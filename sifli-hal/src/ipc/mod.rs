//! 核间 IPC Queue（MAILBOX doorbell + 共享内存环形缓冲）。
//!
//! 目标：在 HCPU Rust 侧复刻 SiFli SDK `ipc_queue` 的"线协议"（共享内存布局与环形缓冲算法），
//! 以便在无需给 LCPU 编程的前提下，与 ROM/固件侧保持一致行为。
//!
//! # 简化用法
//!
//! ```no_run
//! use sifli_hal::{bind_interrupts, ipc};
//!
//! bind_interrupts!(struct Irqs {
//!     MAILBOX2_CH1 => ipc::InterruptHandler;
//! });
//!
//! let p = sifli_hal::init(Default::default());
//! let rev = sifli_hal::syscfg::read_idr().revision();
//! let mut ipc = ipc::Ipc::new(p.MAILBOX1_CH1, ipc::Config::default());
//! let mut q = ipc.open_queue(ipc::QueueConfig::qid0_hci(rev)).unwrap();
//! ```

mod circular_buf;

use core::future::poll_fn;
use core::marker::PhantomData;
use core::mem;
use core::ptr;
use core::sync::atomic::{fence, AtomicBool, AtomicUsize, Ordering};
use core::task::Poll;

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::{self, InterruptExt};
use crate::lcpu::ram::IpcRegion;
use crate::{peripherals, rcc, syscfg};

use circular_buf::{CircularBuf, CircularBufMutPtrExt, CircularBufPtrExt};

/// SF32LB52x `ipc_hw_port.h`: `IPC_HW_QUEUE_NUM=8`
pub const HW_QUEUE_NUM: usize = 8;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    InvalidQid,
    AlreadyOpen,
    BufferTooSmall,
    NotOpen,
    TxUnavailable,
    RxUnavailable,
}

/// IPC 初始化配置。
#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub irq_priority: interrupt::Priority,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            // SDK `ipc_hw_enable_interrupt`: HAL_NVIC_SetPriority(..., 3, 0)
            irq_priority: interrupt::Priority::P3,
        }
    }
}

/// 单个队列的配置（对齐 SDK `ipc_queue_cfg_t` 的必要字段）。
#[derive(Debug, Clone, Copy)]
pub struct QueueConfig {
    pub qid: u8,
    pub rx_buf_addr: usize,
    pub tx_buf_addr: usize,
    pub tx_buf_addr_alias: usize,
    pub tx_buf_size: usize,
}

impl QueueConfig {
    /// qid0（SDK 蓝牙/HCI 通道，默认使用 MB_CH1 buffer）。
    pub fn qid0_hci(rev: syscfg::ChipRevision) -> Self {
        let tx = IpcRegion::HCPU_TO_LCPU_CH1;
        Self {
            qid: 0,
            rx_buf_addr: IpcRegion::lcpu_to_hcpu_start(rev),
            tx_buf_addr: tx,
            tx_buf_addr_alias: IpcRegion::hcpu_to_lcpu_addr(tx),
            tx_buf_size: IpcRegion::BUF_SIZE,
        }
    }

    /// qid1（SDK system/data-service 常用，默认使用 MB_CH2 buffer）。
    pub fn qid1_system(rev: syscfg::ChipRevision) -> Self {
        let tx = IpcRegion::HCPU_TO_LCPU_CH2;
        Self {
            qid: 1,
            rx_buf_addr: IpcRegion::lcpu_to_hcpu_ch2_start(rev),
            tx_buf_addr: tx,
            tx_buf_addr_alias: IpcRegion::hcpu_to_lcpu_addr(tx),
            tx_buf_size: IpcRegion::BUF_SIZE,
        }
    }
}

struct QueueState {
    active: AtomicBool,
    rx_len: AtomicUsize,
    rx_waker: AtomicWaker,
    rx_buf: AtomicUsize,
    tx_buf: AtomicUsize,
}

impl QueueState {
    const fn new() -> Self {
        Self {
            active: AtomicBool::new(false),
            rx_len: AtomicUsize::new(0),
            rx_waker: AtomicWaker::new(),
            rx_buf: AtomicUsize::new(0),
            tx_buf: AtomicUsize::new(0),
        }
    }
}

static QUEUES: [QueueState; HW_QUEUE_NUM] = [const { QueueState::new() }; HW_QUEUE_NUM];

/// 在访问 LPSYS 域资源（如 `MAILBOX2` 寄存器、LPSYS SRAM）前，临时请求 LCPU 保持 active。
///
/// SDK `ipc_hw_enable_interrupt2()` 在 HCPU 侧会 `HAL_HPAON_WakeCore(LCPU)`，避免在 LPSYS 休眠时访问
/// mailbox 寄存器导致行为未定义/触发 fault。
struct LcpuActiveGuard;

impl LcpuActiveGuard {
    #[inline]
    fn new() -> Self {
        unsafe { rcc::wake_lcpu() };
        Self
    }
}

impl Drop for LcpuActiveGuard {
    #[inline]
    fn drop(&mut self) {
        unsafe { rcc::cancel_lcpu_active_request() };
    }
}

// ============================================================================
// Interrupt handler
// ============================================================================

/// IPC 中断处理器
///
/// 直接在中断中处理 MAILBOX2_CH1 中断并唤醒对应的 IPC queue，
/// 无需 spawn 单独的 task。
///
/// # Example
///
/// ```no_run
/// use sifli_hal::{bind_interrupts, ipc};
///
/// bind_interrupts!(struct Irqs {
///     MAILBOX2_CH1 => ipc::InterruptHandler;
/// });
/// ```
pub struct InterruptHandler {
    _phantom: PhantomData<()>,
}

impl interrupt::typelevel::Handler<interrupt::typelevel::MAILBOX2_CH1> for InterruptHandler {
    unsafe fn on_interrupt() {
        let regs = crate::pac::MAILBOX2;

        // 读取 masked interrupt status
        let misr = regs.misr(0).read().0 as u16;
        if misr == 0 {
            return;
        }

        // 清除中断
        regs.icr(0).write(|w| w.0 = misr as u32);

        fence(Ordering::SeqCst);

        // 遍历触发的 bit，唤醒对应 queue
        for qid in 0..HW_QUEUE_NUM {
            if misr & (1 << qid) != 0 {
                handle_rx_irq(qid as u8);
            }
        }
    }
}

#[inline]
fn handle_rx_irq(qid: u8) {
    let st = &QUEUES[qid as usize];
    if !st.active.load(Ordering::Acquire) {
        return;
    }

    let rx_ptr = st.rx_buf.load(Ordering::Acquire);
    let len = if rx_ptr == 0 {
        0
    } else {
        unsafe { (rx_ptr as *const CircularBuf).data_len() }
    };

    st.rx_len.store(len, Ordering::Release);
    st.rx_waker.wake();
}

// ============================================================================
// IPC driver
// ============================================================================

/// IPC（HCPU 侧）初始化与队列打开。
///
/// # Example
///
/// ```no_run
/// use sifli_hal::{bind_interrupts, ipc};
///
/// bind_interrupts!(struct Irqs {
///     MAILBOX2_CH1 => ipc::InterruptHandler;
/// });
///
/// let p = sifli_hal::init(Default::default());
/// let rev = sifli_hal::syscfg::read_idr().revision();
/// let mut ipc = ipc::Ipc::new(p.MAILBOX1_CH1, ipc::Config::default());
/// let mut q = ipc.open_queue(ipc::QueueConfig::qid0_hci(rev)).unwrap();
/// ```
pub struct Ipc<'d> {
    _tx_ch: PeripheralRef<'d, peripherals::MAILBOX1_CH1>,
}

impl<'d> Ipc<'d> {
    /// Create a new IPC instance
    pub fn new(
        tx_ch: impl Peripheral<P = peripherals::MAILBOX1_CH1> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(tx_ch);

        // MAILBOX1 在 HPSYS，需要打开 RCC（不 reset，避免影响其它 channel 的配置）。
        rcc::enable::<peripherals::MAILBOX1>();

        // 配置 MAILBOX2_CH1 中断优先级
        let irq = crate::interrupt::MAILBOX2_CH1;
        unsafe {
            irq.set_priority(config.irq_priority);
            irq.unpend();
            irq.enable();
        }

        Self { _tx_ch: tx_ch }
    }

    /// 打开一个 IPC Queue（等价于 SDK 的 init+open 的最小子集）。
    pub fn open_queue(&mut self, cfg: QueueConfig) -> Result<IpcQueue, Error> {
        if cfg.qid as usize >= HW_QUEUE_NUM {
            return Err(Error::InvalidQid);
        }
        if cfg.tx_buf_size <= mem::size_of::<CircularBuf>() {
            return Err(Error::BufferTooSmall);
        }

        let st = &QUEUES[cfg.qid as usize];

        critical_section::with(|_| {
            if st.active.swap(true, Ordering::AcqRel) {
                return Err(Error::AlreadyOpen);
            }

            st.rx_buf.store(cfg.rx_buf_addr, Ordering::Release);
            st.tx_buf.store(cfg.tx_buf_addr, Ordering::Release);

            // sender 初始化 tx ring buffer，并把 rd_buffer_ptr 改成对端可见 alias。
            if cfg.tx_buf_addr != 0 {
                unsafe {
                    let cb = cfg.tx_buf_addr as *mut CircularBuf;
                    let pool_wr =
                        (cfg.tx_buf_addr as *mut u8).add(mem::size_of::<CircularBuf>());
                    cb.wr_init(
                        pool_wr,
                        (cfg.tx_buf_size - mem::size_of::<CircularBuf>()) as i16,
                    );

                    let pool_rd =
                        (cfg.tx_buf_addr_alias as *mut u8).add(mem::size_of::<CircularBuf>());
                    cb.rd_init(pool_rd);
                }
            }

            // SDK 行为：open 时不读取 rx ring buffer（避免对端尚未初始化导致误判/崩溃）。
            st.rx_len.store(0, Ordering::Release);
            debug!(
                "IPC open_queue: qid={} rx=0x{:08X} tx=0x{:08X} tx_alias=0x{:08X} tx_size={}",
                cfg.qid,
                cfg.rx_buf_addr as u32,
                cfg.tx_buf_addr as u32,
                cfg.tx_buf_addr_alias as u32,
                cfg.tx_buf_size,
            );

            // Unmask：按 SDK 语义，在 tx mailbox 上放开 qid；同时也放开 rx 侧，降低丢中断风险。
            let qid_mask = 1u16 << cfg.qid;
            // 直接操作 MAILBOX1 寄存器
            crate::pac::MAILBOX1.ier(0).modify(|w| w.0 |= qid_mask as u32);
            {
                // 访问 MAILBOX2（L2H_MAILBOX）前确保 LPSYS 处于 active。
                let _lcpu_active = LcpuActiveGuard::new();
                // 直接操作 MAILBOX2 寄存器（寄存器操作是原子的）
                let mb2 = crate::pac::MAILBOX2;
                mb2.ier(0).modify(|w| w.0 |= qid_mask as u32);
                mb2.icr(0).write(|w| w.0 = qid_mask as u32);
            }

            Ok(IpcQueue { qid: cfg.qid })
        })
    }
}

/// 已打开的 IPC Queue 句柄。
pub struct IpcQueue {
    qid: u8,
}

impl IpcQueue {
    #[inline]
    pub fn qid(&self) -> u8 {
        self.qid
    }

    /// 当前 rx buffer 中可读字节数（由中断更新，必要时可再读一次共享 ring buffer 自校准）。
    pub fn rx_available(&self) -> Result<usize, Error> {
        let st = &QUEUES[self.qid as usize];
        if !st.active.load(Ordering::Acquire) {
            return Err(Error::NotOpen);
        }
        Ok(st.rx_len.load(Ordering::Acquire))
    }

    /// 读数据（非阻塞）。无数据时返回 `Ok(0)`。
    pub fn read(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        let st = &QUEUES[self.qid as usize];
        if !st.active.load(Ordering::Acquire) {
            return Err(Error::NotOpen);
        }

        let rx_ptr = st.rx_buf.load(Ordering::Acquire);
        if rx_ptr == 0 {
            return Err(Error::RxUnavailable);
        }

        if st.rx_len.load(Ordering::Acquire) == 0 {
            return Ok(0);
        }

        debug!(
            "IPC read: qid={} rx_ptr=0x{:08X} rx_len={} out_len={}",
            self.qid,
            rx_ptr as u32,
            st.rx_len.load(Ordering::Acquire),
            out.len()
        );

        let cb = rx_ptr as *mut CircularBuf;
        let n = critical_section::with(|_| {
            fence(Ordering::SeqCst);
            let n = unsafe { cb.get(out) };
            st.rx_len
                .store(unsafe { (cb as *const CircularBuf).data_len() }, Ordering::Release);
            n
        });
        Ok(n)
    }

    /// 等待直到 rx 有数据。
    pub async fn wait_readable(&mut self) -> Result<(), Error> {
        let st = &QUEUES[self.qid as usize];
        if !st.active.load(Ordering::Acquire) {
            return Err(Error::NotOpen);
        }

        poll_fn(|cx| {
            st.rx_waker.register(cx.waker());
            fence(Ordering::SeqCst);

            if st.rx_len.load(Ordering::Acquire) > 0 {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        Ok(())
    }

    /// 异步读（至少读到 1 字节，除非发生错误）。
    pub async fn read_async(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        self.wait_readable().await?;
        self.read(out)
    }

    /// 写数据到 tx ring buffer，并触发 doorbell。
    ///
    /// 与 SDK 一致：该接口不保证线程安全（避免多个线程并发写同一队列）。
    pub fn write(&mut self, data: &[u8]) -> Result<usize, Error> {
        let st = &QUEUES[self.qid as usize];
        if !st.active.load(Ordering::Acquire) {
            return Err(Error::NotOpen);
        }

        let tx_ptr = st.tx_buf.load(Ordering::Acquire);
        if tx_ptr == 0 {
            return Err(Error::TxUnavailable);
        }

        let cb = tx_ptr as *mut CircularBuf;
        let n = unsafe { cb.put(data) };
        if n > 0 {
            fence(Ordering::SeqCst);
            // 触发 MAILBOX1 中断（寄存器写入是原子的）
            crate::pac::MAILBOX1.itr(0).write(|w| w.0 = 1u32 << self.qid);
        }
        Ok(n)
    }

    /// 输出当前队列/MAILBOX/环形缓冲的关键信息（用于定位“是否送达/是否被消费/是否有回包”）。
    ///
    /// 只做 volatile 读取，不会去解引用 pool 指针，避免在对端未初始化时触发 fault。
    pub fn debug_dump(&self) {
        // debug_dump 会读取 MAILBOX2/LPSYS ring buffer，必须先确保 LPSYS 处于 active，
        // 否则在某些低功耗状态下会直接 HardFault（SDK 文档也说明这类访问“行为未定义”）。
        let _lcpu_active = LcpuActiveGuard::new();

        let st = &QUEUES[self.qid as usize];
        let rx_ptr = st.rx_buf.load(Ordering::Acquire);
        let tx_ptr = st.tx_buf.load(Ordering::Acquire);
        let rx_len_cached = st.rx_len.load(Ordering::Acquire);

        let (tx_raw_size, tx_rd, tx_wr, tx_rd_buf, tx_wr_buf, tx_data_len) = if tx_ptr == 0 {
            (0i16, 0u32, 0u32, 0u32, 0u32, 0usize)
        } else {
            unsafe {
                let cb = tx_ptr as *const CircularBuf;
                let raw_size = ptr::read_volatile(ptr::addr_of!((*cb).buffer_size));
                let rd = ptr::read_volatile(ptr::addr_of!((*cb).read_idx_mirror));
                let wr = ptr::read_volatile(ptr::addr_of!((*cb).write_idx_mirror));
                let rd_buf = ptr::read_volatile(ptr::addr_of!((*cb).rd_buffer_ptr)) as u32;
                let wr_buf = ptr::read_volatile(ptr::addr_of!((*cb).wr_buffer_ptr)) as u32;
                let data_len = cb.data_len();
                (raw_size, rd, wr, rd_buf, wr_buf, data_len)
            }
        };

        let (rx_raw_size, rx_rd, rx_wr, rx_rd_buf, rx_wr_buf, rx_data_len) = if rx_ptr == 0 {
            (0i16, 0u32, 0u32, 0u32, 0u32, 0usize)
        } else {
            unsafe {
                let cb = rx_ptr as *const CircularBuf;
                let raw_size = ptr::read_volatile(ptr::addr_of!((*cb).buffer_size));
                let rd = ptr::read_volatile(ptr::addr_of!((*cb).read_idx_mirror));
                let wr = ptr::read_volatile(ptr::addr_of!((*cb).write_idx_mirror));
                let rd_buf = ptr::read_volatile(ptr::addr_of!((*cb).rd_buffer_ptr)) as u32;
                let wr_buf = ptr::read_volatile(ptr::addr_of!((*cb).wr_buffer_ptr)) as u32;
                let data_len = cb.data_len();
                (raw_size, rd, wr, rd_buf, wr_buf, data_len)
            }
        };

        // 直接读取 mailbox 寄存器（只读操作）
        let mb1 = crate::pac::MAILBOX1;
        let mb2 = crate::pac::MAILBOX2;
        let mb1_ier = mb1.ier(0).read().0 as u16;
        let mb1_isr = mb1.isr(0).read().0 as u16;
        let mb1_misr = mb1.misr(0).read().0 as u16;
        let mb2_ier = mb2.ier(0).read().0 as u16;
        let mb2_isr = mb2.isr(0).read().0 as u16;
        let mb2_misr = mb2.misr(0).read().0 as u16;

        debug!(
            "IPC dbg: qid={} rx_len(cached)={} tx=0x{:08X} tx_size={} tx_rd=0x{:08X} tx_wr=0x{:08X} tx_pool(rd=0x{:08X} wr=0x{:08X}) tx_data_len={} rx=0x{:08X} rx_size={} rx_rd=0x{:08X} rx_wr=0x{:08X} rx_pool(rd=0x{:08X} wr=0x{:08X}) rx_data_len={} mb1(ier=0x{:04X} isr=0x{:04X} misr=0x{:04X}) mb2(ier=0x{:04X} isr=0x{:04X} misr=0x{:04X})",
            self.qid,
            rx_len_cached,
            tx_ptr as u32,
            tx_raw_size,
            tx_rd,
            tx_wr,
            tx_rd_buf,
            tx_wr_buf,
            tx_data_len,
            rx_ptr as u32,
            rx_raw_size,
            rx_rd,
            rx_wr,
            rx_rd_buf,
            rx_wr_buf,
            rx_data_len,
            mb1_ier,
            mb1_isr,
            mb1_misr,
            mb2_ier,
            mb2_isr,
            mb2_misr
        );
    }
}
