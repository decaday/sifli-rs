//! 核间 IPC Queue（MAILBOX doorbell + 共享内存环形缓冲）。
//!
//! 在 HCPU Rust 侧复刻 SiFli SDK `ipc_queue` 的线协议，与 LCPU ROM/固件保持兼容。
//!
//! # 架构说明
//!
//! IPC 机制由两部分组成：**中断通知**（doorbell）和**共享内存**（环形缓冲区）。
//!
//! ## 1. 中断通知（MAILBOX 硬件）
//!
//! SF32LB52x 有两个 MAILBOX 外设：
//! - **MAILBOX1**（4 个通道 C1-C4）：HCPU 写 → 触发 LCPU 中断
//! - **MAILBOX2**（2 个通道 C1-C2）：LCPU 写 → 触发 HCPU 中断
//!
//! **IPC queue 只使用 MAILBOX1_C1 和 MAILBOX2_C1**，每个通道的中断寄存器有 16 个独立的位，
//! IPC 用其中 bit 0-7 对应 qid 0-7。当需要通知对方时，写 ITR 寄存器的对应位触发中断。
//!
//! ```text
//! MAILBOX1_C1.IER/ITR/ISR:  [bit15 ... bit8 | bit7(qid7) ... bit1(qid1) bit0(qid0)]
//! MAILBOX2_C1.IER/ITR/ISR:  [bit15 ... bit8 | bit7(qid7) ... bit1(qid1) bit0(qid0)]
//! ```
//!
//! **MAILBOX1_C2/C3/C4 和 MAILBOX2_C2 在 IPC queue 机制中未使用。**
//!
//! ## 2. 共享内存（SRAM 缓冲区）
//!
//! 数据通过 SRAM 中的环形缓冲区传输。SDK 预分配了两组缓冲区（每组 512 字节）：
//!
//! | 缓冲区名称 | HCPU 地址 | 用途 |
//! |-----------|----------|------|
//! | HCPU2LCPU_BUF1 | 0x2007FE00 | qid0 (HCI) 的 TX |
//! | HCPU2LCPU_BUF2 | 0x2007FC00 | qid1-7 的 TX |
//! | LCPU2HCPU_BUF1 | 0x20405C00 | qid0 (HCI) 的 RX |
//! | LCPU2HCPU_BUF2 | 0x20405E00 | qid1-7 的 RX |
//!
//! 注意：缓冲区名称中的 "BUF1/BUF2" 与 MAILBOX 的物理通道 C1/C2 **没有对应关系**。
//!
//! ## 3. SDK qid 分配（sf32lb52x）
//!
//! | qid | 缓冲区 | 用途 |
//! |-----|--------|------|
//! | 0 | BUF1 | HCI（蓝牙 controller 通信）|
//! | 1 | BUF2 | 系统 IPC（data service 等）|
//! | 6 | BUF2 | 蓝牙音频 |
//! | 7 | BUF2 | 调试/日志 |
//!
//! # 用法示例
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
//! let mut ipc = ipc::Ipc::new(p.MAILBOX1_CH1, Irqs, ipc::Config::default());
//! let mut q = ipc.open_queue(ipc::QueueConfig::qid0_hci(rev)).unwrap();
//! ```

mod circular_buf;

use core::future::poll_fn;
use core::marker::PhantomData;
use core::mem;
use core::sync::atomic::{fence, AtomicBool, AtomicUsize, Ordering};
use core::task::Poll;

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::{self, InterruptExt};
use crate::lcpu::ram::IpcRegion;
use crate::{peripherals, rcc};

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

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::InvalidQid => write!(f, "invalid queue ID"),
            Error::AlreadyOpen => write!(f, "queue already open"),
            Error::BufferTooSmall => write!(f, "buffer too small"),
            Error::NotOpen => write!(f, "queue not open"),
            Error::TxUnavailable => write!(f, "TX unavailable"),
            Error::RxUnavailable => write!(f, "RX unavailable"),
        }
    }
}

impl core::error::Error for Error {}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Error::NotOpen | Error::AlreadyOpen => embedded_io::ErrorKind::NotConnected,
            Error::InvalidQid | Error::BufferTooSmall => embedded_io::ErrorKind::InvalidInput,
            Error::TxUnavailable | Error::RxUnavailable => embedded_io::ErrorKind::Other,
        }
    }
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
    /// qid0：SDK 蓝牙 HCI 通道，使用 BUF1 缓冲区。
    pub fn qid0_hci() -> Self {
        let tx = IpcRegion::HCPU_TO_LCPU_CH1;
        Self {
            qid: 0,
            rx_buf_addr: IpcRegion::lcpu_to_hcpu_start(),
            tx_buf_addr: tx,
            tx_buf_addr_alias: IpcRegion::hcpu_to_lcpu_addr(tx),
            tx_buf_size: IpcRegion::BUF_SIZE,
        }
    }

    /// qid1：SDK 系统 IPC 通道（data-service 等），使用 BUF2 缓冲区。
    pub fn qid1_system() -> Self {
        let tx = IpcRegion::HCPU_TO_LCPU_CH2;
        Self {
            qid: 1,
            rx_buf_addr: IpcRegion::lcpu_to_hcpu_ch2_start(),
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
        // LCPU 写入 circular buffer 数据后触发 mailbox 中断。
        // Acquire load 只保证 rx_ptr 指针可见，不保证 LCPU 对 buffer
        // 内容（write_idx_mirror 等字段）的写入已刷新到共享 SRAM。
        // 显式 SeqCst fence 确保后续 volatile read 能看到最新数据。
        fence(Ordering::SeqCst);
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
/// let mut ipc = ipc::Ipc::new(p.MAILBOX1_CH1, Irqs, ipc::Config::default());
/// let mut q = ipc.open_queue(ipc::QueueConfig::qid0_hci()).unwrap();
/// ```
pub struct Ipc<'d> {
    _tx_ch: PeripheralRef<'d, peripherals::MAILBOX1_CH1>,
}

impl<'d> Ipc<'d> {
    /// Create a new IPC instance
    pub fn new(
        tx_ch: impl Peripheral<P = peripherals::MAILBOX1_CH1> + 'd,
        _irq: impl interrupt::typelevel::Binding<interrupt::typelevel::MAILBOX2_CH1, InterruptHandler>,
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
                    let pool_wr = (cfg.tx_buf_addr as *mut u8).add(mem::size_of::<CircularBuf>());
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
            crate::pac::MAILBOX1
                .ier(0)
                .modify(|w| w.0 |= qid_mask as u32);
            {
                let mb2 = crate::pac::MAILBOX2;
                mb2.ier(0).modify(|w| w.0 |= qid_mask as u32);
                mb2.icr(0).write(|w| w.0 = qid_mask as u32);
            }

            Ok(IpcQueue {
                rx: IpcQueueRx { qid: cfg.qid },
                tx: IpcQueueTx { qid: cfg.qid },
            })
        })
    }
}

/// 已打开的 IPC Queue 句柄。
pub struct IpcQueue {
    rx: IpcQueueRx,
    tx: IpcQueueTx,
}

/// IPC Queue 的接收端（RX）。
///
/// 通过 [`IpcQueue::split()`] 获取，可独立于 TX 端进行读取操作。
pub struct IpcQueueRx {
    qid: u8,
}

/// IPC Queue 的发送端（TX）。
///
/// 通过 [`IpcQueue::split()`] 获取，可独立于 RX 端进行写入操作。
pub struct IpcQueueTx {
    qid: u8,
}

impl IpcQueue {
    /// 返回队列 ID。
    #[inline]
    pub fn qid(&self) -> u8 {
        self.rx.qid
    }

    /// 将 IpcQueue 拆分为独立的 RX 和 TX 端。
    ///
    /// 拆分后，RX 和 TX 可以在不同的任务中并发使用，不会互相阻塞。
    /// 这对于需要同时进行读写操作的场景（如 HCI 通信）非常有用。
    ///
    /// # Example
    ///
    /// ```no_run
    /// let queue = ipc.open_queue(cfg)?;
    /// let (rx, tx) = queue.split();
    /// // rx 和 tx 可以在不同任务中使用
    /// ```
    pub fn split(self) -> (IpcQueueRx, IpcQueueTx) {
        (self.rx, self.tx)
    }

    /// 当前 rx buffer 中可读字节数（由中断更新，必要时可再读一次共享 ring buffer 自校准）。
    #[inline]
    pub fn rx_available(&self) -> Result<usize, Error> {
        self.rx.rx_available()
    }

    /// 读数据（非阻塞）。无数据时返回 `Ok(0)`。
    #[inline]
    pub fn read(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        self.rx.read(out)
    }

    /// 等待直到 rx 有数据。
    #[inline]
    pub async fn wait_readable(&mut self) -> Result<(), Error> {
        self.rx.wait_readable().await
    }

    /// 异步读（至少读到 1 字节，除非发生错误）。
    #[inline]
    pub async fn read_async(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        self.rx.read_async(out).await
    }

    /// 写数据到 tx ring buffer。
    ///
    /// 与 SDK 一致：该接口不保证线程安全（避免多个线程并发写同一队列）。
    #[inline]
    pub fn write(&mut self, data: &[u8]) -> Result<usize, Error> {
        self.tx.write(data)
    }

    /// 触发 doorbell 通知对端有数据可读。
    ///
    /// 调用 `write` 后需要调用 `flush` 才能通知 LCPU。
    /// 这样设计是为了支持多次 write 后只触发一次 doorbell，避免 LCPU 收到不完整的包。
    #[inline]
    pub fn flush(&mut self) -> Result<(), Error> {
        self.tx.flush()
    }
}

// ============================================================================
// IpcQueueRx implementation
// ============================================================================

impl IpcQueueRx {
    #[inline]
    pub fn qid(&self) -> u8 {
        self.qid
    }

    /// 当前 rx buffer 中可读字节数。
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
            st.rx_len.store(
                unsafe { (cb as *const CircularBuf).data_len() },
                Ordering::Release,
            );
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
            if st.rx_len.load(Ordering::Acquire) > 0 {
                return Poll::Ready(());
            }
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
}

impl embedded_io_async::ErrorType for IpcQueueRx {
    type Error = Error;
}

impl embedded_io_async::Read for IpcQueueRx {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.wait_readable().await?;
        IpcQueueRx::read(self, buf)
    }
}

// ============================================================================
// IpcQueueTx implementation
// ============================================================================

impl IpcQueueTx {
    #[inline]
    pub fn qid(&self) -> u8 {
        self.qid
    }

    /// 写数据到 tx ring buffer。
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
        Ok(n)
    }

    /// 触发 doorbell 通知对端有数据可读。
    ///
    /// 调用 `write` 后需要调用 `flush` 才能通知 LCPU。
    pub fn flush(&mut self) -> Result<(), Error> {
        let st = &QUEUES[self.qid as usize];
        if !st.active.load(Ordering::Acquire) {
            return Err(Error::NotOpen);
        }

        fence(Ordering::SeqCst);
        crate::pac::MAILBOX1
            .itr(0)
            .write(|w| w.0 = 1u32 << self.qid);
        Ok(())
    }
}

impl embedded_io_async::ErrorType for IpcQueueTx {
    type Error = Error;
}

impl embedded_io_async::Write for IpcQueueTx {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        IpcQueueTx::write(self, buf)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        IpcQueueTx::flush(self)
    }
}

// ============================================================================
// embedded_io_async trait implementations for IpcQueue
// ============================================================================

impl embedded_io_async::ErrorType for IpcQueue {
    type Error = Error;
}

impl embedded_io_async::Read for IpcQueue {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // embedded_io::Read 语义要求：至少读取 1 字节（除非 EOF 或错误）
        // 这里使用 wait_readable 等待数据可用，然后读取
        self.wait_readable().await?;
        IpcQueue::read(self, buf)
    }
}

impl embedded_io_async::Write for IpcQueue {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        // IpcQueue::write 是非阻塞的，直接调用
        IpcQueue::write(self, buf)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // 触发 doorbell 通知 LCPU
        IpcQueue::flush(self)
    }
}
