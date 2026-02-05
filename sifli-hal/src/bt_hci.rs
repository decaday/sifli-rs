//! bt-hci Transport 实现，基于 IPC 与 LCPU BT controller 通信。
//!
//! 此模块为 [`IpcQueue`] 提供 bt-hci 兼容的 Transport 封装，
//! 使得可以使用 `bt-hci` 和 `trouble` 等上层库与 LCPU 上的蓝牙控制器通信。
//!
//! # Example
//!
//! ```no_run
//! use sifli_hal::{bind_interrupts, ipc, bt_hci::IpcHciTransport};
//! use bt_hci::controller::ExternalController;
//!
//! bind_interrupts!(struct Irqs {
//!     MAILBOX2_CH1 => ipc::InterruptHandler;
//! });
//!
//! async fn example() {
//!     let p = sifli_hal::init(Default::default());
//!     let rev = sifli_hal::syscfg::read_idr().revision();
//!     let mut ipc_driver = ipc::Ipc::new(p.MAILBOX1_CH1, Irqs, ipc::Config::default());
//!     let queue = ipc_driver.open_queue(ipc::QueueConfig::qid0_hci(rev)).unwrap();
//!
//!     let transport = IpcHciTransport::new(queue);
//!     let controller: ExternalController<_, 4> = ExternalController::new(transport);
//!
//!     // 现在可以使用 bt-hci 的类型安全 API
//!     // use bt_hci::cmd::controller_baseband::Reset;
//!     // controller.exec(Reset).await.unwrap();
//! }
//! ```

use bt_hci::transport::{Transport, WithIndicator};
use bt_hci::{ControllerToHostPacket, HostToControllerPacket, ReadHci, ReadHciError, WriteHci};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embedded_io::ReadExactError;

use crate::ipc::{Error as IpcError, IpcQueue, IpcQueueRx, IpcQueueTx};

/// bt-hci Transport 错误类型。
#[derive(Debug)]
pub enum Error {
    /// IPC 读取错误。
    Read(ReadHciError<IpcError>),
    /// IPC 写入错误。
    Write(IpcError),
}

#[cfg(feature = "defmt")]
impl defmt::Format for Error {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Error::Read(e) => defmt::write!(f, "HCI read error: {:?}", defmt::Debug2Format(e)),
            Error::Write(e) => defmt::write!(f, "HCI write error: {}", e),
        }
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Read(e) => write!(f, "HCI read error: {:?}", e),
            Error::Write(e) => write!(f, "HCI write error: {}", e),
        }
    }
}

impl core::error::Error for Error {}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Self::Read(e) => e.kind(),
            Self::Write(e) => e.kind(),
        }
    }
}

impl From<IpcError> for Error {
    fn from(e: IpcError) -> Self {
        Self::Write(e)
    }
}

impl From<ReadHciError<IpcError>> for Error {
    fn from(e: ReadHciError<IpcError>) -> Self {
        Self::Read(e)
    }
}

impl From<ReadExactError<IpcError>> for Error {
    fn from(e: ReadExactError<IpcError>) -> Self {
        Self::Read(e.into())
    }
}

impl From<bt_hci::FromHciBytesError> for Error {
    fn from(e: bt_hci::FromHciBytesError) -> Self {
        Self::Read(e.into())
    }
}

/// IPC HCI Transport，将 [`IpcQueue`] 包装为 bt-hci Transport。
///
/// 使用 H4 格式（带 packet indicator 字节）通信。
/// 内部将 RX 和 TX 分离，使用独立的 Mutex 保护，支持并发读写。
pub struct IpcHciTransport {
    rx: Mutex<CriticalSectionRawMutex, IpcQueueRx>,
    tx: Mutex<CriticalSectionRawMutex, IpcQueueTx>,
}

impl IpcHciTransport {
    /// 创建新的 IPC HCI Transport。
    pub fn new(queue: IpcQueue) -> Self {
        let (rx, tx) = queue.split();
        Self::from_parts(rx, tx)
    }

    /// 从已拆分的 RX/TX 端创建 Transport。
    ///
    /// 当需要在创建 Transport 之前访问 RX 端（如消费 warmup 事件）时使用此方法。
    ///
    /// # Example
    ///
    /// ```no_run
    /// # async fn example(queue: sifli_hal::ipc::IpcQueue) {
    /// use sifli_hal::bt_hci::IpcHciTransport;
    ///
    /// let (mut rx, tx) = queue.split();
    /// // 可以先用 rx 做一些操作（如 ble_power_on 消费 warmup）
    /// let transport = IpcHciTransport::from_parts(rx, tx);
    /// # }
    /// ```
    pub fn from_parts(rx: IpcQueueRx, tx: IpcQueueTx) -> Self {
        Self {
            rx: Mutex::new(rx),
            tx: Mutex::new(tx),
        }
    }
}

impl embedded_io::ErrorType for IpcHciTransport {
    type Error = Error;
}

impl Transport for IpcHciTransport {
    async fn read<'a>(&self, rx: &'a mut [u8]) -> Result<ControllerToHostPacket<'a>, Self::Error> {
        debug!("bt_hci: Transport::read called, rx.len={}", rx.len());
        let mut q = self.rx.lock().await;
        // bt-hci 的 read_hci_async 会解析 H4 格式的 HCI 包
        ControllerToHostPacket::read_hci_async(&mut *q, rx)
            .await
            .map_err(Error::Read)
    }

    async fn write<T: HostToControllerPacket>(&self, val: &T) -> Result<(), Self::Error> {
        debug!("bt_hci: Transport::write called");
        let mut q = self.tx.lock().await;
        // WithIndicator 会在包前添加 H4 packet indicator
        WithIndicator::new(val)
            .write_hci_async(&mut *q)
            .await
            .map_err(Error::Write)?;
        // 触发 doorbell 通知 LCPU（bt-hci 的 write_hci_async 不会调用 flush）
        q.flush()?;
        Ok(())
    }
}
