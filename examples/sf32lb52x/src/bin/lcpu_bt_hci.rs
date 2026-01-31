//! 使用 bt-hci 库与 LCPU BT controller 通信的示例。
//!
//! 演示如何使用 bt-hci 的类型安全 API 发送 HCI Reset 命令。
//!
//! ## LCPU 通信要点
//!
//! ### 1. 必须保持 LCPU 唤醒
//!
//! `lcpu.power_on()` 结束时会调用 `cancel_lcpu_active_request()`，允许 LCPU 进入睡眠。
//! LCPU 一旦睡眠后被唤醒，BT stack 不会重新初始化，导致 HCI 通信失败。
//!
//! **解决方案**：在 `power_on()` 后立即调用 `rcc::wake_lcpu()` 保持 LCPU 唤醒。
//!
//! ### 2. LCPU 启动事件（warmup）的处理
//!
//! LCPU 启动后会发送一个 VSC 0xFC11 的 Command Complete 事件（7 字节）：
//! ```text
//! 04 0E 04 06 11 FC 00
//! │  │  │  │  └──┴── Opcode: 0xFC11 (Vendor Specific)
//! │  │  │  └─ Num_HCI_Command_Packets: 6
//! │  │  └─ Parameter_Total_Length: 4
//! │  └─ Event Code: 0x0E (Command Complete)
//! └─ H4 Indicator: 0x04 (HCI Event)
//! ```
//!
//! **bt-hci 会自动处理**：`ExternalController::read()` 收到 CommandComplete 时会尝试
//! 匹配等待中的命令 slot。由于没有发送过 opcode=0xFC11 的命令，匹配失败后会自动
//! 忽略该事件并继续读取下一个包。
//!
//! 因此只需确保 `hci_reader_task` 在发送命令前启动即可，无需手动处理 warmup 事件。

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer, WithTimeout};
use panic_probe as _;
use static_cell::StaticCell;

use bt_hci::cmd::controller_baseband::Reset;
use bt_hci::controller::{Controller, ControllerCmdSync, ExternalController};

use sifli_hal::bt_hci::IpcHciTransport;
use sifli_hal::lcpu::{Lcpu, LcpuConfig};
use sifli_hal::{bind_interrupts, ipc, rcc, syscfg};

bind_interrupts!(struct Irqs {
    MAILBOX2_CH1 => ipc::InterruptHandler;
});

type BtController = ExternalController<IpcHciTransport, 4>;
static CONTROLLER: StaticCell<BtController> = StaticCell::new();

/// HCI 事件读取任务，必须持续运行以处理来自 controller 的事件。
///
/// 该任务会自动处理：
/// - LCPU 启动时发送的 warmup 事件（VSC 0xFC11）- 自动忽略
/// - 命令响应（CommandComplete/CommandStatus）- 内部匹配并通知 exec()
/// - 其他事件（LE Meta Event 等）- 返回供上层处理
#[embassy_executor::task]
async fn hci_reader_task(controller: &'static BtController) {
    let mut buf = [0u8; 259]; // Max HCI event size
    loop {
        match controller.read(&mut buf).await {
            Ok(_packet) => {
                // CommandComplete/CommandStatus 事件由 ExternalController 内部处理
                // 其他事件（如 LE Meta Event）会返回到这里
                debug!("HCI event received");
            }
            Err(_e) => {
                error!("HCI read error");
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = sifli_hal::init(Default::default());
    let rev = syscfg::read_idr().revision();

    let lcpu = Lcpu::new(p.LPSYS_AON);
    if let Err(e) = lcpu.power_on(&LcpuConfig::default()) {
        error!("LCPU power_on failed: {:?}", e);
        loop {
            Timer::after_secs(1).await;
        }
    }
    // 保持 LCPU 唤醒（power_on 结束时允许了睡眠，需要重新唤醒）
    unsafe { rcc::wake_lcpu() };
    info!("LCPU is running");

    let mut ipc_driver = ipc::Ipc::new(p.MAILBOX1_CH1, Irqs, ipc::Config::default());
    let queue = match ipc_driver.open_queue(ipc::QueueConfig::qid0_hci(rev)) {
        Ok(q) => q,
        Err(e) => {
            error!("open_queue(qid0) failed: {:?}", e);
            loop {
                Timer::after_secs(1).await;
            }
        }
    };
    info!("IPC queue opened: qid={}", queue.qid());

    // 创建 bt-hci Transport 和 Controller
    let transport = IpcHciTransport::new(queue);
    let controller = CONTROLLER.init(ExternalController::new(transport));

    // 先启动 HCI 事件读取任务
    // 该任务会自动处理 LCPU warmup 事件（匹配失败后忽略）
    spawner.spawn(hci_reader_task(controller).expect("spawn hci_reader_task"));
    info!("HCI reader task started");

    // 等待 reader task 开始运行并处理 warmup 事件
    Timer::after(Duration::from_micros(1000)).await;

    // 发送 HCI Reset 命令
    info!("Sending HCI Reset command...");
    match controller
        .exec(&Reset::new())
        .with_timeout(Duration::from_secs(2))
        .await
    {
        Ok(Ok(_)) => {
            info!("HCI Reset succeeded!");
        }
        Ok(Err(_e)) => {
            error!("HCI Reset command failed");
        }
        Err(_timeout) => {
            error!("Timeout waiting for HCI Reset response");
        }
    }

    info!("Demo completed");
    loop {
        Timer::after_secs(1).await;
    }
}
