//! 使用 trouble-host BLE 栈进行蓝牙广播的示例。
//!
//! 广播设备名 "SiFli-BLE"，可被手机 BLE 扫描器（如 nRF Connect）发现。
//!
//! ## 启动流程
//!
//! 1. HAL 初始化
//! 2. IPC 驱动 + HCI 队列创建
//! 3. LCPU BLE 启动（固件加载、补丁安装、warmup 事件消费）
//! 4. trouble-host Stack 初始化
//! 5. 持续广播，接受连接，断开后重新广播

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use bt_hci::controller::ExternalController;
use trouble_host::prelude::*;

use sifli_hal::bt_hci::IpcHciTransport;
use sifli_hal::lcpu::{Lcpu, LcpuConfig};
use sifli_hal::{bind_interrupts, ipc};

bind_interrupts!(struct Irqs {
    MAILBOX2_CH1 => ipc::InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = sifli_hal::init(Default::default());

    // 等待 probe-rs attach
    // Timer::after(Duration::from_secs(2)).await;

    info!("=== BLE Advertise Example ===");

    // 1. 创建 IPC driver 和 HCI queue
    let mut ipc_driver = ipc::Ipc::new(p.MAILBOX1_CH1, Irqs, ipc::Config::default());
    let queue = match ipc_driver.open_queue(ipc::QueueConfig::qid0_hci()) {
        Ok(q) => q,
        Err(e) => {
            error!("open_queue failed: {:?}", e);
            cortex_m::asm::bkpt();
            loop {
                Timer::after_secs(1).await;
            }
        }
    };

    // 2. BLE 启动
    let (mut rx, tx) = queue.split();
    let lcpu = Lcpu::new(p.LPSYS_AON);
    if let Err(e) = lcpu
        .ble_power_on(&LcpuConfig::default(), p.DMAC2_CH8, &mut rx)
        .await
    {
        error!("ble_power_on failed: {:?}", e);
        cortex_m::asm::bkpt();
        loop {
            Timer::after_secs(1).await;
        }
    }
    info!("LCPU BLE is ready");

    // 3. 创建 HCI transport 和 controller
    let transport = IpcHciTransport::from_parts(rx, tx);
    let controller: ExternalController<_, 4> = ExternalController::new(transport);

    // 4. 初始化 trouble-host Stack
    let address = Address::random([0xC0, 0x12, 0x34, 0x56, 0x78, 0x9A]);
    info!("Address: {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, 1, 1> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        mut runner,
        ..
    } = stack.build();

    // 5. 编码广告数据
    let mut adv_data = [0u8; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(b"SiFli-BLE"),
        ],
        &mut adv_data[..],
    )
    .unwrap();

    // 6. runner + 广播/连接循环并发运行
    info!("Starting advertising...");
    let _ = join(runner.run(), async {
        loop {
            let mut params = AdvertisementParameters::default();
            params.interval_min = Duration::from_millis(100);
            params.interval_max = Duration::from_millis(100);
            let acceptor = match peripheral
                .advertise(
                    &params,
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &adv_data[..len],
                        scan_data: &[],
                    },
                )
                .await
            {
                Ok(a) => {
                    info!("Advertising active, device name: SiFli-BLE");
                    a
                }
                Err(e) => {
                    error!("Advertise failed: {:?}", e);
                    Timer::after(Duration::from_secs(1)).await;
                    continue;
                }
            };

            let conn = match acceptor.accept().await {
                Ok(c) => c,
                Err(e) => {
                    error!("Accept failed: {:?}", e);
                    continue;
                }
            };
            info!("Connected!");

            loop {
                match conn.next().await {
                    ConnectionEvent::Disconnected { reason } => {
                        info!("Disconnected: {:?}", reason);
                        break;
                    }
                    _ => {}
                }
            }
            info!("Re-advertising...");
        }
    })
    .await;
}
