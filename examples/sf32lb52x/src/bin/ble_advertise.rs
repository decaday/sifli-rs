//! 使用 trouble-host BLE 栈进行蓝牙广播的示例。
//!
//! 广播设备名 "SiFli-BLE"，注册 Battery Service (0x180F)，
//! 可被手机 BLE 扫描器（如 nRF Connect）发现并完成 Service Discovery。
//!
//! ## 启动流程
//!
//! 1. HAL 初始化
//! 2. IPC 驱动 + HCI 队列创建
//! 3. LCPU BLE 启动（固件加载、补丁安装、warmup 事件消费）
//! 4. trouble-host Stack 初始化（含 GATT 服务）
//! 5. 持续广播，接受连接，处理 GATT 事件，断开后重新广播

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_time::{Duration, Timer};
use panic_probe as _;

use trouble_host::prelude::*;

use sifli_hal::bt_hci::IpcHciTransport;
use sifli_hal::lcpu::{Lcpu, LcpuConfig};
use sifli_hal::{bind_interrupts, ipc};

bind_interrupts!(struct Irqs {
    MAILBOX2_CH1 => ipc::InterruptHandler;
});

/// GATT 服务定义：Battery Service (UUID 0x180F)
#[gatt_server]
struct Server {
    battery_service: BatteryService,
}

#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 100)]
    level: u8,
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = sifli_hal::init(Default::default());

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

    let mut resources: HostResources<DefaultPacketPool, 1, 2> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        mut runner,
        ..
    } = stack.build();

    // 5. 创建 GATT 服务器
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "SiFli-BLE",
        appearance: &appearance::UNKNOWN,
    }))
    .unwrap();

    // 6. runner + 广播/连接循环并发运行
    info!("Starting advertising...");
    let _ = join(
        async {
            loop {
                if let Err(e) = runner.run().await {
                    error!("[ble] runner error: {:?}", e);
                }
            }
        },
        async {
            loop {
                // 广播
                match advertise(&mut peripheral, &server).await {
                    Ok(conn) => {
                        info!("Connected!");
                        // GATT 事件处理 + 电量通知 并发运行
                        let gatt_task = gatt_events_task(&server, &conn);
                        let notify_task = notification_task(&server, &conn);
                        select(gatt_task, notify_task).await;
                    }
                    Err(e) => {
                        error!("Advertise/accept error: {:?}", e);
                        Timer::after(Duration::from_secs(1)).await;
                    }
                }
                info!("Re-advertising...");
            }
        },
    )
    .await;
}

/// 广播并等待连接
async fn advertise<'values, 'server, C: Controller>(
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut adv_data = [0u8; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[[0x0f, 0x18]]), // Battery Service
            AdStructure::CompleteLocalName(b"SiFli-BLE"),
        ],
        &mut adv_data[..],
    )
    .unwrap();

    let mut params = AdvertisementParameters::default();
    params.interval_min = Duration::from_millis(100);
    params.interval_max = Duration::from_millis(100);

    let acceptor = peripheral
        .advertise(
            &params,
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..len],
                scan_data: &[],
            },
        )
        .await?;

    info!("Advertising active, device name: SiFli-BLE");
    let conn = acceptor.accept().await?.with_attribute_server(server)?;
    Ok(conn)
}

/// GATT 事件处理任务
async fn gatt_events_task(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
) {
    let level = server.battery_service.level;
    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                info!("Disconnected: {:?}", reason);
                break;
            }
            GattConnectionEvent::Gatt { event } => {
                match &event {
                    GattEvent::Read(e) => {
                        if e.handle() == level.handle {
                            let value = server.get(&level);
                            info!("[gatt] Read battery level: {:?}", value);
                        }
                    }
                    GattEvent::Write(e) => {
                        info!("[gatt] Write handle={}", e.handle());
                    }
                    _ => {}
                }
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => warn!("[gatt] error: {:?}", e),
                }
            }
            _ => {}
        }
    }
}

/// 电池电量通知任务：每 30 秒发送一次通知
async fn notification_task(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
) {
    let level = server.battery_service.level;
    let mut battery = 100u8;
    loop {
        Timer::after(Duration::from_secs(30)).await;
        battery = battery.saturating_sub(1).max(20);
        if server.set(&level, &battery).is_ok() {
            info!("[notify] Battery level: {}", battery);
            if let Err(e) = level.notify(conn, &battery).await {
                warn!("[notify] error: {:?}", e);
                break;
            }
        }
    }
}
