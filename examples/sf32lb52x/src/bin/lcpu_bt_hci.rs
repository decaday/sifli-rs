//! 使用 bt-hci 库与 LCPU BT controller 通信的示例。
//!
//! 测试 trouble-host 所需的所有 HCI 命令，验证 LCPU BT controller 的兼容性。
//!
//! ## LCPU BLE 启动流程
//!
//! 使用 [`Lcpu::ble_power_on`] 一次性完成：
//! 1. 启动 LCPU（加载固件、安装补丁）
//! 2. 保持 LCPU 唤醒（避免睡眠后 BT stack 失效）
//! 3. 消费 BT warmup 事件（VSC 0xFC11 Command Complete）

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer, WithTimeout};
use panic_probe as _;
use static_cell::StaticCell;

// Controller & Baseband 命令
use bt_hci::cmd::controller_baseband::{Reset, SetEventMask, SetEventMaskPage2};
// Informational 命令
use bt_hci::cmd::info::{ReadBdAddr, ReadLocalSupportedFeatures, ReadLocalVersionInformation};
// LE 命令
use bt_hci::cmd::le::{
    LeClearFilterAcceptList, LeReadBufferSize, LeReadFilterAcceptListSize,
    LeReadLocalSupportedFeatures, LeReadSupportedStates, LeSetAdvData, LeSetAdvEnable,
    LeSetAdvParams, LeSetEventMask, LeSetRandomAddr, LeSetScanEnable, LeSetScanParams,
};
// 参数类型
use bt_hci::param::{
    AddrKind, AdvChannelMap, AdvFilterPolicy, AdvKind, BdAddr, EventMask, EventMaskPage2,
    LeEventMask, LeScanKind, ScanningFilterPolicy,
};

use bt_hci::controller::{Controller, ControllerCmdSync, ExternalController};

use sifli_hal::bt_hci::IpcHciTransport;
use sifli_hal::lcpu::{Lcpu, LcpuConfig};
use sifli_hal::{bind_interrupts, ipc, syscfg};

bind_interrupts!(struct Irqs {
    MAILBOX2_CH1 => ipc::InterruptHandler;
});

type BtController = ExternalController<IpcHciTransport, 4>;
static CONTROLLER: StaticCell<BtController> = StaticCell::new();

/// 命令超时时间
const CMD_TIMEOUT: Duration = Duration::from_secs(2);

/// HCI 事件读取任务，必须持续运行以处理来自 controller 的事件。
#[embassy_executor::task]
async fn hci_reader_task(controller: &'static BtController) {
    let mut buf = [0u8; 259]; // Max HCI event size
    loop {
        match controller.read(&mut buf).await {
            Ok(_packet) => {
                debug!("HCI event received");
            }
            Err(_e) => {
                error!("HCI read error");
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }
}

/// 执行同步 HCI 命令的辅助宏
macro_rules! exec_cmd {
    ($controller:expr, $cmd:expr, $name:literal) => {
        match $controller.exec(&$cmd).with_timeout(CMD_TIMEOUT).await {
            Ok(Ok(ret)) => {
                info!("{} succeeded", $name);
                Some(ret)
            }
            Ok(Err(e)) => {
                error!("{} failed: {:?}", $name, e);
                None
            }
            Err(_) => {
                error!("{} timeout", $name);
                None
            }
        }
    };
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = sifli_hal::init(Default::default());
    let rev = syscfg::read_idr().revision();

    info!("=== LCPU BT HCI Commands Test ===");
    info!("Testing commands required by trouble-host");

    // 1. 创建 IPC driver 和 HCI queue
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

    // 2. 拆分 queue，以便 ble_power_on 可以读取 warmup 事件
    let (mut rx, tx) = queue.split();

    // 3. BLE 启动
    let lcpu = Lcpu::new(p.LPSYS_AON);
    if let Err(e) = lcpu.ble_power_on(&LcpuConfig::default(), &mut rx).await {
        error!("LCPU ble_power_on failed: {:?}", e);
        loop {
            Timer::after_secs(1).await;
        }
    }
    info!("LCPU BLE is ready");

    // 4. 创建 bt-hci Transport 和 Controller
    let transport = IpcHciTransport::from_parts(rx, tx);
    let controller = CONTROLLER.init(ExternalController::new(transport));

    // 5. 启动 HCI 事件读取任务
    spawner.spawn(hci_reader_task(controller).unwrap());
    info!("HCI reader task started");

    // ========================================
    // 测试 trouble-host 初始化所需的命令
    // ========================================

    info!("");
    info!("=== Phase 1: Controller Reset & Info ===");

    // HCI Reset
    exec_cmd!(controller, Reset::new(), "Reset");

    // Read Local Version Information
    if let Some(ret) = exec_cmd!(
        controller,
        ReadLocalVersionInformation::new(),
        "ReadLocalVersionInformation"
    ) {
        let hci_version = ret.hci_version;
        let hci_subversion = ret.hci_subversion;
        let lmp_version = ret.lmp_version;
        let lmp_subversion = ret.lmp_subversion;
        let company_identifier = ret.company_identifier;
        info!(
            "  HCI Version: {:?}, Subversion: {}",
            hci_version, hci_subversion
        );
        info!(
            "  LMP Version: {:?}, Subversion: {}",
            lmp_version, lmp_subversion
        );
        info!("  Company: 0x{:04X}", company_identifier);
    }

    // Read Local Supported Features
    if let Some(ret) = exec_cmd!(
        controller,
        ReadLocalSupportedFeatures::new(),
        "ReadLocalSupportedFeatures"
    ) {
        info!("  LMP Features: {:?}", ret);
    }

    // Read BD_ADDR
    if let Some(ret) = exec_cmd!(controller, ReadBdAddr::new(), "ReadBdAddr") {
        info!("  BD_ADDR: {:?}", ret.raw());
    }

    info!("");
    info!("=== Phase 2: Event Masks ===");

    // Set Event Mask
    let event_mask = EventMask::new()
        .enable_le_meta(true)
        .enable_conn_request(true)
        .enable_conn_complete(true)
        .enable_hardware_error(true)
        .enable_disconnection_complete(true)
        .enable_encryption_change_v1(true);
    exec_cmd!(controller, SetEventMask::new(event_mask), "SetEventMask");

    // Set Event Mask Page 2
    let event_mask_page2 = EventMaskPage2::new().enable_encryption_change_v2(true);
    exec_cmd!(
        controller,
        SetEventMaskPage2::new(event_mask_page2),
        "SetEventMaskPage2"
    );

    // LE Set Event Mask
    let le_event_mask = LeEventMask::new()
        .enable_le_conn_complete(true)
        .enable_le_enhanced_conn_complete(true)
        .enable_le_conn_update_complete(true)
        .enable_le_adv_set_terminated(true)
        .enable_le_adv_report(true)
        .enable_le_scan_timeout(true)
        .enable_le_ext_adv_report(true)
        .enable_le_long_term_key_request(true)
        .enable_le_phy_update_complete(true)
        .enable_le_data_length_change(true);
    exec_cmd!(
        controller,
        LeSetEventMask::new(le_event_mask),
        "LeSetEventMask"
    );

    info!("");
    info!("=== Phase 3: LE Parameters ===");

    // LE Read Buffer Size
    if let Some(ret) = exec_cmd!(controller, LeReadBufferSize::new(), "LeReadBufferSize") {
        let acl_len = ret.le_acl_data_packet_length;
        let num_packets = ret.total_num_le_acl_data_packets;
        info!(
            "  ACL Data Packet Length: {}, Num Packets: {}",
            acl_len, num_packets
        );
    }

    // LE Read Local Supported Features
    if let Some(ret) = exec_cmd!(
        controller,
        LeReadLocalSupportedFeatures::new(),
        "LeReadLocalSupportedFeatures"
    ) {
        info!("  LE Features: {:?}", ret);
    }

    // LE Read Supported States
    if let Some(ret) = exec_cmd!(
        controller,
        LeReadSupportedStates::new(),
        "LeReadSupportedStates"
    ) {
        info!("  LE States: {:?}", ret);
    }

    // LE Read Filter Accept List Size
    if let Some(ret) = exec_cmd!(
        controller,
        LeReadFilterAcceptListSize::new(),
        "LeReadFilterAcceptListSize"
    ) {
        info!("  Filter Accept List Size: {}", ret);
    }

    // LE Clear Filter Accept List
    exec_cmd!(
        controller,
        LeClearFilterAcceptList::new(),
        "LeClearFilterAcceptList"
    );

    info!("");
    info!("=== Phase 4: Random Address ===");

    // LE Set Random Address (使用静态随机地址)
    // 最高两位必须是 11（static random address）
    let random_addr = BdAddr::new([0xC0, 0x12, 0x34, 0x56, 0x78, 0x9A]);
    exec_cmd!(
        controller,
        LeSetRandomAddr::new(random_addr),
        "LeSetRandomAddr"
    );

    info!("");
    info!("=== Phase 5: Advertising ===");

    // LE Set Advertising Parameters
    let adv_params = LeSetAdvParams::new(
        bt_hci::param::Duration::from_millis(100), // min interval
        bt_hci::param::Duration::from_millis(100), // max interval
        AdvKind::AdvNonconnInd,                    // non-connectable, non-scannable
        AddrKind::RANDOM,                          // use random address
        AddrKind::PUBLIC,                          // peer address kind (not used)
        BdAddr::default(),                         // peer address (not used)
        AdvChannelMap::ALL,                        // all channels
        AdvFilterPolicy::Unfiltered,               // accept all
    );
    exec_cmd!(controller, adv_params, "LeSetAdvParams");

    // LE Set Advertising Data
    let mut adv_data = [0u8; 31];
    // Simple advertisement: Flags + Complete Local Name
    adv_data[0] = 0x02; // length
    adv_data[1] = 0x01; // type: Flags
    adv_data[2] = 0x06; // LE General Discoverable + BR/EDR Not Supported

    adv_data[3] = 0x08; // length
    adv_data[4] = 0x09; // type: Complete Local Name
    adv_data[5..12].copy_from_slice(b"SF32-BT"); // "SF32-BT"

    exec_cmd!(
        controller,
        LeSetAdvData::new(12, adv_data),
        "LeSetAdvData"
    );

    // LE Set Advertising Enable (enable)
    exec_cmd!(
        controller,
        LeSetAdvEnable::new(true),
        "LeSetAdvEnable(true)"
    );

    // 广播一会儿
    info!("Advertising for 2 seconds...");
    Timer::after_secs(2).await;

    // LE Set Advertising Enable (disable)
    exec_cmd!(
        controller,
        LeSetAdvEnable::new(false),
        "LeSetAdvEnable(false)"
    );

    info!("");
    info!("=== Phase 6: Scanning ===");

    // LE Set Scan Parameters
    let scan_params = LeSetScanParams::new(
        LeScanKind::Passive,
        bt_hci::param::Duration::from_millis(100), // scan interval
        bt_hci::param::Duration::from_millis(50),  // scan window
        AddrKind::RANDOM,                          // own address type
        ScanningFilterPolicy::BasicUnfiltered,     // accept all
    );
    exec_cmd!(controller, scan_params, "LeSetScanParams");

    // LE Set Scan Enable (enable, no duplicates filter)
    exec_cmd!(
        controller,
        LeSetScanEnable::new(true, false),
        "LeSetScanEnable(true)"
    );

    // 扫描一会儿
    info!("Scanning for 2 seconds...");
    Timer::after_secs(2).await;

    // LE Set Scan Enable (disable)
    exec_cmd!(
        controller,
        LeSetScanEnable::new(false, false),
        "LeSetScanEnable(false)"
    );

    info!("");
    info!("=== All tests completed ===");

    loop {
        Timer::after_secs(1).await;
    }
}
