#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer, WithTimeout};
use panic_probe as _;

use sifli_hal::{
    bind_interrupts, ipc,
    lcpu::{Lcpu, LcpuConfig},
    syscfg,
};

bind_interrupts!(struct Irqs {
    MAILBOX2_CH1 => ipc::InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = sifli_hal::init(Default::default());
    let rev = syscfg::read_idr().revision();

    let mut ipc = ipc::Ipc::new(p.MAILBOX1_CH1, ipc::Config::default());

    let mut q = match ipc.open_queue(ipc::QueueConfig::qid0_hci(rev)) {
        Ok(q) => q,
        Err(e) => {
            error!("open_queue(qid0) failed: {:?}", e);
            loop {
                Timer::after_secs(1).await;
            }
        }
    };
    info!("IPC queue opened: qid={}", q.qid());

    info!("Powering on LCPU...");
    let lcpu = Lcpu::new(p.LPSYS_AON);
    if let Err(e) = lcpu.power_on(&LcpuConfig::default()) {
        error!("LCPU power_on failed: {:?}", e);
        loop {
            Timer::after_secs(1).await;
        }
    }
    info!("LCPU is running");

    // release 模式下主核代码更快，过早发送 HCI Reset 可能导致 LCPU 侧尚未完成 BT controller/IPC 初始化，
    // 从而丢包或不回包。这里先等待一条“任意”HCI Event（或小延时兜底），用于把时序对齐到 SDK 常见的启动窗口。
    let mut chunk = [0u8; 64];
    let mut warmup: heapless::Vec<u8, 256> = heapless::Vec::new();

    info!("Waiting initial HCI event (warmup, 20ms)...");
    let warmup_wait = async {
        loop {
            let n = q.read_async(&mut chunk).await?;
            if n == 0 {
                continue;
            }
            warmup.extend_from_slice(&chunk[..n]).ok();
            return Ok::<(), ipc::Error>(());
        }
    };

    match warmup_wait.with_timeout(Duration::from_millis(20)).await {
        Ok(Ok(())) => {
            // 尝试把 ring buffer 中已到达的数据一次性读空（避免把启动噪声混入后续 HCI Reset 判定）。
            loop {
                let n = q.read(&mut chunk).unwrap_or(0);
                if n == 0 {
                    break;
                }
                warmup.extend_from_slice(&chunk[..n]).ok();
            }
            info!("Warmup rx");
            dump_hci_events(warmup.as_slice());
        }
        Ok(Err(e)) => warn!("Warmup read failed: {:?}", e),
        Err(_timeout) => {
            // 如果 LCPU 不会主动发事件，则退化成短延时，避免发命令过早。
            warn!("No warmup event; delaying 20ms");
            Timer::after_millis(20).await;
        }
    }

    q.debug_dump();

    // HCI Reset: packet_type=CMD(0x01), opcode=0x0C03, param_len=0.
    let cmd = [0x01, 0x03, 0x0C, 0x00];
    let written = q.write(&cmd).unwrap_or(0);
    if written != cmd.len() {
        warn!("Partial HCI write: {}/{}", written, cmd.len());
    } else {
        info!("Sent HCI Reset: {=[u8]:02X}", cmd);
    }
    q.debug_dump();

    let mut acc: heapless::Vec<u8, 256> = heapless::Vec::new();

    let wait = async {
        loop {
            let n = q.read_async(&mut chunk).await?;
            if n == 0 {
                continue;
            }
            acc.extend_from_slice(&chunk[..n]).ok();
            if contains_hci_reset_ok(&acc) {
                return Ok::<(), ipc::Error>(());
            }
        }
    };

    match wait.with_timeout(Duration::from_secs(2)).await {
        Ok(Ok(())) => {
            info!("Got HCI Reset response");
            dump_hci_events(acc.as_slice());
            q.debug_dump();
        }
        Ok(Err(e)) => warn!("IPC read failed: {:?}", e),
        Err(_timeout) => {
            warn!("Timeout waiting HCI response");
            dump_hci_events(acc.as_slice());
            q.debug_dump();
        }
    }

    loop {
        Timer::after_secs(1).await;
    }
}

fn contains_hci_reset_ok(bytes: &[u8]) -> bool {
    // HCI Event:
    // - Command Complete (0x0E): [04 0E 04 N 03 0C 00] 其中 N=Num_HCI_Command_Packets，值不固定（SDK/ROM 可能是 0x06）。
    // - Command Status   (0x0F): [04 0F 04 00 N 03 0C] status=0 表示已接受命令。
    bytes.windows(7).any(|w| {
        (w[0] == 0x04
            && w[1] == 0x0E
            && w[2] == 0x04
            && w[4] == 0x03
            && w[5] == 0x0C
            && w[6] == 0x00)
            || (w[0] == 0x04
                && w[1] == 0x0F
                && w[2] == 0x04
                && w[3] == 0x00
                && w[5] == 0x03
                && w[6] == 0x0C)
    })
}

fn dump_hci_events(bytes: &[u8]) {
    fn opcode_name(opcode: u16) -> &'static str {
        match opcode {
            0x0C03 => "HCI_Reset",
            0xFC11 => "VSC(0xFC11)",
            _ => "?",
        }
    }

    info!("HCI rx raw: {=[u8]:02X}", bytes);

    let mut i = 0usize;
    while i < bytes.len() {
        if bytes[i] != 0x04 {
            warn!(
                "HCI: unexpected packet_type=0x{:02X} at offset {}",
                bytes[i], i
            );
            i += 1;
            continue;
        }
        if i + 3 > bytes.len() {
            warn!("HCI: truncated header at offset {}", i);
            break;
        }

        let evt = bytes[i + 1];
        let plen = bytes[i + 2] as usize;
        let end = i + 3 + plen;
        if end > bytes.len() {
            warn!(
                "HCI: truncated event at offset {} evt=0x{:02X} plen={}",
                i, evt, plen
            );
            break;
        }
        let params = &bytes[i + 3..end];

        match evt {
            0x0E => {
                // Command Complete:
                // [num_hci_command_packets, opcode(LE u16), return_parameters...]
                if params.len() < 3 {
                    warn!(
                        "HCI evt=0x0E(Command Complete) malformed params={=[u8]:02X}",
                        params
                    );
                } else {
                    let num = params[0];
                    let opcode = u16::from_le_bytes([params[1], params[2]]);
                    let ret = &params[3..];
                    if ret.is_empty() {
                        info!(
                            "HCI evt=0x0E(Command Complete) opcode=0x{:04X}({=str}) num_cmd_pkts=0x{:02X} ret=[]",
                            opcode,
                            opcode_name(opcode),
                            num
                        );
                    } else {
                        let status = ret[0];
                        info!(
                            "HCI evt=0x0E(Command Complete) opcode=0x{:04X}({=str}) num_cmd_pkts=0x{:02X} status=0x{:02X} ret={=[u8]:02X}",
                            opcode,
                            opcode_name(opcode),
                            num,
                            status,
                            ret
                        );
                    }
                }
            }
            0x0F => {
                // Command Status:
                // [status, num_hci_command_packets, opcode(LE u16)]
                if params.len() < 4 {
                    warn!(
                        "HCI evt=0x0F(Command Status) malformed params={=[u8]:02X}",
                        params
                    );
                } else {
                    let status = params[0];
                    let num = params[1];
                    let opcode = u16::from_le_bytes([params[2], params[3]]);
                    info!(
                        "HCI evt=0x0F(Command Status) opcode=0x{:04X}({=str}) num_cmd_pkts=0x{:02X} status=0x{:02X}",
                        opcode,
                        opcode_name(opcode),
                        num,
                        status
                    );
                }
            }
            _ => {
                info!("HCI evt=0x{:02X} params={=[u8]:02X}", evt, params);
            }
        }

        i = end;
    }
}
