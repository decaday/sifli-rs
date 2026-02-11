//! LPAON (Low-Power Always-On) helpers.
//!
//! System-level always-on registers for LCPU start address and wake control.
//! Accessed directly via PAC (no peripheral singleton needed).

use crate::lcpu;
use crate::pac;

fn regs() -> pac::lpsys_aon::LpsysAon {
    pac::LPSYS_AON
}

/// Read the first two entries `(SP, PC)` from the LCPU vector table in memory.
fn read_start_vector_from_mem() -> (u32, u32) {
    let vector_addr = lcpu::LpsysRam::CODE_START as *const u32;

    unsafe {
        // vector[0] (0x20400000): initial stack pointer (SP)
        // vector[1] (0x20400004): Reset_Handler address (PC)
        let sp = core::ptr::read_volatile(vector_addr);
        let pc = core::ptr::read_volatile(vector_addr.add(1));
        (sp, pc)
    }
}

/// Write the given `(sp, pc)` into LPSYS_AON SPR/PCR registers.
fn write_start_vector_to_regs(sp: u32, pc: u32) {
    let r = regs();
    // On LCPU start/wake, hardware loads SP and PC from these registers.
    r.spr().write(|w| w.set_sp(sp));
    r.pcr().write(|w| w.set_pc(pc));
}

/// Configure LCPU start address: read `(SP, PC)` from the LCPU vector table and write into LPAON registers.
pub(crate) fn configure_lcpu_start() {
    let (sp, pc) = read_start_vector_from_mem();
    write_start_vector_to_regs(sp, pc);
}

/// Read the currently configured LCPU start vector `(SP, PC)` from LPAON registers.
#[allow(dead_code)]
pub(crate) fn read_start_vector() -> (u32, u32) {
    let r = regs();
    let spr = r.spr().read();
    let pcr = r.pcr().read();
    (spr.sp(), pcr.pc())
}

/// Read the CPUWAIT flag from `LPSYS_AON.PMR`.
pub(crate) fn cpuwait() -> bool {
    regs().pmr().read().cpuwait()
}

/// Set/clear the CPUWAIT flag in `LPSYS_AON.PMR`.
pub(crate) fn set_cpuwait(enable: bool) {
    regs().pmr().modify(|w| w.set_cpuwait(enable));
}

/// Read the SLEEP_STATUS flag from `LPSYS_AON.SLP_CTRL`.
pub(crate) fn sleep_status() -> bool {
    regs().slp_ctrl().read().sleep_status()
}

/// Set/clear the WKUP_REQ flag in `LPSYS_AON.SLP_CTRL` to request a wakeup.
pub(crate) fn set_wkup_req(enable: bool) {
    regs().slp_ctrl().modify(|w| w.set_wkup_req(enable));
}
