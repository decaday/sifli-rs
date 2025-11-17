//! LPAON (Low-Power Always-On) module.
//!
//! Provides helpers to configure LCPU start address and related wake control.
//!
//! ```no_run
//! use sifli_hal::lpaon::LpAon;
//!
//! LpAon::configure_lcpu_start();
//! let (sp, pc) = LpAon::read_start_vector();
//! ```

use crate::{lcpu, pac};

/// LPAON driver.
///
/// Zero-sized type used as a namespace around `LPSYS_AON` operations.
/// Always-on peripheral, no clock gating or init required.
pub struct LpAon;

impl LpAon {
    /// Read the first two entries `(SP, PC)` from the LCPU vector table in memory.
    ///
    /// # Safety
    ///
    /// Caller must ensure:
    /// - LCPU image has been loaded to `lcpu::LCPU_CODE_START_ADDR`.
    /// - The first two entries form a valid stack pointer and reset handler.
    fn read_start_vector_from_mem() -> (u32, u32) {
        let vector_addr = lcpu::LCPU_CODE_START_ADDR as *const u32;

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
        // On LCPU start/wake, hardware loads SP and PC from these registers.
        pac::LPSYS_AON.spr().write(|w| w.set_sp(sp));
        pac::LPSYS_AON.pcr().write(|w| w.set_pc(pc));
    }

    /// Configure LCPU start address: read `(SP, PC)` from the LCPU vector table and write into LPAON registers.
    ///
    /// ```no_run
    /// use sifli_hal::lpaon::LpAon;
    ///
    /// LpAon::configure_lcpu_start();
    /// ```
    pub fn configure_lcpu_start() {
        let (sp, pc) = Self::read_start_vector_from_mem();
        Self::write_start_vector_to_regs(sp, pc);
    }

    /// Read the currently configured LCPU start vector `(SP, PC)` from LPAON registers.
    ///
    /// ```no_run
    /// use sifli_hal::lpaon::LpAon;
    ///
    /// let (sp, pc) = LpAon::read_start_vector();
    /// ```
    pub fn read_start_vector() -> (u32, u32) {
        let spr = pac::LPSYS_AON.spr().read();
        let pcr = pac::LPSYS_AON.pcr().read();
        (spr.sp(), pcr.pc())
    }

    /// Read the CPUWAIT flag from `LPSYS_AON.PMR`.
    #[allow(dead_code)]
    pub(crate) fn cpuwait() -> bool {
        pac::LPSYS_AON.pmr().read().cpuwait()
    }

    /// Set/clear the CPUWAIT flag in `LPSYS_AON.PMR`.
    #[allow(dead_code)]
    pub(crate) fn set_cpuwait(enable: bool) {
        pac::LPSYS_AON.pmr().modify(|w| w.set_cpuwait(enable));
    }

    /// Read the SLEEP_STATUS flag from `LPSYS_AON.SLP_CTRL`.
    #[allow(dead_code)]
    pub(crate) fn sleep_status() -> bool {
        pac::LPSYS_AON.slp_ctrl().read().sleep_status()
    }

    /// Set/clear the WKUP_REQ flag in `LPSYS_AON.SLP_CTRL` to request a wakeup.
    #[allow(dead_code)]
    pub(crate) fn set_wkup_req(enable: bool) {
        pac::LPSYS_AON
            .slp_ctrl()
            .modify(|w| w.set_wkup_req(enable));
    }
}
