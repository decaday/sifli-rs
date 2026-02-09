//! Post-boot BLE controller initialization.
//!
//! SDK equivalent: `bluetooth_init()` in
//! `SiFli-SDK/middleware/bluetooth/service/bluetooth.c:563`.
//!
//! The SDK runs this on the LCPU side after boot. Since the Rust HAL
//! doesn't run LCPU middleware, we replicate critical steps from HCPU
//! by writing directly to LCPU shared memory.
//!
//! ## SDK `bluetooth_init()` steps
//!
//! | # | SDK function              | Status | Notes                        |
//! |---|---------------------------|--------|------------------------------|
//! | 1 | `rf_ptc_config(1)`        | TODO   | PTC2 timer chain (AGC/CFO)   |
//! | 2 | `bluetooth_isr_init()`    | Skip   | LCPU NVIC, ROM handles       |
//! | 3 | `ble_xtal_less_init()`    | **Done** | lld_prog_delay + clock cfg |
//! | 4 | `bluetooth_pm_init()`     | TODO   | Power management hooks       |
//! | 5 | `bluetooth_config()`      | Skip   | LCPU ROM functions, ROM has defaults |
//! | 6 | `HAL_RCC_SetMacFreq()`    | TODO   | LPSYS_RCC MACDIV (target 8MHz) |
//! | 7 | `wlan_coex_config()`      | TODO   | BT_MAC coexistence regs      |

use super::config::ControllerConfig;
use super::ram::BtRomConfig;
use crate::syscfg;

/// LCPU ROM runtime variable addresses.
///
/// Source: `SiFli-SDK/example/rom_bin/lcpu_boot_loader/lcpu_rom_micro.map`
/// and `SiFli-SDK/example/rom_bin/lcpu_general_ble_img/lcpu_img_56x_52.axf`
mod addr {
    use super::BtRomConfig;

    /// `rwip_prog_delay` address for A3 revision.
    pub const RWIP_PROG_DELAY_A3: *mut u8 = 0x2040_FA94 as _;

    /// `rwip_prog_delay` address for Letter Series (A4/B4).
    pub const RWIP_PROG_DELAY_LETTER: *mut u8 = 0x2041_6338 as _;

    /// `g_rom_config` base address for A3 revision (21 bytes).
    pub const G_ROM_CONFIG_A3: *mut BtRomConfig = 0x2040_E48C as _;
}

/// `g_rom_config.bit_valid` bit definitions.
///
/// Each bit indicates the corresponding field has been explicitly configured.
/// ROM uses internal defaults for fields whose valid bit is NOT set.
///
/// Source: SDK `rom_config_set_*()` functions in `ble_rom_config.c`.
mod bit {
    /// `lld_prog_delay` (field +9).
    pub const LLD_PROG_DELAY: u32 = 1 << 2;
    /// `default_sleep_mode` (field +11).
    pub const SLEEP_MODE: u32 = 1 << 4;
    /// `default_sleep_enabled` (field +12).
    pub const SLEEP_ENABLED: u32 = 1 << 5;
    /// `default_xtal_enabled` (field +13).
    pub const XTAL_ENABLED: u32 = 1 << 6;
    /// `default_rc_cycle` (field +14).
    pub const RC_CYCLE: u32 = 1 << 7;
}

/// Apply post-boot controller configuration to LCPU memory.
///
/// SDK equivalent: `ble_xtal_less_init()` in `bluetooth.c:79-101`.
///
/// Must be called after warmup event consumption and before BLE connections.
///
/// Critical: Each `rom_config_set_*()` in SDK both writes the field value AND
/// sets the corresponding bit in `bit_valid`. The ROM ignores fields whose
/// valid bit is not set, falling back to internal defaults (which may enable
/// sleep or XTAL timing that doesn't match the board).
pub(crate) fn apply(config: &ControllerConfig) {
    let is_letter = syscfg::read_idr().revision().is_letter_series();

    // ── rwip_prog_delay (runtime cache) ──
    //
    // Direct runtime variable read by sch_arb_insert(), sch_arb_prog_timer(), etc.
    let rwip_addr = if is_letter {
        addr::RWIP_PROG_DELAY_LETTER
    } else {
        addr::RWIP_PROG_DELAY_A3
    };

    unsafe {
        core::ptr::write_volatile(rwip_addr, config.lld_prog_delay);
    }

    debug!(
        "Controller init: rwip_prog_delay={} written to 0x{:08X}",
        config.lld_prog_delay, rwip_addr as usize
    );

    // ── g_rom_config fields + bit_valid (A3 only) ──
    //
    // For A3: write directly to g_rom_config in LCPU BSS and update bit_valid.
    // For Letter Series: already configured via RomControlBlock.bt_config in ram::rom_config().
    if !is_letter {
        let cfg_ptr = addr::G_ROM_CONFIG_A3;

        // Dump before modification.
        let rom_cfg = unsafe { core::ptr::read_volatile(cfg_ptr) };
        debug!(
            "g_rom_config BEFORE: bit_valid=0x{:08X} prog_delay={} sleep_mode={} sleep_enabled={} xtal_enabled={} rc_cycle={}",
            rom_cfg.bit_valid,
            rom_cfg.lld_prog_delay,
            rom_cfg.default_sleep_mode,
            rom_cfg.default_sleep_enabled,
            rom_cfg.default_xtal_enabled,
            rom_cfg.default_rc_cycle,
        );

        unsafe {
            // Write fields (mirrors SDK rom_config_set_* functions).
            let base = cfg_ptr as *mut u8;

            // lld_prog_delay (offset +9)
            core::ptr::write_volatile(base.add(9), config.lld_prog_delay);

            // default_sleep_mode (offset +11) = 0 (no sleep)
            core::ptr::write_volatile(base.add(11), 0);

            // default_sleep_enabled (offset +12) = 0 (disable sleep)
            // Without bluetooth_pm_init(), sleep would cause missed connection events.
            core::ptr::write_volatile(base.add(12), 0);

            // default_xtal_enabled (offset +13)
            core::ptr::write_volatile(base.add(13), config.xtal_enabled as u8);

            // default_rc_cycle (offset +14)
            core::ptr::write_volatile(base.add(14), config.rc_cycle);

            // Update bit_valid — tell ROM these fields are explicitly configured.
            let old_valid = core::ptr::read_volatile(cfg_ptr as *const u32);
            let new_valid = old_valid
                | bit::LLD_PROG_DELAY
                | bit::SLEEP_MODE
                | bit::SLEEP_ENABLED
                | bit::XTAL_ENABLED
                | bit::RC_CYCLE;
            core::ptr::write_volatile(cfg_ptr as *mut u32, new_valid);
        }

        // Verify.
        let rom_cfg = unsafe { core::ptr::read_volatile(cfg_ptr) };
        debug!(
            "g_rom_config AFTER: bit_valid=0x{:08X} prog_delay={} sleep_mode={} sleep_enabled={} xtal_enabled={} rc_cycle={}",
            rom_cfg.bit_valid,
            rom_cfg.lld_prog_delay,
            rom_cfg.default_sleep_mode,
            rom_cfg.default_sleep_enabled,
            rom_cfg.default_xtal_enabled,
            rom_cfg.default_rc_cycle,
        );
    }

    // TODO Step 1: rf_ptc_config — PTC2 timer chain
    //   SF32LB52x: ptc_config(0, PTC_LCPU_BT_PKTDET, 0, 0) for CFO sampling

    // Step 6: set_mac_freq — MAC clock divider
    //   SDK: HAL_RCC_SetMacFreq() in bf0_hal_rcc.c:2047
    //   Also called in ble_standby_sleep_after_handler() after every BLE sleep wakeup.
    {
        use crate::rcc;
        let hclk = rcc::get_lpsys_hclk_freq().unwrap_or(crate::time::Hertz(24_000_000));
        let mac_div = (hclk.0 / 8_000_000) as u8;
        if mac_div == 0 {
            warn!("LPSYS HCLK {}Hz < 8MHz, MAC clock divider would be 0, using 1", hclk.0);
            rcc::config_lpsys_mac_clock(1, 0x08);
        } else {
            rcc::config_lpsys_mac_clock(mac_div, 0x08);
        }
        debug!("MAC clock configured: HCLK={}Hz, MACDIV={}, MACFREQ=8", hclk.0, mac_div.max(1));
    }
}
