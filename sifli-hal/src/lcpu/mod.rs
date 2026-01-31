//! LCPU power-on和管理（阻塞模式）。
//!
//! ```no_run
//! use sifli_hal::lcpu::{Lcpu, LcpuConfig};
//!
//! # fn example() -> Result<(), sifli_hal::lcpu::LcpuError> {
//! let p = sifli_hal::init(Default::default());
//! let cfg = LcpuConfig::default();
//! let lcpu = Lcpu::new(p.LPSYS_AON);
//! lcpu.power_on(&cfg)?;
//! # Ok(()) }
//! ```

// Re-export RAM management module
pub mod ram;
pub use ram::LpsysRam;

mod bt_cal;

use core::fmt;

use crate::peripherals;
use crate::syscfg::{self, ChipRevision};
use crate::Peripheral;
use crate::{lpaon, patch, rcc};

#[cfg(feature = "sf32lb52x-lcpu")]
mod sf32lb52x_lcpu_data {
    include!(concat!(env!("OUT_DIR"), "/sf32lb52x_lcpu.rs"));
}

//=============================================================================
// Configuration
//=============================================================================

/// LCPU power-on configuration.
#[derive(Debug, Clone, Copy)]
pub struct LcpuConfig {
    /// LCPU firmware image bytes.
    ///
    /// - A3 and earlier: must be provided and copied to LPSYS RAM.
    /// - Letter Series: optional, firmware is in ROM.
    pub firmware: Option<&'static [u8]>,

    /// ROM configuration parameters.
    pub rom: ram::RomConfig,

    /// Patch data for A3 and earlier (record + code format).
    pub patch_a3: Option<PatchData>,

    /// Patch data for Letter Series (A4/B4) (header + code format).
    pub patch_letter: Option<PatchData>,

    /// Skip LPSYS HCLK frequency check during image loading (use with care).
    pub skip_frequency_check: bool,

    /// Disable RF calibration (normally runs after patch installation).
    pub disable_rf_cal: bool,
}

impl LcpuConfig {
    /// Create a new config with all options unset.
    pub const fn new() -> Self {
        Self {
            firmware: None,
            rom: ram::RomConfig {
                wdt_time: 10,
                wdt_clk: 32_768,
                enable_lxt: true,
            },
            patch_a3: None,
            patch_letter: None,
            skip_frequency_check: false,
            disable_rf_cal: false,
        }
    }
}

impl Default for LcpuConfig {
    fn default() -> Self {
        #[allow(unused_mut)]
        let mut cfg = Self::new();

        #[cfg(feature = "sf32lb52x-lcpu")]
        {
            cfg.firmware = Some(sf32lb52x_lcpu_data::SF32LB52X_LCPU_FIRMWARE);
            cfg.patch_a3 = Some(PatchData {
                list: sf32lb52x_lcpu_data::SF32LB52X_LCPU_PATCH_A3_LIST,
                bin: sf32lb52x_lcpu_data::SF32LB52X_LCPU_PATCH_A3_BIN,
            });
            cfg.patch_letter = Some(PatchData {
                list: sf32lb52x_lcpu_data::SF32LB52X_LCPU_PATCH_LETTER_LIST,
                bin: sf32lb52x_lcpu_data::SF32LB52X_LCPU_PATCH_LETTER_BIN,
            });
            cfg.disable_rf_cal = true;
        }

        cfg
    }
}

/// Patch data (entry list + code binary), matching `g_lcpu_patch_list` / `g_lcpu_patch_bin` in SDK.
#[derive(Debug, Clone, Copy)]
pub struct PatchData {
    /// Patch entry list array.
    pub list: &'static [u8],
    /// Patch code bytes.
    pub bin: &'static [u8],
}

//=============================================================================
// Error types
//=============================================================================

/// LCPU power-on error.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LcpuError {
    /// Image installation error.
    ImageInstall(ram::Error),

    /// Patch installation error.
    PatchInstall(patch::Error),

    /// Missing firmware image for A3 and earlier revisions.
    FirmwareMissing,

    /// Frequency check failed (LPSYS HCLK exceeded 24MHz during loading).
    FrequencyTooHigh {
        /// Actual frequency (Hz).
        actual_hz: u32,
        /// Maximum allowed frequency (Hz).
        max_hz: u32,
    },

    /// HPAON operation error.
    HpaonError,

    /// RCC operation error.
    RccError,

    /// ROM configuration error.
    RomConfigError,

    /// Wake reference count overflow (limit is 20).
    RefCountOverflow,

    /// Timeout waiting for LP_ACTIVE.
    WakeCoreTimeout,
}

impl From<ram::Error> for LcpuError {
    fn from(err: ram::Error) -> Self {
        Self::ImageInstall(err)
    }
}

impl From<patch::Error> for LcpuError {
    fn from(err: patch::Error) -> Self {
        Self::PatchInstall(err)
    }
}

//=============================================================================
// Core IDs
//=============================================================================

/// Core ID (for multi-core wake operations).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CoreId {
    Default = 0,
    /// HCPU (High-performance CPU)
    Hcpu = 1,
    /// LCPU (Low-power CPU)
    Lcpu = 2,
    /// ACPU (58x)
    Acpu = 3,
}

//=============================================================================
// LCPU driver type
//=============================================================================

/// LCPU driver（Blocking）。
pub struct Lcpu<'d> {
    lpaon: lpaon::LpAon<'d, peripherals::LPSYS_AON>,
}

impl<'d> fmt::Debug for Lcpu<'d> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Lcpu").finish_non_exhaustive()
    }
}

impl<'d> Lcpu<'d> {
    /// 创建一个阻塞版 LCPU 驱动。
    pub fn new(lpsys_aon: impl Peripheral<P = peripherals::LPSYS_AON> + 'd) -> Self {
        Self {
            lpaon: lpaon::LpAon::new(lpsys_aon),
        }
    }
}

//=============================================================================
// Power-on flow
//=============================================================================

impl<'d> Lcpu<'d> {
    /// 阻塞版 LCPU 启动流程。
    pub fn power_on(&self, config: &LcpuConfig) -> Result<(), LcpuError> {
        // 1. Wake LCPU。
        debug!("Step 1: Waking up LCPU");
        unsafe {
            rcc::wake_lcpu();
        }

        // 2. Reset and halt LCPU.
        debug!("Step 2: Resetting and halting LCPU");
        self.reset_and_halt_lcpu()?;

        // 3. Configure ROM parameters (bf0_lcpu_init.c:168).
        debug!("Step 3: Configuring ROM parameters");
        let idr = syscfg::read_idr();
        let revision = idr.revision();
        ram::rom_config(revision, &config.rom)?;

        // 4. Enforce frequency limit while loading (bf0_lcpu_init.c:170-176).
        if !config.skip_frequency_check {
            debug!("Step 4: Checking LCPU frequency (must be ≤ 24MHz during loading)");
            check_lcpu_frequency()?;
        } else {
            warn!("Step 4: Skipping frequency check (as requested by config)");
        }

        // 5. Install image for A3 and earlier (bf0_lcpu_init.c:178-182).
        if !revision.is_letter_series() {
            debug!("Step 5: Installing LCPU firmware image (A3/earlier)");

            if let Some(firmware) = config.firmware {
                ram::img_install(revision, firmware)?;
            } else {
                error!("Firmware required for A3 and earlier revisions");
                return Err(LcpuError::FirmwareMissing);
            }
        } else {
            debug!("Step 5: Skipping image install (Letter Series, firmware in ROM)");
        }

        // 6. Configure LCPU start address (bf0_lcpu_init.c:184).
        debug!(
            "Step 6: Configuring LCPU start address (0x{:08X})",
            LpsysRam::CODE_START
        );
        self.lpaon.configure_lcpu_start();

        // 7. Install patches and perform RF calibration (bf0_lcpu_init.c:185).
        debug!("Step 7: Installing patches and RF calibration");
        install_patch_and_calibrate(config, revision)?;

        // 8. Release LCPU to run (bf0_lcpu_init.c:186).
        debug!("Step 8: Releasing LCPU to run");
        self.release_lcpu()?;

        // 9. 清除 HP2LP_REQ，允许 LP 进入低功耗。
        unsafe {
            rcc::cancel_lcpu_active_request();
        }

        Ok(())
    }

    /// 阻塞版关机：仅复位并保持 CPUWAIT。
    pub fn power_off(&self) -> Result<(), LcpuError> {
        info!("Powering off LCPU");

        self.reset_and_halt_lcpu()?;

        info!("LCPU powered off successfully");

        Ok(())
    }

    fn reset_and_halt_lcpu(&self) -> Result<(), LcpuError> {
        // Only perform reset flow when CPUWAIT is not set.
        if !self.lpaon.cpuwait() {
            // 1. Set CPUWAIT so LCPU stays halted.
            self.lpaon.set_cpuwait(true);

            // 2. Reset LCPU and MAC (SF32LB52X requires both).
            rcc::set_lp_lcpu_reset(true);
            rcc::set_lp_mac_reset(true);

            // Wait until reset bits take effect.
            while !rcc::lp_lcpu_reset_asserted() || !rcc::lp_mac_reset_asserted() {}

            // 3. If LPSYS is sleeping, wake it up.
            if self.lpaon.sleep_status() {
                self.lpaon.set_wkup_req(true);
                while self.lpaon.sleep_status() {}
            }

            // 4. Clear reset bits, keep CPUWAIT = 1.
            rcc::set_lp_lcpu_reset(false);
            rcc::set_lp_mac_reset(false);
        }

        Ok(())
    }

    fn release_lcpu(&self) -> Result<(), LcpuError> {
        // Clear CPUWAIT so LCPU can run.
        self.lpaon.set_cpuwait(false);

        Ok(())
    }
}

/// Ensure LPSYS HCLK stays ≤ 24 MHz while loading the LCPU image.
///
/// Mirrors `bf0_lcpu_init.c:170-176` in the SDK.
/// Returns the final HCLK frequency on success.
fn check_lcpu_frequency() -> Result<u32, LcpuError> {
    const MAX_LOAD_FREQ_HZ: u32 = 24_000_000;

    // 1. Compute current LPSYS HCLK frequency.
    let hclk_hz = rcc::get_lpsys_hclk_freq().ok_or(LcpuError::RccError)?.0;
    let hdiv = rcc::get_lpsys_hclk_div();

    // 2. Enforce limit; panic if超出。
    assert!(
        hclk_hz <= MAX_LOAD_FREQ_HZ,
        "LPSYS HCLK {} Hz exceeds limit {} Hz for LCPU loading (HDIV1={})",
        hclk_hz,
        MAX_LOAD_FREQ_HZ,
        hdiv
    );

    debug!(
        "LPSYS HCLK within limit for LCPU loading: {} Hz (HDIV1={})",
        hclk_hz, hdiv
    );

    Ok(hclk_hz)
}

/// install patches and perform rf calibration.
///
/// selects the appropriate patch data based on chip revision, installs it,
/// and then runs rf calibration (not yet implemented).
///
/// # errors
///
/// - [`LcpuError::PatchInstall`][]: patch installation failed.
fn install_patch_and_calibrate(
    config: &LcpuConfig,
    revision: ChipRevision,
) -> Result<(), LcpuError> {
    // Choose patch data based on revision.
    let patch_data = if revision.is_letter_series() {
        debug!("Using Letter Series patch data");
        config.patch_letter
    } else {
        debug!("Using A3 patch data");
        config.patch_a3
    };

    // Install patch if data is provided.
    if let Some(data) = patch_data {
        debug!(
            "Installing patches (list: {} bytes, bin: {} bytes)",
            data.list.len(),
            data.bin.len()
        );

        // Use patch module to install the patch.
        patch::install(revision, data.list, data.bin)?;
    } else {
        warn!("No patch data provided, skipping patch installation");
    }

    if !config.disable_rf_cal {
        debug!("Performing RF calibration");

        // Reference SDK `lcpu_ble_patch_install`, perform BT RF calibration after patch installation.
        // Here only the minimum necessary subset is implemented: reset RFC and write BT_TXPWR configuration.
        crate::lcpu::bt_cal::bt_rf_cal(revision);
    } else {
        warn!("RF calibration disabled by config");
    }

    Ok(())
}
