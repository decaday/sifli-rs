//! System configuration (SYSCFG) and chip identification
//!
//! This module provides access to chip identification information stored in
//! hardware registers, primarily the `HPSYS_CFG->IDR` register.
//!
//! # Examples
//!
//! ```no_run
//! use sifli_hal::syscfg::{Syscfg, ChipRevision};
//!
//! // Read chip signature
//! let sig = Syscfg::read();
//! let rev = sig.revision();
//!
//! // Check chip version
//! match rev.patch_type() {
//!     Some(PatchType::A3) => {
//!         // Use A3 patch
//!     }
//!     Some(PatchType::LetterSeries) => {
//!         // Use Letter Series patch
//!     }
//!     None => {
//!         // Invalid revision
//!     }
//! }
//! ```

use crate::pac;

/// Chip/system identification extracted from `HPSYS_CFG->IDR`.
///
/// 包含芯片变体与版本的 4 个标识字段。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Syscfg {
    /// Revision ID (bit[7:0]) - Hardware revision
    ///
    /// - 0x00-0x03: A3 or earlier (including engineering samples)
    /// - 0x07: A4 (Letter Series)
    /// - 0x0F: B4 (Letter Series)
    pub revid: u8,

    /// Package ID (bit[15:8]) - Package type
    pub pid: u8,

    /// Company ID (bit[23:16]) - Manufacturer/company identifier
    pub cid: u8,

    /// Series ID (bit[31:24]) - Product series identifier
    pub sid: u8,
}

impl Syscfg {
    /// Read chip signature from hardware
    ///
    /// Reads the `HPSYS_CFG->IDR` register to get all chip identification fields.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// let sig = Syscfg::read();
    /// println!("REVID: 0x{:02x}", sig.revid);
    /// ```
    pub fn read() -> Self {
        let idr = pac::HPSYS_CFG.idr().read();
        Self {
            revid: idr.revid(),
            pid: idr.pid(),
            cid: idr.cid(),
            sid: idr.sid(),
        }
    }

    /// Get chip revision information
    ///
    /// Parses the REVID field into a structured enum with SDK-compatible
    /// validity checks.
    pub fn revision(&self) -> ChipRevision {
        ChipRevision::from_revid(self.revid)
    }

    /// Get raw IDR register value
    ///
    /// Returns the complete 32-bit IDR register value.
    pub fn idr_raw(&self) -> u32 {
        ((self.sid as u32) << 24) | ((self.cid as u32) << 16) | ((self.pid as u32) << 8) | (self.revid as u32)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Syscfg {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "Syscfg {{ revid: 0x{:02x}, pid: 0x{:02x}, cid: 0x{:02x}, sid: 0x{:02x} }}",
            self.revid,
            self.pid,
            self.cid,
            self.sid
        );
    }
}

/// Chip revision information
///
/// Matches SDK's `__HAL_SYSCFG_CHECK_REVID()` logic:
/// - Valid revisions: <= 0x03, 0x07, 0x0F
/// - Invalid: all others
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChipRevision {
    /// A3 and earlier revisions (0x00-0x03)
    ///
    /// Includes early engineering samples. All use the same A3 patch.
    /// Corresponds to SDK's `<= HAL_CHIP_REV_ID_A3` check.
    A3OrEarlier(u8),

    /// A4 revision (0x07) - Letter Series
    ///
    /// Uses Letter Series patches from `lcpu_patch_rev_b.c`.
    A4,

    /// B4 revision (0x0F) - Letter Series
    ///
    /// Latest Letter Series revision, uses same patches as A4.
    B4,

    /// Invalid or unknown revision
    ///
    /// Not recognized by SDK's validity checks.
    Invalid(u8),
}

impl ChipRevision {
    /// Parse revision from REVID value
    ///
    /// Implements SDK's `__HAL_SYSCFG_CHECK_REVID()` logic.
    pub fn from_revid(revid: u8) -> Self {
        match revid {
            0x00..=0x03 => ChipRevision::A3OrEarlier(revid),
            0x07 => ChipRevision::A4,
            0x0F => ChipRevision::B4,
            _ => ChipRevision::Invalid(revid),
        }
    }

    /// Check if this is a valid revision per SDK rules
    ///
    /// Returns `true` for all revisions recognized by SDK:
    /// - 0x00-0x03 (A3 and earlier)
    /// - 0x07 (A4)
    /// - 0x0F (B4)
    pub fn is_valid(&self) -> bool {
        !matches!(self, ChipRevision::Invalid(_))
    }

    /// Check if this is a Letter Series revision (A4 or B4)
    ///
    /// Letter Series chips can run LCPU from ROM without loading from flash.
    pub fn is_letter_series(&self) -> bool {
        matches!(self, ChipRevision::A4 | ChipRevision::B4)
    }

    /// Get human-readable revision name
    pub fn name(&self) -> &'static str {
        match self {
            ChipRevision::A3OrEarlier(0x00) => "Pre-A3 (ES v0.0)",
            ChipRevision::A3OrEarlier(0x01) => "Pre-A3 (ES v0.1)",
            ChipRevision::A3OrEarlier(0x02) => "Pre-A3 (ES v0.2)",
            ChipRevision::A3OrEarlier(0x03) => "A3",
            ChipRevision::A3OrEarlier(_) => "A3 or Earlier",
            ChipRevision::A4 => "A4 (Letter Series)",
            ChipRevision::B4 => "B4 (Letter Series)",
            ChipRevision::Invalid(_) => "Invalid",
        }
    }

    /// Get raw REVID value
    pub fn raw_value(&self) -> u8 {
        match self {
            ChipRevision::A3OrEarlier(v) => *v,
            ChipRevision::A4 => 0x07,
            ChipRevision::B4 => 0x0F,
            ChipRevision::Invalid(v) => *v,
        }
    }

    /// Get the patch type for this revision
    ///
    /// Matches SDK's `lcpu_ble_patch_install()` logic:
    /// - `<= 0x03`: Use A3 patch (`lcpu_patch.c`)
    /// - `0x07, 0x0F`: Use Letter Series patch (`lcpu_patch_rev_b.c`)
    ///
    /// Returns `None` for invalid revisions.
    pub fn patch_type(&self) -> Option<PatchType> {
        match self {
            ChipRevision::A3OrEarlier(_) => Some(PatchType::A3),
            ChipRevision::A4 | ChipRevision::B4 => Some(PatchType::LetterSeries),
            ChipRevision::Invalid(_) => None,
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for ChipRevision {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            ChipRevision::A3OrEarlier(id) => {
                defmt::write!(fmt, "A3OrEarlier(0x{:02x})", id)
            }
            ChipRevision::A4 => defmt::write!(fmt, "A4"),
            ChipRevision::B4 => defmt::write!(fmt, "B4"),
            ChipRevision::Invalid(id) => defmt::write!(fmt, "Invalid(0x{:02x})", id),
        }
    }
}

/// LCPU patch type corresponding to chip revision
///
/// Different chip revisions require different LCPU ROM patches.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PatchType {
    /// A3 patch from `lcpu_patch.c`
    ///
    /// Used for REVID <= 0x03 (A3 and earlier revisions).
    /// Requires loading LCPU image from Flash.
    A3,

    /// Letter Series patch from `lcpu_patch_rev_b.c`
    ///
    /// Used for REVID 0x07 (A4) and 0x0F (B4).
    /// LCPU can run directly from ROM.
    LetterSeries,
}

#[cfg(feature = "defmt")]
impl defmt::Format for PatchType {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            PatchType::A3 => defmt::write!(fmt, "A3"),
            PatchType::LetterSeries => defmt::write!(fmt, "LetterSeries"),
        }
    }
}

// ============================================================================
// Future extensions (placeholder implementations)
// ============================================================================

/// Boot mode of the chip
///
/// Determined by the `HPSYS_CFG->BMR` register.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootMode {
    /// Normal boot mode
    Normal,
    /// Download/firmware update mode
    Download,
}

/// Read current boot mode
///
/// # Note
///
/// This function is not yet implemented. It will read the `HPSYS_CFG->BMR`
/// register to determine if the chip booted in download mode.
pub fn boot_mode() -> BootMode {
    todo!("boot_mode: read HPSYS_CFG->BMR register")
}

