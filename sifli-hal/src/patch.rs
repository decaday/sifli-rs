//! LCPU patch installation module.
//!
//! Chooses A3 or Letter-Series patch layout based on chip revision and writes into the proper memory region.
//!
//! ```no_run
//! use sifli_hal::{patch, syscfg};
//!
//! let idr = syscfg::read_idr();
//! patch::install(&idr, &PATCH_LIST, &PATCH_BIN)?;
//! ```

use crate::syscfg::Idr;

//=============================================================================
// Memory layouts
//=============================================================================

/// LCPU patch memory layout for A3 and earlier (HCPU view).
///
/// Addresses and sizes match SDK `mem_map.h`; only used internally.
#[derive(Debug, Clone, Copy)]
struct A3PatchLayout;

impl A3PatchLayout {
    /// Patch code start address.
    ///
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:328`
    const CODE_START: usize = 0x2040_6000;

    /// Patch record area address.
    ///
    /// Located at the last 256 bytes of the patch region (0x20406000 + 0x2000 - 0x100 = 0x20407F00).
    ///
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:331` (`LCPU_PATCH_RECORD_ADDR`)
    const RECORD_ADDR: usize = 0x2040_7F00;

    /// Total patch area size.
    ///
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:300`
    const TOTAL_SIZE: usize = 8 * 1024;
}

/// LCPU patch memory layout for Letter Series (A4+/Rev B, HCPU view).
///
/// Addresses and sizes match SDK `mem_map.h`; only used internally.
#[derive(Debug, Clone, Copy)]
struct LetterPatchLayout;

impl LetterPatchLayout {
    /// Patch buffer start address.
    ///
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:334`
    const BUF_START: usize = 0x2040_5000;

    /// Patch code start address (after 12-byte header, 0x20405000 + 12 = 0x2040500C).
    ///
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:335`
    const CODE_START: usize = 0x2040_500C;

    /// Patch buffer size.
    ///
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:337`
    const BUF_SIZE: usize = 0x3000; // 12KB

    /// Patch code usable size.
    ///
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/mem_map.h:338`
    const CODE_SIZE: usize = 0x2FF4; // 12KB - 12 bytes

    /// Letter Series patch header magic value.
    ///
    /// Reference: `SiFli-SDK/drivers/cmsis/sf32lb52x/lcpu_patch_rev_b.c:60`
    const MAGIC: u32 = 0x4843_4150; // "PACH" (little-endian)

    /// Fixed entry_count value in header.
    const ENTRY_COUNT: u32 = 7;
}

// ===== 通用常量 =====

/// Patch tag magic number.
///
/// Reference: `SiFli-SDK/drivers/Include/bf0_hal_patch.h:83`
#[allow(dead_code)]
const PATCH_TAG: u32 = 0x5054_4348; // "PTCH" (big-endian in memory)

//=============================================================================
// Error types
//=============================================================================

/// Patch installation error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Patch record list is empty.
    EmptyRecord,

    /// Patch code array is empty.
    EmptyCode,

    /// Patch code exceeds available space.
    CodeTooLarge {
        /// Actual size (bytes).
        size_bytes: usize,
        /// Maximum allowed size (bytes).
        max_bytes: usize,
    },

    /// Invalid or unsupported chip revision.
    InvalidRevision {
        /// Revision ID (`REVID`).
        revid: u8,
    },
}

//=============================================================================
// Core API
//=============================================================================

/// High-level helper to install LCPU patches based on chip revision.
///
/// ```no_run
/// use sifli_hal::{patch, syscfg};
///
/// let idr = syscfg::read_idr();
/// patch::install(&idr, &PATCH_LIST, &PATCH_BIN)?;
/// ```
pub fn install(idr: &Idr, list: &[u32], bin: &[u32]) -> Result<(), Error> {
    // Parameter validation.
    if list.is_empty() {
        return Err(Error::EmptyRecord);
    }
    if bin.is_empty() {
        return Err(Error::EmptyCode);
    }

    let revision = idr.revision();
    if !revision.is_valid() {
        return Err(Error::InvalidRevision { revid: idr.revid });
    }

    // Dispatch to A3 or Letter-Series patch installer based on revision.
    if revision.is_letter_series() {
        install_letter(list, bin)
    } else {
        install_a3(list, bin)
    }
}

/// Install A3 / earlier-format patches (internal).
fn install_a3(list: &[u32], bin: &[u32]) -> Result<(), Error> {
    let code_size = core::mem::size_of_val(bin);
    if code_size > A3PatchLayout::TOTAL_SIZE {
        return Err(Error::CodeTooLarge {
            size_bytes: code_size,
            max_bytes: A3PatchLayout::TOTAL_SIZE,
        });
    }

    debug!(
        "Installing A3 patch: record={} words, code={} bytes",
        list.len(),
        code_size
    );

    unsafe {
        // 1. Copy patch record list (entry list).
        let record_dst = A3PatchLayout::RECORD_ADDR as *mut u32;
        core::ptr::copy_nonoverlapping(list.as_ptr(), record_dst, list.len());

        // 2. Clear patch code area.
        let code_dst = A3PatchLayout::CODE_START as *mut u8;
        core::ptr::write_bytes(code_dst, 0, A3PatchLayout::TOTAL_SIZE);

        // 3. Copy patch code.
        let code_dst = A3PatchLayout::CODE_START as *mut u32;
        core::ptr::copy_nonoverlapping(bin.as_ptr(), code_dst, bin.len());
    }

    debug!("A3 patch installed successfully");
    Ok(())
}

/// Install Letter-Series patches (internal).
fn install_letter(_list: &[u32], bin: &[u32]) -> Result<(), Error> {
    let code_size = core::mem::size_of_val(bin);
    if code_size > LetterPatchLayout::CODE_SIZE {
        return Err(Error::CodeTooLarge {
            size_bytes: code_size,
            max_bytes: LetterPatchLayout::CODE_SIZE,
        });
    }

    debug!(
        "Installing Letter Series patch: code={} bytes",
        code_size
    );

    unsafe {
        // 1. Write header (12 bytes). Reference: lcpu_patch_rev_b.c:60-66.
        let header = [
            LetterPatchLayout::MAGIC,                 // magic: "PACH"
            LetterPatchLayout::ENTRY_COUNT,           // entry_count (fixed)
            LetterPatchLayout::CODE_START as u32 + 1, // code_addr (Thumb bit)
        ];
        let header_dst = LetterPatchLayout::BUF_START as *mut u32;
        core::ptr::copy_nonoverlapping(header.as_ptr(), header_dst, 3);

        // 2. Clear patch code area.
        let code_dst = LetterPatchLayout::CODE_START as *mut u8;
        core::ptr::write_bytes(code_dst, 0, LetterPatchLayout::CODE_SIZE);

        // 3. Copy patch code.
        let code_dst = LetterPatchLayout::CODE_START as *mut u32;
        core::ptr::copy_nonoverlapping(bin.as_ptr(), code_dst, bin.len());
    }

    info!("Letter Series patch installed successfully");
    Ok(())
}
