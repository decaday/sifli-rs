//! LCPU image installation helper.
//!
//! Chooses whether to copy firmware into LPSYS RAM based on chip revision.

use core::ptr;

use crate::syscfg::Idr;

/// LPSYS RAM layout (HCPU view, SF32LB52x only).
///
/// Base address and size are taken from `mem_map.h`; currently only LCPU code area is modeled.
#[derive(Debug, Clone, Copy)]
pub struct LpsysRam;

impl LpsysRam {
    /// LPSYS RAM base address (HCPU view)
    pub const BASE: usize = 0x2040_0000;

    /// LPSYS RAM size for A3 and earlier revisions (24KB)
    /// Reference: SiFli-SDK `mem_map.h` `LPSYS_RAM_SIZE`
    pub const SIZE: usize = 24 * 1024;

    /// LCPU code start address for SF32LB52x (same as LPSYS RAM base).
    pub const CODE_START: usize = Self::BASE;
}

/// LPSYS RAM base address (HCPU view).
///
/// Prefer using [`LpsysRam::BASE`] in new code.
pub const LPSYS_RAM_BASE: usize = LpsysRam::BASE;

/// LCPU code start address for SF32LB52x.
///
/// Prefer using [`LpsysRam::CODE_START`] in new code.
pub const LCPU_CODE_START_ADDR: usize = LpsysRam::CODE_START;

/// LPSYS RAM size for A3 and earlier revisions (24KB).
/// Reference: `LPSYS_RAM_SIZE` in `mem_map.h`.
///
/// Prefer using [`LpsysRam::SIZE`] in new code.
pub const LPSYS_RAM_SIZE: usize = LpsysRam::SIZE;

/// LCPU image installation errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Image is empty
    EmptyImage,
    /// Image size exceeds LPSYS RAM capacity
    ImageTooLarge { size_bytes: usize, max_bytes: usize },
    /// Unknown or invalid chip revision
    InvalidRevision { revid: u8 },
}

/// Install an LCPU firmware image, automatically deciding whether to write into LPSYS RAM.
///
/// ```no_run
/// use sifli_hal::{lcpu_img, syscfg};
///
/// let idr = syscfg::read_idr();
/// lcpu_img::install(&idr, &[0u8])?;
/// ```
#[inline]
pub fn install(idr: &Idr, image: &[u8]) -> Result<(), Error> {
    if image.is_empty() {
        return Err(Error::EmptyImage);
    }

    let revision = idr.revision();

    if !revision.is_valid() {
        let revid = idr.revid;

        warn!("Invalid chip revision: 0x{:02x}", revid);

        return Err(Error::InvalidRevision { revid });
    }

    // Only A3 or Earlier is required to load LCPU image
    if !revision.is_letter_series() {
        let size_bytes = image.len();
        if size_bytes > LpsysRam::SIZE {
            error!(
                "LCPU image too large: {} bytes (max {} bytes)",
                size_bytes,
                LpsysRam::SIZE
            );

            return Err(Error::ImageTooLarge {
                size_bytes,
                max_bytes: LpsysRam::SIZE,
            });
        }

        debug!("Installing LCPU image: {} bytes", size_bytes);

        unsafe {
            install_image_unsafe(image);
        }

        debug!("LCPU image installed successfully");
    } else {
        debug!("Letter Series detected, skipping image install");
    }

    Ok(())
}

/// Internal unsafe helper to copy the image into LPSYS RAM.
///
/// # Safety
///
/// Caller must ensure size is valid and LCPU does not access the target region while copying.
#[inline]
unsafe fn install_image_unsafe(image: &[u8]) {
    let dst = LpsysRam::CODE_START as *mut u8;
    let len = image.len();

    // Efficiently copy LCPU image into LPSYS RAM.
    ptr::copy_nonoverlapping(image.as_ptr(), dst, len);
}
