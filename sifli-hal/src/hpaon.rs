//! HPAON (High-Power Always-On) helper.
//!
//! Wraps the small subset of `HPSYS_AON` registers used to wake LCPU and
//! mirrors the SDK helpers:
//! - `HAL_HPAON_WakeCore(CORE_ID_LCPU)` (HP2LP_REQ / LP_ACTIVE logic only)
//! - `HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST()`

use crate::pac::HPSYS_AON;

/// HPAON driver namespace.
pub struct Hpaon;

impl Hpaon {
    /// Async wake of the LPSYS/LP domain that hosts LCPU.
    ///
    /// - Sets `HP2LP_REQ`.
    /// - Polls `LP_ACTIVE` with a timeout using the Embassy timer.
    ///
    /// Returns:
    /// - `true` if `LP_ACTIVE` is seen before the timeout.
    /// - `false` if the timeout expires.
    ///
    /// Mirrors `HAL_HPAON_WakeCore(CORE_ID_LCPU)` in `bf0_hal_hpaon.c:190`.
    pub async fn wake_lcpu_with_timeout(timeout_us: u32) -> bool {
        use embassy_time::{with_timeout, Duration, Timer};

        // Set HP2LP_REQ to request LPSYS/LP activation.
        HPSYS_AON.issr().modify(|w| w.set_hp2lp_req(true));

        // Embassy-style timeout wrapper around a polling future.
        let fut = async {
            loop {
                if HPSYS_AON.issr().read().lp_active() {
                    return true;
                }
                Timer::after_micros(10).await;
            }
        };

        match with_timeout(Duration::from_micros(timeout_us as u64), fut).await {
            Ok(result) => result,
            Err(_) => false,
        }
    }

    /// Blocking wake of the LPSYS/LP domain that hosts LCPU.
    ///
    /// Equivalent to the SDK implementation based on `HAL_Delay_us`.
    pub fn wake_lcpu_with_timeout_blocking(timeout_us: u32) -> bool {
        // Set HP2LP_REQ to request LPSYS/LP activation.
        HPSYS_AON.issr().modify(|w| w.set_hp2lp_req(true));

        // Use a simple time-based wait-with-timeout loop:
        // - Poll LP_ACTIVE on each iteration.
        // - Busy-wait for a small fixed step.
        // - Stop and return false if total wait exceeds timeout_us.
        let step_us: u32 = 10;
        let mut waited: u32 = 0;

        loop {
            if HPSYS_AON.issr().read().lp_active() {
                return true;
            }

            if waited >= timeout_us {
                return false;
            }

            crate::cortex_m_blocking_delay_us(step_us);
            waited = waited.saturating_add(step_us);
        }
    }

    /// Cancel an LCPU LP_ACTIVE request.
    ///
    /// Clears the `HP2LP_REQ` bit; mirrors `HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST()`.
    pub fn cancel_lp_active_request() {
        HPSYS_AON.issr().modify(|w| w.set_hp2lp_req(false));
    }
}
