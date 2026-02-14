use crate::time::Hertz;

macro_rules! define_clock_token {
    ($name:ident, $field:ident) => {
        pub struct $name {
            _private: (),
        }
        impl $name {
            pub(crate) fn new() -> Self {
                Self { _private: () }
            }
            pub fn frequency(&self) -> Option<Hertz> {
                crate::rcc::clocks().$field.into()
            }
        }
    };
}

define_clock_token!(Hclk, hclk);
define_clock_token!(Pclk, pclk);
define_clock_token!(Pclk2, pclk2);
define_clock_token!(ClkPeri, clk_peri);
define_clock_token!(ClkPeriDiv2, clk_peri_div2);
define_clock_token!(ClkUsb, clk_usb);
define_clock_token!(ClkWdt, clk_wdt);
define_clock_token!(ClkRtc, clk_rtc);
define_clock_token!(ClkMpi1, clk_mpi1);
define_clock_token!(ClkMpi2, clk_mpi2);
define_clock_token!(ClkAudPll, clk_aud_pll);
define_clock_token!(ClkAudPllDiv16, clk_aud_pll_div16);
/// LPSYS HCLK token — always reads from hardware registers,
/// because LPSYS clocks may be changed by LCPU independently.
pub struct LpHclk {
    _private: (),
}
impl LpHclk {
    pub(crate) fn new() -> Self {
        Self { _private: () }
    }
    pub fn frequency(&self) -> Option<Hertz> {
        crate::rcc::get_lpsys_hclk_freq()
    }
}

/// LPSYS MAC clock token — always reads from hardware registers.
pub struct LpMacClk {
    _private: (),
}
impl LpMacClk {
    pub(crate) fn new() -> Self {
        Self { _private: () }
    }
    pub fn frequency(&self) -> Option<Hertz> {
        crate::rcc::get_lpsys_mac_clk_freq()
    }
}

/// All clock tokens collected together.
///
/// Each field is a ZST token representing a clock domain. Peripheral constructors
/// borrow these tokens (`&'d Token`) to tie their lifetime to the clock configuration.
/// Reconfiguration functions take `&mut Token` to ensure no peripheral is using that clock.
///
/// Rust's split borrowing on struct fields guarantees that borrowing different clock
/// domains is independent — you can hold `&clk.clk_peri` while taking `&mut clk.hclk`.
pub struct ClockControl {
    pub hclk: Hclk,
    pub pclk: Pclk,
    pub pclk2: Pclk2,
    pub clk_peri: ClkPeri,
    pub clk_peri_div2: ClkPeriDiv2,
    pub clk_usb: ClkUsb,
    pub clk_wdt: ClkWdt,
    pub clk_rtc: ClkRtc,
    pub clk_mpi1: ClkMpi1,
    pub clk_mpi2: ClkMpi2,
    pub clk_aud_pll: ClkAudPll,
    pub clk_aud_pll_div16: ClkAudPllDiv16,
    pub lp_hclk: LpHclk,
    pub lp_mac_clk: LpMacClk,
}

impl ClockControl {
    pub(crate) fn new() -> Self {
        Self {
            hclk: Hclk::new(),
            pclk: Pclk::new(),
            pclk2: Pclk2::new(),
            clk_peri: ClkPeri::new(),
            clk_peri_div2: ClkPeriDiv2::new(),
            clk_usb: ClkUsb::new(),
            clk_wdt: ClkWdt::new(),
            clk_rtc: ClkRtc::new(),
            clk_mpi1: ClkMpi1::new(),
            clk_mpi2: ClkMpi2::new(),
            clk_aud_pll: ClkAudPll::new(),
            clk_aud_pll_div16: ClkAudPllDiv16::new(),
            lp_hclk: LpHclk::new(),
            lp_mac_clk: LpMacClk::new(),
        }
    }
}
