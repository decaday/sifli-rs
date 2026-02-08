//! RFC command sequence generation for BLE MAC hardware.
//!
//! The BLE MAC controller automatically executes RFC command sequences during
//! TX/RX operations to control the RF front-end (power, PLL, VCO, ADC, etc.).
//! These sequences are stored in RFC SRAM and their addresses written to
//! `CU_ADDR_REG1/2/3`.
//!
//! Without these command sequences, the MAC cannot operate the RF front-end,
//! and no BLE packets will be transmitted or received.
//!
//! Based on SDK `bt_rfc_init()` in `bt_rf_fulcal.c`.

use crate::pac::BT_RFC;

// RFC command opcodes (16-bit instructions, two packed per 32-bit word)
const fn rd(n: u16) -> u16 {
    0x1800 + n
}
const fn wr(n: u16) -> u16 {
    0x2800 + n
}
const fn and(n: u16) -> u16 {
    0x3000 + n
}
const fn or(n: u16) -> u16 {
    0x4000 + n
}
const fn wait(n: u16) -> u16 {
    0x5000 + n
}
const RD_FULCAL: u16 = 0x6000;
const RD_DCCAL1: u16 = 0x7000;
const RD_DCCAL2: u16 = 0x8000;
const END: u16 = 0xF000;

/// Helper to build a command sequence in a fixed-size buffer.
struct CmdBuilder {
    buf: [u16; 128],
    len: usize,
}

impl CmdBuilder {
    const fn new() -> Self {
        Self {
            buf: [0; 128],
            len: 0,
        }
    }

    fn push(&mut self, cmd: u16) {
        self.buf[self.len] = cmd;
        self.len += 1;
    }

    /// Pad to even length (commands are packed as pairs into 32-bit words).
    fn pad_even(&mut self) {
        if self.len % 2 != 0 {
            self.push(END);
        }
    }

    /// Write the command sequence to RFC SRAM starting at `offset`.
    /// Returns the next available offset (after the last written word).
    fn write_to_sram(&self, offset: u32) -> u32 {
        let base = super::BT_RFC_MEM_BASE;
        let mut addr = offset;
        for i in (0..self.len).step_by(2) {
            let lo = self.buf[i] as u32;
            let hi = self.buf[i + 1] as u32;
            let word = lo | (hi << 16);
            unsafe {
                core::ptr::write_volatile((base + addr) as *mut u32, word);
            }
            addr += 4;
        }
        addr
    }
}

/// Build the RXON command sequence (BLE RX startup).
fn build_rxon() -> CmdBuilder {
    let mut c = CmdBuilder::new();

    // VDDPSW/RFBG_EN/LO_IARY_EN
    c.push(rd(0x10));
    c.push(rd(0x10));
    c.push(or(18));
    c.push(or(17));
    c.push(or(16));
    c.push(wr(0x10));

    // wait 1us
    c.push(wait(2));

    // FULCAL RSLT
    c.push(RD_FULCAL);
    c.push(wr(0x08));

    // VCO5G_EN
    c.push(rd(0x00));
    c.push(or(12));
    c.push(wr(0x00));

    // PFDCP_EN
    c.push(rd(0x1C));
    c.push(or(19));
    c.push(wr(0x1C));

    // FBDV_EN
    c.push(rd(0x14));
    c.push(or(12));
    c.push(wr(0x14));

    // FBDV_RSTB
    c.push(rd(0x14));
    c.push(and(0x7));
    c.push(wr(0x14));

    // wait 30us for LO lock
    c.push(wait(45));

    // VCO_FLT_EN
    c.push(rd(0x00));
    c.push(or(7));
    c.push(wr(0x00));

    // LDO11_EN & LNA_SHUNTSW
    c.push(rd(0x44));
    c.push(or(22));
    c.push(and(6));
    c.push(wr(0x44));

    // ADC & LDO_ADC & LDO_ADCREF
    c.push(rd(0x60));
    c.push(or(4));
    c.push(or(9));
    c.push(or(21));
    c.push(or(20));
    c.push(wr(0x60));

    // LDO_RBB
    c.push(rd(0x48));
    c.push(or(13));
    c.push(wr(0x48));

    // PA_TX_RX
    c.push(rd(0x38));
    c.push(and(9));
    c.push(wr(0x38));

    // EN_IARRAY & EN_OSDAC
    c.push(rd(0x58));
    c.push(or(5));
    c.push(or(6));
    c.push(or(7));
    c.push(wr(0x58));

    // EN_CBPF & EN_RVGA
    c.push(rd(0x4C));
    c.push(or(27));
    c.push(or(6));
    c.push(or(7));
    c.push(wr(0x4C));

    // EN_PKDET
    c.push(rd(0x50));
    c.push(or(0));
    c.push(or(1));
    c.push(or(2));
    c.push(or(3));
    c.push(wr(0x50));

    // wait 4us
    c.push(wait(5));

    // LODIST5G_RX_EN
    c.push(rd(0x10));
    c.push(or(9));
    c.push(wr(0x10));

    // LNA_PU & MX_PU
    c.push(rd(0x44));
    c.push(or(3));
    c.push(or(17));
    c.push(wr(0x44));

    // START INCCAL
    c.push(rd(0x74));
    c.push(or(29));
    c.push(wr(0x74));

    c.push(wait(30));

    // END
    c.push(END);
    c.pad_even();
    c
}

/// Build the RXOFF command sequence (BLE RX shutdown).
fn build_rxoff() -> CmdBuilder {
    let mut c = CmdBuilder::new();

    // VDDPSW/RFBG/LODIST5G_RX_EN/LO_IARY_EN
    c.push(rd(0x10));
    c.push(rd(0x10));
    c.push(and(18));
    c.push(and(17));
    c.push(and(16));
    c.push(and(9));
    c.push(wr(0x10));

    // VCO5G_EN & VCO_FLT_EN
    c.push(rd(0x00));
    c.push(and(12));
    c.push(and(7));
    c.push(wr(0x00));

    // FBDV_EN / FBDV_RSTB
    c.push(rd(0x14));
    c.push(and(12));
    c.push(or(0x7));
    c.push(wr(0x14));

    // PFDCP_EN
    c.push(rd(0x1C));
    c.push(and(19));
    c.push(wr(0x1C));

    // LNA_PU & MX_PU & LDO11_EN & LNA_SHUNTSW
    c.push(rd(0x44));
    c.push(and(3));
    c.push(or(6));
    c.push(and(17));
    c.push(and(22));
    c.push(wr(0x44));

    // ADC & LDO_ADC & LDO_ADCREF
    c.push(rd(0x60));
    c.push(and(4));
    c.push(and(9));
    c.push(and(21));
    c.push(and(20));
    c.push(wr(0x60));

    // LDO_RBB
    c.push(rd(0x48));
    c.push(and(13));
    c.push(wr(0x48));

    // PA_TX_RX
    c.push(rd(0x38));
    c.push(or(9));
    c.push(wr(0x38));

    // EN_IARRAY & EN_OSDAC
    c.push(rd(0x58));
    c.push(and(5));
    c.push(and(6));
    c.push(and(7));
    c.push(wr(0x58));

    // EN_CBPF & EN_RVGA
    c.push(rd(0x4C));
    c.push(and(27));
    c.push(and(6));
    c.push(and(7));
    c.push(wr(0x4C));

    // EN_PKDET
    c.push(rd(0x50));
    c.push(and(0));
    c.push(and(1));
    c.push(and(2));
    c.push(and(3));
    c.push(wr(0x50));

    // END
    c.push(END);
    c.pad_even();
    c
}

/// Build the TXON command sequence (BLE TX startup).
fn build_txon() -> CmdBuilder {
    let mut c = CmdBuilder::new();

    // VDDPSW/RFBG_EN/LO_IARY_EN
    c.push(rd(0x10));
    c.push(rd(0x10));
    c.push(or(17));
    c.push(or(18));
    c.push(or(16));
    c.push(wr(0x10));

    // wait 1us
    c.push(wait(2));

    // RD FULCAL
    c.push(RD_FULCAL);
    c.push(wr(0x08));

    // VCO5G_EN
    c.push(rd(0x00));
    c.push(or(12));
    c.push(wr(0x00));

    // FBDV_EN
    c.push(rd(0x14));
    c.push(or(12));
    c.push(wr(0x14));

    // PFDCP_EN
    c.push(rd(0x1C));
    c.push(or(19));
    c.push(wr(0x1C));

    // FBDV_RSTB
    c.push(rd(0x14));
    c.push(and(0x7));
    c.push(wr(0x14));

    // wait 30us for LO lock
    c.push(wait(30));

    // VCO_FLT_EN
    c.push(rd(0x00));
    c.push(or(7));
    c.push(wr(0x00));

    // LODIST5G_BLETX_EN
    c.push(rd(0x10));
    c.push(or(8));
    c.push(wr(0x10));

    // EDR_IARRAY_EN
    c.push(rd(0x3C));
    c.push(or(20));
    c.push(wr(0x3C));

    // PA_BUF_PU for normal TX
    c.push(rd(0x34));
    c.push(or(22));
    c.push(wr(0x34));

    // EDR_XFMR_SG
    c.push(rd(0x40));
    c.push(and(11));
    c.push(wr(0x40));

    // wait 4us
    c.push(wait(5));

    // PA_OUT_PU & TRF_SIG_EN
    c.push(rd(0x34));
    c.push(or(16));
    c.push(or(21));
    c.push(wr(0x34));

    // START INCCAL
    c.push(rd(0x74));
    c.push(or(29));
    c.push(wr(0x74));
    c.push(wait(9));

    // END
    c.push(END);
    c.pad_even();
    c
}

/// Build the TXOFF command sequence (BLE TX shutdown).
fn build_txoff() -> CmdBuilder {
    let mut c = CmdBuilder::new();

    // VDDPSW/RFBG_EN/LO_IARY_EN/LODIST5G_BLETX_EN
    c.push(rd(0x10));
    c.push(rd(0x10));
    c.push(and(8));
    c.push(and(16));
    c.push(and(17));
    c.push(and(18));
    c.push(wr(0x10));

    // VCO5G_EN & VCO_FLT_EN
    c.push(rd(0x00));
    c.push(and(12));
    c.push(and(7));
    c.push(wr(0x00));

    // FBDV_EN / FBDV_RSTB
    c.push(rd(0x14));
    c.push(and(12));
    c.push(or(0x7));
    c.push(wr(0x14));

    // PFDCP_EN
    c.push(rd(0x1C));
    c.push(and(19));
    c.push(wr(0x1C));

    // PA_BUF_PU & PA_OUT_PU & TRF_SIG_EN
    c.push(rd(0x34));
    c.push(and(22));
    c.push(and(16));
    c.push(and(21));
    c.push(wr(0x34));

    // TRF_EDR_IARRAY_EN
    c.push(rd(0x3C));
    c.push(and(20));
    c.push(wr(0x3C));

    // Redundancy from bt_txoff:
    // DAC_STOP / EN_TBB_IARRY & EN_LDO_DAC_AVDD & EN_LDO_DAC_DVDD & EN_DAC
    c.push(rd(0x64));
    c.push(and(8));
    c.push(and(9));
    c.push(and(10));
    c.push(and(11));
    c.push(and(12));
    c.push(wr(0x64));

    // EDR_PACAP_EN & EDR_PA_XFMR_SG
    c.push(rd(0x40));
    c.push(and(11));
    c.push(and(17));
    c.push(wr(0x40));

    // TRF_EDR_IARRAY_EN
    c.push(rd(0x3C));
    c.push(and(2));
    c.push(and(12));
    c.push(and(19));
    c.push(wr(0x3C));

    // EDR_EN_OSLO
    c.push(rd(0x28));
    c.push(and(11));
    c.push(wr(0x28));

    // VCO3G_EN/EDR_VCO_FLT_EN
    c.push(rd(0x00));
    c.push(and(13));
    c.push(and(7));
    c.push(wr(0x00));

    // EDR_FBDV_RSTB
    c.push(rd(0x14));
    c.push(or(7));
    c.push(wr(0x14));

    // EDR PFDCP_EN
    c.push(rd(0x1C));
    c.push(and(19));
    c.push(wr(0x1C));

    // EDR FBDV_EN/MOD_STG/SDM_CLK_SEL
    c.push(rd(0x14));
    c.push(and(12));
    c.push(or(5));
    c.push(and(4));
    c.push(or(3));
    c.push(wr(0x14));

    // ACAL_VH_SEL=3/ACAL_VL_SEL=1
    c.push(rd(0x04));
    c.push(and(2));
    c.push(and(6));
    c.push(wr(0x04));

    // LDO_RBB
    c.push(rd(0x48));
    c.push(and(13));
    c.push(wr(0x48));

    // EDR VCO3G_EN/EDR_VCO5G_EN
    c.push(rd(0x00));
    c.push(and(13));
    c.push(wr(0x24));

    // VDDPSW/RFBG_EN/LO_IARY_EN/LODISTEDR_EN
    c.push(rd(0x10));
    c.push(and(0));
    c.push(and(16));
    c.push(and(17));
    c.push(and(18));
    c.push(wr(0x10));

    // END
    c.push(END);
    c.pad_even();
    c
}

/// Build the BT_TXON command sequence (BR/EDR TX startup).
fn build_bt_txon() -> CmdBuilder {
    let mut c = CmdBuilder::new();

    // VDDPSW/RFBG_EN/LO_IARY_EN
    c.push(rd(0x10));
    c.push(rd(0x10));
    c.push(or(16));
    c.push(or(17));
    c.push(or(18));
    c.push(wr(0x10));

    // wait 1us
    c.push(wait(2));

    // LDO_RBB
    c.push(rd(0x48));
    c.push(or(13));
    c.push(wr(0x48));

    // RD FULCAL
    c.push(RD_FULCAL);
    c.push(wr(0x24));
    c.push(wr(0x6C));

    // VCO3G_EN
    c.push(rd(0x00));
    c.push(or(13));
    c.push(wr(0x00));

    // PFDCP_EN ICP_SET
    c.push(rd(0x1C));
    c.push(or(19));
    c.push(or(11));
    c.push(and(13));
    c.push(wr(0x1C));

    // FBDV_EN/MOD_STG/SDM_CLK_SEL
    c.push(rd(0x14));
    c.push(or(12));
    c.push(and(5));
    c.push(or(4));
    c.push(and(3));
    c.push(wr(0x14));

    // FBDV_RSTB
    c.push(rd(0x14));
    c.push(and(7));
    c.push(wr(0x14));

    // ACAL_VH_SEL=7/ACAL_VL_SEL=5
    c.push(rd(0x04));
    c.push(or(2));
    c.push(or(6));
    c.push(wr(0x04));

    // EDR_VCO_FLT_EN
    c.push(rd(0x00));
    c.push(or(7));
    c.push(wr(0x00));

    // EDR_EN_OSLO
    c.push(rd(0x28));
    c.push(or(11));
    c.push(wr(0x28));

    // LODISTEDR_EN
    c.push(rd(0x10));
    c.push(or(0));
    c.push(wr(0x10));

    // EN_TBB_IARRY & EN_LDO_DAC_AVDD & EN_LDO_DAC_DVDD & EN_DAC
    c.push(rd(0x64));
    c.push(or(8));
    c.push(or(9));
    c.push(or(10));
    c.push(or(11));
    c.push(wr(0x64));

    // TRF_EDR_IARRAY_EN
    c.push(rd(0x3C));
    c.push(or(20));
    c.push(wr(0x3C));

    // EDR_PACAP_EN & EDR_PA_XFMR_SG
    c.push(rd(0x40));
    c.push(or(11));
    c.push(or(17));
    c.push(wr(0x40));

    // RD DCCAL
    c.push(RD_DCCAL1);
    c.push(wr(0xA8));
    c.push(RD_DCCAL2);
    c.push(wr(0xAC));

    // EDR_TMXBUF_PU EDR_TMX_PU
    c.push(rd(0x3C));
    c.push(or(12));
    c.push(or(19));
    c.push(wr(0x3C));

    // cmd for cal: RBB_REG5 EN_IARRAY
    c.push(rd(0x58));
    c.push(or(5));
    c.push(wr(0x58));

    // EN_RVGA_I
    c.push(rd(0x4C));
    c.push(or(7));
    c.push(wr(0x4C));

    // ADC & LDO_ADC & LDO_ADCREF
    c.push(rd(0x60));
    c.push(or(4));
    c.push(or(9));
    c.push(or(21));
    c.push(wr(0x60));

    // wait 5us
    c.push(wait(8));

    // pwrmtr_en
    c.push(rd(0x40));
    c.push(or(10));
    c.push(wr(0x40));

    // wait 3us
    c.push(wait(5));

    // lpbk en
    c.push(rd(0x58));
    c.push(or(0));
    c.push(wr(0x58));

    // wait 30us for LO lock
    c.push(wait(20));

    // START INCCAL
    c.push(rd(0x74));
    c.push(or(29));
    c.push(wr(0x74));
    c.push(wait(9));

    // DAC_START
    c.push(rd(0x64));
    c.push(or(12));
    c.push(wr(0x64));

    // EDR_PA_PU
    c.push(rd(0x3C));
    c.push(or(2));
    c.push(wr(0x3C));

    // END
    c.push(END);
    c.pad_even();
    c
}

/// Build the BT_TXOFF command sequence (BR/EDR TX shutdown).
fn build_bt_txoff() -> CmdBuilder {
    let mut c = CmdBuilder::new();

    // EDR_PA_PU / EDR_TMXBUF_PU / EDR_TMX_PU
    c.push(rd(0x3C));
    c.push(rd(0x3C));
    c.push(and(2));
    c.push(and(12));
    c.push(and(19));
    c.push(wr(0x3C));

    // DAC_STOP / EN_TBB_IARRY & EN_LDO_DAC_AVDD & EN_LDO_DAC_DVDD & EN_DAC
    c.push(rd(0x64));
    c.push(and(8));
    c.push(and(9));
    c.push(and(10));
    c.push(and(11));
    c.push(and(12));
    c.push(wr(0x64));

    // EDR_PACAP_EN & EDR_PA_XFMR_SG
    c.push(rd(0x40));
    c.push(and(11));
    c.push(and(17));
    c.push(wr(0x40));

    // cmd for cal: lpbk en
    c.push(rd(0x58));
    c.push(and(0));
    c.push(wr(0x58));

    // wait 1us
    c.push(wait(2));

    // pwrmtr_en
    c.push(rd(0x40));
    c.push(and(10));
    c.push(wr(0x40));

    // wait 1us
    c.push(wait(2));

    // EN_IARRAY
    c.push(rd(0x58));
    c.push(and(5));
    c.push(wr(0x58));

    // EN_RVGA_I
    c.push(rd(0x4C));
    c.push(and(7));
    c.push(wr(0x4C));

    // ADC & LDO_ADC & LDO_ADCREF
    c.push(rd(0x60));
    c.push(and(4));
    c.push(and(9));
    c.push(and(21));
    c.push(wr(0x60));

    // TRF_EDR_IARRAY_EN
    c.push(rd(0x3C));
    c.push(and(20));
    c.push(wr(0x3C));

    // EDR_EN_OSLO
    c.push(rd(0x28));
    c.push(and(11));
    c.push(wr(0x28));

    // VCO3G_EN/EDR_VCO_FLT_EN
    c.push(rd(0x00));
    c.push(and(13));
    c.push(and(7));
    c.push(wr(0x00));

    // EDR_FBDV_RSTB
    c.push(rd(0x14));
    c.push(or(7));
    c.push(wr(0x14));

    // EDR PFDCP_EN ICP_SET
    c.push(rd(0x1C));
    c.push(and(19));
    c.push(and(11));
    c.push(or(13));
    c.push(wr(0x1C));

    // EDR FBDV_EN/MOD_STG/SDM_CLK_SEL
    c.push(rd(0x14));
    c.push(and(12));
    c.push(or(5));
    c.push(and(4));
    c.push(or(3));
    c.push(wr(0x14));

    // ACAL_VH_SEL=3/ACAL_VL_SEL=1
    c.push(rd(0x04));
    c.push(and(2));
    c.push(and(6));
    c.push(wr(0x04));

    // LDO_RBB
    c.push(rd(0x48));
    c.push(and(13));
    c.push(wr(0x48));

    // EDR VCO3G_EN/EDR_VCO5G_EN
    c.push(rd(0x00));
    c.push(and(13));
    c.push(wr(0x24));

    // VDDPSW/RFBG_EN/LO_IARY_EN/LODISTEDR_EN
    c.push(rd(0x10));
    c.push(and(0));
    c.push(and(16));
    c.push(and(17));
    c.push(and(18));
    c.push(wr(0x10));

    // Redundant commands to fix control change while txoff:
    // VCO5G_EN & VCO_FLT_EN
    c.push(rd(0x00));
    c.push(and(12));
    c.push(and(7));
    c.push(wr(0x00));

    // FBDV_EN / FBDV_RSTB
    c.push(rd(0x14));
    c.push(and(12));
    c.push(or(0x7));
    c.push(wr(0x14));

    // PFDCP_EN
    c.push(rd(0x1C));
    c.push(and(19));
    c.push(wr(0x1C));

    // PA_BUF_PU & PA_OUT_PU & TRF_SIG_EN
    c.push(rd(0x34));
    c.push(and(22));
    c.push(and(16));
    c.push(and(21));
    c.push(wr(0x34));

    // END
    c.push(END);
    c.pad_even();
    c
}

/// Initialize INCCAL timing registers.
fn init_inccal_timing() {
    // Use write() instead of modify() to clear residual idac_offset/pdx_offset fields.
    BT_RFC.inccal_reg1().write(|w| {
        w.set_vco3g_auto_incacal_en(false);
        w.set_vco3g_auto_incfcal_en(false);
        w.set_vco3g_incacal_wait_time(0x3F);
        w.set_vco3g_incfcal_wait_time(0x3F);
        w.set_vco3g_idac_offset(0);
        w.set_vco3g_pdx_offset(0);
        w.set_frc_inccal_clk_on(false);
    });
    BT_RFC.inccal_reg2().write(|w| {
        w.set_vco5g_auto_incacal_en(false);
        w.set_vco5g_auto_incfcal_en(false);
        w.set_vco5g_incacal_wait_time(0x3F);
        w.set_vco5g_incfcal_wait_time(0x3F);
        w.set_vco5g_idac_offset(0);
        w.set_vco5g_pdx_offset(0);
    });
}

/// Generate all RFC command sequences and write them to RFC SRAM.
///
/// This is the core function that makes BLE TX/RX work. It:
/// 1. Initializes INCCAL timing registers
/// 2. Builds 6 command sequences (rxon/rxoff/txon/txoff/bt_txon/bt_txoff)
/// 3. Writes them to RFC SRAM
/// 4. Sets CU_ADDR_REG1/2/3 to point to the sequences
///
/// Must be called after `reset_bluetooth_rf()` and the basic `rfc_init()`.
///
/// Returns the next free SRAM offset after all sequences.
pub fn generate_rfc_cmd_sequences() -> u32 {
    // Initialize INCCAL timing
    init_inccal_timing();

    // Starting offset in RFC SRAM (same as SDK: reg_addr = 0)
    let mut addr: u32 = 0;

    // === RXON ===
    let rxon = build_rxon();
    let rxon_addr = addr;
    BT_RFC.cu_addr_reg1().write(|w| {
        w.set_rxon_cfg_addr(rxon_addr as u16);
    });
    addr = rxon.write_to_sram(rxon_addr);

    // === RXOFF ===
    let rxoff = build_rxoff();
    let rxoff_addr = addr + 4; // gap between sequences (SDK: rxoff_addr = rxon_addr + 4)
    BT_RFC.cu_addr_reg1().modify(|w| {
        w.set_rxoff_cfg_addr(rxoff_addr as u16);
    });
    addr = rxoff.write_to_sram(rxoff_addr);

    // === TXON ===
    let txon = build_txon();
    let txon_addr = addr + 4;
    BT_RFC.cu_addr_reg2().write(|w| {
        w.set_txon_cfg_addr(txon_addr as u16);
    });
    addr = txon.write_to_sram(txon_addr);

    // === TXOFF ===
    let txoff = build_txoff();
    let txoff_addr = addr + 4;
    BT_RFC.cu_addr_reg2().modify(|w| {
        w.set_txoff_cfg_addr(txoff_addr as u16);
    });
    addr = txoff.write_to_sram(txoff_addr);

    // === BT_TXON ===
    let bt_txon = build_bt_txon();
    let bt_txon_addr = addr + 4;
    BT_RFC.cu_addr_reg3().write(|w| {
        w.set_bt_txon_cfg_addr(bt_txon_addr as u16);
    });
    addr = bt_txon.write_to_sram(bt_txon_addr);

    // === BT_TXOFF ===
    let bt_txoff = build_bt_txoff();
    let bt_txoff_addr = addr + 4;
    BT_RFC.cu_addr_reg3().modify(|w| {
        w.set_bt_txoff_cfg_addr(bt_txoff_addr as u16);
    });
    addr = bt_txoff.write_to_sram(bt_txoff_addr);

    addr
}
