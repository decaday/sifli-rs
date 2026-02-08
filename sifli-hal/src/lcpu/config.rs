//! LCPU BLE configuration types.
//!
//! User-facing configuration for ROM parameters, Exchange Memory layout,
//! and BLE/BT activity limits. These are written to the ROM configuration
//! area before LCPU startup (Letter Series only for EM/ACT).

/// User-configurable ROM parameters.
#[derive(Debug, Clone, Copy)]
pub struct RomConfig {
    /// Watchdog timeout (in seconds, default 10).
    pub wdt_time: u32,
    /// Watchdog clock frequency (Hz, default 32768).
    pub wdt_clk: u16,
    /// Enable external low-speed crystal (default true).
    pub enable_lxt: bool,
    /// BLE Exchange Memory buffer layout (Letter Series only).
    /// `None` = use ROM defaults.
    pub em_config: Option<EmConfig>,
    /// BLE/BT activity configuration (Letter Series only).
    /// `None` = use ROM defaults.
    pub act_config: Option<ActConfig>,
}

impl Default for RomConfig {
    fn default() -> Self {
        Self {
            wdt_time: 10,
            wdt_clk: 32_768,
            enable_lxt: true,
            em_config: Some(EmConfig::DEFAULT),
            act_config: Some(ActConfig::DEFAULT),
        }
    }
}

/// BLE Exchange Memory buffer configuration.
///
/// Defines internal memory layout of the BLE controller's Exchange Memory.
/// Written to ROM config area at offset 32 (Letter Series only).
/// Reference: `HAL_LCPU_CONFIG_BT_EM_BUF` in SDK.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct EmConfig {
    /// Validity flag (1 = valid, 0 = use ROM defaults).
    pub is_valid: u8,
    // 1 byte implicit padding (repr(C) aligns em_buf to u16)
    /// EM buffer offset table (up to 40 entries).
    pub em_buf: [u16; Self::MAX_NUM],
}

impl EmConfig {
    /// Maximum number of EM buffer entries.
    pub const MAX_NUM: usize = 40;

    /// ROM config offset for this structure.
    pub(crate) const ROM_OFFSET: usize = 32;

    /// Default configuration matching SDK `g_em_offset` (SF32LB52X peripheral example).
    pub const DEFAULT: Self = Self {
        is_valid: 1,
        em_buf: [
            0x178, 0x178, 0x740, 0x7A0, 0x810, 0x880, 0xA00, 0xBB0, 0xD48,
            0x133C, 0x13A4, 0x19BC, 0x21BC, 0x21BC, 0x21BC, 0x21BC, 0x21BC, 0x21BC,
            0x21BC, 0x21BC, 0x263C, 0x265C, 0x2734, 0x2784, 0x28D4, 0x28E8, 0x28FC,
            0x29EC, 0x29FC, 0x2BBC, 0x2BD8, 0x3BE8, 0x5804, 0x5804, 0x5804,
            0, 0, 0, 0, 0,
        ],
    };

    /// Set EM buffer offset table.
    pub const fn em_buf(mut self, buf: [u16; Self::MAX_NUM]) -> Self {
        self.em_buf = buf;
        self
    }
}

impl Default for EmConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// BLE/BT activity configuration.
///
/// Defines maximum connection counts for BLE and classic BT.
/// Written to ROM config area at offset 116 (Letter Series only).
/// Reference: `HAL_LCPU_CONFIG_BT_ACT_CFG` in SDK.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ActConfig {
    /// Bitmask indicating which fields are valid (bits 0-4).
    pub bit_valid: u32,
    /// Maximum classic BT ACL connections.
    pub bt_max_acl: u8,
    /// Maximum classic BT SCO connections.
    pub bt_max_sco: u8,
    /// Maximum BLE activities.
    pub ble_max_act: u8,
    /// Maximum BLE resolving address list entries.
    pub ble_max_ral: u8,
    /// Maximum BLE ISO streams.
    pub ble_max_iso: u8,
    /// BLE RX descriptor count.
    pub ble_rx_desc: u8,
    /// BT RX descriptor count.
    pub bt_rx_desc: u8,
    /// BT device name max length.
    pub bt_name_len: u8,
}

impl ActConfig {
    /// ROM config offset for this structure.
    pub(crate) const ROM_OFFSET: usize = 116;

    /// Default configuration matching SDK peripheral example.
    pub const DEFAULT: Self = Self {
        bit_valid: (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4),
        bt_max_acl: 7,
        bt_max_sco: 0,
        ble_max_act: 6,
        ble_max_ral: 3,
        ble_max_iso: 0,
        ble_rx_desc: 0,
        bt_rx_desc: 0,
        bt_name_len: 0,
    };

    /// Set maximum BLE activities.
    pub const fn ble_max_act(mut self, val: u8) -> Self {
        self.ble_max_act = val;
        self
    }

    /// Set maximum BLE resolving address list entries.
    pub const fn ble_max_ral(mut self, val: u8) -> Self {
        self.ble_max_ral = val;
        self
    }

    /// Set maximum BLE ISO streams.
    pub const fn ble_max_iso(mut self, val: u8) -> Self {
        self.ble_max_iso = val;
        self
    }

    /// Set maximum classic BT ACL connections.
    pub const fn bt_max_acl(mut self, val: u8) -> Self {
        self.bt_max_acl = val;
        self
    }

    /// Set maximum classic BT SCO connections.
    pub const fn bt_max_sco(mut self, val: u8) -> Self {
        self.bt_max_sco = val;
        self
    }
}

impl Default for ActConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}
