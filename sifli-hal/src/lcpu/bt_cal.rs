//! LCPU 侧蓝牙 RF 校准辅助。
//!
//! 这里实现的是 SDK `bt_rf_cal` 的最小必要子集，用于：
//! - 通过 LPSYS_RCC 复位 Bluetooth RF 模块
//! - 将 BT 发射功率参数写入 LCPU ROM 配置区
//!
//! 完整的模拟射频自校准算法（VCO / TXDC / IQ 等，见
//! `SiFli-SDK/drivers/cmsis/sf32lb52x/bt_rf_fulcal.c`）暂未移植。

use super::ram;
use crate::lpsys_rcc::{get_rfc_reset, set_rfc_reset};
use crate::syscfg::Idr;

/// 复位 Bluetooth RF 模块。
///
/// 对应 SDK 中的 `HAL_RCC_ResetBluetoothRF`。
fn reset_bluetooth_rf() {
    // 置位 RFC 复位位
    set_rfc_reset(true);
    // 等待位生效
    while !get_rfc_reset() {}
    // 清除 RFC 复位位
    set_rfc_reset(false);
}

/// 默认的 BT 射频功率参数。
///
/// 对应 SDK 中 `bt_rf_get_max_tx_pwr` / `bt_rf_get_min_tx_pwr` /
/// `bt_rf_get_init_tx_pwr` 和 `bt_is_in_BQB_mode` 的弱默认实现。
fn default_tx_power_params() -> (i8, i8, i8, u8) {
    let max_pwr: i8 = 10;
    let init_pwr: i8 = 0;
    let min_pwr: i8 = 0;
    let is_bqb: u8 = 0;
    (max_pwr, min_pwr, init_pwr, is_bqb)
}

/// 将功率参数编码为 32bit 打包格式。
///
/// 等价于 SDK 中的 `RF_PWR_PARA` 宏：
/// `(is_bqb << 24) | (init << 16) | (min << 8) | (int8_t)(max)`.
fn encode_tx_power(max: i8, min: i8, init: i8, is_bqb: u8) -> u32 {
    let max_u = max as u8 as u32;
    let min_u = min as u8 as u32;
    let init_u = init as u8 as u32;
    let is_bqb_u = is_bqb as u32;

    (is_bqb_u << 24) | (init_u << 16) | (min_u << 8) | max_u
}

/// 执行基础的蓝牙射频“校准”步骤。
///
/// 当前实现：
/// - 复位 Bluetooth RF 模块；
/// - 根据默认参数计算 BT TX 功率编码；
/// - 写入 LCPU ROM 配置区的 `bt_txpwr` 字段。
///
/// 注意：这不会执行 SDK 中完整的模拟射频自校准流程，只保证
/// LCPU 启动所需的配置字段已正确设置。
pub fn bt_rf_cal(idr: &Idr) {
    // 1. 复位 Bluetooth RF
    reset_bluetooth_rf();

    // 2. 计算 TX 功率编码
    let (max_pwr, min_pwr, init_pwr, is_bqb) = default_tx_power_params();
    let tx_pwr = encode_tx_power(max_pwr, min_pwr, init_pwr, is_bqb);

    // 3. 写入 LCPU ROM 配置区（BT_TXPWR）
    ram::set_bt_tx_power(idr, tx_pwr);
}
