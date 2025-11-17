#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_probe as _;

use sifli_hal::gpio;
use sifli_hal::lcpu::{Lcpu, LcpuConfig, PatchData};

#[path = "../patch_data.rs"]
mod patch_data;

#[path = "../patch_data_rev_b.rs"]
mod patch_data_rev_b;

#[path = "../lcpu_image_52x.rs"]
mod lcpu_image_52x;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = sifli_hal::init(Default::default());

    let mut led = gpio::Output::new(p.PA26, gpio::Level::Low);

    info!("Starting LCPU power-on example");

    let lcpu = Lcpu::new().into_async();

    let cfg = LcpuConfig::new()
        .with_firmware(&lcpu_image_52x::G_LCPU_BIN_U32)
        .with_patch_a3(PatchData {
            list: &patch_data::G_LCPU_PATCH_LIST_U32,
            bin: &patch_data::G_LCPU_PATCH_BIN_U32,
        })
        .with_patch_letter(PatchData {
            list: &patch_data_rev_b::G_LCPU_PATCH_LIST_U32,
            bin: &patch_data_rev_b::G_LCPU_PATCH_BIN_U32,
        })
        .disable_rf_cal();

    match lcpu.power_on(&cfg).await {
        Ok(()) => info!("LCPU power-on succeeded"),
        Err(e) => {
            warn!("LCPU power-on failed: {:?}", e);
            // Blink fast to indicate error
            loop {
                led.toggle();
                Timer::after_millis(100).await;
            }
        }
    }

    info!("LCPU is running; blinking LED");

    loop {
        led.toggle();
        Timer::after_secs(1).await;
    }
}
