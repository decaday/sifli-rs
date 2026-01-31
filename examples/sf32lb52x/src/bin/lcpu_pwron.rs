#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_probe as _;

use sifli_hal::gpio;
use sifli_hal::lcpu::{Lcpu, LcpuConfig};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = sifli_hal::init(Default::default());

    let mut led = gpio::Output::new(p.PA26, gpio::Level::Low);

    info!("Starting LCPU power-on example");

    let lcpu = Lcpu::new(p.LPSYS_AON);

    let cfg = LcpuConfig::default();

    match lcpu.power_on(&cfg) {
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
    // LCPU会发送一个HCI事件，如果没有被消费，500ms超时后会自己消费
    // LCPU此时不再被阻塞，power_on()最后一步的允许lp休眠生效

    loop {
        led.toggle();
        Timer::after_secs(1).await;
    }
}
