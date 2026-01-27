#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use sifli_hal::timer::{SimplePwm, PwmPin, Channel, CountingMode};
use sifli_hal::time::Hertz;
use sifli_hal::usart::{Config, Uart};
use embedded_io::Write;
use {defmt_rtt as _, panic_probe as _};

// WS2812 LED 驱动 - HSV 彩虹色环效果
// 
// 硬件连接：
//   PA32 (GPTIM2_CH1) -> WS2812 DIN
//
// 效果：
//   🌈 彩虹色环循环（0-360度）
//   ⚡ 50ms 刷新一次 (20Hz)
//
// 技术特性：
//   ✅ 使用 DMA 自动更新 CCR，时序完美
//   ✅ CPU 占用极低
//   ✅ HSV 颜色空间平滑过渡

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = sifli_hal::init(Default::default());
    
    // UART 调试
    let mut config = Config::default();
    config.baudrate = 1000000;
    let mut usart = Uart::new_blocking(p.USART1, p.PA18, p.PA19, config).unwrap();
    
    // 启用 RGB LED 电源
    sifli_hal::pac::PMUC.peri_ldo().modify(|w| {
        w.set_en_vdd33_ldo3(true);
        w.set_vdd33_ldo3_pd(false);
    });
    
    let _ = writeln!(usart, "\r\n=== WS2812 with DMA (Perfect Solution!) ===");
    
    // 🎯 创建 PWM with DMA（关键！）
    let mut pwm = SimplePwm::new_with_dma(
        p.GPTIM2,
        Some(PwmPin::new(p.PA32)),  // 自动配置 GPIO
        None, None, None,
        p.DMAC1_CH1,                // DMA channel for Update events
        Hertz::khz(800),
        CountingMode::EdgeAlignedUp,
    );
    
    let max = pwm.max_duty_cycle();
    let bit0 = max * 32 / 100;
    let bit1 = max * 64 / 100;
    
    let _ = writeln!(usart, "PWM configured with DMA:");
    let _ = writeln!(usart, "  Frequency: 800 kHz");
    let _ = writeln!(usart, "  Max duty: {}", max);
    let _ = writeln!(usart, "  Bit0: {} (32%), Bit1: {} (64%)", bit0, bit1);
    let _ = writeln!(usart, "  DMA: DMAC1_CH1 for GPTIM2_UPDATE\n");
    
    let _ = writeln!(usart, "🌈 Starting HSV Rainbow Cycle...");
    let _ = writeln!(usart, "   Hue: 0-360° @ 50ms/step (20Hz refresh)\n");
    
    let mut ws2812_data = [0u16; 25];
    let mut hue: u16 = 0;  // 0-359 degrees
    
    loop {
        // HSV 转 RGB (H: 0-359, S: 100%, V: 50% for eye safety)
        let (r, g, b) = hsv_to_rgb(hue, 100, 50);
        
        // 编码颜色（GRB 格式，MSB first）
        for i in 0..8 {
            ws2812_data[i] = if (g & (0x80 >> i)) != 0 { bit1 } else { bit0 };
            ws2812_data[8 + i] = if (r & (0x80 >> i)) != 0 { bit1 } else { bit0 };
            ws2812_data[16 + i] = if (b & (0x80 >> i)) != 0 { bit1 } else { bit0 };
        }
        ws2812_data[24] = 0;  // Reset
        
        // 🎯 使用 DMA 发送（可以重复调用）
        pwm.waveform_up_blocking(Channel::Ch1, &ws2812_data);
        
        // Reset 时序
        Timer::after_micros(80).await;
        
        // 每 30 度打印一次（减少输出）
        if hue % 30 == 0 {
            let _ = writeln!(usart, "Hue: {:3}° -> RGB({:3}, {:3}, {:3})", 
                hue, r, g, b);
        }
        
        // 增加 hue，循环 0-359
        hue = (hue + 1) % 360;
        
        // 50ms 刷新一次（20Hz，完整循环需要 360 × 50ms = 18秒）
        Timer::after_millis(10).await;
    }
}

/// HSV 转 RGB 颜色空间转换
/// 
/// # Arguments
/// * `h` - Hue (色相): 0-359 degrees
/// * `s` - Saturation (饱和度): 0-100%
/// * `v` - Value (明度): 0-100%
/// 
/// # Returns
/// (r, g, b) where each component is 0-255
fn hsv_to_rgb(h: u16, s: u8, v: u8) -> (u8, u8, u8) {
    let s = s as u16;
    let v = v as u16;
    
    let c = (v * s) / 100;  // Chroma
    let h_prime = (h % 360) / 60;  // 0-5
    let x = (c * (60 - ((h % 120) as i16 - 60).abs() as u16)) / 60;
    
    let (r1, g1, b1) = match h_prime {
        0 => (c, x, 0),
        1 => (x, c, 0),
        2 => (0, c, x),
        3 => (0, x, c),
        4 => (x, 0, c),
        _ => (c, 0, x),  // 5
    };
    
    let m = v - c;
    
    let r = ((r1 + m) * 255 / 100) as u8;
    let g = ((g1 + m) * 255 / 100) as u8;
    let b = ((b1 + m) * 255 / 100) as u8;
    
    (r, g, b)
}

