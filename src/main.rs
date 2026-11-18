#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::{
    i2c,
    pac::{self},
    prelude::*,
};

struct Calib {
    t1: u16,
    t2: i16,
    t3: i16,
    p1: u16,
    p2: i16,
    p3: i16,
    p4: i16,
    p5: i16,
    p6: i16,
    p7: i16,
    p8: i16,
    p9: i16,
}

impl Calib {
    fn from_bytes(b: &[u8; 24]) -> Self {
        let u = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
        let s = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
        Calib {
            t1: u(0),
            t2: s(2),
            t3: s(4),
            p1: u(6),
            p2: s(8),
            p3: s(10),
            p4: s(12),
            p5: s(14),
            p6: s(16),
            p7: s(18),
            p8: s(20),
            p9: s(22),
        }
    }

    fn compensate_temp(&self, adc_t: i32) -> (i32, i32) {
        let t1 = self.t1 as i32;
        let var1 = (((adc_t >> 3) - (t1 << 1)) * self.t2 as i32) >> 11;
        let d = (adc_t >> 4) - t1;
        let var2 = (((d * d) >> 12) * self.t3 as i32) >> 14;
        let t_fine = var1 + var2;
        ((t_fine * 5 + 128) >> 8, t_fine)
    }

    fn compensate_press(&self, adc_p: i32, t_fine: i32) -> u32 {
        let mut var1 = t_fine as i64 - 128000;
        let mut var2 = var1 * var1 * self.p6 as i64;
        var2 += (var1 * self.p5 as i64) << 17;
        var2 += (self.p4 as i64) << 35;
        var1 = ((var1 * var1 * self.p3 as i64) >> 8) + ((var1 * self.p2 as i64) << 12);
        var1 = (((1i64 << 47) + var1) * self.p1 as i64) >> 33;
        if var1 == 0 {
            return 0; // avoid division by zero
        }
        let mut p = 1048576 - adc_p as i64;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (self.p9 as i64 * (p >> 13) * (p >> 13)) >> 25;
        var2 = (self.p8 as i64 * p) >> 19;
        p = ((p + var1 + var2) >> 8) + ((self.p7 as i64) << 4);
        (p / 256) as u32 // Q24.8 -> Pa
    }
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Start initializing peripherals");
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

    // LED2
    //let gpioa = dp.GPIOA.split();
    //let mut led = gpioa.pa5.into_push_pull_output();
    let mut delay = cp.SYST.delay(&clocks);

    // SPI: GPIOB PB8 -> I2C1_SCL, PB9 -> I2C1_SDA
    let gpiob = dp.GPIOB.split();
    let scl = gpiob.pb8.into_alternate_open_drain();
    let sda = gpiob.pb9.into_alternate_open_drain();

    let mut i2c_config = i2c::I2c::new(
        dp.I2C1,
        (scl, sda),
        i2c::Mode::Standard {
            frequency: 100.kHz(),
        },
        &clocks,
    );
    rprintln!("Initialized pins and I2C");
    let sensor_addr = 0x76;

    // Read calibration data from registers
    let mut calib_raw = [0u8; 24];
    loop {
        match i2c_config.write_read(sensor_addr, &[0x88], &mut calib_raw) {
            Ok(_) => break,
            Err(e) => {
                rprintln!("Calibration read error {:?}", e);
                delay.delay_ms(500_u16);
            }
        }
    }

    let cal = Calib::from_bytes(&calib_raw);
    // Configure temp x2, pressure x16, normal mode
    // 0xF4: register for controlling measurement
    // osrs_t(010) osrs_p(101) mode(11) = 0x57
    while let Err(e) = i2c_config.write(sensor_addr, &[0xF4, 0x57]) {
        rprintln!("Config error: {:?}", e);
        delay.delay_ms(500_u16);
    }
    delay.delay_ms(100_u16);
    rprintln!("T1={} T2={} T3={}", cal.t1, cal.t2, cal.t3);

    loop {
        // burst read of pressure and temperature registers (6 bytes from 0xF7)
        let mut raw = [0u8; 6];
        match i2c_config.write_read(sensor_addr, &[0xF7], &mut raw) {
            Ok(_) => {
                let adc_p =
                    ((raw[0] as i32) << 12) | ((raw[1] as i32) << 4) | ((raw[2] as i32) >> 4);
                let adc_t =
                    ((raw[3] as i32) << 12) | ((raw[4] as i32) << 4) | ((raw[5] as i32) >> 4);

                let (t_centi, t_fine) = cal.compensate_temp(adc_t);
                let p_pa = cal.compensate_press(adc_p, t_fine);

                let sign = if t_centi < 0 { "-" } else { "" };
                let t = t_centi.abs();
                rprintln!(
                    "T = {}{}.{:02} C, P = {}.{:02} hPa",
                    sign,
                    t / 100,
                    t % 100,
                    p_pa / 100,
                    p_pa % 100
                );
            }
            Err(e) => {
                rprintln!("Read error: {:?}", e);
            }
        }
        delay.delay_ms(1000_u16);
    }
}
