#![no_std]
#![no_main]
#![deny(unsafe_code)]

use core::fmt::Write;

use cortex_m_rt::entry;

use panic_halt as _;

use stm32f4xx_hal::{
    i2c::{DutyCycle, I2c, Mode},
    pac,
    prelude::*,
    serial::Config,
};
use stts22h_rs::*;
use stts22h_rs::prelude::*;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();

    let mut delay = cp.SYST.delay(&clocks);

    let gpiob = dp.GPIOB.split();
    let gpioa = dp.GPIOA.split();

    let scl = gpiob.pb8;
    let sda = gpiob.pb9;

    let i2c: I2c<pac::I2C1> = I2c::new(
        dp.I2C1,
        (scl, sda),
        Mode::Fast {
            frequency: 400.kHz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        &clocks,
    );

    let tx_pin = gpioa.pa2.into_alternate();

    let mut tx = dp
        .USART2
        .tx(
            tx_pin,
            Config::default()
                .baudrate(115200.bps())
                .wordlength_8()
                .parity_none(),
            &clocks,
        )
        .unwrap();

    delay.delay_ms(5);

    let mut sensor = Stts22h::new_i2c(i2c, I2CAddress::I2cAddH);

    let whoami = sensor.dev_id_get().unwrap();
    if whoami != ID {
        loop {}
    }

    sensor.auto_increment_set(1).unwrap();
    sensor.temp_data_rate_set(OdrTemp::Odr1Hz).unwrap();

    // Read samples in polling mode (no interrupt)
    loop {
        let flag = sensor.temp_flag_data_ready_get().unwrap();

        if flag == 1 {
            let data = sensor.temperature_raw_get().unwrap();
            let temp_c = stts22h_rs::from_lsb_to_celsius(data);

            writeln!(tx, "Temperature [degC]: {:.2}", temp_c).unwrap();
        }

        delay.delay_ms(1000);
    }
}
