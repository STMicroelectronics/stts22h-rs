use defmt::info;
use maybe_async::maybe_async;
use crate::*;

#[maybe_async]
pub async fn run<B, D, L>(bus: B, mut tx: L, mut delay: D, _irq: ()) -> !
where
    B: BusOperation,
    D: DelayNs + Clone,
    L: embedded_io::Write
{
    use stts22h::prelude::*;
    use stts22h::*;

    info!("Configuring the sensor");
    let mut sensor = Stts22h::from_bus(bus);

    // boot time
    delay.delay_ms(5).await;

    // Check device ID
    let id = sensor.dev_id_get().await.unwrap();
    info!("Device ID: {:x}", id);
    if id != ID {
        info!("Unexpected device ID: {:x}", id);
        writeln!(tx, "Unexpected device ID: {:x}", id).unwrap();
        loop {}
    }

    sensor.temp_data_rate_set(OdrTemp::Odr1Hz).await.unwrap();

    // Read samples in polling mode (no interrupt)
    loop {
        let flag = sensor.temp_flag_data_ready_get().await.unwrap();

        if flag == 1 {
            let data = sensor.temperature_raw_get().await.unwrap();
            let temp_c = from_lsb_to_celsius(data);

            writeln!(tx, "Temperature [degC]: {:.2}", temp_c,).unwrap();
        }

        delay.delay_ms(1000).await;
    }
}
