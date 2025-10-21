use stts22h_rs::{Stts22h, I2CAddress, ID, register::main::*};
use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};
use rstest::rstest;

const DEVICE_ADDR: u8 = I2CAddress::I2cAddH as u8;

/// Test the WHO_AM_I register to verify device identity.
/// This test mocks an I2C read of the WHO_AM_I register and checks the returned ID.
#[test]
fn test_who_am_i() {
    let expectations = vec![
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x01], vec![0xA0]),
    ];

    let mut i2c = I2cMock::new(&expectations);

    let mut sensor = Stts22h::new_i2c(i2c.clone(), I2CAddress::I2cAddH);

    let who_am_i = sensor.dev_id_get().unwrap();
    assert_eq!(who_am_i, ID);

    i2c.done()
}

/// Test setting and getting temperature threshold high and low values.
/// This test writes and reads back the threshold values to ensure correctness.
#[rstest]
#[case(0xA2, 0x7F)]
#[case(0x3C, 0xD1)]
#[case(0x55, 0xE0)]
#[case(0x09, 0xB4)]
#[case(0xFE, 0x12)]
#[case(0x67, 0x8A)]
fn temp_trshld(#[case] raw_high: u8, #[case] raw_low: u8) {

    let expectations = vec![
        I2cTransaction::write(DEVICE_ADDR, vec![0x02, raw_high]),
        I2cTransaction::write(DEVICE_ADDR, vec![0x03, raw_low]),
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x02], vec![raw_high]),
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x03], vec![raw_low]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = Stts22h::new_i2c(i2c.clone(), I2CAddress::I2cAddH);

    sensor.temp_trshld_high_set(raw_high).unwrap();
    sensor.temp_trshld_low_set(raw_low).unwrap();
    let raw_high_get = sensor.temp_trshld_high_get().unwrap();
    let raw_low_get = sensor.temp_trshld_low_get().unwrap();

    assert_eq!(raw_high, raw_high_get);
    assert_eq!(raw_low, raw_low_get);

    i2c.done();
}

/// Test reading the temperature threshold source register.
/// This test checks the status of over/under temperature threshold flags.
#[test]
fn trshld_src_get() {

    let expectations = vec![
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x05], vec![0x00]),
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x05], vec![0x02]),
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x05], vec![0x04]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = Stts22h::new_i2c(i2c.clone(), I2CAddress::I2cAddH);

    // Assume no temperature threshold is reached
    let temp_trshld = sensor.temp_trshld_src_get().unwrap();

    assert_eq!(temp_trshld.under_thl, 0);
    assert_eq!(temp_trshld.over_thh, 0);

    // Assume over temperature threshold is reached
    let temp_trshld = sensor.temp_trshld_src_get().unwrap();

    assert_eq!(temp_trshld.under_thl, 0);
    assert_eq!(temp_trshld.over_thh, 1);

    // Assume under temperature threshold is reached
    let temp_trshld = sensor.temp_trshld_src_get().unwrap();

    assert_eq!(temp_trshld.under_thl, 1);
    assert_eq!(temp_trshld.over_thh, 0);

    i2c.done();
}

/// Test setting and getting the temperature data rate.
/// This test sets the ODR (Output Data Rate) and verifies it is correctly set and read back.
#[rstest]
#[case(0x00, OdrTemp::PowerDown)]
#[case(0x01, OdrTemp::OneShot)]
#[case(0x80, OdrTemp::Odr1Hz)]
#[case(0x04, OdrTemp::Odr25Hz)]
#[case(0x14, OdrTemp::Odr50Hz)]
#[case(0x24, OdrTemp::Odr100Hz)]
#[case(0x34, OdrTemp::Odr200Hz)]
fn temp_data_rate_test(#[case] expected: u8, #[case] odr: OdrTemp) {
    let expectations = [
        // set part
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x04], vec![0x0]),
        I2cTransaction::write(DEVICE_ADDR, vec![0x04, expected]),
        // get part
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x04], vec![expected])
    ];
    let mut i2c = I2cMock::new(&expectations);

    let mut sensor = Stts22h::new_i2c(i2c.clone(), I2CAddress::I2cAddH);

    sensor.temp_data_rate_set(odr).unwrap();
    let odr_get = sensor.temp_data_rate_get().unwrap();

    assert_eq!(odr, odr_get);

    i2c.done();
}

/// Test enabling and reading the block data update (BDU) feature.
/// This ensures that BDU can be set and read back correctly.
#[test]
fn block_data_update() {
    let expectations = [
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x04], vec![0x0]),
        I2cTransaction::write(DEVICE_ADDR, vec![0x04, 0x40]),
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x04], vec![0x40])
    ];

    let mut i2c = I2cMock::new(&expectations);

    let mut sensor = Stts22h::new_i2c(i2c.clone(), I2CAddress::I2cAddH);

    sensor.block_data_update_set(1).unwrap();
    let bdu = sensor.block_data_update_get().unwrap();

    assert_eq!(1, bdu);

    i2c.done();
}

/// Test reading the data ready flag.
/// This checks if the temperature data is ready to be read.
#[test]
fn flag_data_ready() {
    let expectations = vec![
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x05], vec![0x00])
    ];

    let mut i2c = I2cMock::new(&expectations);

    let mut sensor = Stts22h::new_i2c(i2c.clone(), I2CAddress::I2cAddH);

    let data_ready = sensor.temp_flag_data_ready_get().unwrap();
    assert_eq!(data_ready, 1);

    i2c.done()
}

/// Test reading the raw temperature value, both with and without auto-increment enabled.
/// This ensures correct reading of 16-bit temperature data.
#[rstest]
#[case(0x0000)]
#[case(0x1010)]
#[case(0x1110)]
#[case(0x0010)]
fn temperature_raw(#[case] raw: u16) {

    let bytes = raw.to_le_bytes().to_vec();

    /* two separate reads because if_add_inc by default is not enabled */
    let expectations = vec![
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x06], vec![bytes[0]]),
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x07], vec![bytes[1]])
    ];

    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = Stts22h::new_i2c(i2c.clone(), I2CAddress::I2cAddH);

    let raw_get = sensor.temperature_raw_get().unwrap();
    assert_eq!(raw_get, raw as i16);

    i2c.done();

    /* enabling if_add_inc */
    let expectations = vec![
        // enable if_add_inc (auto_increment_set)
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x04], vec![0x00]),
        I2cTransaction::write(DEVICE_ADDR, vec![0x04, 0x08]),
        // read temperature raw
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x06], bytes),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = Stts22h::new_i2c(i2c.clone(), I2CAddress::I2cAddH);

    sensor.auto_increment_set(1).unwrap();

    let raw_get = sensor.temperature_raw_get().unwrap();
    assert_eq!(raw_get, raw as i16);

    i2c.done()
}

/// Test enabling and reading the auto-increment feature.
#[test]
fn auto_increment_set() {
    let expectations = vec![
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x04], vec![0x00]),
        I2cTransaction::write(DEVICE_ADDR, vec![0x04, 0x08]),
        I2cTransaction::write_read(DEVICE_ADDR, vec![0x04], vec![0x08]),
    ];

    let mut i2c = I2cMock::new(&expectations);
    let mut sensor = Stts22h::new_i2c(i2c.clone(), I2CAddress::I2cAddH);

    sensor.auto_increment_set(1).unwrap();
    let auto_inc = sensor.auto_increment_get().unwrap();

    assert_eq!(auto_inc, 1);

    i2c.done()
}
