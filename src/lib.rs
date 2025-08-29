#![no_std]
use core::fmt::Debug;
use embedded_hal::i2c::{I2c, SevenBitAddress};

use st_mems_bus::*;

pub mod prelude;
pub mod register;

use prelude::*;

/// Driver for the STTS22H sensor.
///
/// The struct takes a bus to write to the registers.
/// The bus is generalized over the BusOperation trait, however actually only the
/// I2C protocol is supported; this also allows the user to implement sharing
/// techniques to share the underlying bus.
pub struct Stts22h<B> {
    pub bus: B,
    /// Handles the split of reads/writes.
    ///
    /// If set to 1 singular r/w are used, if set more than 1 multiple r/w are
    /// used.
    chunk_size: usize,
}

/// Error that driver could generates
#[derive(Debug)]
pub enum Error<B> {
    /// Incapsulates errors coming from the bus.
    Bus(B),
    /// Unexpected value read from a register
    UnexpectedValue,
}
impl<B> Stts22h<B>
where
    B: BusOperation,
{
    /// Constructor method based on general BusOperation implementation.
    ///
    /// # Arguments
    ///
    /// * `bus`: An object that implements `BusOperation` trait
    pub fn from_bus(bus: B) -> Self {
        Self { bus, chunk_size: 1 }
    }
}
impl<P> Stts22h<i2c::I2cBus<P>>
where
    P: I2c,
{
    /// Constructor method for using the I2C bus.
    ///
    /// # Arguments
    ///
    /// * `i2c`: The I2C peripheral.
    /// * `address`: The I2C address of the STTS22H sensor.
    pub fn new_i2c(i2c: P, address: I2CAddress) -> Self {
        let bus = i2c::I2cBus::new(i2c, address as SevenBitAddress);
        Self { bus, chunk_size: 1 }
    }
}

impl<B: BusOperation> Stts22h<B> {
    #[inline]
    fn write_to_register(&mut self, mut reg: u8, buf: &[u8]) -> Result<(), Error<B::Error>> {
        let mut tmp: [u8; MAX_CHUNK_SIZE] = [0; MAX_CHUNK_SIZE];
        for chunk in buf.chunks(self.chunk_size) {
            tmp[0] = reg;
            tmp[1..1 + chunk.len()].copy_from_slice(chunk);
            self.bus
                .write_bytes(&tmp[..1 + chunk.len()])
                .map_err(Error::Bus)?;

            reg = reg.wrapping_add(chunk.len() as u8);
        }
        Ok(())
    }

    #[inline]
    pub fn read_from_register(
        &mut self,
        mut reg: u8,
        buf: &mut [u8],
    ) -> Result<(), Error<B::Error>> {
        for chunk in buf.chunks_mut(self.chunk_size) {
            self.bus
                .read_from_register(reg, chunk)
                .map_err(Error::Bus)?;
            reg = reg.wrapping_add(chunk.len() as u8);
        }

        Ok(())
    }

    /// Set temperature sensor data rate.
    ///
    /// Set the CTRL register according to the datasheet.
    ///
    /// # Arguments
    ///
    /// * `val`: OdrTemp enum struct to select different ODRs.
    pub fn temp_data_rate_set(&mut self, val: OdrTemp) -> Result<(), Error<B::Error>> {
        let mut ctrl = Ctrl::read(self)?;

        ctrl.set_one_shot((val as u8) & 0x01);
        ctrl.set_freerun(((val as u8) & 0x02) >> 1);
        ctrl.set_low_odr_start(((val as u8) & 0x04) >> 2);
        ctrl.set_avg(((val as u8) & 0x30) >> 4);

        ctrl.write(self)
    }

    /// Get temperature sensor data rate selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `OdrTemp`: Temperature data rate.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn temp_data_rate_get(&mut self) -> Result<OdrTemp, Error<B::Error>> {
        let ctrl = Ctrl::read(self)?;

        let odr_value = ctrl.one_shot()
            | (ctrl.freerun() << 1)
            | (ctrl.low_odr_start() << 2)
            | (ctrl.avg() << 4);

        let odr_temp = match odr_value {
            0x00 => OdrTemp::PowerDown,
            0x01 => OdrTemp::OneShot,
            0x04 => OdrTemp::Odr1Hz,
            0x02 => OdrTemp::Odr25Hz,
            0x12 => OdrTemp::Odr50Hz,
            0x22 => OdrTemp::Odr100Hz,
            0x32 => OdrTemp::Odr200Hz,
            _ => OdrTemp::PowerDown,
        };

        Ok(odr_temp)
    }

    /// Set Block data update.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of bdu in CTRL register.
    pub fn block_data_update_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl = Ctrl::read(self)?;
        ctrl.set_bdu(val);
        ctrl.write(self)
    }

    /// Get Block data update settings.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Get the values of bdu in register CTRL.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn block_data_update_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ctrl::read(self).map(|reg| reg.bdu())
    }

    /// Get a flag about new data available from temperature sensor.
    ///
    /// If flag equals to 1: new data is available to be read,
    /// then use temperature_raw_get.
    pub fn temp_flag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        let status = Status::read(self)?;
        let val = if status.busy() == 1 { 0 } else { 1 };

        Ok(val)
    }

    /// Read raw temperature data from registers.
    ///
    /// The L and H registers together express a 16-bit word in two's complement.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `i16`: Temperature raw value.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn temperature_raw_get(&mut self) -> Result<i16, Error<B::Error>> {
        TempOut::read(self).map(|reg| reg.temperature())
    }

    /// Get WHO_AM_I register. Used to identify the sensor.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: WHO_AM_I content
    ///     * `Err`: Returns an error if the operation fails.
    pub fn dev_id_get(&mut self) -> Result<u8, Error<B::Error>> {
        WhoAmI::read(self).map(|reg| reg.who_am_i())
    }

    /// Get device status register.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `DevStatus`: Contains the status of the device.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn dev_status_get(&mut self) -> Result<DevStatus, Error<B::Error>> {
        let status = Status::read(self)?;
        let dev_status = DevStatus {
            busy: status.busy(),
        };

        Ok(dev_status)
    }

    /// Set SMBus mode.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of `time_out_dis` in register .
    pub fn smbus_interface_set(&mut self, val: SmbusMd) -> Result<(), Error<B::Error>> {
        let mut ctrl = Ctrl::read(self)?;
        ctrl.set_time_out_dis(val as u8);
        ctrl.write(self)
    }

    /// Get SMBus mode setting.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `SmbusMd`: Describes the SMBus timeout setting.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn smbus_interface_get(&mut self) -> Result<SmbusMd, Error<B::Error>> {
        let ctrl = Ctrl::read(self)?;

        let val = match ctrl.time_out_dis() {
            0 => SmbusMd::TimeoutEnable,
            1 => SmbusMd::TimeoutDisable,
            _ => SmbusMd::TimeoutEnable,
        };

        Ok(val)
    }

    /// Enable/Disable AutoIncrement: Register address automatically incremented
    /// during a multiple byte access with a serial interface.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of `if_add_inc` in register .
    pub fn auto_increment_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut ctrl = Ctrl::read(self)?;
        ctrl.set_if_add_inc(val);
        if val == 1 {
            self.chunk_size = CHUNK_SIZE;
        } else {
            self.chunk_size = 1;
        }

        ctrl.write(self)
    }

    /// Get status(enable/disable) of AutoIncrement: If enabled register address is
    /// automatically incremented during a multiple byte access with a serial interface.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Gets the value of if_add_inc in reg CTRL.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn auto_increment_get(&mut self) -> Result<u8, Error<B::Error>> {
        Ctrl::read(self).map(|reg| reg.if_add_inc())
    }

    /// Sets the high temperature interrupt threshold register (TEMP_H_LIMIT).
    ///
    /// When the temperature exceeds this threshold, a high temperature interrupt is triggered.
    ///
    /// - The temperature threshold corresponding to `val` is:
    ///   `threshold (°C) = (val - 63) * 0.64`
    /// - Setting `val` to 0 disables the high temperature interrupt.
    ///
    /// # Arguments
    /// * `val` - The raw register value to write to TEMP_H_LIMIT.
    pub fn temp_trshld_high_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        TempHLimit::new().with_thl(val).write(self)
    }

    /// Gets the current high temperature interrupt threshold register value (TEMP_H_LIMIT).
    ///
    /// Returns the raw register value `val`, which corresponds to a temperature threshold:
    ///     `threshold (°C) = (val - 63) * 0.64`
    /// If `val` is 0, the high temperature interrupt is disabled.
    ///
    /// # Returns
    /// * `val` - The raw register value from TEMP_H_LIMIT.
    pub fn temp_trshld_high_get(&mut self) -> Result<u8, Error<B::Error>> {
        TempHLimit::read(self).map(|reg| reg.thl())
    }

    /// Sets the low temperature interrupt threshold register (TEMP_L_LIMIT).
    ///
    /// When the temperature falls below this threshold, a low temperature interrupt is triggered.
    ///
    /// - The temperature threshold corresponding to `val` is:
    ///   `threshold (°C) = (val - 63) * 0.64`
    /// - Setting `val` to 0 disables the low temperature interrupt.
    ///
    /// # Arguments
    /// * `val` - The raw register value to write to TEMP_L_LIMIT.
    pub fn temp_trshld_low_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        TempLLimit::new().with_tll(val).write(self)
    }

    /// Gets the current low temperature interrupt threshold register value (TEMP_L_LIMIT).
    ///
    /// Returns the raw register value `val`, which corresponds to a temperature threshold:
    ///     `threshold (°C) = (val - 63) * 0.64`
    /// If `val` is 0, the low temperature interrupt is disabled.
    ///
    /// # Returns
    /// * `val` - The raw register value from TEMP_L_LIMIT.
    pub fn temp_trshld_low_get(&mut self) -> Result<u8, Error<B::Error>> {
        TempLLimit::read(self).map(|reg| reg.tll())
    }

    /// Reads the temperature threshold interrupt source status.
    ///
    /// This function checks the status register for temperature threshold events:
    /// - `under_thl`: Indicates if the temperature has fallen below the low threshold.
    ///     - `0`: Low limit not exceeded (or interrupt disabled).
    ///     - `1`: Low limit exceeded. This bit is automatically reset to 0 upon reading
    ///       the status register.
    /// - `over_thh`: Indicates if the temperature has exceeded the high threshold.
    ///     - `0`: High limit not exceeded (or interrupt disabled).
    ///     - `1`: High limit exceeded. This bit is automatically reset to 0 upon reading
    ///       the status register.
    ///
    /// # Returns
    ///
    /// * `Ok(TempTrlhdSrc)`: Contains the current threshold status bits.
    /// * `Err`: Returns an error if the operation fails.
    pub fn temp_trshld_src_get(&mut self) -> Result<TempTrlhdSrc, Error<B::Error>> {
        let status = Status::read(self)?;

        let under_thl = status.under_thl();
        let over_thh = status.over_thh();

        Ok(TempTrlhdSrc {
            under_thl,
            over_thh,
        })
    }
}

/// Converts a raw temperature value from LSB (least significant bits) to degrees Celsius.
///
/// The conversion formula is: `celsius = lsb / 100.0`
///
/// # Arguments
/// * `lsb` - The raw temperature value as read from the sensor register.
///
/// # Returns
/// * Temperature in degrees Celsius as an `f32`.
pub fn from_lsb_to_celsius(lsb: i16) -> f32 {
    lsb as f32 / 100.0
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum I2CAddress {
    I2cAddH = 0x38,
    I2cAdd15kohm = 0x3C,
    I2cAdd56kohm = 0x3E,
    I2cAddL = 0x3F,
}

pub const ID: u8 = 0xA0;

/// CHUNK_SIZE represent how to split reads and writes.
///
/// CHUNK_SIZE is used when if_add_inc bit is enabled, otherwise 1 (singular
/// writes and reads) are used.
///
/// # Safety
/// CHUNK_SIZE should always be less than or equal to (MAX_CHUNK_SIZE - 1).
const CHUNK_SIZE: usize = 255;
const MAX_CHUNK_SIZE: usize = 256;
