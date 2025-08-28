#![no_std]
use bitfield_struct::bitfield;
use core::fmt::Debug;
use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::SpiDevice;

use st_mems_bus::*;

/// The COMPONENT generic driver struct.
pub struct Stts22h<B> {
    /// The bus driver.
    pub bus: B,
    chunk_size: usize
}

/// Driver errors.
#[derive(Debug)]
pub enum Error<B> {
    Bus(B),          // Error at the bus level
    WhoAmIError(u8), // Incorrect COMPONENT identifier
    UnexpectedValue, // Unexpected value read from a register
}
impl<B> Stts22h<B>
where
    B: BusOperation,
{
    /// Constructor method based on general BusOperation implementation.
    /// Could take another sensor as bus to use it as passthrough
    ///
    /// # Arguments
    ///
    /// * `bus`: An object that implements `BusOperation` trait
    ///
    /// # Returns
    ///
    /// * `Self`: A new `` instance
    pub fn new_bus(bus: B) -> Self {
        Self { bus, chunk_size: 1 }
    }
}
impl<P> Stts22h<Owned<i2c::I2cBus<P>>>
where
    P: I2c,
{
    /// Constructor method for using the I2C bus.
    ///
    /// # Arguments
    ///
    /// * `i2c`: The I2C peripheral.
    /// * `address`: The I2C address of the COMPONENT sensor.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `Self`: Returns an instance of ``.
    ///     * `Err`: Returns an error if the initialization fails.
    pub fn new_i2c(i2c: P, address: I2CAddress) -> Result<Self, Error<P::Error>> {
        // Initialize the I2C bus with the COMPONENT address
        let bus = Owned::new(i2c::I2cBus::new(i2c, address as SevenBitAddress));
        let instance = Self { bus, chunk_size: 1 };

        Ok(instance)
    }
}
impl<P> Stts22h<Owned<spi::SpiBus<P>>>
where
    P: SpiDevice,
{
    /// Constructor method for using the SPI bus.
    ///
    /// # Arguments
    ///
    /// * `spi`: The SPI peripheral.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `Self`: Returns an instance of ``.
    ///     * `Err`: Returns an error if the initialization fails.
    pub fn new_spi(spi: P) -> Result<Self, Error<P::Error>> {
        // Initialize the SPI bus
        let bus = Owned::new(spi::SpiBus::new(spi));
        let instance = Self { bus, chunk_size: 1 };

        Ok(instance)
    }
}

impl<B: BusOperation> Stts22h<B> {
    #[inline]
    fn write_to_register(&mut self, mut reg: u8, buf: &[u8]) -> Result<(), Error<B::Error>> {
        let mut tmp: [u8; MAX_CHUNK_SIZE] = [0; MAX_CHUNK_SIZE];
        for chunk in buf.chunks(self.chunk_size) {
            tmp[0] = reg;
            tmp[1..1 + chunk.len()].copy_from_slice(chunk);
            self.bus.write_bytes(&tmp[..1 + chunk.len()]).map_err(Error::Bus)?;

            reg = reg.wrapping_add(chunk.len() as u8);
        }
        Ok(())
    }

    #[inline]
    pub fn read_from_register(&mut self, mut reg: u8, buf: &mut [u8]) -> Result<(), Error<B::Error>> {
        for chunk in buf.chunks_mut(self.chunk_size) {
            self.bus.read_from_register(reg, chunk).map_err(Error::Bus)?;
            reg = reg.wrapping_add(chunk.len() as u8);
        }

        Ok(())
    }


    /// Temperature sensor data rate selection. (set)
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of "one_shot" in reg .
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn temp_data_rate_set(&mut self, val: OdrTemp) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl as u8, &mut arr)?;
        let mut ctrl = Ctrl(arr[0]);

        ctrl.set_one_shot((val as u8) & 0x01);
        ctrl.set_freerun(((val as u8) & 0x02) >> 1);
        ctrl.set_low_odr_start(((val as u8) & 0x04) >> 2);
        ctrl.set_avg(((val as u8) & 0x30) >> 4);

        self.write_to_register(Reg::Ctrl as u8, &[ctrl.0])?;

        Ok(())
    }
    /// Temperature sensor data rate selection.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `OdrTemp`: Temperature data rate.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn temp_data_rate_get(&mut self) -> Result<OdrTemp, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl as u8, &mut arr)?;
        let ctrl = Ctrl(arr[0]);

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
    /// Block data update.(set)
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of bdu in reg CTRL.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn block_data_update_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl as u8, &mut arr)?;
        let mut ctrl = Ctrl(arr[0]);
        ctrl.set_bdu(val);
        self.write_to_register(Reg::Ctrl as u8, &[ctrl.0])?;

        Ok(())
    }
    /// Block data update.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Get the values of bdu in register CTRL.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn block_data_update_get(&mut self) -> Result<u8, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl as u8, &mut arr)?;

        Ok(arr[0])
    }
    /// New data available from temperature sensor.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Returns an option of `uint8_t`.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn temp_flag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Status as u8, &mut arr)?;
        let status = Status(arr[0]);

        let val = if status.busy() == 1 { 0 } else { 1 };

        Ok(val)
    }
    /// Temperature data output register.
    ///
    /// The L and H registers together express a 16-bit word in two's complement.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `i16`: Temperature raw value.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn temperature_raw_get(&mut self) -> Result<i16, Error<B::Error>> {
        let mut buff: [u8; 2] = [0; 2];
        self.read_from_register(Reg::TempLOut as u8, &mut buff)?;
        Ok(i16::from_le_bytes(buff))
    }
    /// Device Who am I..(get)
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Buffer that stores the data read.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn dev_id_get(&mut self) -> Result<u8, Error<B::Error>> {
        let mut buff: [u8; 1] = [0];
        self.read_from_register(Reg::WhoAmI as u8, &mut buff)?;

        Ok(buff[0])
    }
    /// Device status register.(get)
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `DevStatus`: Contains the status of the device.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn dev_status_get(&mut self) -> Result<DevStatus, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Status as u8, &mut arr)?;
        let status = Status(arr[0]);
        let dev_status = DevStatus {
            busy: status.busy(),
        };

        Ok(dev_status)
    }
/// SMBus mode set.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of `time_out_dis` in register .
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn smbus_interface_set(&mut self, val: SmbusMd) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl as u8, &mut arr)?;
        let mut ctrl = Ctrl(arr[0]);
        ctrl.set_time_out_dis(val as u8);
        self.write_to_register(Reg::Ctrl as u8, &[ctrl.0])?;

        Ok(())
    }
    /// SMBus mode.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `SmbusMd`: Describes the SMBus timeout setting.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn smbus_interface_get(&mut self) -> Result<SmbusMd, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl as u8, &mut arr)?;
        let ctrl = Ctrl(arr[0]);

        let val = match ctrl.time_out_dis() {
            0 => SmbusMd::TimeoutEnable,
            1 => SmbusMd::TimeoutDisable,
            _ => SmbusMd::TimeoutEnable,
        };

        Ok(val)
    }
    /// Register address automatically incremented during a multiple
    /// byte access with a serial interface.
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of `if_add_inc` in register .
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`: Returns a result indicating success.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn auto_increment_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl as u8, &mut arr)?;
        let mut ctrl = Ctrl(arr[0]);
        ctrl.set_if_add_inc(val);
        if val == 1 {
            self.chunk_size = CHUNK_SIZE;
        } else {
            self.chunk_size = 1;
        }

        self.write_to_register(Reg::Ctrl as u8, &[ctrl.0])?;

        Ok(())
    }
    /// Register address is automatically incremented during a multiple
    /// byte access with a serial interface.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Gets the value of if_add_inc in reg CTRL.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn auto_increment_get(&mut self) -> Result<u8, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl as u8, &mut arr)?;

        Ok(arr[0])
    }
    /// Over temperature interrupt value. (degC / 0.64) + 63. (set)
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of thl in reg TEMP_H_LIMIT.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn temp_trshld_high_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::TempHLimit as u8, &mut arr)?;
        let mut temp_h_limit = TempHLimit(arr[0]);
        temp_h_limit.set_thl(val);
        self.write_to_register(Reg::TempHLimit as u8, &[temp_h_limit.0])?;

        Ok(())
    }
    /// Over temperature interrupt value. (degC / 0.64) + 63. (get)
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Get the values of `thl` in register `TEMP_H_LIMIT`.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn temp_trshld_high_get(&mut self) -> Result<u8, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::TempHLimit as u8, &mut arr)?;
        let val: u8 = TempHLimit(arr[0]).thl();

        Ok(val)
    }
    /// Under temperature interrupt value. ( degC / 0.64 ) + 63.(set)
    ///
    /// # Arguments
    ///
    /// * `val`: Change the values of tll in reg TEMP_L_LIMIT.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `()`
    ///     * `Err`: Returns an error if the operation fails.
    pub fn temp_trshld_low_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::TempLLimit as u8, &mut arr)?;
        let mut temp_l_limit = TempLLimit(arr[0]);
        temp_l_limit.set_tll(val);
        self.write_to_register(Reg::TempLLimit as u8, &[temp_l_limit.0])?;

        Ok(())
    }
    /// Under temperature interrupt value. ( degC / 0.64 ) + 63. (get)
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `u8`: Get the values of tll in reg TEMP_L_LIMIT.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn temp_trshld_low_get(&mut self) -> Result<u8, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::TempLLimit as u8, &mut arr)?;
        let val: u8 = TempLLimit(arr[0]).tll();

        Ok(val)
    }
    /// Temperature interrupt on threshold source.
    ///
    /// # Returns
    ///
    /// * `Result`
    ///     * `TempTrlhdSrc`: Contains threshold status.
    ///     * `Err`: Returns an error if the operation fails.
    pub fn temp_trshld_src_get(&mut self) -> Result<TempTrlhdSrc, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Status as u8, &mut arr)?;
        let status = Status(arr[0]);

        let under_thl = status.under_thl();
        let over_thh = status.over_thh();

        Ok(TempTrlhdSrc {
            under_thl,
            over_thh,
        })
    }
}

pub fn from_lsb_to_celsius(lsb: i16) -> f32 {
    lsb as f32 / 100.0
}

#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl {
    #[bits(1)]
    pub one_shot: u8,
    #[bits(1)]
    pub time_out_dis: u8,
    #[bits(1)]
    pub freerun: u8,
    #[bits(1)]
    pub if_add_inc: u8,
    #[bits(2)]
    pub avg: u8,
    #[bits(1)]
    pub bdu: u8,
    #[bits(1)]
    pub low_odr_start: u8,
}

#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Status {
    #[bits(1)]
    pub busy: u8,
    #[bits(1)]
    pub over_thh: u8,
    #[bits(1)]
    pub under_thl: u8,
    #[bits(5, access = RO)]
    pub not_used_01: u8,
}

#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct TempHLimit {
    #[bits(8)]
    pub thl: u8,
}

#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct TempLLimit {
    #[bits(8)]
    pub tll: u8,
}

pub struct DevStatus {
    pub busy: u8,
}

pub struct TempTrlhdSrc {
    pub under_thl: u8,
    pub over_thh: u8,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum OdrTemp {
    PowerDown = 0x00,
    OneShot = 0x01,
    Odr1Hz = 0x04,
    Odr25Hz = 0x02,
    Odr50Hz = 0x12,
    Odr100Hz = 0x22,
    Odr200Hz = 0x32,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum SmbusMd {
    TimeoutEnable = 0,
    TimeoutDisable = 1,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum Reg {
    WhoAmI = 0x01,
    TempHLimit = 0x02,
    TempLLimit = 0x03,
    Ctrl = 0x04,
    Status = 0x05,
    TempLOut = 0x06,
    TempHOut = 0x07,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum I2CAddress {
    I2cAddH = 0x71,
    I2cAddL = 0x7F,
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
