use bitfield_struct::bitfield;

/// CTRL (0x04)
///
/// Control register (R/W)
/// Used to configure the operating mode, averaging, address increment, and other features.
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Ctrl {
    /// ONE_SHOT (bit 0)
    /// If set to 1, triggers a new one-shot temperature acquisition.
    /// After conversion, device returns to power-down.
    #[bits(1)]
    pub one_shot: u8,

    /// TIME_OUT_DIS (bit 1)
    /// If set to 1, disables the SMBus timeout function.
    /// Timeout is enabled by default at power-up.
    #[bits(1)]
    pub time_out_dis: u8,

    /// FREERUN (bit 2)
    /// Enables freerun mode when set to 1.
    /// In freerun, temperature is measured continuously at the selected ODR.
    #[bits(1)]
    pub freerun: u8,

    /// IF_ADD_INC (bit 3)
    /// Enables automatic address increment for I²C/SMBus multi-byte read/write when set to 1.
    /// Default is 1 (auto-increment enabled).
    #[bits(1)]
    pub if_add_inc: u8,

    /// AVG[1:0] (bits 4-5)
    /// Selects the number of averages and, in freerun mode, the output data rate (ODR).
    /// 00: 25 Hz, 01: 50 Hz, 10: 100 Hz, 11: 200 Hz (see Table 13 in datasheet).
    #[bits(2)]
    pub avg: u8,

    /// BDU (bit 6)
    /// Block Data Update.
    /// 0: Output registers are updated continuously.
    /// 1: Output registers are not updated until both TEMP_L_OUT and TEMP_H_OUT are read.
    #[bits(1)]
    pub bdu: u8,

    /// LOW_ODR_START (bit 7)
    /// Enables 1 Hz low ODR mode when set to 1.
    /// Only one of FREERUN or LOW_ODR_START should be set at a time.
    #[bits(1)]
    pub low_odr_start: u8,
}

/// STATUS (0x05)
///
/// Status register (Read-only)
/// Provides information about conversion status and threshold events.
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Status {
    /// BUSY (bit 0)
    /// Indicates conversion status in one-shot mode.
    /// 0: Conversion complete, 1: Conversion in progress.
    #[bits(1)]
    pub busy: u8,

    /// OVER_THH (bit 1)
    /// High temperature threshold exceeded.
    /// 0: Not exceeded, 1: Exceeded (auto-cleared on STATUS read).
    #[bits(1)]
    pub over_thh: u8,

    /// UNDER_THL (bit 2)
    /// Low temperature threshold exceeded.
    /// 0: Not exceeded, 1: Exceeded (auto-cleared on STATUS read).
    #[bits(1)]
    pub under_thl: u8,

    /// Reserved (bits 3-7)
    /// Not used, always reads as 0.
    #[bits(5, access = RO)]
    pub not_used_01: u8,
}

/// TEMP_H_LIMIT (0x02)
///
/// High temperature threshold register (R/W)
/// Sets the high temperature interrupt threshold.
/// Threshold = (TEMP_H_LIMIT - 63) * 0.64°C.
/// Writing 0 disables the high limit interrupt.
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct TempHLimit {
    /// THL[7:0]
    /// High temperature threshold value.
    #[bits(8)]
    pub thl: u8,
}

/// TEMP_L_LIMIT (0x03)
///
/// Low temperature threshold register (R/W)
/// Sets the low temperature interrupt threshold.
/// Threshold = (TEMP_L_LIMIT - 63) * 0.64°C.
/// Writing 0 disables the low limit interrupt.
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct TempLLimit {
    /// TLL[7:0]
    /// Low temperature threshold value.
    #[bits(8)]
    pub tll: u8,
}

/// Device status
///
/// Software representation of the BUSY status bit.
pub struct DevStatus {
    /// BUSY
    /// 0: Conversion complete, 1: Conversion in progress (one-shot mode only).
    pub busy: u8,
}

/// Temperature threshold event source
///
/// Indicates which threshold event(s) occurred.
pub struct TempTrlhdSrc {
    /// UNDER_THL
    /// 1: Low temperature threshold exceeded.
    pub under_thl: u8,

    /// OVER_THH
    /// 1: High temperature threshold exceeded.
    pub over_thh: u8,
}

/// Temperature Output Data Rate (ODR) selection
///
/// Defines the ODR or power mode for the temperature sensor.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum OdrTemp {
    /// Power-down mode (default)
    PowerDown = 0x00,
    /// One-shot mode
    OneShot = 0x01,
    /// 1 Hz ODR (low power)
    Odr1Hz = 0x04,
    /// 25 Hz ODR
    Odr25Hz = 0x02,
    /// 50 Hz ODR
    Odr50Hz = 0x12,
    /// 100 Hz ODR
    Odr100Hz = 0x22,
    /// 200 Hz ODR
    Odr200Hz = 0x32,
}

/// SMBus Timeout Mode
///
/// Controls the SMBus timeout feature.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum SmbusMd {
    /// Timeout enabled (default)
    TimeoutEnable = 0,
    /// Timeout disabled
    TimeoutDisable = 1,
}

/// Register addresses
///
/// Enumerates the register map of the STTS22H.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum Reg {
    /// WHO_AM_I register (device identification)
    WhoAmI = 0x01,
    /// TEMP_H_LIMIT register (high temperature threshold)
    TempHLimit = 0x02,
    /// TEMP_L_LIMIT register (low temperature threshold)
    TempLLimit = 0x03,
    /// CTRL register (configuration and mode control)
    Ctrl = 0x04,
    /// STATUS register (conversion and threshold status)
    Status = 0x05,
    /// TEMP_L_OUT register (temperature output, low byte)
    TempLOut = 0x06,
    /// TEMP_H_OUT register (temperature output, high byte)
    TempHOut = 0x07,
}
