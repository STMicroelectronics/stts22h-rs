# stts22h-rs
[![Crates.io][crates-badge]][crates-url]
[![BSD 3-Clause licensed][bsd-badge]][bsd-url]

[crates-badge]: https://img.shields.io/crates/v/stts22h-rs
[crates-url]: https://crates.io/crates/stts22h-rs
[bsd-badge]: https://img.shields.io/crates/l/stts22h-rs
[bsd-url]: https://opensource.org/licenses/BSD-3-Clause

Provides a platform-agnostic, no_std-compatible driver for the ST STTS22H temperature sensor, supporting both I2C and SPI communication interfaces.

## Sensor Overview

The STTS22H is a digital temperature sensor that communicates over a 2-wire
I²C/SMBus 3.0 serial interface.
Thanks to its factory calibration, the STTS22H offers high-end accuracy
performance over the entire operating
temperature range reaching as low as ±0.5 °C without requiring any further
calibration at the application level.

The sensor operating mode is user-configurable and allows selecting between
different ODRs (down to 1 Hz) or
the one-shot mode for battery saving. In one-shot mode, the sensor current
consumption falls to 1.75 µA.

The STTS22H comes in a 6-pin device that supports user-configurable slave
addresses. By connecting properly
the Addr pin (see Table 2), four different addresses can be specified, thus
allowing to have up to four STTS22H
sharing the same I²C/SMBus bus line. An interrupt pin is also available to
signal the application whenever the
user-selectable high or low threshold has been exceeded.

For more info, please visit the device page at [https://www.st.com/en/mems-and-sensors/stts22h.html](https://www.st.com/en/mems-and-sensors/stts22h.html)

## Installation

Add the driver to your `Cargo.toml` dependencies:

```toml
[dependencies]
stts22h-rs = "1.1.0"
```

Or, add it directly from the terminal:

```sh
cargo add stts22h-rs
```

## Usage

Include the crate and its prelude
```rust
use stts22h_rs as stts22h;
use stts22h::*;
use stts22h::prelude::*;
```

### Create an instance

Create an instance of the driver with the `new_<bus>` associated function, by passing an I2C (`embedded_hal::i2c::I2c`) instance and I2C address, or an SPI (`embedded_hal::spi::SpiDevice`) instance.

An example with I2C:

```rust
let mut sensor = Stts22h::new_i2c(i2c, I2CAddress::I2cAddH).unwrap();
```

### Check "Who Am I" Register

This step ensures correct communication with the sensor. It returns a unique ID to verify the sensor's identity.

```rust
let whoami = sensor.dev_id_get().unwrap();
if whoami != ID {
    panic!("Invalid sensor ID");
}
```

### Configure

See details in specific examples; the following are common api calls:

```rust
// Set Output Data Rate
sensor.temp_data_rate_set(OdrTemp::Odr1Hz).unwrap();
```

## License

Distributed under the BSD-3 Clause license.

More Information: [http://www.st.com](http://st.com/MEMS).

**Copyright (C) 2025 STMicroelectronics**
