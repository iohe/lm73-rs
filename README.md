# Rust LM73 Temperature Sensor and Thermal Watchdog Driver

[![crates.io](https://img.shields.io/crates/v/lm73.svg)](https://crates.io/crates/lm73)
[![Docs](https://docs.rs/lm73/badge.svg)](https://docs.rs/lm73)
[![Build Status](https://travis-ci.org/eldruin/lm73-rs.svg?branch=master)](https://travis-ci.org/iohe/lm73-rs)
[![Coverage Status](https://coveralls.io/repos/github/eldruin/lm73-rs/badge.svg?branch=master)](https://coveralls.io/github/iohe/lm73-rs?branch=master)
![Maintenance Intention](https://img.shields.io/badge/maintenance-actively--developed-brightgreen.svg)

This is based on LM75 work of Diego Barrios Romero

This is a platform agnostic Rust driver for the LM73 temperature sensor
and thermal watchdog, based on the
[`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits.


This driver allows you to:
- Enable/disable the device.
- Read the temperature.

## The device
The LM73 temperature sensor includes a delta-sigma analog-to-digital
converter, and a digital overtemperature detector. The host can
query the LM73 through its I2C interface to read temperature at any
time. 

Datasheet:
- [LM73](http://www.ti.com/lit/ds/symlink/lm73.pdf)

### Usage

```rust
extern crate linux_embedded_hal as hal;
extern crate lm73;

use hal::I2cdev;
use lm73::{ Lm73, SlaveAddr };

fn main() {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let address = SlaveAddr::default();
    let mut sensor = Lm73::new(dev, address);
    let temp_celsius = sensor.read_temperature().unwrap();
    println!("Temperature: {}ºC", temp_celsius);
}
```

## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT) at your option.

### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

