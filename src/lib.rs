//! This is a platform agnostic Rust driver for the lm73 temperature
//! sensor and thermal watchdog, based on the [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
//!
//! This driver allows you to:
//! - Enable/disable the device.
//! - Read the temperature.
//! - Set the OS temperature.
//! - Set the OS operation mode.
//!
//! ## The device
//!
//! The lm73 temperature sensor includes a delta-sigma analog-to-digital
//! converter, and a digital overtemperature detector. The host can
//! query the lm73 through its I2C interface to read temperature at any
//! time. 
//! 
//! Datasheet:
//! - [lm73](https://datasheets.maximintegrated.com/en/ds/lm73.pdf)
//!
//!
//! ## Usage examples (see also examples folder)
//!
//! To use this driver, import this crate and an `embedded_hal` implementation,
//! then instantiate the device.
//!
//! ### Read temperature
//!
//! ```no_run
//! extern crate linux_embedded_hal as hal;
//! extern crate lm73;
//!
//! use hal::I2cdev;
//! use lm73::{ Lm73, SlaveAddr };
//!
//! # fn main() {
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let address = SlaveAddr::default();
//! let mut sensor = Lm73::new(dev, address);
//! let temp_celsius = sensor.read_temperature().unwrap();
//! println!("Temperature: {}ºC", temp_celsius);
//! # }
//! ```
//!
//! ### Provide an alternative address
//!
//! ```no_run
//! extern crate linux_embedded_hal as hal;
//! extern crate lm73;
//!
//! use hal::I2cdev;
//! use lm73::{ Lm73, SlaveAddr };
//!
//! # fn main() {
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let (a2, a1, a0) = (false, false, true);
//! let address = SlaveAddr::Alternative(a2, a1, a0);
//! let mut sensor = Lm73::new(dev, address);
//! # }
//! ```
//!
//!
//! ### Set the OneShot operation mode
//!
//! ```no_run
//! extern crate linux_embedded_hal as hal;
//! extern crate lm73;
//!
//! use hal::I2cdev;
//! use lm73::{ Lm73, SlaveAddr, OsMode };
//!
//! # fn main() {
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let mut sensor = Lm73::new(dev, SlaveAddr::default());
//! sensor.set_os_mode(OsMode::Enabled).unwrap();
//! # }
//! ```
//!
//! ### Set the HIGH_TEMP temperature
//!
//! ```no_run
//! extern crate linux_embedded_hal as hal;
//! extern crate lm73;
//!
//! use hal::I2cdev;
//! use lm73::{ Lm73, SlaveAddr };
//!
//! # fn main() {
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let mut sensor = Lm73::new(dev, SlaveAddr::default());
//! let temp_celsius = 50.0;
//! sensor.set_temperature_high(temp_celsius).unwrap();
//! # }
//! ```
//!
//! ### Enable / disable the sensor
//!
//! ```no_run
//! extern crate linux_embedded_hal as hal;
//! extern crate lm73;
//!
//! use hal::I2cdev;
//! use lm73::{ Lm73, SlaveAddr };
//!
//! # fn main() {
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let mut sensor = Lm73::new(dev, SlaveAddr::default());
//! sensor.disable().unwrap(); // shutdown
//! sensor.enable().unwrap();
//! # }
//! ```

#![deny(missing_docs, unsafe_code, warnings)]
#![no_std]

extern crate embedded_hal as hal;

use hal::blocking::i2c;

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error
    I2C(E),
    /// Invalid input data
    InvalidInputData,
}

/// Possible slave addresses
#[derive(Debug, Clone)]
pub enum SlaveAddr {
    /// Default slave address
    Default,
    /// Alternative slave address providing bit values for A2, A1 and A0
    Alternative(bool, bool, bool),
}

impl Default for SlaveAddr {
    /// Default slave address
    fn default() -> Self {
        SlaveAddr::Default
    }
}

impl SlaveAddr {
    fn addr(self, default: u8) -> u8 {
        match self {
            SlaveAddr::Default => default,
            SlaveAddr::Alternative(a2, a1, a0) => {
                default | ((a2 as u8) << 2) | ((a1 as u8) << 1) | a0 as u8
            }
        }
    }
}

/// Resolution of temperature
///
/// Number of consecutive faults necessary to trigger OS condition.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Resolution {
    /// 11 bits, 0.25(default)
    _11,
    /// 12 bits, 0.125
    _12,
    /// 13 bits, 0.0625
    _13,
    /// 14 bits, 0.03125
    _14,
}

/// OS polarity
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OsPolarity {
    /// Active low (default)
    ActiveLow,
    /// Active high
    ActiveHigh,
}

/// OneShot operation mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OsMode {
    /// Disabled (default)
    Disabled,
    /// Enabled
    Enabled,
}

/// Alert Enable operation mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AlertEnable {
    /// Disabled (default)
    Disabled,
    /// Enabled
    Enabled,
}

const DEVICE_BASE_ADDRESS: u8 = 0b100_1000;

struct Register;

impl Register {
    const TEMPERATURE: u8 = 0x00;
    const CONFIGURATION: u8 = 0x01;
    const T_HIGH: u8 = 0x02;
    const T_LOW: u8 = 0x03;
    const CONTROL_STATUS: u8 = 0x04;
    const IDENTIFICATION: u8 = 0x07;
}

struct ConfigBitFlags;

impl ConfigBitFlags {
    //   const RESERVED0_0 : u8 = 0b0000_0001;
    //   const RESERVED1_0 : u8 = 0b0000_0010;
    const ONE_SHOT: u8 = 0b0000_0100;
    const ALRT_RST: u8 = 0b0000_1000;
    const ALRT_POL: u8 = 0b0001_0000;
    const ALRT_EN: u8 = 0b0010_0000;
    //   const RESERVED6_1 : u8 = 0b0100_0000;
    const PD: u8 = 0b1000_0000;
}

#[derive(Debug, Clone, Copy)]
struct Config {
    bits: u8,
}

impl Config {
    fn with_high(self, mask: u8) -> Self {
        Config {
            bits: self.bits | mask,
        }
    }
    fn with_low(self, mask: u8) -> Self {
        Config {
            bits: self.bits & !mask,
        }
    }
}

impl Default for Config {
    fn default() -> Self {
        Config { bits: 0x40 }
    }
}

struct CsrBitFlags;

impl CsrBitFlags {
    //   const DAV         : u8 = 0b0000_0001;
    //   const TLOW        : u8 = 0b0000_0010;
    //   const THI         : u8 = 0b0000_0100;
    //   const ALRT_STAT   : u8 = 0b0000_1000;
    //   const RESERVED4_0 : u8 = 0b0001_0000;
    const RES0: u8 = 0b0010_0000;
    const RES1: u8 = 0b0100_0000;
    //   const TO_DIS      : u8 = 0b1000_0000;
}

#[derive(Debug, Clone, Copy)]
struct Csr {
    bits: u8,
}

impl Csr {
    fn with_high(self, mask: u8) -> Self {
        Csr {
            bits: self.bits | mask,
        }
    }
    fn with_low(self, mask: u8) -> Self {
        Csr {
            bits: self.bits & !mask,
        }
    }
}

impl Default for Csr {
    fn default() -> Self {
        Csr { bits: 0b0000_1000 }
    }
}

/// Lm73 device driver.
#[derive(Debug, Default)]
pub struct Lm73<I2C> {
    /// The concrete I²C device implementation.
    i2c: I2C,
    /// The I²C device address.
    address: u8,
    /// Configuration register status.
    config: Config,
    /// Control and Status register.
    csr: Csr,
}

mod conversion;

impl<I2C, E> Lm73<I2C>
where
    I2C: i2c::Write<Error = E>,
{
    /// Create new instance of the lm73 device.
    pub fn new(i2c: I2C, address: SlaveAddr) -> Self {
        Lm73 {
            i2c,
            address: address.addr(DEVICE_BASE_ADDRESS),
            config: Config::default(),
            csr: Csr::default(),
        }
    }

    /// Destroy driver instance, return I²C bus instance.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Enable the sensor (default state).
    pub fn enable(&mut self) -> Result<(), Error<E>> {
        let csr = self.csr;
        let _result = self.write_csr(csr)?;

        let config = self.config;
        self.write_config(config.with_low(ConfigBitFlags::PD))
    }

    /// Disable the sensor (shutdown).
    pub fn disable(&mut self) -> Result<(), Error<E>> {
        let config = self.config;
        self.write_config(config.with_high(ConfigBitFlags::PD))
    }

    /// Set temperature resolution.
    ///
    /// Set temperature conversion resolution.
    pub fn set_resolution(&mut self, res: Resolution) -> Result<(), Error<E>> {
        let csr = self.csr;
        match res {
            Resolution::_11 => {
                self.write_csr(csr.with_low(CsrBitFlags::RES1).with_low(CsrBitFlags::RES0))
            }
            Resolution::_12 => {
                self.write_csr(csr.with_low(CsrBitFlags::RES1).with_high(CsrBitFlags::RES0))
            }
            Resolution::_13 => {
                self.write_csr(csr.with_high(CsrBitFlags::RES1).with_low(CsrBitFlags::RES0))
            }
            Resolution::_14 => self.write_csr(
                csr.with_high(CsrBitFlags::RES1)
                    .with_high(CsrBitFlags::RES0),
            ),
        }
    }

    /// Set the OneShot operation mode.
    pub fn set_os_mode(&mut self, mode: OsMode) -> Result<(), Error<E>> {
        let config = self.config;
        match mode {
            OsMode::Enabled => self.write_config(config.with_high(ConfigBitFlags::ONE_SHOT)),
            OsMode::Disabled => self.write_config(config.with_low(ConfigBitFlags::ONE_SHOT)),
        }
    }

    /// Reset Alert pin.
    pub fn set_alert_reset(&mut self) -> Result<(), Error<E>> {
        let config = self.config;
        self.write_config(config.with_high(ConfigBitFlags::ALRT_RST))
    }

    /// Set the ALRT_POL polarity.
    pub fn set_alert_polarity(&mut self, polarity: OsPolarity) -> Result<(), Error<E>> {
        let config = self.config;
        match polarity {
            OsPolarity::ActiveLow => self.write_config(config.with_low(ConfigBitFlags::ALRT_POL)),
            OsPolarity::ActiveHigh => self.write_config(config.with_high(ConfigBitFlags::ALRT_POL)),
        }
    }

    /// Set the ALRT_EN
    pub fn set_alert_enable(&mut self, enabled: AlertEnable) -> Result<(), Error<E>> {
        let config = self.config;
        match enabled {
            AlertEnable::Disabled => self.write_config(config.with_high(ConfigBitFlags::ALRT_EN)),
            AlertEnable::Enabled => self.write_config(config.with_low(ConfigBitFlags::ALRT_EN)),
        }
    }

    /// Set temperature_high (celsius).
    pub fn set_temperature_high(&mut self, temperature: f32) -> Result<(), Error<E>> {
        if temperature < -256.0 || temperature > 255.75 {
            return Err(Error::InvalidInputData);
        }
        let (msb, lsb) = conversion::convert_temp_to_register(temperature);
        let _data: u16 = (msb as u16 * 256 as u16 + lsb as u16).into();
        self.i2c
            .write(Register::T_HIGH, &[msb, lsb])
            .map_err(Error::I2C)
    }

    /// Set temperature_low (celsius).
    pub fn set_temperature_low(&mut self, temperature: f32) -> Result<(), Error<E>> {
        if temperature < -256.0 || temperature > 255.75 {
            return Err(Error::InvalidInputData);
        }
        let (msb, lsb) = conversion::convert_temp_to_register(temperature);
        let _data: u16 = (msb as u16 * 256 as u16 + lsb as u16).into();
        self.i2c
            .write(Register::T_LOW, &[msb, lsb])
            .map_err(Error::I2C)
    }

    fn write_config(&mut self, config: Config) -> Result<(), Error<E>> {
        self.i2c
            .write(Register::CONFIGURATION, &[config.bits])
            .map_err(Error::I2C)?;
        self.config = config;
        Ok(())
    }

    fn write_csr(&mut self, csr: Csr) -> Result<(), Error<E>> {
        self.i2c
            .write(Register::CONTROL_STATUS, &[csr.bits])
            .map_err(Error::I2C)?;
        self.csr = csr;
        Ok(())
    }
}

impl<I2C, E> Lm73<I2C>
where
    I2C: i2c::WriteRead<Error = E>,
{
    /// Read the temperature from the sensor (celsius).
    pub fn read_temperature(&mut self) -> Result<f32, Error<E>> {
        let mut data = [0; 2];
        self.i2c
            .write_read(self.address, &[Register::TEMPERATURE], &mut data)
            .map_err(Error::I2C)?;
        Ok(conversion::convert_temp_from_register(data[0], data[1]))
    }

    /// Read the identification sensor register.
    pub fn read_identification(&mut self) -> Result<u16, Error<E>> {
        let mut data = [0; 2];
        self.i2c
            .write_read(self.address, &[Register::IDENTIFICATION], &mut data)
            .map_err(Error::I2C)?;

        Ok( (data[0] as u16) << 8 + data[1])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn can_get_default_address() {
        let addr = SlaveAddr::default();
        assert_eq!(DEVICE_BASE_ADDRESS, addr.addr(DEVICE_BASE_ADDRESS));
    }

    #[test]
    fn can_generate_alternative_addresses() {
        assert_eq!(
            0b100_1001,
            SlaveAddr::Alternative(false, false, true).addr(DEVICE_BASE_ADDRESS)
        );
        assert_eq!(
            0b100_1010,
            SlaveAddr::Alternative(false, true, false).addr(DEVICE_BASE_ADDRESS)
        );
        assert_eq!(
            0b100_1100,
            SlaveAddr::Alternative(true, false, false).addr(DEVICE_BASE_ADDRESS)
        );
        assert_eq!(
            0b100_1101,
            SlaveAddr::Alternative(true, false, true).addr(DEVICE_BASE_ADDRESS)
        );
        assert_eq!(
            0b100_1110,
            SlaveAddr::Alternative(true, true, false).addr(DEVICE_BASE_ADDRESS)
        );
    }
}
