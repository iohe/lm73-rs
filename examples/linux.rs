extern crate embedded_hal as hal;
extern crate lm73;
pub extern crate i2cdev;

use std::{ops};
use std::path::{Path, PathBuf};
use lm73::{ Lm73, SlaveAddr };
use i2cdev::core::I2CDevice;

/// Newtype around [`i2cdev::linux::LinuxI2CDevice`] that implements the `embedded-hal` traits
///
/// [`i2cdev::linux::LinuxI2CDevice`]: https://docs.rs/i2cdev/0.3.1/i2cdev/linux/struct.LinuxI2CDevice.html
pub struct I2cdev {
    inner: i2cdev::linux::LinuxI2CDevice,
    path: PathBuf,
    address: Option<u8>,
}

impl I2cdev {
    /// See [`i2cdev::linux::LinuxI2CDevice::new`][0] for details.
    ///
    /// [0]: https://docs.rs/i2cdev/0.3.1/i2cdev/linux/struct.LinuxI2CDevice.html#method.new
    pub fn new<P>(path: P) -> Result<Self, i2cdev::linux::LinuxI2CError>
    where
        P: AsRef<Path>,
    {
        let dev = I2cdev {
            path: path.as_ref().to_path_buf(),
            inner: i2cdev::linux::LinuxI2CDevice::new(path, 0)?,
            address: None,
        };
        Ok(dev)
    }

    fn set_address(&mut self, address: u8) -> Result<(), i2cdev::linux::LinuxI2CError> {
        if self.address != Some(address) {
            self.inner = i2cdev::linux::LinuxI2CDevice::new(&self.path, address as u16)?;
            self.address = Some(address);
        }
        Ok(())
    }
}

impl hal::blocking::i2c::Read for I2cdev {
    type Error = i2cdev::linux::LinuxI2CError;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.set_address(address)?;
        let data = self.inner.smbus_read_byte_data(buffer[0]).unwrap();
        buffer[0] = data;
        Ok(())
    }
}

impl hal::blocking::i2c::Write for I2cdev {
    type Error = i2cdev::linux::LinuxI2CError;

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.set_address(address)?;
        self.inner.smbus_write_byte_data(address, bytes[0])
    }
}

impl hal::blocking::i2c::WriteRead for I2cdev {
    type Error = i2cdev::linux::LinuxI2CError;

    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        
        self.set_address(address)?;
        let data = self.inner.smbus_read_word_data(bytes[0]).unwrap();
        buffer[1] = ((data >> 8) & 0xff) as u8;
        buffer[0] = (data & 0xff) as u8;
        Ok(())
    }
}

impl ops::Deref for I2cdev {
    type Target = i2cdev::linux::LinuxI2CDevice;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl ops::DerefMut for I2cdev {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}


fn main() {
    let dev = I2cdev::new("/dev/i2c-0").unwrap();
    let mut sensor = Lm73::new(dev, SlaveAddr::default());
    let temperature = sensor.read_temperature().unwrap();
    println!("Temperature: {}", temperature);
    
}
