#![cfg_attr(not(feature = "std"), no_std)]

use embedded_hal_async::i2c::SevenBitAddress;

use crate::registers::Register;

pub mod registers;

pub const ADDRESS: SevenBitAddress = 0b0011010;

/// Driver for the NAU88C22 audio codec.
pub struct Nau88c22<I2C> {
    interface: I2C,
}

impl<I2C> Nau88c22<I2C> {
    /// Constructs a new instance of the NAU88C22 audio codec driver.
    pub fn new(interface: I2C) -> Self {
        Self { interface }
    }
}

/// Trait implementation for asyncronous I2C.
impl<I2C: embedded_hal_async::i2c::I2c> Nau88c22<I2C>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    /// Reset the audio codec.
    pub async fn reset(&mut self) -> Result<(), I2C::Error> {
        // Writing anything to register 0 triggers a reset.
        self.write_register(Register::SoftwareReset, 0x1FF).await
    }

    /// Reads the specified register.
    ///
    /// Register values are in the lower 9 bits of the `u16`.
    pub async fn read_register(
        &mut self,
        register: registers::Register,
    ) -> Result<u16, I2C::Error> {
        let mut buffer = [0u8; 2];
        self.interface
            .write_read(ADDRESS, &[(register as u8) * 2], &mut buffer)
            .await?;
        let mut result = ((buffer[0] as u16) & 1) << 8;
        result |= buffer[1] as u16;
        Ok(result)
    }

    /// Write to the specified register.
    ///
    /// The value is contained in the lower 9 bits of the `u16`.
    pub async fn write_register(
        &mut self,
        register: registers::Register,
        value: u16,
    ) -> Result<(), I2C::Error> {
        let buffer = [((register as u8) * 2) | (value >> 8) as u8, value as u8];
        self.interface.write(ADDRESS, &buffer).await?;
        Ok(())
    }

    /// Reads the value of a register, passes it to the included
    /// closure for modification, and writes the results.
    pub async fn modify_register<F>(
        &mut self,
        register: registers::Register,
        f: F,
    ) -> Result<(), I2C::Error>
    where
        F: FnOnce(u16) -> u16,
    {
        let value = self.read_register(register).await?;
        let new_value = f(value);
        self.write_register(register, new_value).await
    }
}
