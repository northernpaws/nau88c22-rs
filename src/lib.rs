#![cfg_attr(not(feature = "std"), no_std)]

use paste::paste;

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
        self.write_register_value(Register::SoftwareReset, 0x1FF)
            .await
    }

    /// Reads the specified register.
    ///
    /// Register values are in the lower 9 bits of the `u16`.
    pub async fn read_register_value(
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
    pub async fn write_register_value(
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
    pub async fn modify_register_value<F>(
        &mut self,
        register: registers::Register,
        f: F,
    ) -> Result<(), I2C::Error>
    where
        F: FnOnce(u16) -> u16,
    {
        let value = self.read_register_value(register).await?;
        let new_value = f(value);
        self.write_register_value(register, new_value).await
    }
}

/// Macro rule to generate the read_, write_
/// and modify_ methods for a given register.
macro_rules! register_methods {
    ($name:ident) => {
        paste! {
            /// Reads from the $name register.
            pub async fn [<read_ $name:lower >](
                &mut self,
            ) -> Result<registers::$name, I2C::Error> {
                // Read the raw value from the register.
                let value = self.read_register_value(registers::Register::$name).await?;

                // Convert it into the register struct.
                Ok(registers::$name(value))
            }

            /// Writes to the $name register.
            pub async fn [<write_ $name:lower >](
                &mut self,
                register: registers::$name,
            ) -> Result<(), I2C::Error> {
                // Write the underlying `u16` storage value to the register.
                self.write_register_value(registers::Register::$name, register.into()).await?;
                Ok(())
            }

            /// Modifies the $name register.
            pub async fn [<modify_ $name:lower >] <F>(
                &mut self,
                f: F,
            ) -> Result<(), I2C::Error>
            where
                F: FnOnce(registers::$name) -> registers::$name,
            {
                let value = self.[<read_ $name:lower >]().await?;
                let new_value = f(value);
                self.[<write_ $name:lower >](new_value).await?;
                Ok(())
            }
        }
    };
}

/// Implements the various register methods.
impl<I2C: embedded_hal_async::i2c::I2c> Nau88c22<I2C>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    register_methods!(PowerManagement1);
    register_methods!(PowerManagement2);
    register_methods!(PowerManagement3);

    // General Audio Controls
    register_methods!(AudioInterface);
    register_methods!(Companding);
    register_methods!(ClockControl1);
    register_methods!(ClockControl2);
    register_methods!(Gpio);
    register_methods!(JackDetect1);
    register_methods!(DACControl);
    register_methods!(LeftDACVolume);
    register_methods!(RightDACVolume);
    register_methods!(JackDetect2);
    register_methods!(ADCControl);
    register_methods!(LeftADCVolume);
    register_methods!(RightADCVolume);

    // Equalizer
    register_methods!(EQ1HighCutoff);
    register_methods!(EQ2Peak1);
    register_methods!(EQ3Peak2);
    register_methods!(EQ4Peak3);
    register_methods!(EQ5LowCutoff);

    // DAC Limiter
    register_methods!(DACLimiter1);
    register_methods!(DACLimiter2);

    // Notch filter
    register_methods!(NotchFilter1);
    register_methods!(NotchFilter2);
    register_methods!(NotchFilter3);
    register_methods!(NotchFilter4);

    // ALC and Noise Gate Controls
    register_methods!(ALCControl1);
    register_methods!(ALCControl2);
    register_methods!(ALCControl3);
    register_methods!(NoiseGate);

    // Phase Locked Loop
    register_methods!(PllN);
    register_methods!(PllK1);
    register_methods!(PllK2);
    register_methods!(PllK3);

    // Miscellaneous
    register_methods!(Control3D);
    register_methods!(RightSpeakerSubmixer);
    register_methods!(InputControl);
    register_methods!(LeftInputPGAGain);
    register_methods!(RightInputPGAGain);
    register_methods!(LeftADCBoost);
    register_methods!(RightADCBoost);
    register_methods!(OutputControl);
    register_methods!(LeftMixer);
    register_methods!(RightMixer);
    register_methods!(LHPVolume);
    register_methods!(RHPVolume);
    register_methods!(LSPKOutVolume);
    register_methods!(RSPKOutVolume);
    register_methods!(AUX2Mixer);
    register_methods!(AUX1Mixer);
    register_methods!(PowerManagement4);

    // PCM Time Slot and ADCOUT Impedance Option Control
    register_methods!(LeftTimeSlot);
    register_methods!(Misc);
    register_methods!(RightTimeSlot);

    // Silicon Revision and Device ID
    register_methods!(DeviceRevisionNumber);
    register_methods!(DeviceID);

    register_methods!(DACDither);
    register_methods!(ALCEnhancement1);
    register_methods!(ALCEnhancement2);
    register_methods!(MiscControls);
    register_methods!(InputTieOffDirectManualControl);
    register_methods!(ReductionAndOutputTieOffDirectManualControl);
    register_methods!(AGCPeakToPeakReadout);
    register_methods!(AGCPeakDetectorReadout);
    register_methods!(AutomuteControlAndStatusReadout);
    register_methods!(OutputTieOffDirectManualControls);
}
