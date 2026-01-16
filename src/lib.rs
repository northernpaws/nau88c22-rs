#![cfg_attr(not(feature = "std"), no_std)]

use core::fmt;

use paste::paste;

use embedded_hal_async::{
    delay::DelayNs,
    i2c::{self, ErrorType, SevenBitAddress},
};

use crate::{
    clock::ClockParameters,
    registers::{AudioInterfaceDataFormat, MasterClockSourceScaling, Register, WordLength},
};

pub mod clock;
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

/// Supplies the configuration for the left or right PGA.
///
/// The Programmable Gain Amplifer handles gain for the
/// differential or single-ended mic input and line-in.
///
/// Note that the AUX inputs bypass the PGA.
#[derive(Debug)]
pub struct PGAConfig {
    /// Specifies if the input PGA should be muted.
    pub muted: bool,

    /// Connects mic positive input to PGA.
    pub mic_n: bool,
    /// Connects mic negative input to PGA.
    pub mic_p: bool,
    /// Connects the line-input to the PGA.
    pub line_in: bool,
}

impl Default for PGAConfig {
    fn default() -> Self {
        Self {
            muted: false,
            mic_n: false,
            mic_p: false,
            line_in: false,
        }
    }
}

/// Configures the left or right ADC input mixer.
#[derive(Debug)]
pub struct InputMixerConfig {
    /// Optionally enables and configures the
    /// PGA associated with the input mixer.
    pub pga: Option<PGAConfig>,

    // Apply a +20dB gain boost to the PGA input.
    pub pga_boost: bool,
    pub pga_gain: u8, // TODO: use gain type?
}

/// Configures the left or right ADC.
#[derive(Debug)]
pub struct ADCChannelConfig {
    /// Gain applied the ADC input from the mix/boost stage.
    pub gain: u8,
}

impl Default for ADCChannelConfig {
    fn default() -> Self {
        Self {
            gain: 0b11111111, // 0.0dB attenuation (no attenuation)
        }
    }
}

#[derive(Debug)]
pub struct ADCConfig {
    /// Enables x128 oversampling instead of the
    /// default x64 at a modest power increase.
    pub oversample_128: bool,

    /// Enables and configures the left ADC.
    pub adc_left: Option<ADCChannelConfig>,
    /// Enables and configures the right ADC.
    pub adc_right: Option<ADCChannelConfig>,
}

impl Default for ADCConfig {
    fn default() -> Self {
        Self {
            oversample_128: false,
            adc_left: None,
            adc_right: None,
        }
    }
}

/// Configures the audio format used by the ADCs and DACs.
#[derive(Debug)]
pub struct AudioFormat {
    /// Sets the digital audio word length.
    pub word_length: WordLength,
    /// Sets the digital audio format.
    pub data_format: AudioInterfaceDataFormat,
}

impl Default for AudioFormat {
    fn default() -> Self {
        Self {
            word_length: WordLength::Word24Bit,
            data_format: AudioInterfaceDataFormat::StandardI2S,
        }
    }
}

/// Configures the left or right DAC channel.
#[derive(Debug)]
pub struct DACChannelConfig {
    /// Sets the gain reduction for the DAC.
    ///
    /// 0b11111111 is 0dB
    pub gain: u8,
}

impl Default for DACChannelConfig {
    fn default() -> Self {
        Self {
            gain: 0b11111111, // 0dB
        }
    }
}

#[derive(Debug)]
pub struct DACConfig {
    /// Enables x128 oversampling instead of the
    /// default x64 at a modest power increase.
    pub oversample_128: bool,

    /// Configures the left DAC.
    pub dac_left: Option<DACChannelConfig>,
    /// Configures the right DAC.
    pub dac_right: Option<DACChannelConfig>,
}

impl Default for DACConfig {
    fn default() -> Self {
        Self {
            oversample_128: false,
            dac_left: None,
            dac_right: None,
        }
    }
}

/// Configures the right output mixer.
#[derive(Debug)]
pub struct RightOutputMixerConfig {
    pub aux_gain: u8,
    pub aux_input: bool,
    pub adc_input_gain: u8,
    pub adc_input: bool,
    pub dac_input: bool,
}

impl Default for RightOutputMixerConfig {
    fn default() -> Self {
        Self {
            aux_gain: 0, // -15dB
            aux_input: false,
            adc_input_gain: 0, // -15dB
            adc_input: false,
            dac_input: false,
        }
    }
}

/// Configures the left output mixer.
#[derive(Debug)]
pub struct LeftOutputMixerConfig {
    pub aux_gain: u8,
    pub aux_input: bool,
    pub adc_input_gain: u8,
    pub adc_input: bool,
    pub dac_input: bool,
}

impl Default for LeftOutputMixerConfig {
    fn default() -> Self {
        Self {
            aux_gain: 0, // -15dB
            aux_input: false,
            adc_input_gain: 0, // -15dB
            adc_input: false,
            dac_input: false,
        }
    }
}

/// Configures the aux1 output mixer.
///
/// Note that for audio output, use AUX1 for right channel
/// because AUX2 doens't have a right channel mixer.
#[derive(Debug)]
pub struct Aux1OutputConfig {
    /// AUXOUT1 output mute control.
    pub muted: bool,
    /// AUXOUT1 6dB attenuation enable.
    pub attenuation_6_0db: bool,
    /// Left LMAIN MIXER output to AUX1 MIXER input path control.
    pub left_mixer_input: bool,
    /// Left DAC output to AUX1 MIXER input path control.
    pub left_dac_input: bool,
    /// Right RADC Mix/Boost output RINMIX path control to AUX1 MIXER input.
    pub right_adc_mixer_input: bool,
    /// Right RMIX output to AUX1 MIXER input path control.
    pub right_mixer_input: bool,
    /// Right DAC output to AUX1 MIXER input path control.
    pub right_dac_input: bool,
}

/// Configures the aux2 output mixer.
///
/// Note that for audio output, use AUX2 for left channel.
#[derive(Debug)]
pub struct Aux2OutputConfig {
    /// AUXOUT2 output mute control.
    pub muted: bool,
    /// AUX1 Mixer output to AUX2 MIXER input path control.
    pub aux1_interconnect_input: bool,
    /// Left LADC Mix/Boost output LINMIX path control to AUX2 MIXER input.
    pub left_adc_mix_input: bool,
    /// Left LMAIN MIXER output to AUX2 MIXER input path control.
    pub left_mixer_input: bool,
    /// Left DAC output to AUX2 MIXER input path control.
    pub left_dac_input: bool,
}

#[derive(Debug)]
pub struct AudioConfig {
    /// Enables the AUX1 output mixer.
    ///
    /// Note that these are configured seperate from other outputs
    /// because they have sets of independent mixers from the main.
    pub aux1_output: Option<Aux1OutputConfig>,

    /// Enables the AUX2 output mixer.
    ///
    /// Note that these are configured seperate from other outputs
    /// because they have sets of independent mixers from the main.
    pub aux2_output: Option<Aux2OutputConfig>,

    /// Enables the micbias power regulator.
    ///
    /// This does not apply micbias power directly
    /// to mic inputs, you must tie the micbias
    /// input as a pull-up to the desired inputs.
    pub enable_micbias: bool,

    /// Enables the right headphone driver.
    pub enable_headphone_right: bool,
    /// Enables the left headphone driver.
    pub enable_headphone_left: bool,

    /// Enables the right channel mix/boost input mixer that
    /// sits between the right input PGA and the right ADC.
    ///
    /// Note that the mixer can also be routed through a
    /// bypass to the right main output mixer.
    ///
    /// Note that we don't group the input mixer settings under
    /// the ADC settings because the input mixers can be used
    /// independently via their output mixer bypass path.
    pub input_mixer_right: Option<InputMixerConfig>,

    /// Enables the left channel mix/boost input mixer that
    /// sits between the left input PGA and the left ADC.
    ///
    /// Note that the mixer can also be routed through a
    /// bypass to the left main output mixer.
    ///
    /// Note that we don't group the input mixer settings under
    /// the ADC settings because the input mixers can be used
    /// independently via their output mixer bypass path.
    pub input_mixer_left: Option<InputMixerConfig>,

    /// Enables and configures the right output mixer.
    pub output_mixer_right: Option<RightOutputMixerConfig>,

    /// Enables and configures the left output mixer.
    pub output_mixer_left: Option<LeftOutputMixerConfig>,

    /// Enables the right speaker driver.
    pub enable_speaker_right: bool,
    /// Enables the left speaker driver.
    pub enable_speaker_left: bool,

    /// Enables and configures the left and right ADCs.
    pub adc: Option<ADCConfig>,

    /// Enables and configures the left and right DACs.
    pub dac: Option<DACConfig>,

    /// Configures the digital audio format for the ADCs and DACs.
    pub format: AudioFormat,
}

impl AudioConfig {
    /// Constructs an audio config configured for using the AUX left and
    /// right inputs as ADC inputs, and the AUX1/2 outputs as DAC outputs.
    pub const fn aux_in_aux_out(format: AudioFormat) -> Self {
        AudioConfig {
            aux1_output: Some(Aux1OutputConfig {
                muted: false,
                attenuation_6_0db: false,
                left_mixer_input: false,
                left_dac_input: false,
                right_adc_mixer_input: false,
                right_mixer_input: false,
                right_dac_input: true,
            }),
            aux2_output: Some(Aux2OutputConfig {
                muted: false,
                aux1_interconnect_input: false,
                left_adc_mix_input: false,
                left_mixer_input: false,
                left_dac_input: true,
            }),

            enable_micbias: false,

            enable_headphone_right: false,
            enable_headphone_left: false,

            enable_speaker_right: false,
            enable_speaker_left: false,

            input_mixer_right: Some(InputMixerConfig {
                pga: None, // aux inputs bypass the PGA
                pga_boost: false,
                pga_gain: 0,
            }),
            input_mixer_left: Some(InputMixerConfig {
                pga: None, // aux inputs bypass the PGA
                pga_boost: false,
                pga_gain: 0,
            }),

            // Output mixers are disabled as we'll just pass the
            // DAC output into the AUX output mixers directly.
            output_mixer_right: None,
            output_mixer_left: None,

            adc: Some(ADCConfig {
                oversample_128: false,
                adc_left: Some(ADCChannelConfig {
                    gain: 0b11111111, // 0dB - no attenuation
                }),
                adc_right: Some(ADCChannelConfig {
                    gain: 0b11111111, // 0dB - no attenuation
                }),
            }),

            dac: Some(DACConfig {
                oversample_128: false,
                dac_left: Some(DACChannelConfig {
                    gain: 0b11111111, // 0dB - no attenuation
                }),
                dac_right: Some(DACChannelConfig {
                    gain: 0b11111111, // 0dB - no attenuation
                }),
            }),

            format: format,
        }
    }
}

impl Default for AudioConfig {
    fn default() -> Self {
        Self {
            aux1_output: None,
            aux2_output: None,
            enable_micbias: false,
            enable_headphone_right: false,
            enable_headphone_left: false,
            input_mixer_right: None,
            input_mixer_left: None,
            output_mixer_right: None,
            output_mixer_left: None,
            enable_speaker_right: false,
            enable_speaker_left: false,
            adc: None,
            dac: None,
            format: Default::default(),
        }
    }
}

#[derive(Debug)]
pub struct ClockConfig {
    /// The input master clock frequency
    /// in hertz applied to the MCLK pin.
    pub mclk: f32,

    /// The desired sample rate in hertz.
    pub sample_rate: f32,
}

/// Config for performing initialization of the audio paths and clock.
///
/// Ideally should be used once when program starts, but could
/// be called multiple times without any downsides.
#[derive(Debug)]
pub struct InitializationConfig {
    pub audio: AudioConfig,

    /// Optionally configures the codec's IMCLK PLL.
    pub clock: Option<ClockConfig>,
}

/// Returned if there was an error configuring the clocks.
#[derive(Debug)]
pub enum ClockError<I2CError: i2c::Error> {
    PLLError(clock::CodecClockError),
    MCLKDividerInvalid,
    I2CError(I2CError),
}

impl<I2CError: i2c::Error> From<I2CError> for ClockError<I2CError> {
    fn from(value: I2CError) -> Self {
        Self::I2CError(value)
    }
}

/// Returned if there was an error configuring the audio paths.
#[derive(Debug)]
pub enum AudioError<I2CError: i2c::Error> {
    I2CError(I2CError),
}

impl<I2CError: i2c::Error> From<I2CError> for AudioError<I2CError> {
    fn from(value: I2CError) -> Self {
        Self::I2CError(value)
    }
}

/// Returned if there was an error initializing the codec.
#[derive(Debug)]
pub enum InitError<I2CError: i2c::Error> {
    /// Indicates there was an error configuring the clocks.
    ClockError(ClockError<I2CError>),
    /// Indicates there was an error configuring the audio.
    AudioError(AudioError<I2CError>),
    /// Indicates there was an I2C error resetting the codec.
    I2CError(I2CError),
}

impl<I2CError: i2c::Error> fmt::Display for InitError<I2CError> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl<I2CError: i2c::Error> core::error::Error for InitError<I2CError> {}

impl<I2CError: i2c::Error> From<ClockError<I2CError>> for InitError<I2CError> {
    fn from(value: ClockError<I2CError>) -> Self {
        Self::ClockError(value)
    }
}

impl<I2CError: i2c::Error> From<AudioError<I2CError>> for InitError<I2CError> {
    fn from(value: AudioError<I2CError>) -> Self {
        Self::AudioError(value)
    }
}

impl<I2CError: i2c::Error> From<I2CError> for InitError<I2CError> {
    fn from(value: I2CError) -> Self {
        Self::I2CError(value)
    }
}

/// Utility and convenience methods for the codec.
impl<I2C> Nau88c22<I2C>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    pub async fn mute_all(&mut self) -> Result<(), InitError<I2C::Error>> {
        self.modify_rightspeakersubmixer(|reg| reg.with_rauxsmut(true).with_rmixmut(true))
            .await?;

        self.modify_lhpvolume(|reg| reg.with_lhpmute(true)).await?;
        self.modify_rhpvolume(|reg| reg.with_rhpmute(true)).await?;

        self.modify_lspkoutvolume(|reg| reg.with_lspkmute(true))
            .await?;
        self.modify_rspkoutvolume(|reg| reg.with_rspkmute(true))
            .await?;

        self.modify_aux1mixer(|reg| reg.with_auxout1mt(true))
            .await?;
        self.modify_aux2mixer(|reg| reg.with_auxout2mt(true))
            .await?;

        Ok(())
    }

    /// Convenince methods that performs the manufacture recommended initialization sequence.
    pub async fn initialize<DELAY: DelayNs>(
        &mut self,
        config: InitializationConfig,
        mut delay: DELAY,
    ) -> Result<(), InitError<I2C::Error>> {
        // Software reset the codec to a known state.
        self.reset().await?; // register 0

        // Give time for the reset to finish.
        loop {
            delay.delay_ms(25).await;

            // Check that the device has finished reset and is responsive.
            let device_id = self.read_deviceid().await?;
            if device_id.id() == 26 {
                break;
            }
        }

        // Mute all outputs to help prevent popping.
        self.mute_all().await?;

        // Configure the boost for 3.3v "low-power" operation.
        self.modify_outputcontrol(|reg| {
            reg.with_spkbst(false)
                .with_aux1bst(false)
                .with_aux2bst(false)
        })
        .await?;

        // First enable the aux mixers,internal tie-off, and slow-charge impedance.
        self.modify_powermanagement1(|reg| {
            reg.with_dcbufen(false) // false for lower then 3.6v operation
                .with_abiasen(true) // Enable internal Analog Bias Buffer.
                .with_iobufen(true) // Internal Tie-off Buffer In Non-boost 1.0X Mode
                .with_refimp(0b10) // VREF Impedance Select - 80k
        })
        .await?;

        // Wait the recommended minimum 250ms for the caps to
        // charge to avoid popping/clicking on the outputs.
        delay.delay_ms(500).await;

        // Configure the audio paths, ADC, and DAC.
        self.configure_audio(config.audio).await?;

        // Configure the digital audio clock and internal PLL.
        if let Some(clock) = config.clock {
            // PLL is only required if the master frequency doesn't match
            // the required frequency for the desired sample rate.
            if (clock.sample_rate * 256.0) != clock.mclk {
                self.configure_pll(clock, &mut delay).await?;
            } else {
                // Ensure PLL is disabled if not required.
                self.modify_powermanagement1(|reg| {
                    reg.with_pllen(false) // Ensure the PLL is disabled.
                })
                .await?;

                // Register 6
                //
                // Set MCLK (pin#11) as a master clock input instead of
                // the internal PLL, and disable the input clock divider.
                self.modify_clockcontrol1(|reg| {
                    reg.with_clkm(false) // Internal PLL is disabled
                        .with_mclksel(MasterClockSourceScaling::Divide1) // divide PLL before MCLK
                        .with_clkioen(false) // fs and bclk are inputs
                })
                .await?;
            }
        }

        Ok(())
    }

    /// Convenince method that configures the
    /// audio chain based on a supplied config.
    ///
    /// Note that this does not disable or reset
    /// any unconfigured registers, so it should
    /// only be called after a reset().
    pub async fn configure_audio(
        &mut self,
        config: AudioConfig,
    ) -> Result<(), AudioError<I2C::Error>> {
        // Enable the aux output mixers and mic biasing.
        self.modify_powermanagement1(|reg| {
            reg.with_aux1mxen(config.aux1_output.is_some()) // Enable the aux 1&2 output mixers.
                .with_aux2mxen(config.aux2_output.is_some())
                .with_micbiasen(config.enable_micbias) // Enable the micbias buffer output.
        })
        .await?;

        // Register 2.
        //
        // Enable the configured headphone outputs and ADC inputs.
        self.modify_powermanagement2(|reg| {
            reg.with_rhpen(config.enable_headphone_right) // Right headphone driver
                .with_lphen(config.enable_headphone_left) // Left headphone driver
                .with_sleep(false) // Normal mode
                .with_rbsten(config.input_mixer_right.as_ref().is_some()) // Right channel mixer/boost input
                .with_lbsten(config.input_mixer_left.as_ref().is_some()) // Left channel mixer/boost input
                .with_rpgaen(
                    config
                        .input_mixer_right
                        .as_ref()
                        .is_some_and(|mixer| mixer.pga.is_some()),
                ) // Right channel input PGA enable
                .with_lpgaen(
                    config
                        .input_mixer_left
                        .as_ref()
                        .is_some_and(|mixer| mixer.pga.is_some()),
                ) // Left channel input PGA enable
                .with_radcen(
                    config
                        .adc
                        .as_ref()
                        .is_some_and(|adc| adc.adc_right.is_some()),
                ) // Right channel ADC enable
                .with_ladcen(
                    config
                        .adc
                        .as_ref()
                        .is_some_and(|adc| adc.adc_left.is_some()),
                ) // Left channel ADC enable
        })
        .await?;

        // Input routing
        {
            // CConfigure the input ADC if enabled.
            if let Some(adc) = config.adc {
                // Register 14.
                self.modify_adccontrol(|reg| {
                    reg.with_adcos(adc.oversample_128) //128x oversampling
                        .with_hpf(0)
                        .with_hpfen(false) // disable high pass filter
                })
                .await?;

                // Configure the right input ADC if enabled.
                if let Some(adc_right) = adc.adc_right {
                    self.modify_rightadcvolume(|reg| reg.with_radcgain(adc_right.gain))
                        .await?;
                }

                // Configure the left input ADC if enabled.
                if let Some(adc_left) = adc.adc_left {
                    self.modify_leftadcvolume(|reg| reg.with_ladcgain(adc_left.gain))
                        .await?;
                }
            }

            // Configure the right input mixer and PGA if enabled.
            if let Some(input_mixer) = config.input_mixer_right {
                // Enable the right aux in as the right ADC source.
                self.modify_rightadcboost(|reg| {
                    reg.with_rpgabst(input_mixer.pga_boost) // disable the PGA input
                        .with_rauxbstgain(0b101) // Enable the right aux in at 0db
                        .with_rpgabstgain(input_mixer.pga_gain) // Right line-in boost stage input
                })
                .await?;

                // Configure the right input PGA if the right mixer was enabled.
                if let Some(pga) = input_mixer.pga {
                    // Mute left and right PGA (registers 45, 46)
                    self.modify_rightinputpgagain(|reg| reg.with_rpgamt(pga.muted))
                        .await?;

                    // Register 44.
                    //
                    // Configure the right PGA inputs.
                    self.modify_inputcontrol(|reg| {
                        reg.with_rmicnrpga(pga.mic_n)
                            .with_rmicprpga(pga.mic_p)
                            .with_rlinrpga(pga.line_in)
                    })
                    .await?;
                }
            }

            // Configure the left input mixer and PGA if enabled.
            if let Some(input_mixer) = config.input_mixer_left {
                // Enable the left aux in as the left ADC source.
                self.modify_leftadcboost(|reg| {
                    reg.with_lpgabst(input_mixer.pga_boost) // Disable the left PGA input
                        .with_lauxbstgain(0b101) // Enable the left aux in at 0db
                        .with_lpgabstgain(input_mixer.pga_gain) // Left line-in to boost stage
                })
                .await?;

                // Configure the left input PGA if the left mixer was enabled.
                if let Some(pga) = input_mixer.pga {
                    self.modify_leftinputpgagain(|reg| reg.with_lpgamt(pga.muted))
                        .await?;

                    // Register 44.
                    //
                    // Configure the left PGA inputs.
                    self.modify_inputcontrol(|reg| {
                        reg.with_lmicnlpga(pga.mic_n)
                            .with_lmicplpga(pga.mic_p)
                            .with_llinlpga(pga.line_in)
                    })
                    .await?;
                }
            }
        }

        // Output routing and DAC setup

        // Register 3 - output power
        self.modify_powermanagement3(|reg| {
            reg.with_auxout1en(config.aux1_output.is_some()) // Aux out 1 (pin#21) enable
                .with_auxout2en(config.aux2_output.is_some()) // Aux out 2 (pin#22) enable
                .with_lspken(config.enable_speaker_left) // Left speaker output driver enable
                .with_rspken(config.enable_speaker_right) // Right speaker output driver enable
                .with_rmixen(config.output_mixer_right.is_some()) // Right output main mixer enable
                .with_lmixen(config.output_mixer_left.is_some()) // Left output main mixer enable
                .with_rdacen(
                    config
                        .dac
                        .as_ref()
                        .is_some_and(|dac| dac.dac_right.is_some()),
                ) // Right DAC enable
                .with_ldacen(
                    config
                        .dac
                        .as_ref()
                        .is_some_and(|dac| dac.dac_left.is_some()),
                ) // Left DAC enable
        })
        .await?;

        // Configure the left and right DACs if enabled.
        if let Some(dac) = config.dac {
            // Register 10
            self.modify_daccontrol(|reg| {
                reg.with_dacos(dac.oversample_128) // 128x oversampling
            })
            .await?;

            // Configures the right DAC.
            if let Some(dac_right) = dac.dac_right {
                self.modify_rightdacvolume(|reg| {
                    reg.with_rdacvu(true).with_rdacgain(dac_right.gain)
                })
                .await?;
            }

            // Configures the left DAC.
            if let Some(dac_left) = dac.dac_left {
                self.modify_leftdacvolume(|reg| reg.with_ldacvu(true).with_ldacgain(dac_left.gain))
                    .await?;
            }
        }

        // Configure the right output mixer.
        if let Some(mixer) = config.output_mixer_right {
            // Register 51
            // Set the right main mix to use the right aux input.
            self.modify_rightmixer(|reg| {
                reg.with_rauxmxgain(mixer.aux_gain)
                    .with_rauxrmx(mixer.aux_input)
                    .with_rbypmxgain(mixer.adc_input_gain)
                    .with_rbyprmx(mixer.adc_input)
                    .with_rdacrmx(mixer.dac_input)
            })
            .await?;
        }

        // Configure the left output mixer.
        if let Some(mixer) = config.output_mixer_left {
            // Register 50
            // Set the left main mix to use the left aux input.
            self.modify_leftmixer(|reg| {
                reg.with_lauxmxgain(mixer.aux_gain)
                    .with_lauxlmx(mixer.aux_input)
                    .with_lbypmxgain(mixer.adc_input_gain)
                    .with_lbyplmx(mixer.adc_input)
                    .with_ldaclmx(mixer.dac_input)
            })
            .await?;
        }

        // TODO: remove after testing clock!!
        // Internal ADC -> DAC Loopback
        // Test routing ADC output to DAC input
        // codec
        //     .modify_companding(|reg| reg.with_addap(true))
        //     .await?;

        // Configure the aux1 output mixer.
        if let Some(aux1) = config.aux1_output {
            // Connect the left output mixer to the aux1 out.
            //
            // AUX1 can only connect to LMIX or RMIX.
            self.modify_aux1mixer(|reg| {
                reg.with_auxout1mt(aux1.muted)
                    .with_aux1half(aux1.attenuation_6_0db)
                    .with_lmixaux1(aux1.left_mixer_input)
                    .with_ldacaux1(aux1.left_dac_input)
                    .with_radcaux1(aux1.right_adc_mixer_input)
                    .with_rmixaux1(aux1.right_mixer_input)
                    .with_rdacaux1(aux1.right_dac_input)
            })
            .await?;
        }

        // Configure the aux2 output mixer.
        if let Some(aux2) = config.aux2_output {
            // Connect the left output mixer to the aux2 out.
            //
            // AUX2 can only connect to LMIX but not RMIX.
            self.modify_aux2mixer(|reg| {
                reg.with_auxout2mt(aux2.muted)
                    .with_aux1mix_2(aux2.aux1_interconnect_input)
                    .with_ladcaux2(aux2.left_adc_mix_input)
                    .with_lmixaux2(aux2.left_mixer_input)
                    .with_ldacaux2(aux2.left_dac_input)
            })
            .await?;
        }

        // Register 4
        //
        // Configure the audio format
        self.modify_audiointerface(|reg| {
            reg.with_wlen(config.format.word_length) // 16, 24, 32
                .with_aifmt(config.format.data_format) // i2s, pcm, tdm
        })
        .await?;

        Ok(())
    }

    /// Convenince method that configures the
    /// internal MCLK and PLL based on the
    /// required sample rate and input clock.
    pub async fn configure_pll<DELAY: DelayNs>(
        &mut self,
        config: ClockConfig,
        mut delay: DELAY,
    ) -> Result<(), ClockError<I2C::Error>> {
        // First, ensure the PLL is powered down.
        self.modify_powermanagement1(|reg| {
            reg.with_pllen(false) // Ensure the PLL is disabled.
        })
        .await?;

        // PLL is only required if the master frequency doesn't match
        // the required frequency for the desired sample rate.
        //
        // Short-circuit early if the PLL is not required, and ensure
        // that the external clock input is used instead.
        if (config.sample_rate * 256.0) == config.mclk {
            // Register 6
            //
            // Set MCLK (pin#11) as a master clock input instead of PLL.
            self.modify_clockcontrol1(|reg| {
                reg.with_clkm(false) // Internal PLL is disabled
                    .with_mclksel(MasterClockSourceScaling::Divide1) // divide PLL before MCLK
                    .with_clkioen(false) // fs and bclk are inputs
            })
            .await?;

            return Ok(());
        }

        // Calculate the codec master clock divisor and PLL for the
        // provided input clock frequency and desired sample rate.
        let clock = clock::calculate_pll(config.mclk, config.sample_rate)
            .map_err(|e| ClockError::PLLError(e))?;

        let mclk_div: MasterClockSourceScaling = clock
            .mclk_div
            .try_into()
            .map_err(|_| ClockError::MCLKDividerInvalid)?;

        // Register 6
        //
        // Set PLL as a internal master clock input, configure
        // the clock pins (BCLK and FS) in slave mode, and set
        // the desired MCLK divider.
        self.modify_clockcontrol1(|reg| {
            reg.with_clkm(true) // Internal PLL is used
                .with_mclksel(mclk_div)
                .with_clkioen(false) // fs and bclk are inputs
        })
        .await?;

        // Configure the PLL integer component and mclk divide by 2.
        self.modify_plln(|reg| reg.with_pllmclk(clock.pll_mclk).with_plln(clock.pll_int))
            .await?;

        // Configure the fractional PLL portion.
        self.modify_pllk1(|reg| reg.with_pllk(clock.pll_k1)).await?;
        self.modify_pllk2(|reg| reg.with_pllk(clock.pll_k2)).await?;
        self.modify_pllk3(|reg| reg.with_pllk(clock.pll_k3)).await?;

        // Enable the PLL.
        self.modify_powermanagement1(|reg| reg.with_pllen(true))
            .await?;

        // Wait for the PLL to stabalize.
        delay.delay_ms(150).await;

        Ok(())
    }
}
