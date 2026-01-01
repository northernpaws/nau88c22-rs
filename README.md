# NAU88C22-rs

A Rust `embedded-hal-async` based driver for the [Nuvoton NAU88C22](https://www.nuvoton.com/products/smart-home-audio/audio-converters/audio-codec-series/nau88c22yg/) 24-bit stereo audio codec.

The NAU88C22 audio codec includes drivers for speaker, headphone, differential and stereo line outputs, along with integrated preamps for stereo differential microphones. Along with having a low ADC SNR of 89dB @ 0dB gain and a DAC SNR of 89dB @ 0dB gain, and it's low price of $1CAD per unit makes it a very flexible and cost effective audio codec for stereo audio applications.

Uses `libm` to avoid requiring nightly and `core_intrinsics`.

[**Data Sheet**](https://www.nuvoton.com/export/resource-files/en-us--DS_NAU88C22_DataSheet_EN_Rev2.2.pdf)

[**docs.rs Link**](https://docs.rs/nau88c22-rs/0.1.0/nau88c22_rs/)

## Codec Clock/PLL Configuration

I've copied the calculations from the manual for initializing the codec's IMCLK and PLL to work with any desired input MCLK frequency and put them in the `clock` module. These have been tested on my boards to output the correct values for a range of source clocks with a desired 48kHz sampling rate.

```rust
// Calculate the codec master clock divisor and PLL for the
// provided input clock frequency and desired sample rate.
let clock = clock::calculate_pll(config.mclk, config.sample_rate).unwrap();
// clock.pll_n, clock.pll_k1, etc..
```

These can be calculated and directly applied to the codec using the `configure_pll` convenience method:
```rust
codec.configure_pll(ClockConfig{
    // The frequency of the MCLK signal applied to the codec on pin 11.
    //
    // If this is matched closely to your desired sample rate frequency
    // (sample_rate*256) then you won't even need to configure the PLL.
    // 
    // I.e., for 48kHz sample rate you'd, want a clock at or slightly under
    // 12.288MHz (codec can't handle higher without audio quality issues).
    mclk: 2_884_000,

    // Desired sample rate.
    //
    // Note that if you can't exactly match the sample rate on your digital audio
    // output, set this to the actual skewed sample rate to avoid sound glitches.
    sample_rate: 48000,
}).await?;
```

## Example

```rust
use nau88c22_rs::Nau88c22;

async fn main () {
    // Set up the I2C device based on your HAL.
    let i2c;

    // Initialize the codec using the I2C device.
    let codec = Nau88c22yg::new(i2c);

    // Software reset the codec to known default register values.
    codec.reset().await.unwrap();
}
```

## License

This crate is licensed under the MIT license.
