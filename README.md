# NAU88C22-rs

A Rust `embedded-hal-async` based driver for the [Nuvoton NAU88C22](https://www.nuvoton.com/products/smart-home-audio/audio-converters/audio-codec-series/nau88c22yg/) 24-bit stereo audio codec.

The NAU88C22 audio codec includes drivers for speaker, headphone, differential and stereo line outputs, along with integrated preamps for stereo differential microphones. Along with having a low ADC SNR of 89dB @ 0dB gain and a DAC SNR of 89dB @ 0dB gain, and it's low price of $1CAD per unit makes it a very flexible and cost effective audio codec for stereo audio applications.

[**Data Sheet**](https://www.nuvoton.com/export/resource-files/en-us--DS_NAU88C22_DataSheet_EN_Rev2.2.pdf)

[**docs.rs Link**](https://docs.rs/nau88c22-rs/0.1.0/nau88c22_rs/)

# Example

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
