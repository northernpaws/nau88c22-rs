# NAU88C22-rs

A Rust embedded-hal driver for the [Nuvoton NAU88C22](https://www.nuvoton.com/products/smart-home-audio/audio-converters/audio-codec-series/nau88c22yg/) audio codec.

[**Data Sheet**](https://www.nuvoton.com/export/resource-files/en-us--DS_NAU88C22_DataSheet_EN_Rev2.2.pdf)

# Example

```rust
use nau88c22_rs::Nau88c22;

fn main () {
    let i2c;
    // set up the I2C device based on your HAL.

    let codec = Nau88c22yg::new(i2c);
}

```
