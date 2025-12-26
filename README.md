# nau88c22

A Rust embedded-hal driver for the nau88c22 audio codec.

# Example

```rust
use nau88c22_rs::Nau88c22;

fn main () {
    let i2c;
    // set up the I2C device based on your HAL.

    let codec = Nau88c22::new(i2c);
}

```