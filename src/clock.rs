use core::fmt::Display;

/// Shortcut of the known mclk divisors for the codec.
const NAU_MCLKSEL: [f32; 8] = [1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0];

/// Returned if a suitable clock divisor and
/// PLL config could not be calculated.
#[derive(Debug)]
pub enum CodecClockError {
    SampleRateTooLow,
    SampleRateTooHigh,
    MCLKTooHigh,
    MCLKTooLow,
    UnresolvablePLL,
}

impl Display for CodecClockError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        todo!()
    }
}

pub struct ClockParameters {
    // R6[7:5](MCLKSEL)
    pub mclk_div: u8,
    /// R36[4](PLLMCLK)
    pub pll_mclk: bool,
    /// R36[3:0](PLLN)
    pub pll_int: u8,
    /// R37(PLLK[23:18])
    pub pll_k1: u8,
    /// R38(PLLK[17:9])
    pub pll_k2: u16,
    /// R38(PLLK[8:0])
    pub pll_k3: u16,
}

/// Calculates a suitable set of clock control parameters for the NAU88C22
/// given the provided master block and sample rate inputs.
///
///  "IMCLK should not exceed 12.288MHz under any condition without enabling PLL49MOUT bit R72[2].
///   IMCLK is output from the Master Clock Prescaler. The prescaler reduces by an integer division
///   factor the input frequency input clock. The source of this input frequency clock is either
///   the external MCLK pin, or the output from the internal PLL Block."
///
///  "the optimum PLL oscillator frequency is in the range between 90MHz and
///   100MHz, and thus, it is best to keep f2 within this range."
pub fn calculate_pll(
    mckl_in: f32,
    sample_rate_hz: f32,
) -> Result<ClockParameters, CodecClockError> {
    // 11.999kHz
    if sample_rate_hz < 11999.0 {
        return Err(CodecClockError::SampleRateTooLow);
    }

    // 48kHz
    if sample_rate_hz > 48001.0 {
        return Err(CodecClockError::SampleRateTooHigh);
    }

    // First, derrive the desired IMCLK frequency
    // which can be calculated as 256 * sample_Rate.
    //
    // IMCLK = desired Master Clock = (256)*(desired codec sample rate)
    //
    // 12_288_000hz for 48k
    let frq_imclk = 256.0 * sample_rate_hz;

    // Check that the MCLK being provided to the codec is within the
    // workable PLL reference frequency range of 8MHz to 33MHz.
    if mckl_in > 33_000_000.0 {
        return Err(CodecClockError::MCLKTooHigh);
    }

    if mckl_in < 9_000_000.0 {
        return Err(CodecClockError::MCLKTooLow);
    }

    let mut mclkseldiv: u8 = 0;

    // f2 = (4)*(P)(IMCLK) or (2)*(P)(IMCLK) when PLL49MOUT bit R72[2] = 1
    // > where P is the Master Clock Prescale integer value
    //
    // optimal f2: 90MHz< f2 <100MHz
    let mut pll_f2: f32 = 0.0; // PLL Oscillator f2

    // Try to find a divisor that hits the optimal f2 frequency.
    for i in 0..8 {
        pll_f2 = 4.0 * NAU_MCLKSEL[i] * frq_imclk;
        if (pll_f2 >= 80000000.0) && (pll_f2 < 110000000.0) {
            mclkseldiv = i as u8;
            break;
        };
    }

    if mclkseldiv == 0 {
        for i in 0..8 {
            pll_f2 = 2.0 * NAU_MCLKSEL[i] * frq_imclk;
            if (pll_f2 >= 80000000.0) && (pll_f2 < 110000000.0) {
                mclkseldiv = i as u8;
                break;
            };
        }
    };

    // f1 = (MCLK)/(D)
    //  D = PLL Prescale factor of 1, or 2
    //  MCLK = is the frequency at the MCLK pin
    let pll_f1: f32 = mckl_in; // input MCLK frequency

    let mut pres_mckl: u8 = 3;

    // Fractional frequency multiplication factor for the PLL.
    //
    // R = f2/f1 = xy.abcdefgh
    // Fractional multipler R=f2/f1
    let mut pll_r: f32 = pll_f2 / pll_f1;

    if (pll_r > 6.0) && (pll_r < 13.0) {
        pres_mckl = 1;
    } else {
        pll_r = pll_f2 / (pll_f1 * 2.0);
        pres_mckl = 2;
    };

    // Truncated integer portion of the R value, and limited
    // to decimal value 6, 7, 8, 9, 10, 11, or 12.
    //
    // N = xy
    let mut integer_portion_n: u8 = 0;

    // Integer fractional portion of the PLL.
    let mut pll_frac_int: u32 = 0;

    // Check that the fractional multiplier is within the supported PLL clock bounds,
    // and then calculate the derrived integer (H) and fractional (K) portions.
    if (pll_r > 6.0) && (pll_r < 13.0) {
        // Get the intergel part by flooring the fractional multiplier.
        integer_portion_n = libm::floorf(pll_r) as u8; // N = xy

        // TODO check that N is 6, 7, 8, 9, 10, 11, or 12

        // Derrive the fractional portion K from the floored intger portion.
        let mut pll: f32 = 0.0;
        pll = pll_r - (integer_portion_n as f32); // (0.abcdefgh)

        // Multiply by 2^24 to derrive the 24-bit binary fractional value.
        pll *= 16777216.0; // K = (2^24)*(0.abcdefgh)

        // Cast to u32 to round to nearest whole integer.
        pll_frac_int = pll as u32;
    } else {
        return Err(CodecClockError::UnresolvablePLL);
    }

    // Convert the fractional binary value into it's bit
    // segments to write to the corrosponding registers.
    let pll_k1: u8 = ((pll_frac_int >> 18) & 0x3F) as u8; // Highest order 6-bits of 24-bit fraction
    let pll_k2: u16 = ((pll_frac_int >> 9) & 0x1FF) as u16; // Middle 9-bits of 24-bit fraction
    let pll_k3: u16 = ((pll_frac_int >> 0) & 0x1FF) as u16; // Lowest order 9-bits of 24-bit fraction

    Ok(ClockParameters {
        mclk_div: mclkseldiv,
        pll_mclk: (pres_mckl - 1) > 0,
        pll_int: integer_portion_n,
        pll_k1,
        pll_k2,
        pll_k3,
    })
}
