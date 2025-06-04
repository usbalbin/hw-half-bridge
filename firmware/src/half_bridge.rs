use stm32_hrtim::pac::dac1::mcr::HFSEL;
use stm32g4xx_hal::{
    dac::{self, Dac1Ch1},
    rcc::Rcc,
};

use crate::{
    math::{atan, pow2, sqrt, tan},
    types::{Amperes, Henries, Seconds, Voltage},
};

pub struct Input {
    /// Voltage at LO-side
    ///
    /// This is typically the
    /// * **output** when running in **buck** mode, or the
    /// * **input** when running in **boost** mode
    v_lo: f32,

    /// Voltage at HI-side
    ///
    /// This is typically the
    /// * **input** when running in **buck** mode, or the
    /// * **output** when running in **boost** mode
    v_hi: f32,

    /// The instantanious current measured in the inductor
    i: f32,
}

/*fn slope_peak_to_peak(
    rcc: &mut Rcc,
    vin: Voltage,
    vout: Voltage,
    dac: Dac1Ch1<M_INT_SIG>,
) -> Voltage {
    let t_sw = Seconds(1.0e-6);
    let l_inv = 1.0 / Henries(1.76e-6);
    let current_sense_gain = Voltage(0.066) / Amperes(1.0);
    let duty = vout / vin;

    let f_ahb = rcc.clocks.ahb_clk;
    let hfsel = 0b10; // <-- Setting of DAC

    let slope_p2p = (0.18 - duty) * current_sense_gain * t_sw * l_inv * vin;
    let ahb_cycles = match dac::hfsel(rcc) {
        HFSEL::Disabled => 3,
        HFSEL::More80mhz => 5,
        HFSEL::More160mhz => 7,
    };

    let t_dac_sample = ahb_cycles as f32 / f_ahb;
    let steps_per_sw_period = (t_sw / t_dac_sample).floor();
    let step_size = slope_p2p / steps_per_sw_period;
    let step_size = (-step_size).to_dac_value();
    dac.enable_generator(dac::GeneratorConfig::sawtooth(amplitude), rcc);
}*/

pub struct HalfBridge<P: embedded_hal::pwm::SetDutyCycle> {
    pwm_control: P,
}

impl<P: embedded_hal::pwm::SetDutyCycle> HalfBridge<P> {
    /// vout = vin * d
    //#[cfg(feature = "current-mode")]
    pub const fn update_buck(&mut self, target_u_lo: f32, measured: Input) {
        // https://www.biricha.com/articles/step-by-step-design-guide-for-digital-peak-current-mode-control-a-single-chip-solution
        // https://www.st.com/en/embedded-software/x-cube-dpower.html
        // https://www.ti.com/lit/an/sprabe7a/sprabe7a.pdf?ts=1723931480534
        // https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/171/Presentation_5F002D005F00_Mr._5F00_Ali_5F00_Shirsavar.pdf
        // https://www.st.com/resource/en/application_note/an5497-introduction-to-the-buck-current-mode-with-the-bg474edpow1-discovery-kit-stmicroelectronics.pdf
        // https://centaur.reading.ac.uk/31751/1/Microcontroller%20Based%20Peak%20Current%20Mode%20Control%20Using%20Digital%20Slope%20Compensation%20-%20Hallworth%202012.pdf

        
    }

    /// vout = vin * d
    #[cfg(feature = "voltage-mode")]
    pub fn update_buck(&mut self, target_u_lo: f32, measured: Input) {}

    /// vout = vin/(1-d)
    #[cfg(feature = "current-mode")]
    pub fn update_boost(&mut self, target_u_hi: f32, measured: Input) {}

    /// vout = vin/(1-d)
    #[cfg(feature = "voltage-mode")]
    pub fn update_boost(&mut self, target_u_hi: f32, measured: Input) {}
}