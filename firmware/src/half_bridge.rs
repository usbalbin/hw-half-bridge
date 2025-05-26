use stm32_hrtim::pac::dac1::mcr::HFSEL;
use stm32g4xx_hal::{
    dac::{self, Dac1Ch1},
    rcc::Rcc,
};

use crate::types::{Amperes, Henries, Seconds, Voltage};

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

        use core::f64::consts::PI;

        const V_IN: f64 = 48.0;
        const V_OUT: f64 = 12.0;
        const C_OUT: f64 = 2.0 * 7.7e-6; // 2 * ~7.7uF @ 12V
        const F_SW: f64 = 1e6;
        const T_SW_PERIOD: f64 = 1.0 / F_SW;
        const L_INDUCTOR: f64 = 2e-6; // 2.2 @ 0A, 2.0 at 8A, ~1.5 @ 24A
        const R_ESR_INDUCTOR: f64 = 4.08e-3; // 4.08mOhm typical
        const R_ESR_OUT_CAP: f64 = 1.5e-3; // todo
        const CURRENT_SENSE_GAIN: f64 = 66e-3; // 66mV/A
        const I_LOAD: f64 = 10.0; // 10A
        const R_LOAD: f64 = V_OUT / I_LOAD; // ohm

        let steady_state_duty = V_OUT / V_IN;
        let inv_steady_state_duty = 1.0 - steady_state_duty;

        // S_n
        let inductor_current_up_slope = (V_IN - V_OUT) * CURRENT_SENSE_GAIN; // volts/second

        // m_c
        let slope_compensation_factor = (1.0 + PI / 2.0) / (PI * steady_state_duty);

        // m_c
        //let slope_compensation_factor = 1.0 + dac_down_slope / inductor_current_up_slope;

        // S_e
        let dac_down_slope = -(slope_compensation_factor - 1.0) * inductor_current_up_slope; // volts/second

        //

        let h_dc = R_LOAD / CURRENT_SENSE_GAIN * 1.0
            / (1.0
                + (R_LOAD * T_SW_PERIOD / L_INDUCTOR)
                    * (slope_compensation_factor * inv_steady_state_duty - 0.5));

        let ohmega_p1 = (1.0 / (R_LOAD * C_OUT))
            + (T_SW_PERIOD / (L_INDUCTOR * C_OUT)
                * slope_compensation_factor
                * inv_steady_state_duty
                - 0.5);
        //let h_p_ideal = |s| 1.0 / (1.0 + s / ohmega_p1);

        const OHMEGA_ESR: f64 = 1.0 / C_OUT * R_ESR_OUT_CAP;

        //let h_cap = |s| 1.0 + s / OHMEGA_ESR;
        //let h_p = |s| h_cap(s) * h_p_ideal(s);
        // let h_p = |s| (1.0 + s / OHMEGA_ESR) / (1.0 + s / ohmega_p1);
        //(1.0 + s / OHMEGA_ESR) / (1.0 + s / ohmega_p1);

        //
        let ohmega_n = PI * F_SW;
        let q_inv = PI * (slope_compensation_factor * inv_steady_state_duty - 0.5);

        let h_h = |s: Complex| 1.0 / (s * s / (ohmega_n * ohmega_n) + s * q_inv / ohmega_n + 1.0);
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

#[derive(Debug, Clone, Copy)]
struct Complex {
    re: f64,
    im: f64,
}

impl Complex {
    pub const fn add(self, rhs: Self) -> Self {
        Self {
            re: self.re + rhs.re,
            im: self.im + rhs.im,
        }
    }

    pub const fn add_r(self, rhs: f64) -> Self {
        Self {
            re: self.re + rhs,
            im: self.im,
        }
    }

    const fn mul(self, rhs: Self) -> Self {
        Self {
            re: self.re * rhs.re,
            im: self.im * rhs.im,
        }
    }

    const fn div(self, rhs: Self) -> Self {
        Self {
            re: self.re / rhs.re,
            im: self.im / rhs.im,
        }
    }

    const fn div_r(self, rhs: f64) -> Self {
        Self {
            re: self.re / rhs,
            im: self.im / rhs,
        }
    }

    const fn mul_r(self, rhs: f64) -> Self {
        Self {
            re: self.re * rhs,
            im: self.im * rhs,
        }
    }
}

impl core::ops::Add for Complex {
    type Output = Complex;

    fn add(self, rhs: Complex) -> Self::Output {
        Complex {
            re: self.re + rhs.re,
            im: self.im + rhs.im,
        }
    }
}

impl core::ops::Add<f64> for Complex {
    type Output = Complex;

    fn add(self, rhs: f64) -> Self::Output {
        Complex {
            re: self.re + rhs,
            im: self.im,
        }
    }
}

impl core::ops::Add<Complex> for f64 {
    type Output = Complex;

    fn add(self, rhs: Complex) -> Self::Output {
        Complex {
            re: self + rhs.re,
            im: rhs.im,
        }
    }
}

impl core::ops::Mul for Complex {
    type Output = Complex;

    fn mul(self, rhs: Self) -> Self::Output {
        self.mul(rhs)
    }
}

impl core::ops::Mul<f64> for Complex {
    type Output = Complex;

    fn mul(self, rhs: f64) -> Self::Output {
        self.mul_r(rhs)
    }
}

impl core::ops::Mul<Complex> for f64 {
    type Output = Complex;

    fn mul(self, rhs: Complex) -> Self::Output {
        Complex {
            re: self * rhs.re,
            im: self * rhs.im,
        }
    }
}

impl core::ops::Div for Complex {
    type Output = Complex;

    fn div(self, rhs: Self) -> Self::Output {
        self.div(rhs)
    }
}

impl core::ops::Div<f64> for Complex {
    type Output = Complex;

    fn div(self, rhs: f64) -> Self::Output {
        self.div_r(rhs)
    }
}

impl core::ops::Div<Complex> for f64 {
    type Output = Complex;

    fn div(self, rhs: Complex) -> Self::Output {
        Complex {
            re: self / rhs.re,
            im: self / rhs.im,
        }
    }
}
