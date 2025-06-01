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

        // TODO: Figure out this
        // Time taken in seconds from the ADC reading of Vout, the calculation of the control function and to setting the DAC value
        let t_adc_sample_to_dac_out = 0.0;

        let steady_state_duty = V_OUT / V_IN; // Assuming zero Rds(on) and Rdc drops
        let inv_steady_state_duty = 1.0 - steady_state_duty;

        // S_n
        let inductor_current_up_slope = ((V_IN - V_OUT) * CURRENT_SENSE_GAIN) / L_INDUCTOR; // volts/second

        // m_c
        let slope_compensation_factor = (1.0 + PI / 2.0) / (PI * inv_steady_state_duty);

        // m_c
        //let slope_compensation_factor = 1.0 + dac_down_slope / inductor_current_up_slope;

        // S_e
        let dac_down_slope = -(slope_compensation_factor - 1.0) * inductor_current_up_slope; // volts/second

        //

        let q_inv_no_pi = slope_compensation_factor * inv_steady_state_duty - 0.5;

        // This turns out to be 1.0
        let q_inv = q_inv_no_pi * PI;

        // st+ba
        let h_dc = R_LOAD / CURRENT_SENSE_GAIN * 1.0
            / (1.0 + (q_inv_no_pi * R_LOAD * T_SW_PERIOD / (PI * L_INDUCTOR)));

        let ohmega_p1 =
            (1.0 / (R_LOAD * C_OUT)) + (q_inv_no_pi * T_SW_PERIOD / (L_INDUCTOR * C_OUT));

        const OHMEGA_ESR: f64 = 1.0 / C_OUT * R_ESR_OUT_CAP;

        let h_p = |s: Complex| (1.0 + s / OHMEGA_ESR) / (1.0 + s / ohmega_p1);

        // st+ba
        let ohmega_n = 0.5 * F_SW; // F_SW in Hz, or PI * F_SW with F_SW in rad/s // TODO

        // st+ba
        // High frequency transfer function
        let h_h = |s: Complex| 1.0 / (s * s / (ohmega_n * ohmega_n) + s * q_inv / ohmega_n + 1.0);

        let h_dc = 8.367313080152826;
        let ohmega_p1 = 133817.56250707916;
        //const OHMEGA_ESR: f64 = 97.4025974025974;
        let ohmega_n = 3141592.653589793;
        let q_inv = 6.141592653589793;

        let h_p = |s: Complex| (1.0 + s / 97.4025974025974) / (1.0 + s / 133817.56250707916);
        let h_h = |s: Complex| {
            1.0 / (s * s / (3141592.653589793 * 3141592.653589793)
                + s * 6.141592653589793 / 3141592.653589793
                + 1.0)
        };

        let h_ctrl_to_output = |s| h_h(s) * h_p(s) * h_dc;

        //------------------------

        // Crossover frequency
        // TODO: Is this a good value?
        let f_x = F_SW / 13.0;

        // Crossover frequency as rad/s
        let ohmega_x: f64 = 2.0 * PI * f_x;

        let phase_erosion = 2.0 * PI * f_x * t_adc_sample_to_dac_out;
        assert!(phase_erosion < 90.0f64.to_radians());

        // TODO: Is this enough?
        let phase_margin: f64 = 50.0f64.to_radians() + phase_erosion;

        // Compensate for pole placed at frequency of the zero formed by capacitor and its esr
        let ohmega_n1;
        let ohmega_n2;

        {
            let sqrt = sqrt(1.0 / (ohmega_n * ohmega_n) - 2.0);
            ohmega_n1 = -0.5 * ohmega_n + sqrt;
            ohmega_n2 = -0.5 * ohmega_n - sqrt;
        }

        let phi_v = -0.5 * PI
            + phase_margin
            + atan(ohmega_x / ohmega_p1)
            + atan(ohmega_x / ohmega_n1)
            + atan(ohmega_x / ohmega_n2);

        let ohmega_cp1 = OHMEGA_ESR;
        let ohmega_cz1 = ohmega_x / tan(phi_v);

        let k1 =
            sqrt(1.0 + pow2(ohmega_x / ohmega_cz1)) / sqrt(1.0 + (ohmega_x * ohmega_x) / ohmega_p1);
        let k2 = 1.0 / sqrt(pow2(1.0 - ohmega_x / ohmega_n) + pow2(ohmega_x / ohmega_n));

        // pole at origin
        let ohmega_cp0 = ohmega_x / (h_dc * k1 * k2);

        // Compensator transfer function in the analog domain
        let h_c = |s: Complex| ohmega_cp0 / s * (1.0 + s / ohmega_cz1) / (1.0 + s / ohmega_cp1);

        /// -----
        // Control parameters for 2p2z controller
        let t_sw = T_SW_PERIOD;
        let b0 = t_sw * ohmega_cp0 * ohmega_cp1 * (2.0 + t_sw * ohmega_cz1)
            / (2.0 * (2.0 + t_sw * ohmega_cp1) * ohmega_cz1);

        let b1 = pow2(t_sw) * ohmega_cp0 * ohmega_cp1 / (2.0 + t_sw * ohmega_cp1);

        let b2 = t_sw * ohmega_cp0 * ohmega_cp1 * (-2.0 + t_sw * ohmega_cz1)
            / (2.0 * (2.0 + t_sw * ohmega_cp1) * ohmega_cz1);

        let a1 = 4.0 / (2.0 + t_sw * ohmega_cp1);
        let a2 = (-2.0 + t_sw * ohmega_cp1) / (2.0 + t_sw * ohmega_cp1);        
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
        // (x + iy) * (z + iw) =
        // = xz + xiw + iyz + iyiw = // i*i=-1
        // = xz + ixw + iyz - yw =
        // = (xz - yw) + i(xw + yz)
        Self {
            re: (self.re * rhs.re - self.im * rhs.im),
            im: (self.re * rhs.im + self.im * rhs.re),
        }
    }

    /// Returns the square of the norm, i.e. `re^2 + im^2`.
    #[inline]
    pub const fn norm_sqr(&self) -> f64 {
        // https://docs.rs/num-complex/0.4.6/src/num_complex/lib.rs.html#149
        self.re * self.re + self.im * self.im
    }

    const fn div(self, rhs: Self) -> Self {
        // https://docs.rs/num-complex/0.4.6/src/num_complex/lib.rs.html#820
        let norm_sqr = rhs.norm_sqr();
        let re = self.re * rhs.re + self.im * rhs.im;
        let im = self.im * rhs.re - self.re * rhs.im;
        Self {
            re: re / norm_sqr,
            im: im / norm_sqr,
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
