use core::f64::consts::PI;

#[cfg(feature = "hardware")]
use defmt::println;

use crate::math::{atan, pow2, sqrt, tan};

pub trait Scalar:
    Sized + Copy + core::ops::Add<Self, Output = Self> + core::ops::Mul<Self, Output = Self>
{
    const ZERO: Self;
    fn from_f32(f: f32) -> Self;
}

impl Scalar for f32 {
    const ZERO: Self = 0.0;
    fn from_f32(f: f32) -> Self {
        f as _
    }
}

macro_rules! impl_scalar {
    ($($t:ident),*) => {$(
        impl Scalar for fixed::types::$t {
            const ZERO: Self = Self::ZERO;
            fn from_f32(f: f32) -> Self {
                Self::from_num(f)
            }
        }
    )*};
}

impl_scalar!(
    I32F0, I31F1, I30F2, I29F3, I28F4, I27F5, I26F6, I25F7, I24F8, I23F9, I22F10, I21F11, I20F12,
    I19F13, I18F14, I17F15, I16F16, I15F17, I14F18, I13F19, I12F20, I11F21, I10F22, I9F23, I8F24,
    I7F25, I6F26, I5F27, I4F28, I3F29, I2F30, I1F31, I0F32
);
impl_scalar!(
    I0F16, I1F15, I2F14, I3F13, I4F12, I5F11, I6F10, I7F9, I8F8, I9F7, I10F6, I11F5, I12F4, I13F3,
    I14F2, I15F1, I16F0
);

#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct TwoPoleTwoZeroParams<T> {
    pub a1: T,
    pub a2: T,

    pub b0: T,
    pub b1: T,
    pub b2: T,
}

impl<T: Scalar> TwoPoleTwoZeroParams<T> {
    #[inline(always)]
    pub const fn to_controller(self) -> TwoPoleTwoZero<T> {
        TwoPoleTwoZero {
            params: self,
            outputs: [T::ZERO; _],
            errors: [T::ZERO; _],
        }
    }
}

impl TwoPoleTwoZeroParams<f32> {
    pub fn to_t<T: Scalar>(self) -> TwoPoleTwoZeroParams<T> {
        let Self { a1, a2, b0, b1, b2 } = self;
        TwoPoleTwoZeroParams {
            a1: T::from_f32(a1),
            a2: T::from_f32(a2),
            b0: T::from_f32(b0),
            b1: T::from_f32(b1),
            b2: T::from_f32(b2),
        }
    }
}

pub struct TwoPoleTwoZero<T> {
    params: TwoPoleTwoZeroParams<T>,

    /// History of outputs with newest value at index 0
    outputs: [T; 2],

    /// History of errors with newest value at index 0
    errors: [T; 2],
}

impl<T: Scalar> TwoPoleTwoZero<T> {
    #[inline(always)]
    pub fn update(&mut self, error: T) -> T {
        let output = self.params.a1 * self.outputs[0]
            + self.params.a2 * self.outputs[1]
            + self.params.b0 * error
            + self.params.b1 * self.errors[0]
            + self.params.b2 * self.errors[1];
        self.outputs.rotate_right(1);
        self.outputs[0] = output;

        self.errors.rotate_right(1);
        self.errors[0] = error;

        output
    }

    pub fn reset(&mut self) {
        *self = self.params.to_controller();
    }
}

pub struct ParametersBuck {
    pub v_in: f64,
    pub v_out: f64,
    pub v_diode: f64,
    pub c_out: f64,
    pub f_sw: f64,
    pub l_inductor: f64,
    pub r_esr_out_cap: f64,
    pub current_sense_gain: f64,
    pub i_load: f64,

    /// TODO: Figure out this
    /// Time taken in seconds from the ADC reading of Vout, the calculation of the control function and to setting the DAC value
    pub phase_margin: PhaseMargin,
}

#[derive(Copy, Clone, Debug, defmt::Format)]
pub struct DacSettings {
    /// DAC slope in Volts/second
    pub dac_slope: f64,

    /// Voltage peak to peak in Volts
    vpp: f64,
}

macro_rules! p {
    ($val:expr, $val2:expr) => {
        #[cfg(false)]
        eprintln!(
            "[{}:{}:{}] {} = {:#?}, {}",
            file!(),
            line!(),
            column!(),
            stringify!($val),
            &$val,
            $val2
        );
    };
}

impl ParametersBuck {
    pub const fn to_transfer_function(self) -> (TransferFunction, DacSettings) {
        //
        // https://centaur.reading.ac.uk/31751/1/Microcontroller%20Based%20Peak%20Current%20Mode%20Control%20Using%20Digital%20Slope%20Compensation%20-%20Hallworth%202012.pdf
        //

        // https://www.biricha.com/articles/step-by-step-design-guide-for-digital-peak-current-mode-control-a-single-chip-solution
        // https://www.st.com/en/embedded-software/x-cube-dpower.html
        // https://www.ti.com/lit/an/sprabe7a/sprabe7a.pdf?ts=1723931480534
        // https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/171/Presentation_5F002D005F00_Mr._5F00_Ali_5F00_Shirsavar.pdf
        // https://www.st.com/resource/en/application_note/an5497-introduction-to-the-buck-current-mode-with-the-bg474edpow1-discovery-kit-stmicroelectronics.pdf

        use core::f64::consts::PI;

        let ParametersBuck {
            v_in,
            v_out,
            c_out,
            v_diode: diode_drop,
            f_sw,
            l_inductor,
            r_esr_out_cap,
            current_sense_gain,
            i_load,
            phase_margin,
        } = self;

        p!(v_in, "16");
        p!(v_out, "8");
        p!(i_load, "2");
        p!(c_out, "440e-6");
        p!(l_inductor, "22e-6");
        p!(current_sense_gain, "0.48");
        p!(r_esr_out_cap, "31e-3");

        p!(f_sw, "200e3");

        let t_sw = 1.0 / f_sw;
        let r_load = v_out / i_load; // ohm

        let steady_state_duty = (v_out + diode_drop) / v_in; // Assuming zero Rds(on) and Rdc drops
        let inv_steady_state_duty = 1.0 - steady_state_duty;

        p!(steady_state_duty, "0.5375");

        // S_n
        let inductor_current_up_slope =
            ((v_in - v_out - diode_drop) * current_sense_gain) / l_inductor; // volts/second

        // m_c
        let slope_compensation_factor = (1.0 + PI / 2.0) / (PI * inv_steady_state_duty);

        // S_e
        let dac_down_slope = -(slope_compensation_factor - 1.0) * inductor_current_up_slope; // volts/second

        let vpp = -dac_down_slope * t_sw; // V_PP is positive (paper eq. 6: V_PP = S_E * T_S)

        //
        let q_inv_no_pi = slope_compensation_factor * inv_steady_state_duty - 0.5;

        // This turns out to be 1.0
        let q_inv = q_inv_no_pi * PI;

        // st+ba
        let h_dc = r_load / current_sense_gain * 1.0
            / (1.0 + (q_inv_no_pi * r_load * t_sw / (l_inductor)));

        let ohmega_p1 = (1.0 / (r_load * c_out)) + (q_inv_no_pi * t_sw / (l_inductor * c_out));
        let ohmega_esr = 1.0 / (c_out * r_esr_out_cap);
        p!(ohmega_p1, "732.6");
        p!(ohmega_esr, "aka ωCP1 (and ωZ1 ?) 73 310");
        // let h_ctrl_to_output = |s| h_h(s) * h_p(s) * h_dc;

        //------------------------
        p!(h_dc, "6.4631");
        (
            TransferFunction {
                f_sw,
                phase_margin,

                q_inv,

                ohmega_p1,
                ohmega_esr,
                h_dc,
            },
            DacSettings {
                dac_slope: dac_down_slope,
                vpp,
            },
        )
    }
}

#[derive(Debug, defmt::Format, Copy, Clone)]
pub enum PhaseMargin {
    Manual {
        phase_margin: f64,
    },
    Calculated {
        t_adc: f64,
        t_processing: f64,
        t_dac: f64,
    },
}

#[derive(Debug, defmt::Format)]
pub struct TransferFunction {
    f_sw: f64,
    phase_margin: PhaseMargin,

    q_inv: f64,

    ohmega_p1: f64,
    ohmega_esr: f64,
    h_dc: f64,
}

impl TransferFunction {
    pub const fn to_2p2z(self) -> TwoPoleTwoZeroParams<f32> {
        let TransferFunction {
            f_sw,
            phase_margin,
            q_inv: _,
            ohmega_p1,
            ohmega_esr,
            h_dc,
        } = self;

        let ohmega_n = self.ohmega_n();
        p!(ohmega_n, "628 300");

        // Crossover frequency
        // TODO: Is this a good value?
        let f_x = f_sw / 13.33333333333333333333;
        //p!(f_x, "15000");

        //println!("----------------------------------");
        //println!("----------------------------------");
        //println!("----------------------------------");

        // Crossover frequency as rad/s
        let ohmega_x = 2.0 * PI * f_x;
        p!(ohmega_x, "?");

        let phase_margin = match phase_margin {
            PhaseMargin::Manual { phase_margin } => {
                phase_margin
            },
            PhaseMargin::Calculated { t_adc, t_processing, t_dac } => {
                let phase_erosion = 2.0 * PI * f_x * (t_adc + t_processing + t_dac);
                // TODO: Is 50 enough?
                50.0f64.to_radians() + phase_erosion
            },
        };


        //#[cfg(not(feature = "hardware"))]
        //assert!(phase_erosion < 90.0f64.to_radians());

        // ChatGPT's suggestion
        let r = ohmega_x / ohmega_n;
        let complex_pole_pair = atan(r / (1.0 - pow2(r)));
        //dbg!(complex_pole_pair);

        // p1 ok
        let phi_v = -0.5 * PI + phase_margin + atan(ohmega_x / ohmega_p1) + complex_pole_pair;
        //dbg!(ohmega_x);

        let ohmega_cp1 = ohmega_esr; // eq. 8
        let ohmega_cz1 = ohmega_x / tan(phi_v); // eq. 10
        p!(ohmega_cp1, "73_310");
        p!(ohmega_cz1, "11_110");

        let k1 = sqrt((1.0 + pow2(ohmega_x / ohmega_cz1)) / (1.0 + pow2(ohmega_x / ohmega_p1))); // eq. 18
        let k2 = sqrt(1.0 / (pow2(1.0 - pow2(ohmega_x / ohmega_n)) + pow2(ohmega_x / ohmega_n))); // eq. 19 (Q_C=1)

        // pole at origin, eq. 16
        let ohmega_cp0 = ohmega_x / (h_dc * k1 * k2);
        p!(ohmega_cp0, "217_100");

        // Compensator transfer function in the analog domain
        // let h_c = |s: Complex| ohmega_cp0 / s * (1.0 + s / ohmega_cz1) / (1.0 + s / ohmega_cp1);

        // ----------

        // Control parameters for 2p2z controller
        let t_sw = 1.0 / f_sw;
        let b0 = t_sw * ohmega_cp0 * ohmega_cp1 * (2.0 + t_sw * ohmega_cz1)
            / (2.0 * (2.0 + t_sw * ohmega_cp1) * ohmega_cz1);

        let b1 = pow2(t_sw) * ohmega_cp0 * ohmega_cp1 / (2.0 + t_sw * ohmega_cp1);

        let b2 = t_sw * ohmega_cp0 * ohmega_cp1 * (-2.0 + t_sw * ohmega_cz1)
            / (2.0 * (2.0 + t_sw * ohmega_cp1) * ohmega_cz1);

        let a1 = 4.0 / (2.0 + t_sw * ohmega_cp1);
        let a2 = (-2.0 + t_sw * ohmega_cp1) / (2.0 + t_sw * ohmega_cp1);

        TwoPoleTwoZeroParams {
            a1: a1 as _,
            a2: a2 as _,
            b0: b0 as _,
            b1: b1 as _,
            b2: b2 as _,
        }
    }

    pub const fn ohmega_n(&self) -> f64 {
        PI * self.f_sw // F_SW in Hz, or PI * F_SW with F_SW in rad/s // TODO
    }

    pub fn print_h_p_transfer_func(&self) {
        let ohmega_esr = self.ohmega_esr;
        let ohmega_p1 = self.ohmega_p1;

        //let h_p = |s: Complex| (1.0 + s / ohmega_esr) / (1.0 + s / ohmega_p1);

        #[cfg(feature = "hardware")]
        println!("(1.0 + s / {}) / (1.0 + s / {})", ohmega_esr, ohmega_p1);

        #[cfg(not(feature = "hardware"))]
        println!(
            "(1.0 + s / {:.2}) / (1.0 + s / {:.2})",
            ohmega_esr, ohmega_p1
        );
    }

    pub fn print_high_freq_transfer_func(&self) {
        let ohmega_n = self.ohmega_n();
        let q_inv = self.q_inv;
        // High frequency transfer function
        // let h_h = |s: Complex| 1.0 / (s * s / (ohmega_n * ohmega_n) + s * q_inv / ohmega_n + 1.0);

        #[cfg(feature = "hardware")]
        println!(
            "1.0 / (s * s / {}^2 + s * {} / {} + 1.0)",
            ohmega_n, q_inv, ohmega_n
        );

        #[cfg(not(feature = "hardware"))]
        println!(
            "1.0 / (s * s / {:.2}^2 + s * {:.2} / {:.2} + 1.0)",
            ohmega_n, q_inv, ohmega_n
        );
    }

    pub fn print_dc_gain(&self) {
        #[cfg(feature = "hardware")]
        println!("{}", self.h_dc);

        #[cfg(not(feature = "hardware"))]
        println!("{:.2}", self.h_dc);
    }
}
