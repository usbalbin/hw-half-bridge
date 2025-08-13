use core::f64::consts::PI;

#[cfg(feature = "hardware")]
use defmt::{assert, dbg, println, println as eprintln};

use crate::math::{atan, pow2, sqrt, tan, Complex};

#[derive(Debug, defmt::Format)]
pub struct TwoPoleTwoZero {
    a1: f32,
    a2: f32,

    b0: f32,
    b1: f32,
    b2: f32,
}

const P: ParametersBuck = ParametersBuck {
    v_in: 16.0,
    v_out: 8.0,
    c_out: 440.0e-6, // 2 * ~7.7uF @ 12V
    f_sw: 2e5,
    l_inductor: 22e-6, // 2.2 @ 0A, 2.0 at 8A, ~1.5 @ 24A
    //r_esr_inductor: 4.08e-3,   // 4.08mOhm typical
    r_esr_out_cap: 31e-3,      // todo
    current_sense_gain: 66e-3, // 66mV/A
    i_load: 2.0,               // 10A
};

pub struct ParametersBuck {
    pub v_in: f64,
    pub v_out: f64,
    pub c_out: f64,
    pub f_sw: f64,
    pub l_inductor: f64,
    pub r_esr_out_cap: f64,
    pub current_sense_gain: f64,
    pub i_load: f64,
}

pub struct DacSettings {
    /// DAC slope in Volts/second
    dac_down_slope: f64,
}

macro_rules! p {
    ($val:expr, $val2:expr) => {
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
    pub fn to_transfer_function(self) -> (TransferFunction, DacSettings) {
        // https://www.biricha.com/articles/step-by-step-design-guide-for-digital-peak-current-mode-control-a-single-chip-solution
        // https://www.st.com/en/embedded-software/x-cube-dpower.html
        // https://www.ti.com/lit/an/sprabe7a/sprabe7a.pdf?ts=1723931480534
        // https://e2e.ti.com/cfs-file/__key/communityserver-discussions-components-files/171/Presentation_5F002D005F00_Mr._5F00_Ali_5F00_Shirsavar.pdf
        // https://www.st.com/resource/en/application_note/an5497-introduction-to-the-buck-current-mode-with-the-bg474edpow1-discovery-kit-stmicroelectronics.pdf
        // https://centaur.reading.ac.uk/31751/1/Microcontroller%20Based%20Peak%20Current%20Mode%20Control%20Using%20Digital%20Slope%20Compensation%20-%20Hallworth%202012.pdf

        use core::f64::consts::PI;

        let ParametersBuck {
            v_in,
            v_out,
            c_out,
            f_sw,
            l_inductor,
            r_esr_out_cap,
            current_sense_gain,
            i_load,
        } = self;

        p!(v_in, "16");
        p!(v_out, "8");
        p!(i_load, "2");
        p!(c_out, "440e-6");
        p!(l_inductor, "22e-6");
        p!(current_sense_gain, "0.48");
        p!(r_esr_out_cap, "31e-3");

        // TODO: Dont
        let diode_drop = 0.6;
        p!(diode_drop, "0.6");
        p!(f_sw, "200e3");

        let t_sw = 1.0 / f_sw;
        let r_load = v_out / i_load; // ohm

        // TODO: Figure out this
        // Time taken in seconds from the ADC reading of Vout, the calculation of the control function and to setting the DAC value
        let t_adc_sample_to_dac_out = 0.0;

        let steady_state_duty = (v_out + diode_drop) / v_in; // Assuming zero Rds(on) and Rdc drops
        let inv_steady_state_duty = 1.0 - steady_state_duty;

        p!(steady_state_duty, "0.5375");

        // S_n
        let inductor_current_up_slope =
            ((v_in - v_out - diode_drop) * current_sense_gain) / l_inductor; // volts/second

        // m_c
        let slope_compensation_factor = (1.0 + PI / 2.0) / (PI * inv_steady_state_duty);
        {
            let m_c = slope_compensation_factor;
            p!(m_c, "1.7693");
        }
        // m_c
        //let slope_compensation_factor = 1.0 + dac_down_slope / inductor_current_up_slope;

        // S_e
        let dac_down_slope = -(slope_compensation_factor - 1.0) * inductor_current_up_slope; // volts/second

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
                t_adc_sample_to_dac_out,

                q_inv,

                ohmega_p1,
                ohmega_esr,
                h_dc,
            },
            DacSettings { dac_down_slope },
        )
    }
}

#[derive(Debug)]
pub struct TransferFunction {
    f_sw: f64,
    t_adc_sample_to_dac_out: f64,

    q_inv: f64,

    ohmega_p1: f64,
    ohmega_esr: f64,
    h_dc: f64,
}

impl TransferFunction {
    pub fn to_2p2z(self) -> TwoPoleTwoZero {
        let TransferFunction {
            f_sw,
            t_adc_sample_to_dac_out,
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
        p!(f_x, "15000");

        println!("----------------------------------");
        println!("----------------------------------");
        println!("----------------------------------");

        // Crossover frequency as rad/s
        let ohmega_x = 2.0 * PI * f_x;
        p!(ohmega_x, "?");

        let phase_erosion = 2.0 * PI * f_x * t_adc_sample_to_dac_out;
        assert!(phase_erosion < 90.0f64.to_radians());

        // TODO: Is this enough?
        let phase_margin: f64 = 75.0f64.to_radians(); //50.0f64.to_radians() + phase_erosion;

        dbg!(phase_margin);

        // ChatGPT's suggestion
        let r = ohmega_x / ohmega_n;
        let complex_pole_pair = (r / (1.0 - pow2(r))).atan();
        dbg!(complex_pole_pair);

        // p1 ok
        let phi_v = -0.5 * PI + phase_margin + (ohmega_x / ohmega_p1).atan() + complex_pole_pair;
        dbg!(ohmega_x);

        /*let phi_v = ((Complex::r_div(ohmega_x, ohmega_n1)).atan())
        .add((Complex::r_div(ohmega_x, ohmega_n2)).atan())
        .add_r((-0.5 * PI + phase_margin) + (ohmega_x / ohmega_p1).atan());*/

        // phi_v should end up being only real at this point
        //assert_eq!(phi_v.im, 0.0);

        //let phi_v = phi_v.re;
        dbg!(phi_v);
        //p!(phi_v.to_degrees(), "?");

        //p!(phi_v.tan(), "0.874095");

        let ohmega_cp1 = ohmega_esr; // Rätt
        let ohmega_cz1 = ohmega_x / phi_v.tan();
        p!(ohmega_cp1, "73_310"); // Rätt
        p!(ohmega_cz1, "11_110"); // Fel

        let k1 =
            f64::sqrt((1.0 + pow2(ohmega_x / ohmega_cz1)) / (1.0 + pow2(ohmega_x / ohmega_p1)));
        //let k2 = 1.0 / f64::sqrt(pow2(1.0 - ohmega_x / pow2(ohmega_n)) + pow2(ohmega_x / ohmega_n));
        let k2 =
            f64::sqrt(1.0 / (pow2(1.0 - pow2(ohmega_x / ohmega_n)) + pow2(ohmega_x / ohmega_n)));

        dbg!(k1);
        dbg!(k2);
        // pole at origin
        let ohmega_cp0 = ohmega_x / (h_dc * k1 * k2); // Fel
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

        TwoPoleTwoZero {
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

        println!(
            "1.0 / (s * s / {:.2}^2 + s * {:.2} / {:.2} + 1.0)",
            ohmega_n, q_inv, ohmega_n
        );
    }

    pub fn print_dc_gain(&self) {
        println!("{:.2}", self.h_dc);
    }
}
