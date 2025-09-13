use core::f64;
use core::convert::From;
use fugit::NanosDurationU32;
use half_bridge::{
    control_2p2z::{DacSettings, ParametersBuck, TransferFunction, TwoPoleTwoZeroParams},
};
const T_ADC: NanosDurationU32 = NanosDurationU32::nanos(761);
const T_PROCESSING: NanosDurationU32 = NanosDurationU32::nanos(400);
const T_DAC: NanosDurationU32 = NanosDurationU32::nanos(125);

const S: ParametersBuck = ParametersBuck {
    v_in: 16.0,
    v_out: 8.0,
    v_diode: 0.6,
    c_out: 440.0e-6,
    f_sw: 200e3,
    l_inductor: 22e-6,
    //r_esr_inductor: 4.08e-3,   // 4.08mOhm typical
    r_esr_out_cap: 31e-3,     // todo
    current_sense_gain: 0.48, // 66mV/A
    i_load: 2.0,              // 10A
    t_adc: 0.0,               // TODO
    t_processing: 0.0,
    t_dac: 0.0,
};

const P: ParametersBuck = ParametersBuck {
    v_in: 16.0,
    v_out: 8.0,
    v_diode: 0.0,
    c_out: 15.4e-6, // 2 * ~7.7uF @ 12V
    f_sw: 1e6,
    l_inductor: 22e-6, // 2.2 @ 0A, 2.0 at 8A, ~1.5 @ 24A
    //r_esr_inductor: 4.08e-3,   // 4.08mOhm typical
    r_esr_out_cap: 31e-3,      // todo
    current_sense_gain: 66e-3, // 66mV/A
    i_load: 10.0,              // 10A
    t_adc: T_ADC.to_nanos() as f64 * 1e-9,
    t_processing: T_PROCESSING.to_nanos() as f64 * 1e-9,
    t_dac: T_DAC.to_nanos() as f64 * 1e-9,
};

const TRANSFER_FUNCTION_AND_DAC_SETTINGS: (TransferFunction, DacSettings) =
    S.to_transfer_function();
const TRANSFER_FUNCTION: TransferFunction = TRANSFER_FUNCTION_AND_DAC_SETTINGS.0;
const DAC_SETTINGS: DacSettings = TRANSFER_FUNCTION_AND_DAC_SETTINGS.1;

const COMPENSATOR: TwoPoleTwoZeroParams = TRANSFER_FUNCTION.to_2p2z();


fn main() {
    println!("{:?}", TRANSFER_FUNCTION);
    println!("DC gain: ");
    TRANSFER_FUNCTION.print_dc_gain();

    println!("h_p_transfer_func: ");
    TRANSFER_FUNCTION.print_h_p_transfer_func();
    println!("high_freq_transfer_funcn: ");
    TRANSFER_FUNCTION.print_high_freq_transfer_func();
    let ohmega_n = TRANSFER_FUNCTION.ohmega_n();
    dbg!(ohmega_n);
    println!("{:?} - DAC settings: {:?}", COMPENSATOR, DAC_SETTINGS);
    println!("TwoPoleTwoZero {{ a1: 1.69, a2: -0.69, b0: 3.11, b1: 0.17, b2: -2.94 }} Expected");
    println!("\n\n\n");
    println!("Mine: {:?} - DAC settings: {:?}", P.to_transfer_function().0.to_2p2z(), P.to_transfer_function().1);
    println!("\n\n\n");

    for v_in in [0.1, 1., 2., 10., 16., 30., 60.] {
        let params = ParametersBuck {
            v_in: f64::from(v_in).max(f64::EPSILON),
            v_out: P.v_out,
            v_diode: P.v_diode,
            c_out: P.c_out,
            f_sw: P.f_sw,
            l_inductor: P.l_inductor,
            r_esr_out_cap: P.r_esr_out_cap,
            current_sense_gain: P.current_sense_gain,
            i_load: P.i_load,
            t_adc: P.t_adc,
            t_processing: P.t_processing,
            t_dac: P.t_dac,
        };

        let (tf, _dac) = params.to_transfer_function();
        let comp = tf.to_2p2z();

        let diff = TwoPoleTwoZeroParams {
            a1: P.to_transfer_function().0.to_2p2z().a1 - comp.a1,
            a2: P.to_transfer_function().0.to_2p2z().a2 - comp.a2,
            b0: P.to_transfer_function().0.to_2p2z().b0 - comp.b0,
            b1: P.to_transfer_function().0.to_2p2z().b1 - comp.b1,
            b2: P.to_transfer_function().0.to_2p2z().b2 - comp.b2,
        };

        println!("vin={} - diff: {:?} - val: {:?}", v_in, diff, comp);
    }

    println!();

    for v_out in [0.1, 1., 2., 10., 15.] {
        let params = ParametersBuck {
            v_in: P.v_in,
            v_out: f64::from(v_out).max(f64::EPSILON),
            v_diode: P.v_diode,
            c_out: P.c_out,
            f_sw: P.f_sw,
            l_inductor: P.l_inductor,
            r_esr_out_cap: P.r_esr_out_cap,
            current_sense_gain: P.current_sense_gain,
            i_load: P.i_load,
            t_adc: P.t_adc,
            t_processing: P.t_processing,
            t_dac: P.t_dac,
        };

        let (tf, _dac) = params.to_transfer_function();
        let comp = tf.to_2p2z();

        let diff = TwoPoleTwoZeroParams {
            a1: P.to_transfer_function().0.to_2p2z().a1 - comp.a1,
            a2: P.to_transfer_function().0.to_2p2z().a2 - comp.a2,
            b0: P.to_transfer_function().0.to_2p2z().b0 - comp.b0,
            b1: P.to_transfer_function().0.to_2p2z().b1 - comp.b1,
            b2: P.to_transfer_function().0.to_2p2z().b2 - comp.b2,
        };

        println!("v_out={} - diff: {:?} - val: {:?}", v_out, diff, comp);
    }

    println!();

    for i_load in [0., 0.1, 1., 2., 10., 20.] {
        let params = ParametersBuck {
            v_in: P.v_in,
            v_out: P.v_out,
            v_diode: P.v_diode,
            c_out: P.c_out,
            f_sw: P.f_sw,
            l_inductor: P.l_inductor,
            r_esr_out_cap: P.r_esr_out_cap,
            current_sense_gain: P.current_sense_gain,
            i_load: f64::from(i_load).max(f64::EPSILON), // 10A
            t_adc: P.t_adc,
            t_processing: P.t_processing,
            t_dac: P.t_dac,
        };

        let (tf, _dac) = params.to_transfer_function();
        let comp = tf.to_2p2z();

        let diff = TwoPoleTwoZeroParams {
            a1: P.to_transfer_function().0.to_2p2z().a1 - comp.a1,
            a2: P.to_transfer_function().0.to_2p2z().a2 - comp.a2,
            b0: P.to_transfer_function().0.to_2p2z().b0 - comp.b0,
            b1: P.to_transfer_function().0.to_2p2z().b1 - comp.b1,
            b2: P.to_transfer_function().0.to_2p2z().b2 - comp.b2,
        };

        println!("i_load={} - diff: {:?} - val: {:?}", i_load, diff, comp);
    }
}
