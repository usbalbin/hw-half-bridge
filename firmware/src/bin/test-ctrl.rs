use fixed::types::{I10F22, I11F21, I12F20};
use fugit::NanosDurationU32;
use half_bridge::control_2p2z::{
    DacSettings, ParametersBuck, TransferFunction, TwoPoleTwoZeroParams,
};

const T_ADC: NanosDurationU32 = NanosDurationU32::nanos(870);
const T_PROCESSING: NanosDurationU32 = NanosDurationU32::nanos(500);
const T_DAC: NanosDurationU32 = NanosDurationU32::nanos(150);

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
    P.to_transfer_function();
const TRANSFER_FUNCTION: TransferFunction = TRANSFER_FUNCTION_AND_DAC_SETTINGS.0;
const DAC_SETTINGS: DacSettings = TRANSFER_FUNCTION_AND_DAC_SETTINGS.1;
const COMPENSATOR_CFG: TwoPoleTwoZeroParams<f32> = TRANSFER_FUNCTION.to_2p2z();

fn main() {
    let mut ctrl = COMPENSATOR_CFG.to_controller();

    for i in 0..10 {
        let r = ctrl.update(0.0);
        if i % 10 == 0 {
            bar(r);
            println!("0.0: {i}: {r}");
        }
    }
    println!();
    for i in 0..1000 {
        let r = ctrl.update(1.0);

        if i % 10 == 0 {
            bar(r);
            println!("1.0: {i}: {r}");
        }
    }


    println!("-----------");

    type T = I12F20;

    let ctrl = COMPENSATOR_CFG.to_t();
    println!("{ctrl:?}");

    let mut ctrl = ctrl.to_controller();

    for i in 0..10 {
        let r = ctrl.update(T::from_num(0.0));
        if i % 10 == 0 {
            bar(r.to_num());
            println!("0.0: {i}: {r}");
        }
    }
    println!();
    for i in 0..1000 {
        let r = ctrl.update(T::from_num(2047.0));

        if i % 10 == 0 {
            bar(r.to_num());
            println!("1.0: {i}: {r}");
        }
    }
}

fn bar(x: f32) {
    for _ in 0..((x * 100.0).abs() as i32) {
        print!("=");
    }
    print!(">");
}
