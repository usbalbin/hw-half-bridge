#![no_std]
#![no_main]

use defmt::{dbg, println};
use half_bridge::control_2p2z::{DacSettings, ParametersBuck, TransferFunction, TwoPoleTwoZero};

const S: ParametersBuck = ParametersBuck {
    v_in: 16.0,
    v_out: 8.0,
    c_out: 440.0e-6,
    f_sw: 200e3,
    l_inductor: 22e-6,
    //r_esr_inductor: 4.08e-3,   // 4.08mOhm typical
    r_esr_out_cap: 31e-3,     // todo
    current_sense_gain: 0.48, // 66mV/A
    i_load: 2.0,              // 10A
};

/*const P: ParametersBuck = ParametersBuck {
    v_in: 16.0,
    v_out: 8.0,
    c_out: 440.0e-6, // 2 * ~7.7uF @ 12V
    f_sw: 2e5,
    l_inductor: 22e-6, // 2.2 @ 0A, 2.0 at 8A, ~1.5 @ 24A
    //r_esr_inductor: 4.08e-3,   // 4.08mOhm typical
    r_esr_out_cap: 31e-3,     // todo
    current_sense_gain: 66e-3, // 66mV/A
    i_load: 2.0,              // 10A
};*/

const TRANSFER_FUNCTION_AND_DAC_SETTINGS: (TransferFunction, DacSettings) =
    S.to_transfer_function();
const TRANSFER_FUNCTION: TransferFunction = TRANSFER_FUNCTION_AND_DAC_SETTINGS.0;

const COMPENSATOR: TwoPoleTwoZero = TRANSFER_FUNCTION.to_2p2z();

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    println!("{}", TRANSFER_FUNCTION);
    println!("DC gain: ");
    TRANSFER_FUNCTION.print_dc_gain();

    println!("h_p_transfer_func: ");
    TRANSFER_FUNCTION.print_h_p_transfer_func();
    println!("high_freq_transfer_funcn: ");
    TRANSFER_FUNCTION.print_high_freq_transfer_func();
    let ohmega_n = TRANSFER_FUNCTION.ohmega_n();
    dbg!(ohmega_n);
    defmt::println!("{}", COMPENSATOR);
    defmt::println!(
        "TwoPoleTwoZero {{ a1: 1.69, a2: -0.69, b0: 3.11, b1: 0.17, b2: -2.94 }} Expected"
    );

    for v_in in 0..60 {
        let params = ParametersBuck {
            v_in: v_in as _,
            v_out: 8.0,
            c_out: 440.0e-6,
            f_sw: 200e3,
            l_inductor: 22e-6,
            //r_esr_inductor: 4.08e-3,   // 4.08mOhm typical
            r_esr_out_cap: 31e-3,     // todo
            current_sense_gain: 0.48, // 66mV/A
            i_load: 2.0,              // 10A
        };

        let (tf, _dac) = params.to_transfer_function();
        let comp = tf.to_2p2z();

        defmt::println!("vin={} - {}", v_in, comp);
    }

    for v_out in 0..15 {
        let params = ParametersBuck {
            v_in: 16.0,
            v_out: v_out as _,
            c_out: 440.0e-6,
            f_sw: 200e3,
            l_inductor: 22e-6,
            //r_esr_inductor: 4.08e-3,   // 4.08mOhm typical
            r_esr_out_cap: 31e-3,     // todo
            current_sense_gain: 0.48, // 66mV/A
            i_load: 2.0 as _,         // 10A
        };

        let (tf, _dac) = params.to_transfer_function();
        let comp = tf.to_2p2z();

        defmt::println!("v_out={} - {}", v_out, comp);
    }

    for i_load in 0..10 {
        let params = ParametersBuck {
            v_in: 16.0,
            v_out: 8.0,
            c_out: 440.0e-6,
            f_sw: 200e3,
            l_inductor: 22e-6,
            //r_esr_inductor: 4.08e-3,   // 4.08mOhm typical
            r_esr_out_cap: 31e-3,     // todo
            current_sense_gain: 0.48, // 66mV/A
            i_load: i_load as _,      // 10A
        };

        let (tf, _dac) = params.to_transfer_function();
        let comp = tf.to_2p2z();

        defmt::println!("i_load={} - {}", i_load, comp);
    }

    loop {}
}
