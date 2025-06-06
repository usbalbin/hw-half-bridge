use half_bridge::{control_2p2z::ParametersBuck, math::Complex};

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

const P: ParametersBuck = ParametersBuck {
    v_in: 48.0,
    v_out: 12.0,
    c_out: 2.0 * 7.7e-6, // 2 * ~7.7uF @ 12V
    f_sw: 1e6,
    l_inductor: 2e-6, // 2.2 @ 0A, 2.0 at 8A, ~1.5 @ 24A
    //r_esr_inductor: 4.08e-3,   // 4.08mOhm typical
    r_esr_out_cap: 1.5e-3,     // todo
    current_sense_gain: 66e-3, // 66mV/A
    i_load: 10.0,              // 10A
};

fn main() {
    //dbg!(Complex::new(0.72, -1.5).atan());

    for i in (-10)..=10 {
        let x = i as f64 / 10.0;
        let expected = x.atan();
        let x = Complex::new(x, 1.0);
        let got = x.atan();
        println!("atan({x}) = {got}");
        //if (expected - got).abs() > 0.000001 {
        //    panic!("got: {got}, expected: {expected}, i: {i}");
        //}
    }

    let div = Complex::r_div(94247.0, Complex::new(-314159.0, 1.4142));
    let div2 = 94247.0 / Complex::new(-314159.0, 1.4142);
    println!("div = {div}");
    println!("div2 = {div2}");
    println!("atan(div) = {}", div.atan());

    let (transfer_function, dac_settings) = S.to_transfer_function();
    //println!("{transfer_function:.2?}");
    //print!("DC gain: "); transfer_function.print_dc_gain();

    //print!("h_p_transfer_func: "); transfer_function.print_h_p_transfer_func();
    //print!("high_freq_transfer_funcn: "); transfer_function.print_high_freq_transfer_func();
    //let ohmega_n = transfer_function.ohmega_n();
    //dbg!(ohmega_n);
    let compensator = transfer_function.to_2p2z();
    println!("{compensator:.2?}");
}
