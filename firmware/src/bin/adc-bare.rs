#![no_main]
#![no_std]

use half_bridge::{self as _}; // global logger + panicking-behavior + memory layout

use fugit::NanosDurationU32;
use half_bridge::{
    control_2p2z::{DacSettings, ParametersBuck, TransferFunction, TwoPoleTwoZeroParams},
    hardware::{
        self, adc::ADC_CLOCK_MODE, DEADTIME_FALLING_TICKS, DEADTIME_RISING_TICKS, SYS_PLL_M_DIV,
        SYS_PLL_N_MUL, SYS_PLL_P_DIV, SYS_PLL_R_DIV, SYS_PLL_SOURCE,
    },
};
use stm32_hrtim::{
    compare_register::HrCompareRegister,
    output::{HrOut, HrOutput},
    pac::{HRTIM_COMMON, HRTIM_TIMA},
    timer::HrTimer,
    timer_eev_cfg::EevCfgs,
    DacResetOnCounterReset, DacStepOnCmp2, HrParts, HrPwmAdvExt, Pscl128,
};
use stm32g4xx_hal::{
    self as hal,
    adc::{AdcClaim, AdcCommonExt},
    dac::SawtoothConfig,
    delay::SYSTDelayExt,
    hrtim::HrPwmBuilderExt,
    interrupt, stm32,
    timer::Instant,
};
use stm32g4xx_hal::{
    gpio::GpioExt,
    hrtim::HrControltExt,
    pwr::{self, PwrExt},
    rcc::{self, RccExt},
    timer::MonoTimer,
};

// Max dac sample rate is 15MSps
pub const NUM_DAC_TRIGGERS_PER_PERIOD: f64 = 15.0;

// The dac register is incremented 16 steps for every step on the output
pub const INC_PER_DAC_INC: f64 = 16.0;
pub const DAC_STEP_SIZE: u16 = -(INC_PER_DAC_INC / NUM_DAC_TRIGGERS_PER_PERIOD
    * DAC_SETTINGS.dac_slope
    / hardware::F_SW.to_Hz() as f64
    / DAC_GAIN) as u16;
pub const DAC_STEP_DIR: hal::dac::CountingDirection = hal::dac::CountingDirection::Increment; // Increment for buck, decrement for boost
pub const DAC_CFG: SawtoothConfig = SawtoothConfig::with_slope(DAC_STEP_DIR, DAC_STEP_SIZE);

const R_HI_VOUT_DIVIDER: f64 = 20_000.0;
const R_LO_VOUT_DIVIDER: f64 = 1_000.0;
const RESISTOR_DIVIDER_GAIN: f64 = R_LO_VOUT_DIVIDER / (R_HI_VOUT_DIVIDER + R_LO_VOUT_DIVIDER);
const ADC_GAIN: f64 = 4095.0 / 3.3;
const DAC_GAIN: f64 = 3.3 / 4095.0;

const INV_GAIN: f32 = (1.0 / (RESISTOR_DIVIDER_GAIN * ADC_GAIN * DAC_GAIN)) as f32;

const T_ADC: NanosDurationU32 = hardware::adc::sampling_time(
    hal::adc::config::SampleTime::Cycles_24_5,
    hal::adc::config::Resolution::Twelve,
);

const TODO_T_PROCESSING: () = ();
const T_PROCESSING: NanosDurationU32 = NanosDurationU32::nanos(500);
const T_DAC: NanosDurationU32 = hardware::dacs::T_FAST_DAC_SETTLE_MIN_TO_MAX_1LSB;

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
const COMPENSATOR_CFG: TwoPoleTwoZeroParams = TRANSFER_FUNCTION.to_2p2z();

type Prescaler = stm32_hrtim::Pscl64;

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::info!("init");

    let dp = stm32g4xx_hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let pwr_cfg = pwr // Enable boost mode to allow f_sys > 150MHz
        .vos(pwr::VoltageScale::Range1 { enable_boost: true })
        .freeze();
    let rcc_cfg = rcc::Config::pll().pll_cfg(rcc::PllConfig {
        mux: SYS_PLL_SOURCE,
        n: SYS_PLL_N_MUL,
        m: SYS_PLL_M_DIV,
        r: Some(SYS_PLL_R_DIV), // Set system frequency to 16MHz * 85/4/2 = 170MHz
        p: Some(SYS_PLL_P_DIV), // Set adc input frequency to 16MHz * 85/4/7 = ~48.6MHz

        ..Default::default()
    });
    let mut rcc = dp.RCC.freeze(rcc_cfg, pwr_cfg);
    let mut delay = cp.SYST.delay(&rcc.clocks);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    let a_hi = gpioa.pa8;
    let a_li = gpioa.pa9;
    let fb_a = gpioc.pc5;

    let (ctrl, _f, _eev_inputs) = dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();

    let deadtime = stm32_hrtim::deadtime::DeadtimeConfig::default()
        .prescaler(stm32_hrtim::deadtime::DeadtimePrescaler::ThrtimDiv8)
        .deadtime_falling_value(DEADTIME_FALLING_TICKS)
        .deadtime_rising_value(DEADTIME_RISING_TICKS);

    let mut hr_control = ctrl.constrain();

    let mut timer = dp
        .HRTIM_TIMA
        .pwm_advanced((a_hi, a_li))
        .deadtime(deadtime)
        .prescaler(Prescaler::default())
        .period(0xFF00)
        .out1_polarity(stm32_hrtim::Polarity::ActiveHigh)
        .out2_polarity(stm32_hrtim::Polarity::ActiveHigh)
        .preload(stm32_hrtim::PreloadSource::OnCounterReset)
        //.timer_mode(stm32_hrtim::HrTimerMode::SingleShotRetriggerable)
        .counting_direction(stm32_hrtim::HrCountingDirection::Up)
        //.eev_cfg(EevCfgs::default())
        //.dac_trigger_cfg(DacResetOnCounterReset, DacStepOnCmp2)
        //.repetition_counter(repetition_counter)
        //.enable_repetition_interrupt()
        .finalize(&mut hr_control);

    let pin = &mut timer.out;

    let li_pin = ();
    let _ = li_pin;
    let pin = &mut pin.0;

    pin.enable_set_event(&timer.timer);
    pin.enable_rst_event(&timer.cr1);

    defmt::info!("Starting timers");

    timer.cr1.set_duty(544); // Set max duty to 50%
                             //timers.timer1.out.0.enable();
                             //timers.timer1.out.1.enable();

    hr_control.adc_trigger1.enable_source(&timer.cr4);
    timer.cr4.set_duty(2048); // Set ADC sample point

    let cfg = ADC_CLOCK_MODE;
    let adc12_common = dp.ADC12_COMMON.claim(cfg, &mut rcc);

    /*let mut adc2 = {
        let cfg = hal::adc::config::AdcConfig::<hal::adc::config::ExternalTrigger12>::default()
            .external_trigger(
                hal::adc::config::TriggerMode::RisingEdge,
                (&hr_control.adc_trigger1).into(),
            )
            .end_of_conversion_interrupt(hal::adc::config::Eoc::Conversion)
            .continuous(hal::adc::config::Continuous::Single)
            .subgroup_len(hal::adc::config::SubGroupLength::One);

        let adc = adc12_common.claim_and_configure(dp.ADC2, cfg, &mut delay).start_conversion();

        let mut adc = adc.into_dynamic_adc();
        adc.reset_sequence();
        adc.configure_channel(
            &fb_a,
            hal::adc::config::Sequence::One,
            hal::adc::config::SampleTime::Cycles_12_5,
        );

        adc
    };*/

    let mut adc2 = adc12_common.claim(dp.ADC2, &mut delay);

    adc2.set_external_trigger((
        hal::adc::config::TriggerMode::RisingEdge,
        (&hr_control.adc_trigger1).into(),
    ));
    adc2.set_continuous(hal::adc::config::Continuous::Discontinuous);
    adc2.reset_sequence();
    adc2.configure_channel(
        &fb_a,
        hal::adc::config::Sequence::One,
        hal::adc::config::SampleTime::Cycles_12_5,
    );
    adc2.set_end_of_conversion_interrupt(stm32g4xx_hal::adc::config::Eoc::Sequence);
    let mut adc2 = adc2.enable().into_dynamic_adc();

    unsafe {
        let hrtim = HRTIM_COMMON::steal();
        defmt::println!("adc1r ac4: {}", hrtim.adc1r().read().ac4().bit());
        defmt::println!("adc1r: {}", hrtim.adc1r().read().bits());

        let adc2 = hal::pac::ADC2::steal();
        let cr = adc2.cr().read();
        defmt::println!("adstart: {}", cr.adstart().bit());
        defmt::println!("aden: {}", cr.aden().bit());
        defmt::println!("addis: {}", cr.addis().bit());
        defmt::println!("adstp: {}", cr.adstp().bit());
        defmt::println!("advregen: {}", cr.advregen().bit());
        defmt::println!("deeppwd: {}", cr.deeppwd().bit());

        let cfgr = adc2.cfgr().read();
        defmt::println!("cont: {}", cfgr.cont().is_single());
        defmt::println!("discen: {}", cfgr.discen().is_enabled());
        defmt::println!("discnum: {}", cfgr.discnum().bits());
        defmt::println!("exten: {}", cfgr.exten().bits());
        defmt::println!("extsel: {}", cfgr.extsel().bits());
        defmt::println!("ovrmod: {}", cfgr.ovrmod().is_overwrite());
        defmt::println!("res: {}", cfgr.res().is_bits12());

        let cfgr2 = adc2.cfgr2().read();
        defmt::println!("bulb: {}", cfgr2.bulb().is_enabled());
        defmt::println!("gcomp: {}", cfgr2.gcomp().is_enabled());
        defmt::println!("ovsr: {}", cfgr2.ovsr().bits());
        defmt::println!("ovss: {}", cfgr2.ovss().bits());
        defmt::println!("rovse: {}", cfgr2.rovse().is_enabled());
        defmt::println!("rovsm: {}", cfgr2.rovsm().is_continued());
        defmt::println!("smptrig: {}", cfgr2.smptrig().is_enabled());
        defmt::println!("swtrig: {}", cfgr2.swtrig().is_enabled());
        defmt::println!("trovs: {}", cfgr2.trovs().is_triggered());

        for (i, smp) in adc2.smpr1().read().smp_iter().enumerate() {
            defmt::println!("smp{}: {}", i, smp.bits());
        }

        defmt::println!("sqr1 l {}", adc2.sqr1().read().l().bits());

        for (i, sq) in adc2.sqr1().read().sq_iter().enumerate() {
            defmt::println!("sq{}: {}", i, sq.bits());
        }
    }

    let debug_timer = MonoTimer::new(cp.DWT, cp.DCB, &rcc.clocks);
    let instant = debug_timer.now();

    timer.timer.start(&mut hr_control.control);
    adc2.start_conversion();

    defmt::println!("Enable EXTI Interrupt");
    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::ADC1_2);
    }

    /*loop {
        defmt::assert!(!adc2.get_overrun_flag());
        adc2.wait_for_conversion_sequence();
        let v = adc2.current_sample();
        let t = unsafe { hal::pac::HRTIM_TIMA::steal().cntr().read().cnt().bits() };
        defmt::println!("v: {}, t: {}", v, t);
    }*/
    loop {}
}
//use stm32g4xx_hal::interrupt::ADC1_2;
#[cortex_m_rt::interrupt]
fn ADC1_2() {
    let mut adc2: hal::adc::DynamicAdc<stm32::ADC2> = unsafe { core::mem::zeroed() };

    let t = unsafe { hal::pac::HRTIM_TIMA::steal().cntr().read().cnt().bits() };

    let isr = unsafe { hal::pac::ADC2::steal().isr().read() };
    /*defmt::println!("adrdy: {}", isr.adrdy().is_ready());
        for (i, awd) in isr.awd_iter().enumerate() {
            defmt::println!("awd{}: ¸{}", i, awd.is_event())
        }
        defmt::println!("eoc: {}", isr.eoc().is_complete());
        defmt::println!("eos: {}", isr.eos().is_complete());
        defmt::println!("eosmp: {}", isr.eosmp().is_ended());
        defmt::println!("jeoc: {}", isr.jeoc().is_complete());
        defmt::println!("jeos: {}", isr.jeos().is_complete());
        defmt::println!("jqovf: {}", isr.jqovf().is_overflow());
        defmt::println!("ovr: {}", isr.ovr().is_overrun());
    */
    //assert!(adc2.get_end_of_conversion_flag());
    let a = adc2.get_overrun_flag();
    let vout = adc2.current_sample();
    //assert!(!adc2.get_end_of_conversion_flag());

    let b = adc2.get_overrun_flag();
    adc2.clear_overrun_flag();
    let c = adc2.get_overrun_flag();
    defmt::println!("t: {}, a: {}, b: {}, c: {}", t, a, b, c);
    defmt::assert!(!a && !b && !c, "a: {}, b: {}, c: {}", a, b, c,);

    // PC5 D0 LOW
    //defmt::println!("vout: {}", vout);
    /*if !ctx.shared.is_on.lock(|b| *b) {
        ctx.local.controller.reset();
        ctx.local.vout_metric.set(vout);
        ctx.local.master_timer.timer.clear_repetition_interrupt();
        ctx.local.runtime_metric.set(start.elapsed());
        return;
    }*/

    /*let error = ctx.local.target.get() - vout;
    let out = error as f32;
    //let out = ctx.local.controller.update(error as f32);
    let current_limit = out * INV_GAIN;
    let current_limit = current_limit.clamp(0.0, 4095.0) as u16;

    let current_limit = (2047u16).saturating_sub(current_limit).clamp(0, 4095);
    ctx.local.dacs.cc1_cc5.set_value(current_limit);
    ctx.local.target_current.set(current_limit);

    ctx.local
        .cnt2
        .set(unsafe { hal::pac::HRTIM_TIMA::steal().cntr().read().cnt().bits() });
    //ctx.local.vout_metric.set(vout); // TODO
    //ctx.local.runtime_metric.set(start.elapsed());*/
}
