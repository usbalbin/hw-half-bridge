#![no_main]
#![no_std]

use half_bridge::{self as _, hardware::TICK_RATE}; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = stm32g4xx_hal::stm32,
)]
mod app {
    use embedded_hal::delay::DelayNs;
    use fixed::types::I31F1 as T;
    use fugit::NanosDurationU32;
    use half_bridge::{
        control_2p2z::{
            DacSettings, ParametersBuck, TransferFunction, TwoPoleTwoZero, TwoPoleTwoZeroParams,
        },
        hardware::{
            self,
            adc::Adcs,
            dacs::Dacs,
            timers::{MasterTimer, TimerHb1},
        },
    };
    use stm32_hrtim::{compare_register::HrCompareRegister, output::HrOutput, timer::HrTimer};
    use stm32g4xx_hal::{
        self as hal,
        dac::{DacOut, SawtoothConfig},
        stm32,
        timer::{self, CountDownTimer, Instant},
    };
    use stm32g4xx_hal::{adc::config::SampleTime, gpio, timer::MonoTimer};

    use crate::millis_to_ticks;

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

    #[cfg(feature = "fixed-ctrl")]
    type Controller = TwoPoleTwoZero<T>;

    #[cfg(not(feature = "fixed-ctrl"))]
    type Controller = TwoPoleTwoZero<f32>;

    // Shared resources go here
    #[shared]
    struct Shared {
        zero_current_offsets: [u16; 5],
        // TODO: Add resources
        debug_timer: MonoTimer,

        ad_channels: hardware::adc::AdcChannels,

        is_on: bool,
    }

    // Local resources go here
    #[local]
    struct Local {
        is_first: bool,
        dacs: Dacs,
        master_timer: MasterTimer,
        timer1: TimerHb1,

        nucleo_user_button: gpio::PC13<gpio::Input>,
        adc1: hal::adc::Adc<stm32::ADC1, hal::adc::Configured>,
        #[cfg(feature = "hw_triggered_adc2")]
        adc2: hal::adc::DynamicAdc<stm32::ADC2>,
        //#[cfg(not(feature = "hw_triggered_adc2"))]
        //adc2: hal::adc::Adc<stm32::ADC2, hal::adc::Configured>,
        adc3: hal::adc::Adc<stm32::ADC3, hal::adc::Configured>,
        adc4: hal::adc::Adc<stm32::ADC4, hal::adc::Configured>,
        adc5: hal::adc::Adc<stm32::ADC5, hal::adc::Configured>,

        eevs: hardware::external_events::Eevs,
        i: u64,
        btn_iter_pressed: u32,
        is_wait_for_btn_release: bool,

        max_temp_adc: u16,

        controller: Controller,

        vin_metric: probe_plotter::Metric<u16>,
        vout_metric: probe_plotter::Metric<u16>,

        current_metric: probe_plotter::Metric<u16>,
        temp_metric: probe_plotter::Metric<u16>,

        target: probe_plotter::Setting<u16>,
        duty_limit: probe_plotter::Setting<u16>,

        target_current: probe_plotter::Metric<u16>,
        runtime_metric: probe_plotter::Metric<u32>,

        cnt1: probe_plotter::Metric<u16>,
        cnt2: probe_plotter::Metric<u16>,

        instant: Instant,
        slow_tick_timer: CountDownTimer<hal::pac::TIM7>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::dbg!(hardware::PERIOD);
        defmt::dbg!(hardware::F_SW);
        defmt::dbg!(hardware::TICK_RATE);
        //defmt::dbg!(hardware::REPETITION_COUNTER);
        defmt::dbg!(hardware::I_FILTER);
        defmt::dbg!(hardware::ADC_POST_SCALER);
        defmt::dbg!(hardware::DEADTIME);
        defmt::dbg!(hardware::DEADTIME_RISING_TICKS);
        defmt::dbg!(hardware::DEADTIME_FALLING_TICKS);
        defmt::dbg!(COMPENSATOR_CFG);

        defmt::info!("init");
        let hardware::Hardware {
            mut timers,
            mut adcs,
            ad_channels,
            eevs,
            dacs,
            mut delay,
            nucleo_user_button,
            debug_timer,
        } = hardware::Hardware::init(cx.device, cx.core, DAC_CFG);

        delay.delay_ms(1000);

        let mut zero_current_offsets = [0; 5];
        let samples = 100;
        defmt::info!("Measuring zero current");
        for _ in 0..samples {
            zero_current_offsets[0] +=
                adcs.adc3
                    .convert(&ad_channels.cc1, SampleTime::Cycles_640_5) as u32;
            zero_current_offsets[1] += 2047; /*
                                             adcs.adc2
                                                 .convert(&ad_channels.cc2, SampleTime::Cycles_640_5) as u32;*/
            zero_current_offsets[2] +=
                adcs.adc4
                    .convert(&ad_channels.cc3, SampleTime::Cycles_640_5) as u32;
            zero_current_offsets[3] += 2047; /*
                                             adcs.adc2
                                                 .convert(&ad_channels.cc4, SampleTime::Cycles_640_5) as u32;*/
            zero_current_offsets[4] += 2047; /*
                                             adcs.adc2
                                                 .convert(&ad_channels.cc5, SampleTime::Cycles_640_5) as u32;*/

            delay.delay_ms(10);
        }

        let zero_current_offsets = zero_current_offsets.map(|x| (x / samples) as u16);

        defmt::dbg!(zero_current_offsets);

        defmt::info!("Starting timers");
        timers.timer1.cr1.set_duty(544); // Set max duty to 50%
                                         //timers.timer1.out.0.enable();
                                         //timers.timer1.out.1.enable();

        timers
            .control
            .adc_trigger1
            .enable_source(&timers.timer1.cr4);
        timers.timer1.cr4.set_duty(544); // Set ADC sample point

        timers.control.control.start_stop_timers(|w| {
            let w = w.start(&mut timers.master_timer.timer);
            #[cfg(feature = "hv1")]
            let w = w.start(&mut timers.timer1.timer);
            #[cfg(feature = "hv2")]
            let w = w.start(&mut timers.timer2.timer);
            #[cfg(feature = "hv3")]
            let w = w.start(&mut timers.timer3.timer);
            #[cfg(feature = "hv4")]
            let w = w.start(&mut timers.timer4b.timer);
            #[cfg(feature = "hv4")]
            let w = w.start(&mut timers.timer4d.timer);
            #[cfg(feature = "hv5")]
            let w = w.start(&mut timers.timer5.timer);
            w
        });

        //let r_ntc = (r_pull_up * v_adc) / (vcc - v_adc);
        //let r_ntc = (10000 * (x * 3.3 / 4095.0)) / (3.3 - (x * 3.3 / 4095.0));

        //let r_ntc / r_ntc_25c = e.pow(beta * (t1_inv - t0_inv));

        // ln e^x = x;

        //let beta * (1.0/t1 - 1.0/t0) = ln(r_ntc / r_ntc_25c);

        let max_temp_adc = defmt::dbg!(Adcs::degrees_c_to_adc(70.0));
        adcs.adc2.start_conversion();
        adcs.adc2.clear_overrun_flag();

        unsafe {
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
        let instant = debug_timer.now();

        (
            Shared {
                zero_current_offsets,
                ad_channels,
                debug_timer,
                is_on: true,
            },
            Local {
                is_first: true,
                controller: COMPENSATOR_CFG.to_t().to_controller(),
                master_timer: timers.master_timer,
                timer1: timers.timer1,
                dacs,
                nucleo_user_button,
                adc1: adcs.adc1,
                adc2: adcs.adc2,
                adc3: adcs.adc3,
                adc4: adcs.adc4,
                adc5: adcs.adc5,
                eevs,
                i: 0,
                btn_iter_pressed: 0,
                is_wait_for_btn_release: true,

                max_temp_adc,
                temp_metric: probe_plotter::make_metric!(
                    TEMP: u16 = 0,
                    "(1.0 / ((ln((10000 * (TEMP * 3.3 / 4095.0)) / (3.3 - (TEMP * 3.3 / 4095.0)) / 10000) / 4100) + 1.0 / (273.15 + 25))) - 273.15"
                ).unwrap(),
                vin_metric: probe_plotter::make_metric!(VIN: u16 = 0, "(3.3 * VIN / 4095) * (20000 + 1000) / 1000").unwrap(),
                vout_metric: probe_plotter::make_metric!(VOUT: u16 = 0, "(3.3 * VOUT / 4095) * (20000 + 1000) / 1000").unwrap(),
                current_metric: probe_plotter::make_metric!(
                    CURRENT: u16 = 0,
                    "((CURRENT * 3.3 / 4095.0) - (3.3 / 2)) / -0.066"// Negate to get positive current in buck direction
                ).unwrap(),

                target: probe_plotter::make_setting!(TARGET: u16 = 0, 0..=4000, 1.0).unwrap(),
                duty_limit: probe_plotter::make_setting!(DUTY_LIMIT: u16 = 544, 544..=4896, 1.0).unwrap(),
                target_current: probe_plotter::make_metric!(TARGET_CURRENT: u16 = 0, "TARGET_CURRENT").unwrap(),
                runtime_metric: probe_plotter::make_metric!(RUNTIME_NS: u32 = 0, "RUNTIME_NS * 1000/170").unwrap(),

                cnt1: probe_plotter::make_metric!(CNT1: u16 = 0, "CNT1").unwrap(),
                cnt2: probe_plotter::make_metric!(CNT2: u16 = 0, "CNT2").unwrap(),
                instant,
                slow_tick_timer: timers.slow_tick_timer,
            },
        )
    }

    #[task(
        binds = TIM7,
        shared = [is_on, &debug_timer, &ad_channels, &zero_current_offsets],
        local = [timer1, adc1, adc3, adc4, adc5, vin_metric, current_metric, temp_metric, max_temp_adc, duty_limit, nucleo_user_button, is_wait_for_btn_release, btn_iter_pressed, slow_tick_timer],
        priority = 1,
    )]
    fn not_fast(mut ctx: not_fast::Context) {
        ctx.local
            .slow_tick_timer
            .clear_interrupt(timer::Event::TimeOut);
        return;
        //defmt::println!("hej");

        let enable = |ctx: &mut not_fast::Context| {
            ctx.local.timer1.out.0.enable();
            ctx.local.timer1.out.1.enable();
            //ctx.shared.is_on.lock(|b| *b = true);
        };

        let disable = |ctx: &mut not_fast::Context| {
            ctx.local.timer1.out.0.disable();
            ctx.local.timer1.out.1.disable();
            //ctx.shared.is_on.lock(|b| *b = false);
        };

        //*ctx.local.i = ctx.local.i.wrapping_add(1);

        let is_btn_pressed = ctx.local.nucleo_user_button.is_high();
        let status = ctx.local.timer1.out.0.get_state();
        if is_btn_pressed && !*ctx.local.is_wait_for_btn_release {
            match status {
                stm32_hrtim::output::State::Idle => {
                    if *ctx.local.btn_iter_pressed >= millis_to_ticks(2000) {
                        enable(&mut ctx);
                        defmt::info!("Enabled by user");
                        *ctx.local.btn_iter_pressed = 0;
                        *ctx.local.is_wait_for_btn_release = true;
                    }
                }
                stm32_hrtim::output::State::Running => {
                    if *ctx.local.btn_iter_pressed >= millis_to_ticks(50) {
                        disable(&mut ctx);
                        defmt::info!("Disabled by user");
                        *ctx.local.btn_iter_pressed = 0;
                        *ctx.local.is_wait_for_btn_release = true;
                    }
                }
                stm32_hrtim::output::State::Fault => todo!(),
            }
            *ctx.local.btn_iter_pressed = ctx.local.btn_iter_pressed.saturating_add(1);
        } else if !is_btn_pressed {
            *ctx.local.is_wait_for_btn_release = false;
            *ctx.local.btn_iter_pressed = 0;
        }

        let vin = 0; /*ctx
                     .local
                     .adc4
                     .convert(&ctx.shared.ad_channels.fb_d, SampleTime::Cycles_47_5); // PC4 D1 HI*/
        let t = ctx
            .local
            .adc1
            .convert(&ctx.shared.ad_channels.ntc_5, SampleTime::Cycles_247_5);
        let i = ctx
            .local
            .adc3
            .convert(&ctx.shared.ad_channels.cc1, SampleTime::Cycles_12_5);

        //let t = Adcs::adc_to_degreec_c(t);
        ctx.local.temp_metric.set(t);
        ctx.local.current_metric.set(i);
        ctx.local.vin_metric.set(vin);

        let duty_limit = ctx.local.duty_limit.get();
        ctx.local.timer1.cr1.set_duty(duty_limit);

        /*if *ctx.local.i & 0x1FFF == 0 {
            let t = Adcs::adc_to_degreec_c(t);
            let i = Adcs::adc_to_ma_buck(i, ctx.shared.zero_current_offsets[0]);
            match status {
                stm32_hrtim::output::State::Idle => {
                    defmt::warn!(
                        "{}, {}, t: {}°C, i: {}mA",
                        is_btn_pressed as u8,
                        status,
                        t,
                        i as i32
                    )
                }
                stm32_hrtim::output::State::Running => {
                    defmt::info!(
                        "{}, {}, t: {}°C, i: {}mA",
                        is_btn_pressed as u8,
                        status,
                        t,
                        i as i32
                    )
                }
                stm32_hrtim::output::State::Fault => todo!(),
            }
        }*/

        // NTC: Small value is hot
        if t < *ctx.local.max_temp_adc {
            disable(&mut ctx);
            defmt::error!("Disabled due to overheat");
        }
    }

    #[task(
        binds = ADC1_2,
        shared = [is_on, &debug_timer, &ad_channels, &zero_current_offsets],
        local = [
            master_timer,
            is_first,
            adc2,
            dacs,
            target,
            vout_metric,
            target_current,
            runtime_metric,
            controller,
            cnt1,
            cnt2,
            instant,
            i
        ],
        priority = 15
    )]
    fn foo(mut ctx: foo::Context) {
        let start = ctx.shared.debug_timer.now();
        //ctx.local.runtime_metric.set(ctx.local.instant.elapsed());
        //*ctx.local.instant = ctx.shared.debug_timer.now();
        //ctx.local.cnt1.set(t);
        //defmt::println!("Hej");
        //         no off        , no ctrl              1641ns - 1676ns

        // No ADC, no off        , no ctrl              529ns

        // No ADC, no off:                              758ns
        // No ADC, no off, no DAC:                      697ns

        // No ADC, no off, no DAC, no ctrl:             491ns

        // No ADC, no off,       , no ctrl, no i metric 512ns

        // Set metric: 17ns
        // Set dac: 38ns
        // Ctrl w f32: 206ns
        // ADC 24.5: 1112ns-1147ns
        if ctx.local.adc2.get_overrun_flag() {
            let t = unsafe { hal::pac::HRTIM_TIMA::steal().cntr().read().cnt().bits() };
            panic!("Overrun at t: {}", t);
        }
        let vout = ctx.local.adc2.current_sample();

        //defmt::println!("vout: {}", vout);
        if !ctx.shared.is_on.lock(|b| *b) {
            ctx.local.controller.reset();
            ctx.local.vout_metric.set(vout);
            ctx.local.runtime_metric.set(start.elapsed());
            return;
        }

        #[cfg(feature = "fixed-ctrl")]
        let to_t = |x| T::from_num(x);

        #[cfg(not(feature = "fixed-ctrl"))]
        let to_t = |x| x as f32;

        #[cfg(feature = "fixed-ctrl")]
        let t_to_u16 = |x: T| x.to_num();

        #[cfg(not(feature = "fixed-ctrl"))]
        let t_to_u16 = |x| x as u16;

        #[cfg(feature = "fixed-ctrl")]
        let from_f32 = |x| T::from_num(x);

        #[cfg(not(feature = "fixed-ctrl"))]
        let from_f32 = |x| x;

        let error = ctx.local.target.get() - vout;
        let out = ctx.local.controller.update(to_t(error));
        let current_limit = out * from_f32(INV_GAIN);
        let current_limit = t_to_u16(current_limit.clamp(to_t(0), to_t(4095)));

        let current_limit = (2047u16).saturating_sub(current_limit).clamp(0, 4095);
        ctx.local.dacs.cc1_cc5.set_value(current_limit);
        ctx.local.target_current.set(current_limit);

        //ctx.local
        //    .cnt2
        //    .set(unsafe { hal::pac::HRTIM_TIMA::steal().cntr().read().cnt().bits() });
        ctx.local.vout_metric.set(vout);
        ctx.local.runtime_metric.set(start.elapsed());
    }
}

const fn millis_to_ticks(ms: u32) -> u32 {
    TICK_RATE.to_Hz() * ms / 1000
}
