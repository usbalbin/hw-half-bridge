#![no_main]
#![no_std]

use half_bridge::{self as _, hardware::TICK_RATE}; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = stm32g4xx_hal::stm32,
)]
mod app {
    use embedded_hal::delay::DelayNs;
    use half_bridge::{
        control_2p2z::TwoPoleTwoZero,
        half_bridge::HalfBridge,
        hardware::{self, adc::Adcs, dacs::Dacs, timers::Timers},
    };
    use stm32_hrtim::compare_register::HrCompareRegister;
    use stm32g4xx_hal::{self as hal, dac::SawtoothConfig, stm32};
    use stm32g4xx_hal::{adc::config::SampleTime, gpio, timer::MonoTimer};

    use crate::millis_to_ticks;

    const DONT_FORGET_DAC_STEP_SIZE: () = ();
    pub const DAC_STEP_SIZE: u16 = 1;
    pub const DAC_STEP_DIR: hal::dac::CountingDirection = hal::dac::CountingDirection::Increment; // Increment for buck, decrement for boost
    pub const DAC_CFG: SawtoothConfig = SawtoothConfig::with_slope(DAC_STEP_DIR, DAC_STEP_SIZE);

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        debug_timer: MonoTimer,

        ad_channels: hardware::adc::AdcChannels,
    }

    // Local resources go here
    #[local]
    struct Local {
        half_bridge: HalfBridge,
        dacs: Dacs,
        timers: Timers,
        nucleo_user_button: gpio::PC13<gpio::Input>,
        adc1: hal::adc::Adc<stm32::ADC1, hal::adc::Configured>,
        adc2: hal::adc::Adc<stm32::ADC2, hal::adc::Configured>,
        adc3: hal::adc::Adc<stm32::ADC3, hal::adc::Configured>,
        adc4: hal::adc::Adc<stm32::ADC4, hal::adc::Configured>,
        adc5: hal::adc::Adc<stm32::ADC5, hal::adc::Configured>,

        eevs: hardware::external_events::Eevs,
        i: u32,
        btn_iter_pressed: u32,
        is_wait_for_btn_release: bool,

        max_temp_adc: u16,

        controller: TwoPoleTwoZero,

        vin_metric: probe_plotter::Metric<u16>,
        vout_metric: probe_plotter::Metric<u16>,

        current_metric: probe_plotter::Metric<u16>,
        temp_metric: probe_plotter::Metric<u16>,
        duty_limit: probe_plotter::Setting<u16>,
        current_limit: probe_plotter::Setting<i16>,

        current_limit_metric: probe_plotter::Metric<u16>,
        runtime_metric: probe_plotter::Metric<u32>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::dbg!(hardware::PERIOD);
        defmt::dbg!(hardware::F_SW);
        defmt::dbg!(hardware::TICK_RATE);
        defmt::dbg!(hardware::REPETITION_COUNTER);
        defmt::dbg!(hardware::I_FILTER);
        defmt::dbg!(hardware::ADC_POST_SCALER);
        defmt::dbg!(hardware::DEADTIME);
        defmt::dbg!(hardware::DEADTIME_RISING_TICKS);
        defmt::dbg!(hardware::DEADTIME_FALLING_TICKS);

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
            zero_current_offsets[1] +=
                adcs.adc2
                    .convert(&ad_channels.cc2, SampleTime::Cycles_640_5) as u32;
            zero_current_offsets[2] +=
                adcs.adc4
                    .convert(&ad_channels.cc3, SampleTime::Cycles_640_5) as u32;
            zero_current_offsets[3] +=
                adcs.adc2
                    .convert(&ad_channels.cc4, SampleTime::Cycles_640_5) as u32;
            zero_current_offsets[4] +=
                adcs.adc2
                    .convert(&ad_channels.cc5, SampleTime::Cycles_640_5) as u32;

            delay.delay_ms(10);
        }

        let zero_current_offsets = zero_current_offsets.map(|x| (x / samples) as u16);

        defmt::dbg!(zero_current_offsets);

        defmt::info!("Starting timers");
        timers.timer1.cr1.set_duty(544); // Set max duty to 50%
                                         //timers.timer1.out.0.enable();
                                         //timers.timer1.out.1.enable();

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
        (
            Shared {
                ad_channels,
                debug_timer,
            },
            Local {
                half_bridge: HalfBridge::init(timers, dacs, zero_current_offsets),
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
                duty_limit: probe_plotter::make_setting!(DUTY_LIMIT: u16 = 544, 544..=4896, 1.0).unwrap(),
                current_limit_metric: probe_plotter::make_metric!(DUTY_M: u16 = 0, "DUTY_M").unwrap(),
                runtime_metric: probe_plotter::make_metric!(RUNTIME: u32 = 0, "RUNTIME / 170").unwrap()
            },
        )
    }

    #[task(
        binds = SPI1,
        shared = [&debug_timer, &ad_channels],
        local = [adc1, adc3, vin_metric, current_metric, temp_metric, i, max_temp_adc],
        priority = 1,
    )]
    fn not_fast(ctx: not_fast::Context) {
        *ctx.local.i = ctx.local.i.wrapping_add(1);

        let status = ctx.local.half_bridge.status();
        if is_btn_pressed && !*ctx.local.is_wait_for_btn_release {
            match status {
                stm32_hrtim::output::State::Idle => {
                    if *ctx.local.btn_iter_pressed >= millis_to_ticks(2000) {
                        ctx.local.half_bridge.enable();
                        defmt::info!("Enabled by user");
                        *ctx.local.btn_iter_pressed = 0;
                        *ctx.local.is_wait_for_btn_release = true;
                    }
                }
                stm32_hrtim::output::State::Running => {
                    if *ctx.local.btn_iter_pressed >= millis_to_ticks(50) {
                        ctx.local.half_bridge.disable();
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

        let vin = ctx
            .local
            .adc3
            .convert(&ctx.shared.ad_channels.fb_d, SampleTime::Cycles_47_5); // PC4 D1 HI
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
        ctx.local.half_bridge.set_duty(duty_limit);

        let is_btn_pressed = ctx.local.nucleo_user_button.is_high();

        if *ctx.local.i & 0x1FFF == 0 {
            let t = Adcs::adc_to_degreec_c(t);
            let i = Adcs::adc_to_ma_buck(i, ctx.local.half_bridge.zero_current_offsets[0]);
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
        }

        // NTC: Small value is hot
        if t < *ctx.local.max_temp_adc {
            ctx.local.half_bridge.disable();
            defmt::error!("Disabled due to overheat");
        }
    }

    #[task(
        binds = HRTIM_MASTER_IRQN,
        shared = [&debug_timer, &ad_channels],
        local = [adc2, is_wait_for_btn_release, vout_metric, current_limit_metric, runtime_metric, controller],
        priority = 15
    )]
    fn foo(ctx: foo::Context) {
        let start = ctx.shared.debug_timer.now();

        let vout = ctx
            .local
            .adc2
            .convert(&ctx.shared.ad_channels.fb_a, SampleTime::Cycles_24_5); // PC5 D0 LOW

        if !is_on {
            ctx.local.controller.reset();
            ctx.local.vout_metric.set(vout);
            ctx.local.half_bridge.clear_repetition_interrupt();
            ctx.local.runtime_metric.set(start.elapsed());
            return;
        }

        let error = todo!();
        let out = ctx.local.controller.update(error);
        let current_limit = todo!();

        ctx.local.dacs.update_set_all_currents_buck(current_limit);
        ctx.local.current_limit_metric.set(current_limit);

        ctx.local.vout_metric.set(vout); // Move this to other isr?
        ctx.local.half_bridge.clear_repetition_interrupt();
        ctx.local.runtime_metric.set(start.elapsed());
    }
}

const fn millis_to_ticks(ms: u32) -> u32 {
    TICK_RATE.to_Hz() * ms / 1000
}
