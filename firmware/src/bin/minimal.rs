#![no_main]
#![no_std]

use half_bridge::{self as _, hardware::TICK_RATE}; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = stm32g4xx_hal::stm32,
)]
mod app {
    use embedded_hal::delay::DelayNs;
    use fugit::ExtU32;
    use half_bridge::{
        half_bridge::HalfBridge,
        hardware::{self, adc::Adcs, REPETITION_COUNTER, TICK_RATE},
    };
    use stm32_hrtim::compare_register::HrCompareRegister;
    use stm32g4xx_hal::{adc::config::SampleTime, gpio};

    use crate::millis_to_ticks;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        half_bridge: HalfBridge,
        nucleo_user_button: gpio::PC13<gpio::Input>,
        adcs: hardware::adc::Adcs,
        ad_channels: hardware::adc::AdcChannels,
        eevs: hardware::external_events::Eevs,
        i: u32,
        btn_iter_pressed: u32,
        is_wait_for_btn_release: bool,
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
        } = hardware::Hardware::init(cx.device, cx.core);

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
        timers.timer1.cr1.set_duty(hardware::PERIOD / 2); // Set max duty to 50%
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

        (
            Shared {},
            Local {
                half_bridge: HalfBridge::init(timers, dacs, zero_current_offsets),
                nucleo_user_button,
                adcs,
                ad_channels,
                eevs,
                i: 0,
                btn_iter_pressed: 0,
                is_wait_for_btn_release: true,
            },
        )
    }

    #[task(
        binds = HRTIM_MASTER_IRQN,
        shared = [ ],
        local = [adcs, ad_channels, half_bridge, nucleo_user_button, i, btn_iter_pressed, is_wait_for_btn_release],
        priority = 15
    )]
    fn foo(ctx: foo::Context) {
        *ctx.local.i = ctx.local.i.wrapping_add(1);
        let is_btn_pressed = ctx.local.nucleo_user_button.is_high();
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

        if *ctx.local.i & 0xFFF != 0 {
            ctx.local.half_bridge.clear_repetition_interrupt();
            return;
        }

        //ctx.local.adcs.read(ctx.local.ad_channels);
        let t = ctx
            .local
            .adcs
            .adc1
            .convert(&ctx.local.ad_channels.ntc_5, SampleTime::Cycles_640_5);
        let t = Adcs::adc_to_degreec_c(t);

        if *ctx.local.i & 0x1FFF == 0 {
            let i = ctx
                .local
                .adcs
                .adc3
                .convert(&ctx.local.ad_channels.cc1, SampleTime::Cycles_12_5);
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
        ctx.local.half_bridge.update_set_all_currents_buck(50);

        if t > 70.0 {
            ctx.local.half_bridge.disable();
            defmt::error!("Disabled due to overheat");
        }
        ctx.local.half_bridge.clear_repetition_interrupt();
    }
}

const fn millis_to_ticks(ms: u32) -> u32 {
    TICK_RATE.to_Hz() * ms / 1000
}
