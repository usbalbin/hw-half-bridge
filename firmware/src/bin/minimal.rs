#![no_main]
#![no_std]

use half_bridge as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = stm32g4xx_hal::stm32,
)]
mod app {
    use embedded_hal::delay::DelayNs;
    use half_bridge::{half_bridge::HalfBridge, hardware};
    use stm32g4xx_hal::adc::config::SampleTime;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        half_bridge: HalfBridge,
        adcs: hardware::adc::Adcs,
        ad_channels: hardware::adc::AdcChannels,
        eevs: hardware::external_events::Eevs,
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
        timers.control.control.start_stop_timers(|w| {
            w.start(&mut timers.master_timer.timer)
                .start(&mut timers.timer1.timer)
                .start(&mut timers.timer2.timer)
                .start(&mut timers.timer3.timer)
                .start(&mut timers.timer4b.timer)
                .start(&mut timers.timer4d.timer)
                .start(&mut timers.timer5.timer)
        });

        (
            Shared {},
            Local {
                half_bridge: HalfBridge::init(timers, dacs, zero_current_offsets),
                adcs,
                ad_channels,
                eevs,
            },
        )
    }

    #[task(
        binds = HRTIM_MASTER_IRQN,
        shared = [ ],
        local = [adcs, ad_channels, half_bridge],
        priority = 15
    )]
    fn foo(ctx: foo::Context) {
        ctx.local.adcs.read(ctx.local.ad_channels);
        ctx.local.half_bridge.update_set_all_currents_buck(0);
        defmt::println!(".");
    }
}
