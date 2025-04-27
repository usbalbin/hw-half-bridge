#![no_main]
#![no_std]

use half_bridge as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = stm32g4xx_hal::stm32,
)]
mod app {
    use half_bridge::hardware;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        timers: hardware::timers::Timers,
        adcs: hardware::adc::Adcs,
        ad_channels: hardware::adc::AdcChannels,
        eevs: hardware::external_events::Eevs,
        dacs: hardware::dacs::Dacs,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let hardware::Hardware {
            timers,
            adcs,
            ad_channels,
            eevs,
            dacs,
        } = hardware::Hardware::init(cx.device, cx.core);

        (
            Shared {},
            Local {
                timers,
                adcs,
                ad_channels,
                eevs,
                dacs,
            },
        )
    }

    #[task(
        binds = HRTIM_MASTER_IRQN,
        shared = [ ],
        local = [adcs, ad_channels],
        priority = 15
    )]
    fn foo(mut ctx: foo::Context) {
        ctx.local.adcs.read(ctx.local.ad_channels);
    }
}
