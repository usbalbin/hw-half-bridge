use stm32_hrtim::{
    self,
    control::{HrControltExt, HrPwmControl, HrTimCalibrated},
    external_event::{EevInput, EevInputs, EevSamplingFilter, ExternalEventSource},
    output::{HrOut1, HrOut2, HrOutput},
    timer::HrSlaveTimer,
    timer_eev_cfg::EevCfgs,
    HrParts, HrPwmAdvExt,
};
use stm32g4xx_hal::{gpio, stm32::{
    Peripherals, HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME,
    HRTIM_TIMF,
}};

use super::{external_events::Eevs, Prescaler, PERIOD};

macro_rules! init_hrtim {
    ($tim:expr, ($hi_pin:ident $(, $li_pin:ident)*), $comp_eev:expr, $deadtime:expr, $rst_evt:expr, $rcc:expr, $hr_control:expr) => {{
        let mut timer = $tim
            .pwm_advanced(($hi_pin $(, $li_pin)*))
            .deadtime($deadtime)
            .prescaler(Prescaler::default())
            .period(PERIOD)
            .out1_polarity(stm32_hrtim::Polarity::ActiveHigh)
            .out2_polarity(stm32_hrtim::Polarity::ActiveHigh)
            .preload(stm32_hrtim::PreloadSource::OnCounterReset)
            .timer_mode(stm32_hrtim::HrTimerMode::SingleShotRetriggerable)
            .counting_direction(stm32_hrtim::HrCountingDirection::Up)
            .eev_cfg(EevCfgs::default())
            //.repetition_counter(repetition_counter)
            //.enable_repetition_interrupt()
            .finalize(&mut $hr_control);

        let pin = &mut timer.out;
        $(
            let $li_pin = ();
            let pin = &mut pin.0;
        )*

        timer.timer.enable_reset_event(&$rst_evt);
        pin.enable_set_event(&$rst_evt);
        pin.enable_rst_event(&timer.cr1);
        pin.enable_rst_event(&$comp_eev);

        timer
    }};
}

pub struct Timers {
    timer1: HrParts<
        HRTIM_TIMF,
        Prescaler,
        (HrOut1<HRTIM_TIMF, Prescaler>, HrOut2<HRTIM_TIMF, Prescaler>),
    >,
    timer2: HrParts<
        HRTIM_TIMC,
        Prescaler,
        (HrOut1<HRTIM_TIMC, Prescaler>, HrOut2<HRTIM_TIMC, Prescaler>),
    >,
    timer3: HrParts<
        HRTIM_TIME,
        Prescaler,
        (HrOut1<HRTIM_TIME, Prescaler>, HrOut2<HRTIM_TIME, Prescaler>),
    >,

    #[cfg(feature = "hv4")]
    timer4b: HrParts<HRTIM_TIMB, Prescaler, HrOut1<HRTIM_TIMB, Prescaler>>,
    #[cfg(feature = "hv4")]
    timer4d: HrParts<HRTIM_TIMD, Prescaler, HrOut2<HRTIM_TIMD, Prescaler>>,
    #[cfg(feature = "hv5")]
    timer5a: HrParts<
        HRTIM_TIMA,
        Prescaler,
        (HrOut1<HRTIM_TIMA, Prescaler>, HrOut2<HRTIM_TIMA, Prescaler>),
    >,
}

impl Timers {
    pub(crate) fn init(
        hrtim_master: HRTIM_MASTER,
        hrtima: HRTIM_TIMA,
        hrtimb: HRTIM_TIMB,
        hrtimc: HRTIM_TIMC,
        hrtimd: HRTIM_TIMD,
        hrtime: HRTIM_TIME,
        hrtimf: HRTIM_TIMF,

        hi_1: gpio::gpioc::PC6<gpio::Input<gpio::Floating>>,
        li_1: gpio::gpioc::PC7<gpio::Input<gpio::Floating>>,
        hi_2: gpio::gpiob::PB12<gpio::Input<gpio::Floating>>,
        li_2: gpio::gpiob::PB13<gpio::Input<gpio::Floating>>,
        hi_3: gpio::gpioc::PC8<gpio::Input<gpio::Floating>>,
        li_3: gpio::gpioc::PC9<gpio::Input<gpio::Floating>>,
        #[cfg(feature = "hv4")] li_4: gpio::gpiob::PB15<gpio::Input<gpio::Floating>>,
        #[cfg(feature = "hv4")] hi_4: gpio::gpioa::PA10<gpio::Input<gpio::Floating>>,
        #[cfg(feature = "hv5")] hi_5: gpio::gpioa::PA8<gpio::Input<gpio::Floating>>,
        #[cfg(feature = "hv5")] li_5: gpio::gpioa::PA9<gpio::Input<gpio::Floating>>,
        eevs: &Eevs,
        mut hr_ctrl: HrPwmControl,
    ) -> Timers {
        defmt::info!("Initializing Timers...");

        let timer_master = hrtim_master
            .pwm_advanced(())
            .enable_repetition_interrupt()
            .prescaler(stm32_hrtim::Pscl1)
            .period(PERIOD)
            //.repetition_counter(repetition_counter)
            .enable_repetition_interrupt()
            .finalize(&mut hr_ctrl);

        let dt = stm32_hrtim::deadtime::DeadtimeConfig::default();

        let timer1 = init_hrtim!(
            hrtimf,
            (hi_1, li_1),
            eevs.cc1,
            dt,
            timer_master.timer,
            rcc,
            hr_ctrl
        );
        let timer2 = init_hrtim!(
            hrtimc,
            (hi_2, li_2),
            eevs.cc2,
            dt,
            timer_master.cr1,
            rcc,
            hr_ctrl
        );
        let timer3 = init_hrtim!(
            hrtime,
            (hi_3, li_3),
            eevs.cc3,
            dt,
            timer_master.cr2,
            rcc,
            hr_ctrl
        );

        #[cfg(feature = "hv4")]
        let timer4b = init_hrtim!(hrtimb, (hi_4), eevs.cc4, dt, timer_master.cr3, rcc, hr_ctrl);

        #[cfg(feature = "hv4")]
        let timer4d = init_hrtim!(hrtimd, (li_4), eevs.cc4, dt, timer_master.cr3, rcc, hr_ctrl);

        #[cfg(feature = "hv5")]
        let timer5a = init_hrtim!(
            hrtima,
            (hi_5, li_5),
            eevs.cc4,
            dt,
            timer_master.cr4,
            rcc,
            hr_ctrl
        );
        Timers {
            timer1,
            timer2,
            timer3,
            #[cfg(feature = "hv4")]
            timer4b,
            #[cfg(feature = "hv4")]
            timer4d,
            #[cfg(feature = "hv5")]
            timer5a,
        }
    }
}
