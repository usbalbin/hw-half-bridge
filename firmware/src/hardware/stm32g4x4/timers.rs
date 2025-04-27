use stm32_hrtim::{
    self,
    control::{HrControltExt, HrPwmControl, HrTimCalibrated},
    external_event::{EevInput, EevInputs, EevSamplingFilter, ExternalEventSource},
    output::{HrOut1, HrOut2, HrOutput},
    timer::HrSlaveTimer,
    timer_eev_cfg::EevCfgs,
    HrParts, HrPwmAdvExt,
};
use stm32g4xx_hal::{
    gpio,
    stm32::{
        Peripherals, HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME,
        HRTIM_TIMF,
    },
};

use crate::hardware::REPETITION_COUNTER;

use super::{external_events::Eevs, Prescaler, PERIOD};

type Timer<TIM> = HrParts<TIM, Prescaler, (HrOut1<TIM, Prescaler>, HrOut2<TIM, Prescaler>)>;

type TimHb1 = HRTIM_TIMF;
type TimHb2 = HRTIM_TIMC;
type TimHb3 = HRTIM_TIME;
type TimHb4b = HRTIM_TIMB;
type TimHb4d = HRTIM_TIMD;
type TimHb5 = HRTIM_TIMA;

type TimerHb1 = Timer<TimHb1>;
type TimerHb2 = Timer<TimHb2>;
type TimerHb3 = Timer<TimHb3>;
type TimerHb4b = HrParts<TimHb4b, Prescaler, HrOut1<TimHb4b, Prescaler>>;
type TimerHb4d = HrParts<TimHb4d, Prescaler, HrOut2<TimHb4d, Prescaler>>;
type TimerHb5 = Timer<TimHb5>;

macro_rules! init_hrtim {
    ($tim:expr, ($hi_pin:ident $(, $li_pin:ident)*), $deadtime:expr, $rst_evt:expr, $rcc:expr, $hr_control:expr) => {{
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
            .dac_trigger(DacTriggerCfg::edge_aligned_slope())
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

        timer
    }};
}

pub struct Timers {
    timer1: TimerHb1,
    timer2: TimerHb2,
    timer3: TimerHb3,

    #[cfg(feature = "hv4")]
    timer4b: TimerHb4b,
    #[cfg(feature = "hv4")]
    timer4d: TimerHb4d,
    #[cfg(feature = "hv5")]
    timer5: TimerHb5,
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
        mut hr_ctrl: HrPwmControl,
    ) -> Timers {
        defmt::info!("Initializing Timers...");

        let timer_master = hrtim_master
            .pwm_advanced(())
            .prescaler(stm32_hrtim::Pscl1)
            .period(PERIOD)
            .repetition_counter(REPETITION_COUNTER)
            .enable_repetition_interrupt()
            .finalize(&mut hr_ctrl);

        let dt = stm32_hrtim::deadtime::DeadtimeConfig::default();

        let timer1 = init_hrtim!(hrtimf, (hi_1, li_1), dt, timer_master.timer, rcc, hr_ctrl);
        let timer2 = init_hrtim!(hrtimc, (hi_2, li_2), dt, timer_master.cr1, rcc, hr_ctrl);
        let timer3 = init_hrtim!(hrtime, (hi_3, li_3), dt, timer_master.cr2, rcc, hr_ctrl);

        #[cfg(feature = "hv4")]
        let timer4b = init_hrtim!(hrtimb, (hi_4), dt, timer_master.cr3, rcc, hr_ctrl);

        #[cfg(feature = "hv4")]
        let timer4d = init_hrtim!(hrtimd, (li_4), dt, timer_master.cr3, rcc, hr_ctrl);

        #[cfg(feature = "hv5")]
        let timer5 = init_hrtim!(hrtima, (hi_5, li_5), dt, timer_master.cr4, rcc, hr_ctrl);
        Timers {
            timer1,
            timer2,
            timer3,
            #[cfg(feature = "hv4")]
            timer4b,
            #[cfg(feature = "hv4")]
            timer4d,
            #[cfg(feature = "hv5")]
            timer5,
        }
    }

    pub fn connect_comparators(mut self, eevs: &Eevs) -> Self {
        self.timer1.out.0.enable_rst_event(eevs.cc1);
        self.timer2.out.0.enable_rst_event(eevs.cc2);
        self.timer3.out.0.enable_rst_event(eevs.cc3);
        #[cfg(feature = "hv4")]
        self.timer4b.out.enable_rst_event(eevs.cc4);
        #[cfg(feature = "hv4")]
        self.timer4d.out.enable_rst_event(eevs.cc4);
        #[cfg(feature = "hv5")]
        self.timer5.out.0.enable_rst_event(eevs.cc4);
        self
    }
}
