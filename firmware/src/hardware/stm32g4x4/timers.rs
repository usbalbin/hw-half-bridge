use stm32_hrtim::{
    self,
    control::HrPwmControl,
    output::{HrOut1, HrOut2, HrOutput},
    timer::HrSlaveTimer,
    timer_eev_cfg::EevCfgs,
    DacResetOnCounterReset, DacStepOnCmp2, HrParts, HrPwmAdvExt,
};
use stm32g4xx_hal::{
    gpio,
    hrtim::HrPwmBuilderExt,
    stm32::{HRTIM_MASTER, HRTIM_TIMA, HRTIM_TIMB, HRTIM_TIMC, HRTIM_TIMD, HRTIM_TIME, HRTIM_TIMF},
};

use crate::hardware::REPETITION_COUNTER;

use super::{external_events::Eevs, Prescaler, PERIOD};

type DacRst = DacResetOnCounterReset;
type DacStp = DacStepOnCmp2;
type Timer<TIM> = HrParts<
    TIM,
    Prescaler,
    (
        HrOut1<TIM, Prescaler, DacRst, DacStp>,
        HrOut2<TIM, Prescaler, DacRst, DacStp>,
    ),
    DacRst,
    DacStp,
>;

pub type TimHb1 = HRTIM_TIMF;
pub type TimHb2 = HRTIM_TIMC;
pub type TimHb3 = HRTIM_TIME;
pub type TimHb4b = HRTIM_TIMB;
pub type TimHb4d = HRTIM_TIMD;
pub type TimHb5 = HRTIM_TIMA;

pub type MasterTimer = HrParts<HRTIM_MASTER, Prescaler, ()>;
pub type TimerHb1 = Timer<TimHb1>;
pub type TimerHb2 = Timer<TimHb2>;
pub type TimerHb3 = Timer<TimHb3>;
pub type TimerHb4b =
    HrParts<TimHb4b, Prescaler, HrOut1<TimHb4b, Prescaler, DacRst, DacStp>, DacRst, DacStp>;
pub type TimerHb4d =
    HrParts<TimHb4d, Prescaler, HrOut2<TimHb4d, Prescaler, DacRst, DacStp>, DacRst, DacStp>;
pub type TimerHb5 = Timer<TimHb5>;

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
            .dac_trigger_cfg(DacResetOnCounterReset, DacStepOnCmp2)
            //.repetition_counter(repetition_counter)
            //.enable_repetition_interrupt()
            .finalize(&mut $hr_control);

        let pin = &mut timer.out;
        $(
            let $li_pin = ();
            let _ = $li_pin;
            let pin = &mut pin.0;
        )*

        timer.timer.enable_reset_event(&$rst_evt);
        pin.enable_set_event(&$rst_evt);
        pin.enable_rst_event(&timer.cr1);

        timer
    }};
}

pub struct Timers {
    pub control: HrPwmControl,
    pub master_timer: MasterTimer,
    pub timer1: TimerHb1,
    pub timer2: TimerHb2,
    pub timer3: TimerHb3,

    #[cfg(feature = "hv4")]
    pub timer4b: TimerHb4b,
    #[cfg(feature = "hv4")]
    pub timer4d: TimerHb4d,
    #[cfg(feature = "hv5")]
    pub timer5: TimerHb5,
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

        hi_1: gpio::gpioc::PC6,
        li_1: gpio::gpioc::PC7,
        hi_2: gpio::gpiob::PB12,
        li_2: gpio::gpiob::PB13,
        hi_3: gpio::gpioc::PC8,
        li_3: gpio::gpioc::PC9,
        #[cfg(feature = "hv4")] li_4: gpio::gpiob::PB15,
        #[cfg(feature = "hv4")] hi_4: gpio::gpioa::PA10,
        #[cfg(feature = "hv5")] hi_5: gpio::gpioa::PA8,
        #[cfg(feature = "hv5")] li_5: gpio::gpioa::PA9,
        mut hr_ctrl: HrPwmControl,
    ) -> Timers {
        defmt::info!("Initializing Timers...");

        let master_timer = hrtim_master
            .pwm_advanced(())
            .prescaler(stm32_hrtim::Pscl1)
            .period(PERIOD)
            .repetition_counter(REPETITION_COUNTER)
            .enable_repetition_interrupt()
            .finalize(&mut hr_ctrl);

        let dt = stm32_hrtim::deadtime::DeadtimeConfig::default()
            .deadtime_falling_value(super::DEADTIME_FALLING_TICKS)
            .deadtime_rising_value(super::DEADTIME_RISING_TICKS);

        let timer1 = init_hrtim!(hrtimf, (hi_1, li_1), dt, master_timer.timer, rcc, hr_ctrl);
        let timer2 = init_hrtim!(hrtimc, (hi_2, li_2), dt, master_timer.cr1, rcc, hr_ctrl);
        let timer3 = init_hrtim!(hrtime, (hi_3, li_3), dt, master_timer.cr2, rcc, hr_ctrl);

        #[cfg(feature = "hv4")]
        let timer4b = init_hrtim!(hrtimb, (hi_4), dt, master_timer.cr3, rcc, hr_ctrl);

        #[cfg(feature = "hv4")]
        let timer4d = init_hrtim!(hrtimd, (li_4), dt, master_timer.cr3, rcc, hr_ctrl);

        #[cfg(feature = "hv5")]
        let timer5 = init_hrtim!(hrtima, (hi_5, li_5), dt, master_timer.cr4, rcc, hr_ctrl);
        Timers {
            control: hr_ctrl,
            master_timer,
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

    #[cfg(not(feature = "hv5"))]
    pub fn connect_filtered_comparators(mut self, eevs: &Eevs) -> Self {
        self.timer1.out.0.enable_rst_event(&eevs.cc1_filt);
        self.timer2.out.0.enable_rst_event(&eevs.cc2_filt);
        self.timer3.out.0.enable_rst_event(&eevs.cc3_filt);
        #[cfg(feature = "hv4")]
        self.timer4b.out.enable_rst_event(&eevs.cc4_filt);
        #[cfg(feature = "hv4")]
        self.timer4d.out.enable_rst_event(&eevs.cc4_filt);

        self.timer5.out.0.enable_rst_event(&eevs.cc4_filt);
        self
    }

    pub fn connect_fast_comparators(mut self, eevs: &Eevs) -> Self {
        self.timer1.out.0.enable_rst_event(&eevs.cc1_fast);
        self.timer2.out.0.enable_rst_event(&eevs.cc2_fast);
        self.timer3.out.0.enable_rst_event(&eevs.cc3_fast);
        #[cfg(feature = "hv4")]
        self.timer4b.out.enable_rst_event(&eevs.cc4_fast);
        #[cfg(feature = "hv4")]
        self.timer4d.out.enable_rst_event(&eevs.cc4_fast);
        #[cfg(feature = "hv5")]
        self.timer5.out.0.enable_rst_event(&eevs.cc5_fast);
        self
    }
}
