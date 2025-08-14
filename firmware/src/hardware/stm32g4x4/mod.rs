pub mod adc;
pub mod dacs;
pub mod external_events;
pub mod timers;
//pub mod comparators;

use core::mem;

use adc::{AdcChannels, Adcs};
use dacs::Dacs;
use external_events::Eevs;
use fugit::NanosDurationU32;
use stm32_hrtim::external_event::EevSamplingFilter;
use stm32g4xx_hal::{
    self as hal,
    dac::SawtoothConfig,
    delay::{SYSTDelayExt, SystDelay},
    gpio::{self, GpioExt},
    hrtim::HrControltExt,
    opamp::OpampEx,
    pwr::{self, PwrExt},
    rcc::{self, RccExt},
    serial::SerialExt,
    stasis::Freeze,
    stm32::{self, Peripherals},
    time::Hertz,
    timer::MonoTimer,
};
use timers::Timers;

use crate::control_2p2z::DacSettings;

macro_rules! try_down_cast {
    ($x:expr, $from_t:ty, $t:ty) => {{
        if $x > <$t>::MAX as $from_t {
            panic!("Conversion failed")
        }

        $x as $t
    }};
}

// <System Clocks>
pub const SYS_PLL_SOURCE: rcc::PllSrc = rcc::PllSrc::HSI; // 16MHz
pub const SYS_PLL_N_MUL: rcc::PllNMul = rcc::PllNMul::MUL_85;
pub const SYS_PLL_M_DIV: rcc::PllMDiv = rcc::PllMDiv::DIV_4;
pub const SYS_PLL_R_DIV: rcc::PllRDiv = rcc::PllRDiv::DIV_2;

pub const SYS_PLL_P_DIV: rcc::PllPDiv = rcc::PllPDiv::DIV_7; // For ADC
pub const F_ADC: Hertz = Hertz::Hz(
    SYS_PLL_SOURCE.frequency().raw() * SYS_PLL_N_MUL.multiplier()
        / SYS_PLL_M_DIV.divisor()
        / SYS_PLL_P_DIV.divisor(),
);

pub const F_SYS: Hertz = Hertz::Hz(
    SYS_PLL_SOURCE.frequency().raw() * SYS_PLL_N_MUL.multiplier()
        / SYS_PLL_M_DIV.divisor()
        / SYS_PLL_R_DIV.divisor(),
);

pub type Prescaler = stm32_hrtim::Pscl1;

/// Switch frequency
pub const F_SW: Hertz = Hertz::MHz(1);

pub const DEADTIME: NanosDurationU32 = NanosDurationU32::nanos(32);

pub const DEADTIME_TICKS: u16 = try_down_cast!(
    (DEADTIME.ticks() as u32 * 8 * F_SYS.to_MHz() as u32).div_ceil(1000),
    u32,
    u16
);

pub const DEADTIME_FALLING_TICKS: u16 = DEADTIME_TICKS;
pub const DEADTIME_RISING_TICKS: u16 = DEADTIME_TICKS;

/// Switch period in number of ticks
pub const PERIOD: u16 = try_down_cast!(F_SYS.raw() as u64 * 32 / F_SW.raw() as u64, u64, u16);

/// Interrupt tick rate
pub const TICK_RATE: Hertz = Hertz::kHz(10);

pub const REPETITION_COUNTER: u8 = try_down_cast!(F_SW.raw() / TICK_RATE.raw() - 1, u32, u8);

pub const I_FILTER: EevSamplingFilter = EevSamplingFilter::None;

pub const ADC_POST_SCALER: stm32_hrtim::control::AdcTriggerPostscaler =
    stm32_hrtim::control::AdcTriggerPostscaler::Div31;

pub struct Hardware {
    pub timers: Timers,
    pub adcs: Adcs,
    pub ad_channels: AdcChannels,
    pub eevs: Eevs,
    pub dacs: Dacs,
    pub delay: SystDelay,
    pub nucleo_user_button: gpio::PC13<gpio::Input>,
    pub debug_timer: MonoTimer,
}

impl Hardware {
    pub fn init(dp: Peripherals, cp: cortex_m::Peripherals, dac_cfg: SawtoothConfig) -> Hardware {
        defmt::info!("Initializing Hardware...");

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

        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpiob = dp.GPIOB.split(&mut rcc);
        let gpioc = dp.GPIOC.split(&mut rcc);
        let gpiod = dp.GPIOD.split(&mut rcc);
        let gpiof = dp.GPIOF.split(&mut rcc);
        //let gpiog = dp.GPIOG.split(&mut rcc);

        let usb_pd = dp.UCPD1;
        let usb = dp.USB;

        struct FakeUsb {
            #[allow(dead_code)]
            usb: stm32::USB,
            #[allow(dead_code)]
            usb_pd: stm32::UCPD1,

            #[allow(dead_code)]
            usb_dm: gpio::gpioa::PA11,
            #[allow(dead_code)]
            usb_dp: gpio::gpioa::PA12,

            #[allow(dead_code)]
            cc1: gpio::gpiob::PB6,
            #[allow(dead_code)]
            cc2: gpio::gpiob::PB4<gpio::Debugger>,

            #[allow(dead_code)]
            #[cfg(feature = "usb-pd-db")]
            dbcc1: gpio::gpioa::PA9,

            #[allow(dead_code)]
            #[cfg(feature = "usb-pd-db")]
            dbcc2: gpio::gpioa::PA10,

            #[cfg(feature = "usb-pd-db")]
            frs: gpio::gpioc::PC12,

            #[cfg(feature = "usb-pd-db")]
            en_vconn: gpio::gpioc::PC10,

            #[cfg(feature = "usb-pd-db")]
            en_cc: gpio::gpioc::PC13,

            // Used to select cc-line for usb pd cable orientation
            #[cfg(feature = "usb-pd-db")]
            cc_select: gpio::gpioc::PC14,

            // Use to select direction of current measurements 2-5
            #[cfg(feature = "usb-pd-db")]
            cc_dir: gpio::gpioc::PC15,
        }

        struct Swd {
            #[allow(dead_code)]
            swdio: gpio::gpioa::PA13<gpio::Debugger>,
            #[allow(dead_code)]
            swc: gpio::gpioa::PA14<gpio::Debugger>,
        }

        let pa0 = gpioa.pa0;
        let pa1 = gpioa.pa1;
        let pa2 = gpioa.pa2;
        let pa3 = gpioa.pa3;
        let pa4 = gpioa.pa4;
        let pa5 = gpioa.pa5;
        let pa6 = gpioa.pa6;
        let pa7 = gpioa.pa7;
        let pa8 = gpioa.pa8;
        let pa9 = gpioa.pa9;
        let pa10 = gpioa.pa10;
        let pa11 = gpioa.pa11;
        let pa12 = gpioa.pa12;
        let pa13 = gpioa.pa13;
        let pa14 = gpioa.pa14;
        let pa15 = gpioa.pa15;

        let pb0 = gpiob.pb0;
        let pb1 = gpiob.pb1;
        let pb2 = gpiob.pb2;
        let pb3 = gpiob.pb3;
        let pb4 = gpiob.pb4;
        let pb5 = gpiob.pb5;
        let pb6 = gpiob.pb6;
        let pb7 = gpiob.pb7;
        let pb8 = gpiob.pb8;
        let pb9 = gpiob.pb9;
        let pb10 = gpiob.pb10;
        let pb11 = gpiob.pb11;
        let pb12 = gpiob.pb12;
        let pb13 = gpiob.pb13;
        let pb14 = gpiob.pb14;
        let pb15 = gpiob.pb15;

        let pc0 = gpioc.pc0;
        let pc1 = gpioc.pc1;
        let pc2 = gpioc.pc2;
        let pc3 = gpioc.pc3;
        let pc4 = gpioc.pc4;
        let pc5 = gpioc.pc5;
        let pc6 = gpioc.pc6;
        let pc7 = gpioc.pc7;
        let pc8 = gpioc.pc8;
        let pc9 = gpioc.pc9;
        #[cfg(feature = "usb-pd-db")]
        let pc10 = gpioc.pc10;
        let pc11 = gpioc.pc11;
        #[cfg(feature = "usb-pd-db")]
        let pc12 = gpioc.pc12;
        #[cfg(feature = "usb-pd-db")]
        let pc13 = gpioc.pc13;
        #[cfg(not(feature = "usb-pd-db"))]
        let pc13 = gpioc.pc13.into_floating_input();
        #[cfg(feature = "usb-pd-db")]
        let pc14 = gpioc.pc14;
        #[cfg(feature = "usb-pd-db")]
        let pc15 = gpioc.pc15;

        let pd2 = gpiod.pd2;

        let pf0 = gpiof.pf0;
        let pf1 = gpiof.pf1;

        // LO: CC1 and CC2 are connected to PA9 and PA10
        // HI: HI_4, LI_4, HI_5 and LI_5 are connected to PA8,9,10 and PB15
        let cc_hi4_hi5_mux = pd2.into_push_pull_output();

        //let pg10 = gpiog.pg10;comp3_b_fb_d_cc1b_pin
        //let _reset_pin = pg10;
        let _boot_pin = pb8;

        let _swd = Swd {
            swdio: pa13,
            swc: pa14,
        };

        let _usb = FakeUsb {
            usb,
            usb_pd: usb_pd,

            usb_dm: pa11,
            usb_dp: pa12,

            cc1: pb6,
            cc2: pb4,

            #[cfg(feature = "usb-pd-db")]
            dbcc1: pa9,
            #[cfg(feature = "usb-pd-db")]
            dbcc2: pa10,

            #[cfg(feature = "usb-pd-db")]
            frs: pc12,
            #[cfg(feature = "usb-pd-db")]
            en_vconn: pc10,
            #[cfg(feature = "usb-pd-db")]
            en_cc: pc13,

            #[cfg(feature = "usb-pd-db")]
            cc_select: pc14,
            #[cfg(feature = "usb-pd-db")]
            cc_dir: pc15,
        };

        let debug_timer = MonoTimer::new(cp.DWT, cp.DCB, &rcc.clocks);

        // HRTIMF
        let f_hi = pc6;
        let f_li = pc7;

        // HRTIMC
        let c_hi = pb12;
        let c_li = pb13;

        // HRTIME
        let e_hi = pc8;
        let e_li = pc9;

        // HRTIMD
        #[cfg(feature = "hv4")]
        let d_li = pb15; // HRTIMD

        #[cfg(feature = "hv4")]
        let b_li = pa10; // HRTIMB
                         //let li_b = pa11; // HRTIMB <--- Used by USB_DP

        // HRTIMA
        //#[cfg(feature = "hv5")]
        let a_hi = pa8;
        //#[cfg(feature = "hv5")]
        let a_li = pa9; // Used by dbcc1

        //let mosi_pin = pb5.into_alternate(); // 5v tol

        #[cfg(feature = "leds")]
        let pwm_led2 = pb7.into_alternate(); // 5v tol
        #[cfg(feature = "leds")]
        let pwm_led3 = pb9.into_alternate(); // 5v tol

        #[cfg(feature = "leds")]
        let pwm_led4 = pb10.into_alternate(); // 3.6v max

        let pwm_led7_adc2_in11 = pc5.into_analog(); // TIM1_CH4N  // 3.6v max
        #[cfg(feature = "leds")]
        let pwm_led8_adc2_in12 = pb2.into_alternate(); // 3.6v max

        #[cfg(feature = "leds")]
        let (led2_tim4, led3_tim4) = dp.TIM4.pwm((pwm_led2, pwm_led3), 20.kHz(), &mut rcc);
        #[cfg(feature = "leds")]
        let led4_tim2 = dp.TIM2.pwm((pwm_led4/*, pwm_led5*/), 20.kHz(), &mut rcc);
        #[cfg(feature = "leds")]
        let led8_tim5 = dp.TIM5.pwm(pwm_led8_adc2_in12, 20.kHz(), &mut rcc);

        let tx = pb3.into_alternate();
        let rx = pa15.into_alternate();
        let uart = dp
            .USART2
            .usart(
                tx,
                rx,
                stm32g4xx_hal::serial::FullConfig::default(),
                &mut rcc,
            )
            .unwrap();

        //let comp1_cc4_pin = pb1.into_analog();
        let (_, [op1_cc4_pin, comp1_b_cc4_pin]) = pa1.into_analog().freeze();

        //let comp2_cc5_pin = pa3.into_analog(); // No filter and same DAC as comp4
        let (_, [op12_cc5_pin_b, comp2_cc5_pin_b]) = pa7.into_analog().freeze(); // CC5
                                                                                 // comp3_b_fb_d on pc1

        let (_, [op3_cc1_pin, comp4_cc1_pin]) = pb0.into_analog().freeze();
        // let comp4_pin_b = pe7; only on LQFP80 and larger

        //let comp5_pin = pc7.into_analog(); // Used by HRTIMF_CH2
        let (_, [op4_cc2_pin, comp6_cc2_pin]) = pb11.into_analog().freeze();
        //let comp6_pin_b = pd11; only on LQFP100 and larger

        let (_, [op25_cc3_pin, comp7_cc3_pin]) = pb14.into_analog().freeze();

        //let comp7_pin_b = pd14; only on LQFP100 and larger

        let ntc_1 = pc0.into_analog();
        let ntc_2 = pc3.into_analog();
        let ntc_3 = pa2.into_analog();
        let ntc_4 = pf0.into_analog();
        let ntc_5 = pa0.into_analog(); // No filter and same DAC as comp1

        //let op1_comp1_b_pin_fb_a = pa1.into_analog();
        // comp4_op3_pin
        let fb_c = pa6.into_analog();
        //let comp3_b_cc1b_pin = pc1.into_analog();

        let adc12_in8_pot = pc2.into_analog();
        let adc1_in4_pot2_pwm_led5 = pa3.into_analog();
        let fb1_lo_adc2_in17 = pa4.into_analog();
        let fb1_hi_adc2_in13 = pa5.into_analog();
        let fb_d_adc2_in5 = pc4.into_analog();
        let fb_b_adc2_in10 = pf1.into_analog();

        let (mut ctrl, _f, eev_inputs) = dp
            .HRTIM_COMMON
            .hr_control(&mut rcc)
            .set_adc1_trigger_psc(ADC_POST_SCALER)
            .set_adc2_trigger_psc(ADC_POST_SCALER)
            .set_adc3_trigger_psc(ADC_POST_SCALER)
            .set_adc4_trigger_psc(ADC_POST_SCALER)
            .wait_for_calibration();

        // TODO: Figure out something safer
        let dac_tokens = unsafe { mem::transmute(()) };
        let eevs = Eevs::init(
            dac_tokens,
            dp.COMP,
            comp4_cc1_pin,
            //&comp3_b_cc1b_pin,
            comp6_cc2_pin,
            comp7_cc3_pin,
            comp1_b_cc4_pin,
            comp2_cc5_pin_b,
            eev_inputs,
            &mut rcc,
            &mut ctrl,
        );

        let hr_ctrl = ctrl.constrain();

        let timers = Timers::init(
            dp.HRTIM_MASTER,
            dp.HRTIM_TIMA,
            dp.HRTIM_TIMB,
            dp.HRTIM_TIMC,
            dp.HRTIM_TIMD,
            dp.HRTIM_TIME,
            dp.HRTIM_TIMF,
            /*#[cfg(feature = "hv5")]
            a_hi,*/
            /*#[cfg(feature = "hv5")]
            a_li,*/
            #[cfg(feature = "hv1")]
            a_hi, //f_hi,
            #[cfg(feature = "hv1")]
            a_li, //f_li,
            #[cfg(feature = "hv2")]
            c_hi,
            #[cfg(feature = "hv2")]
            c_li,
            #[cfg(feature = "hv3")]
            e_hi,
            #[cfg(feature = "hv3")]
            e_li,
            #[cfg(feature = "hv4")]
            d_li,
            #[cfg(feature = "hv4")]
            b_li,
            hr_ctrl,
        );

        //DAC --ref-voltage--> Comp ----> Eev ----> HRTIM --dac-trigger--> DAC
        let (dacs, dac_tokens) = Dacs::init(
            dp.DAC1, dp.DAC2, dp.DAC3, dp.DAC4, &timers, &mut rcc, dac_cfg,
        );

        let (op1, op2, op3, op4, op5, _op6) = dp.OPAMP.split(&mut rcc);

        #[cfg(feature = "cs-op")]
        let op1 = op1.follower(op1_cc4_pin, InternalOutput); // PA1 PA3(comp2) PA7
        #[cfg(feature = "cs-op")]
        let op2 = op2.follower(op12_cc5_pin_b, InternalOutput); // PA7 PB0(comp4) PB14(comp7)
        #[cfg(feature = "cs-op")]
        let op3 = op3.follower(op3_cc1_pin, InternalOutput); // PA1 PB0(comp4)
        #[cfg(feature = "cs-op")]
        let op4 = op4.follower(op4_cc2_pin, InternalOutput);
        #[cfg(feature = "cs-op")]
        let op5 = op5.follower(op25_cc3_pin, InternalOutput);
        // PB11(comp6)
        //let op5 = op5.follower(ntc_2_op5, None::<gpio::gpioa::PA8<hal::gpio::Analog>>); // PB14(comp7) PC3
        // let op6 = op6.follower((), None);

        let nucleo_user_button = pc13;

        let ad_channels = AdcChannels {
            //op1_comp1_b_cc4_pin_fb_a,
            ntc_1,
            ntc_2,
            ntc_3,
            ntc_4,
            ntc_5,
            adc1_in4_pot2_pwm_led5,
            adc12_in8_pot,

            #[cfg(not(feature = "cs-op"))]
            cc1: op3_cc1_pin,
            //#[cfg(not(feature = "cs-op"))]
            //cc1b: comp3_b_cc1b_pin,
            #[cfg(not(feature = "cs-op"))]
            cc2: op4_cc2_pin,
            #[cfg(not(feature = "cs-op"))]
            cc3: op25_cc3_pin,
            #[cfg(not(feature = "cs-op"))]
            cc4: op1_cc4_pin,
            #[cfg(not(feature = "cs-op"))]
            cc5: op12_cc5_pin_b,

            #[cfg(feature = "cs-op")]
            cc1: op3,
            //#[cfg(feature = "cs-op")]
            //cc1b: comp3_b_cc1b_pin,
            #[cfg(feature = "cs-op")]
            cc2: op4,
            #[cfg(feature = "cs-op")]
            cc3: op5,
            #[cfg(feature = "cs-op")]
            cc4: op1,
            #[cfg(feature = "cs-op")]
            cc5: op2,

            //op12_comp2_cc5_pin_b,
            fb1_lo: fb1_lo_adc2_in17,
            fb1_hi: fb1_hi_adc2_in13,
            fb_a: pwm_led7_adc2_in11,
            fb_b: fb_b_adc2_in10,
            fb_c,
            fb_d: fb_d_adc2_in5,
            //pwm_led8_adc2_in12,
        };

        let mut delay = cp.SYST.delay(&rcc.clocks);
        let adcs = Adcs::init(
            dp.ADC12_COMMON,
            dp.ADC345_COMMON,
            dp.ADC1,
            dp.ADC2,
            dp.ADC3,
            dp.ADC4,
            dp.ADC5,
            &mut delay,
            &mut rcc,
        );

        // With a hardware modification this could be used to drive the WS2812b LEDs
        //let spi_mode = spi::Mode::default();
        //let spi = dp.SPI1.spi(mosi_pin, spi_mode, 3.MHz(), &mut rcc);

        let timers = timers.connect_fast_comparators(&eevs);

        defmt::info!("Initializing Hardware - Done");

        Hardware {
            timers,
            adcs,
            ad_channels,
            eevs,
            dacs,
            delay,
            nucleo_user_button,
            debug_timer,
        }
    }
}
