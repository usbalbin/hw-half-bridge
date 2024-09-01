#![no_main]
#![no_std]

use half_bridge as _; // global logger + panicking-behavior + memory layout

#[rtic::app(
    device = stm32g4xx_hal::stm32,
)]
mod app {
    use cortex_m::delay::Delay;
    use stm32g4xx_hal::{
        self as hal,
        adc::{self, Adc, AdcClaim},
        comparator::{self, split, ComparatorExt, ComparatorSplit},
        dac::{self, Dac3Ch1, Dac3Ch2, Dac4Ch1, Dac4Ch2, DacExt},
        delay::SYSTDelayExt,
        gpio::{
            self,
            gpioa::{PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7},
            gpiob::{PB0, PB11, PB14, PB2},
            gpioc::{PC0, PC1, PC2, PC3, PC4, PC5},
            gpiof::{PF0, PF1},
            GpioExt,
        },
        hrtim::{
            self,
            control::{HrControltExt, HrPwmControl, HrTimCalibrated},
            external_event::{EevInput, EevInputs, EevSamplingFilter, ExternalEventSource},
            output::HrOutput,
            timer::HrSlaveTimer,
            timer_eev_cfg::EevCfgs,
            HrPwmAdvExt,
        },
        opamp::{
            opamp1::{self, Follower, IntoFollower},
            opamp2::{self, IntoFollower as _},
            opamp3::{self, IntoFollower as _},
            opamp4::{self, IntoFollower as _},
            opamp5::{self, IntoFollower as _},
            OpampEx,
        },
        pwm::{self, PwmExt as _},
        pwr::{self, PwrExt},
        rcc::{self, Rcc, RccExt},
        serial::SerialExt,
        stm32::{self, UCPD1},
        time::{Hertz, RateExtU32 as _},
    };

    macro_rules! try_down_cast {
        ($x:expr, $from_t:ty, $t:ty) => {{
            if $x > <$t>::MAX as $from_t {
                panic!("Conversion failed")
            }

            $x as $t
        }};
    }

    // <System Clocks>
    pub const SYS_PLL_SOURCE: rcc::PllSrc = rcc::PllSrc::HSI;
    pub const SYS_PLL_N_MUL: rcc::PllNMul = rcc::PllNMul::MUL_80;
    pub const SYS_PLL_M_DIV: rcc::PllMDiv = rcc::PllMDiv::DIV_4;
    pub const SYS_PLL_R_DIV: rcc::PllRDiv = rcc::PllRDiv::DIV_2;

    pub const F_SYS: Hertz = Hertz::Hz(
        SYS_PLL_SOURCE.frequency().raw() * SYS_PLL_N_MUL.multiplier()
            / SYS_PLL_M_DIV.divisor()
            / SYS_PLL_R_DIV.divisor(),
    );

    pub const F_SW: Hertz = Hertz::MHz(1);
    pub const PERIOD: u16 = try_down_cast!(F_SYS.raw() as u64 * 32 / F_SW.raw() as u64, u64, u16);
    const I_FILTER: EevSamplingFilter = EevSamplingFilter::None;

    const ADC_POST_SCALER: hrtim::control::AdcTriggerPostscaler =
        hrtim::control::AdcTriggerPostscaler::Div31;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        // TODO: Add resources
    }

    macro_rules! init_hrtim {
        ($tim:expr, ($hi_pin:ident $(, $li_pin:ident)*), $comp_eev:expr, $deadtime:expr, $rst_evt:expr, $rcc:expr, $hr_control:expr) => {{
            let (mut timer, (cr1, cr2, _cr3, _cr4), (mut $hi_pin $(, $li_pin)*), _) = $tim
                .pwm_advanced(($hi_pin $(, $li_pin)*), &mut $rcc)
                .deadtime($deadtime)
                .prescaler(hrtim::Pscl1)
                .period(PERIOD)
                .out1_polarity(pwm::Polarity::ActiveHigh)
                .out2_polarity(pwm::Polarity::ActiveHigh)
                .preload(hrtim::PreloadSource::OnCounterReset)
                .timer_mode(hrtim::HrTimerMode::SingleShotRetriggerable)
                .counting_direction(hrtim::HrCountingDirection::Up)
                .eev_cfg(EevCfgs::default())
                //.repetition_counter(repetition_counter)
                //.enable_repetition_interrupt()
                .finalize(&mut $hr_control);

            timer.enable_reset_event(&$rst_evt);
            $hi_pin.enable_set_event(&$rst_evt);
            $hi_pin.enable_rst_event(&cr1);
            $hi_pin.enable_rst_event(&$comp_eev);

            (timer, cr1, cr2, $hi_pin $(, $li_pin)*)
        }};
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let cp = cx.core;
        let dp = cx.device;

        let pwr = dp.PWR.constrain();
        let pwr_cfg = pwr // Enable boost mode to allow f_sys > 150MHz
            .vos(pwr::VoltageScale::Range1 { enable_boost: true })
            .freeze();

        // Set system frequency to 16MHz * 80/4/2 = 160MHz
        // This would lead to HrTim running at 160MHz * 32 = 5.12...
        let rcc_cfg = rcc::Config::pll().pll_cfg(rcc::PllConfig {
            mux: SYS_PLL_SOURCE,
            n: SYS_PLL_N_MUL,
            m: SYS_PLL_M_DIV,
            r: Some(SYS_PLL_R_DIV),

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
            usb: stm32::USB,
            #[allow(dead_code)]
            usb_pd: stm32::UCPD1,

            #[allow(dead_code)]
            usb_dm: gpio::gpioa::PA11<gpio::Input<gpio::Floating>>,
            #[allow(dead_code)]
            usb_dp: gpio::gpioa::PA12<gpio::Input<gpio::Floating>>,

            #[allow(dead_code)]
            cc1: gpio::gpiob::PB6<gpio::Input<gpio::Floating>>,
            #[allow(dead_code)]
            cc2: gpio::gpiob::PB4<gpio::Input<gpio::Floating>>,

            #[allow(dead_code)]
            dbcc1: gpio::gpioa::PA9<gpio::Input<gpio::Floating>>,
            #[allow(dead_code)]
            dbcc2: gpio::gpioa::PA10<gpio::Input<gpio::Floating>>,

            frs: gpio::gpioc::PC12<gpio::Input<gpio::Floating>>,

            en_vconn: gpio::gpioc::PC10<gpio::Input<gpio::Floating>>,

            en_cc: gpio::gpioc::PC13<gpio::Input<gpio::Floating>>,
        }

        struct Swd {
            #[allow(dead_code)]
            swdio: gpio::gpioa::PA13<gpio::Input<gpio::Floating>>,
            #[allow(dead_code)]
            swc: gpio::gpioa::PA14<gpio::Input<gpio::Floating>>,
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
        let pc10 = gpioc.pc10;
        let pc11 = gpioc.pc11;
        let pc12 = gpioc.pc12;
        let pc13 = gpioc.pc13;
        let pc14 = gpioc.pc14;
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

            dbcc1: pa9,
            dbcc2: pa10,

            frs: pc12,
            en_vconn: pc10,
            en_cc: pc13,
        };

        // HRTIMF
        let hi1 = pc6;
        let li1 = pc7;

        // HRTIMC
        let hi_2 = pb12;
        let li_2 = pb13;

        // HRTIME
        let hi_3 = pc8;
        let li_3 = pc9;

        // HRTIMD
        //let hi_5 = pb14; // HRTIMD  <--- Used by comp7
        //let li_4 = pb15; // HRTIMD

        //let hi_4 = pa10; // HRTIMB
                         //let li_b = pa11; // HRTIMB <--- Used by USB_DP

        // HRTIMA
        //let hi_5 = pa8;
        //let li_5 = pa9; // Used by dbcc1

        let pwm_led1 = pb5.into_alternate(); // 5v tol
        let pwm_led2 = pb7.into_alternate(); // 5v tol
        let pwm_led3 = pb9.into_alternate(); // 5v tol

        let pwm_led4 = pb10.into_alternate(); // 3.6v max
        //let pwm_led5 = pa3.into_alternate(); // 3.6v max
        let pwm_led6_pot3_adc1_in12_adc3_in1 = pb1.into_alternate();  // 3.6v max
        let pwm_led7_adc2_in11 = pc5.into_analog(); // TIM1_CH4N  // 3.6v max
        let pwm_led8_adc2_in12 = pb2.into_alternate(); // 3.6v max

        let (led2_tim4, led3_tim4) = dp.TIM4.pwm((pwm_led2, pwm_led3), 20.kHz(), &mut rcc);
        let (led3_tim3, led6_tim3) = dp.TIM3.pwm((pwm_led1, pwm_led6_pot3_adc1_in12_adc3_in1), 20.kHz(), &mut rcc);
        let (led4_tim2/*, led5_tim2*/) = dp.TIM2.pwm((pwm_led4/*, pwm_led5*/), 20.kHz(), &mut rcc);
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
        let op1_comp1_b_cc4_pin_fb_a = pa1.into_analog();

        //let comp2_cc5_pin = pa3.into_analog(); // No filter and same DAC as comp4
        let op12_comp2_cc5_pin_b = pa7.into_analog(); // CC5
        // comp3_b_fb_d on pc1

        let op3_comp4_cc1a_pin = pb0.into_analog();
        // let comp4_pin_b = pe7; only on LQFP80 and larger

        //let comp5_pin = pc7.into_analog(); // Used by HRTIMF_CH2
        let op4_comp6_cc2_pin = pb11.into_analog();
        //let comp6_pin_b = pd11; only on LQFP100 and larger

        let op25_comp7_cc3_pin = pb14.into_analog();

        //let comp7_pin_b = pd14; only on LQFP100 and larger

        let ntc_1 = pc0.into_analog();
        let ntc_2_op5 = pc3.into_analog();
        let ntc_3 = pa2.into_analog();
        let ntc_4 = pf0.into_analog();
        let ntc_5_comp3_pin = pa0.into_analog(); // No filter and same DAC as comp1

        //let op1_comp1_b_pin_fb_a = pa1.into_analog();
        // comp4_op3_pin
        let fb_c = pa6.into_analog();
        let comp3_b_fb_d_cc1b_pin = pc1.into_analog();

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

        let dacs = init_dacs(dp.DAC1, dp.DAC2, dp.DAC3, dp.DAC4, &mut rcc);
        let eevs = init_comparators(
            &dacs,
            dp.COMP,
            &op1_comp1_b_cc4_pin_fb_a,
            &op12_comp2_cc5_pin_b,
            &comp3_b_fb_d_cc1b_pin,
            &op3_comp4_cc1a_pin,
            &op4_comp6_cc2_pin,
            &op25_comp7_cc3_pin,
            eev_inputs,
            &mut rcc,
            &mut ctrl,
        );

        let mut hr_ctrl = ctrl.constrain();
        
        // Used to select cc-line for usb pd cable orientation
        let cc_select = pc14;

        // Use to select direction of current measurements 2-5
        let cc_dir = pc15;

        let (op1, op2, op3, op4, op5, _op6) = dp.OPAMP.split(&mut rcc);

        let op1_fb_a = op1.follower(
            op1_comp1_b_cc4_pin_fb_a,
            None::<gpio::gpioa::PA2<hal::gpio::Analog>>,
        ); // PA1 PA3(comp2) PA7
        let op2 = op2.follower(
            op12_comp2_cc5_pin_b,
            None::<gpio::gpioa::PA6<hal::gpio::Analog>>,
        ); // PA7 PB0(comp4) PB14(comp7)
        let op3 = op3.follower(
            op3_comp4_cc1a_pin,
            None::<gpio::gpiob::PB1<hal::gpio::Analog>>,
        ); // PA1 PB0(comp4)
        let op4 = op4.follower(
            op4_comp6_cc2_pin,
            None::<gpio::gpiob::PB12<hal::gpio::Analog>>,
        );
        let op5 = op5.follower(
            op25_comp7_cc3_pin,
            None::<gpio::gpioa::PA8<hal::gpio::Analog>>,
        );
         // PB11(comp6)
           //let op5 = op5.follower(ntc_2_op5, None::<gpio::gpioa::PA8<hal::gpio::Analog>>); // PB14(comp7) PC3
           // let op6 = op6.follower((), None);

        let ad_channels = AdcChannels {
            op1_fb_a,
            //op1_comp1_b_cc4_pin_fb_a,
            ntc_1,
            ntc_2_op5,
            ntc_3,
            ntc_4,
            ntc_5_comp3_pin,
            adc1_in4_pot2_pwm_led5,
            adc12_in8_pot,

            cc1a: op3_comp4_cc1a_pin,
            cc1b: comp3_b_fb_d_cc1b_pin,
            cc2: op4_comp6_cc2_pin,
            cc3: op25_comp7_cc3_pin,
            cc4: op1_comp1_b_cc4_pin_fb_a,
            cc5: op12_comp2_cc5_pin_b,

            //op12_comp2_cc5_pin_b,
            fb_c,
            fb1_lo_adc2_in17,
            fb1_hi_adc2_in13,
            fb_d_adc2_in5,
            fb_b_adc2_in10,
            pwm_led7_adc2_in11,
            //pwm_led8_adc2_in12,
            op2,
            op3,
            op4,
            op5,
        };

        let (master, (mcmp1, mcmp2, mcmp3, mcmp4), _dma) = dp
            .HRTIM_MASTER
            .pwm_advanced((), &mut rcc)
            .enable_repetition_interrupt()
            .prescaler(hrtim::Pscl1)
            .period(PERIOD)
            //.repetition_counter(repetition_counter)
            .enable_repetition_interrupt()
            .finalize(&mut hr_ctrl);

        let dt = hrtim::deadtime::DeadtimeConfig::default();

        let (tim1, cr1, cr2, _out1, _out2) = init_hrtim!(
            dp.HRTIM_TIMF,
            (hi1, li1),
            eevs.cc1a,
            dt,
            master,
            rcc,
            hr_ctrl
        );
        let (tim2, cr1, cr2, _out1, _out2) = init_hrtim!(
            dp.HRTIM_TIMC,
            (hi_2, li_2),
            eevs.cc2,
            dt,
            mcmp1,
            rcc,
            hr_ctrl
        );
        let (tim3, cr1, cr2, _out1, _out2) = init_hrtim!(
            dp.HRTIM_TIME,
            (hi_3, li_3),
            eevs.cc3,
            dt,
            mcmp2,
            rcc,
            hr_ctrl
        );

        let mut delay = cp.SYST.delay(&rcc.clocks);
        init_adcs(
            dp.ADC1, dp.ADC2, dp.ADC3, dp.ADC4, dp.ADC5, &mut delay, &rcc,
        );

        /*let (tim4b, cr1, cr2, _out1) =
            init_hrtim!(dp.HRTIM_TIMB, (hi_4), eevs.cc4, dt, mcmp3, rcc, hr_ctrl);
        let (tim4d, cr1, cr2, _out2) =
            init_hrtim!(dp.HRTIM_TIMD, (li_4), eevs.cc4, dt, mcmp3, rcc, hr_ctrl);

        let (tim5a, cr1, cr2, _out1 /*_out2*/) = init_hrtim!(
            dp.HRTIM_TIMA,
            (hi_5/*, li_5*/),
            eevs.cc4,
            dt,
            mcmp4,
            rcc,
            hr_ctrl
        );*/

        todo!()
    }

    struct Adcs {
        adc1: Adc<stm32::ADC1, adc::Configured>,
        adc2: Adc<stm32::ADC2, adc::Configured>,
        adc3: Adc<stm32::ADC3, adc::Configured>,
        adc4: Adc<stm32::ADC4, adc::Configured>,
        adc5: Adc<stm32::ADC5, adc::Configured>,
    }

    fn init_adcs(
        adc1: stm32::ADC1,
        adc2: stm32::ADC2,
        adc3: stm32::ADC3,
        adc4: stm32::ADC4,
        adc5: stm32::ADC5,
        delay: &mut Delay,
        rcc: &Rcc,
    ) -> Adcs {
        let adc1 = adc1.claim_and_configure(
            hal::adc::ClockSource::SystemClock,
            &rcc,
            hal::adc::config::AdcConfig::default(),
            delay,
            false,
        );

        let adc2 = adc2.claim_and_configure(
            hal::adc::ClockSource::SystemClock,
            &rcc,
            hal::adc::config::AdcConfig::default(),
            delay,
            false,
        );

        let adc3 = adc3.claim_and_configure(
            hal::adc::ClockSource::SystemClock,
            &rcc,
            hal::adc::config::AdcConfig::default(),
            delay,
            false,
        );

        let adc4 = adc4.claim_and_configure(
            hal::adc::ClockSource::SystemClock,
            &rcc,
            hal::adc::config::AdcConfig::default(),
            delay,
            false,
        );

        let adc5 = adc5.claim_and_configure(
            hal::adc::ClockSource::SystemClock,
            &rcc,
            hal::adc::config::AdcConfig::default(),
            delay,
            false,
        );

        Adcs {
            adc1,
            adc2,
            adc3,
            adc4,
            adc5,
        }
    }

    fn init_dacs(
        _dac1: stm32::DAC1,
        _dac2: stm32::DAC2,
        dac3: stm32::DAC3,
        dac4: stm32::DAC4,
        rcc: &mut Rcc,
    ) -> Dacs {
        let dac_ampl = 0;
        // DAC1 and DAC2 are too slow to be useful for generating the sawtooth shape required for
        // slope compensation
        /*let (dac1ch1, dac1ch2) = {
            let (mut dac1ch1, mut dac1ch2) = dp
                .DAC1
                .constrain((dac::Dac1IntSig1, dac::Dac1IntSig2), &mut rcc);
            (
                dac1ch1.enable_generator(dac::GeneratorConfig::sawtooth(dac_ampl)),
                dac1ch2.enable_generator(dac::GeneratorConfig::sawtooth(dac_ampl)),
            )
        };

        let dac2ch1 = dp
            .DAC2
            .constrain(dac::Dac2IntSig1, &mut rcc)
            .enable_generator(dac::GeneratorConfig::sawtooth(dac_ampl));*/

        let (dac3ch1, dac3ch2) = {
            let (ch1, ch2) = dac3.constrain((dac::Dac3IntSig1, dac::Dac3IntSig2), rcc);
            (
                ch1.enable_generator(dac::GeneratorConfig::sawtooth(dac_ampl)),
                ch2.enable_generator(dac::GeneratorConfig::sawtooth(dac_ampl)),
            )
        };

        let (dac4ch1, dac4ch2) = {
            let (ch1, ch2) = dac4.constrain((dac::Dac4IntSig1, dac::Dac4IntSig2), rcc);
            (
                ch1.enable_generator(dac::GeneratorConfig::sawtooth(dac_ampl)),
                ch2.enable_generator(dac::GeneratorConfig::sawtooth(dac_ampl)),
            )
        };

        Dacs {
            dac3ch1,
            dac3ch2,
            dac4ch1,
            dac4ch2,
        }
    }

    struct Eevs {
        cc1a: ExternalEventSource<7, false>,
        cc1b: ExternalEventSource<5, false>,
        cc2: ExternalEventSource<8, false>,

        cc3: ExternalEventSource<10, false>,
        cc4: ExternalEventSource<6, false>,
        cc5: ExternalEventSource<1, false>,
    }

    struct Dacs {
        dac3ch1: Dac3Ch1<0b11, dac::WaveGenerator>,
        dac3ch2: Dac3Ch2<0b11, dac::WaveGenerator>,

        dac4ch1: Dac4Ch1<0b11, dac::WaveGenerator>,
        dac4ch2: Dac4Ch2<0b11, dac::WaveGenerator>,
    }

    fn init_comparators(
        dacs: &Dacs,
        comp: stm32::COMP,
        cc4_comp1_adc12_in2_pin: &PA1<gpio::Analog>,
        cc5_comp2_adc2_in4_pin: &PA7<gpio::Analog>,
        cc1b_comp3_adc12_in7_pin: &PC1<gpio::Analog>,
        cc1a_comp4_adc1_in15__adc3_in12_pin: &PB0<gpio::Analog>,
        cc2_comp6_adc12_in14_pin: &PB11<gpio::Analog>,
        cc3_comp7_adc1_in5__adc4_in4_pin: &PB14<gpio::Analog>,
        eev_inputs: EevInputs,
        rcc: &mut Rcc,
        ctrl: &mut HrTimCalibrated,
    ) -> Eevs {
        macro_rules! init_comp {
            ($comp:expr, $pos_in:expr, $neg_in:expr, $eev_input:expr, $rcc:expr, $hr_control:expr, $($filter:expr)*) => {{
                use stm32g4xx_hal::hrtim::external_event::ToExternalEventSource;
                let comp = $comp
                    .comparator(
                        $pos_in,
                        &$neg_in,
                        comparator::Config::default(),
                        &$rcc.clocks,
                    )
                    .enable()
                    .lock();

                #[allow(unused_mut)]
                let mut eev = $eev_input
                    .bind(&comp)
                    .edge_or_polarity(hrtim::external_event::EdgeOrPolarity::Polarity(
                        pwm::Polarity::ActiveHigh,
                    ));
                $(eev = eev.filter($filter);)*

                eev.finalize($hr_control)
            }};
        }

        let (comp1, comp2, comp3, comp4, _comp5, comp6, comp7) = comp.split(rcc);

        // filt=eev6 // fast=eev4,
        let comp1_cc4 = init_comp!(
            comp1,
            cc4_comp1_adc12_in2_pin, // ok
            dacs.dac3ch1,
            eev_inputs.eev_input6,
            rcc,
            ctrl,
            I_FILTER
        );

        // fast=eev1 // filt=eev6
        let comp2_cc5 = init_comp!(
            comp2,
            cc5_comp2_adc2_in4_pin, // ok
            dacs.dac3ch2,
            eev_inputs.eev_input1,
            rcc,
            ctrl, /* No filter */
        ); // <-- Same DAC as comp4

        // fast=eev5, // filt=eev8
        let comp3_cc1b = init_comp!(
            comp3,
            cc1b_comp3_adc12_in7_pin, // ok
            dacs.dac3ch1,
            eev_inputs.eev_input5,
            rcc,
            ctrl, /* No filter */
        ); // Same DAC as comp1

        // filt=eev7 // fast=eev2, filt=eev9
        let comp4_cc1a = init_comp!(
            comp4,
            cc1a_comp4_adc1_in15__adc3_in12_pin, // ok, TODO: Is ADC3 good enough?
            dacs.dac3ch2,
            eev_inputs.eev_input7,
            rcc,
            ctrl,
            I_FILTER
        ); // Same DAC as comp2

        // filt=eev9 // fast=eev5
        //let comp5 = init_comp!(comp5, comp5_pin, dac4ch1, eev_input9, rcc, hr_control); // <-- No available in_pos, DAC same as comp7

        // filt=eev8, // fast=eev3,
        let comp6_cc2 = init_comp!(
            comp6,
            cc2_comp6_adc12_in14_pin, // ok
            dacs.dac4ch2,
            eev_inputs.eev_input8,
            rcc,
            ctrl,
            I_FILTER
        );

        // filt=eev10, //fast=eev5
        let comp7_cc3 = init_comp!(
            comp7,
            cc3_comp7_adc1_in5__adc4_in4_pin, // ok
            dacs.dac4ch1,
            eev_inputs.eev_input10,
            rcc,
            ctrl,
            I_FILTER
        );

        let cc1a = comp4_cc1a;
        let cc1b = comp3_cc1b;
        let cc2 = comp6_cc2;

        let cc3 = comp7_cc3;
        let cc4 = comp1_cc4;
        let cc5 = comp2_cc5; // WARNING uses same DAC as cc1, so will have wonky slope compensation, adjust phase order to minimize bad stuff

        Eevs {
            cc1a,
            cc1b,
            cc2,
            cc3,
            cc4,
            cc5,
        }
    }

    struct AdcChannels {
        //op1_comp1_b_cc4_pin_fb_a: PA1<gpio::Analog>,
        ntc_1: PC0<gpio::Analog>,           //ok
        ntc_2_op5: PC3<gpio::Analog>,       //ok
        ntc_3: PA2<gpio::Analog>,           //ok
        ntc_4: PF0<gpio::Analog>,           //ok
        ntc_5_comp3_pin: PA0<gpio::Analog>, //ok
        adc12_in8_pot: PC2<gpio::Analog>,
        adc1_in4_pot2_pwm_led5: PA3<gpio::Analog>,

        cc1a: PB0<gpio::Analog>,
        cc1b: PC1<gpio::Analog>,
        cc2: PB11<gpio::Analog>,
        cc3: PB14<gpio::Analog>,
        cc4: PA1<gpio::Analog>,
        cc5: PA7<gpio::Analog>,

        //op12_comp2_cc5_pin_b: PA7<gpio::Analog>,
        op1_fb_a: opamp1::Follower<PA1<gpio::Analog>>,
        fb_b_adc2_in10: PF1<gpio::Analog>,
        fb_c: PA6<gpio::Analog>,
        fb_d_adc2_in5: PC4<gpio::Analog>,

        fb1_lo_adc2_in17: PA4<gpio::Analog>,
        fb1_hi_adc2_in13: PA5<gpio::Analog>,
        
        pwm_led7_adc2_in11: PC5<gpio::Analog>,
        //pwm_led8_adc2_in12: PB2<gpio::Analog>,// already used

        op2: opamp2::Follower<PA7<gpio::Analog>>, // ok, is also CS1A, CS3 and CS5
        op3: opamp3::Follower<PB0<gpio::Analog>>, // ok, is also CS1A
        op4: opamp4::Follower<PB11<gpio::Analog>>,
        op5: opamp5::Follower<PB14<gpio::Analog>>, // CS3
    }

    fn read_adcs(adcs: &mut Adcs, ad_channels: &AdcChannels) {
        let sample_time = hal::adc::config::SampleTime::Cycles_12_5;
        let fast_sample_time = hal::adc::config::SampleTime::Cycles_6_5; // Should be fine for current signals since come from the current amplifiers with ~20R @ 1MHz
        adcs.adc1.convert(&ad_channels.op1_fb_a, sample_time);
        //adc1.convert(&op1_comp1_b_cc4_pin_fb_a, sample_time);
        adcs.adc1.convert(&ad_channels.ntc_1, sample_time);
        adcs.adc1.convert(&ad_channels.ntc_2_op5, sample_time);
        adcs.adc1.convert(&ad_channels.ntc_3, sample_time);
        adcs.adc1.convert(&ad_channels.ntc_4, sample_time);
        adcs.adc1.convert(&ad_channels.ntc_5_comp3_pin, sample_time);
        adcs.adc1.convert(&ad_channels.adc12_in8_pot, sample_time);

        //adc2.convert(&op1_comp1_b_cc4_pin_fb_a, sample_time);
        //adc2.convert(&op2_pin_fb_b, sample_time);
        adcs.adc2.convert(&ad_channels.ntc_1, sample_time);
        adcs.adc2.convert(&ad_channels.ntc_2_op5, sample_time);
        //adc2.convert(&ntc_3, sample_time);
        //adc2.convert(&ntc_4, sample_time);
        adcs.adc2.convert(&ad_channels.ntc_5_comp3_pin, sample_time);
        adcs.adc2.convert(&ad_channels.fb_c, sample_time);
        adcs.adc2.convert(&ad_channels.adc12_in8_pot, sample_time);
        adcs.adc2.convert(&ad_channels.fb1_lo_adc2_in17, sample_time);
        adcs.adc2.convert(&ad_channels.fb1_hi_adc2_in13, sample_time);
        adcs.adc2.convert(&ad_channels.fb_d_adc2_in5, sample_time);
        adcs.adc2.convert(&ad_channels.fb_b_adc2_in10, sample_time);
        adcs.adc2.convert(&ad_channels.pwm_led7_adc2_in11, sample_time);
        
        adcs.adc2.convert(&ad_channels.op2, sample_time);
        adcs.adc2.convert(&ad_channels.op3, sample_time);
        //adcs.adc2.convert(&ad_channels.pwm_led8_adc2_in12, sample_time);
        
        //adcs.adc2.convert(&ad_channels.cc1a, sample_time); // Use OP2 or OP3 instead
        adcs.adc2.convert(&ad_channels.cc1b, sample_time);
        adcs.adc2.convert(&ad_channels.cc2, sample_time);
        //adcs.adc2.convert(&ad_channels.cc3, sample_time); // Use OP2 or OP5 insead
        adcs.adc2.convert(&ad_channels.cc4, sample_time);
        adcs.adc2.convert(&ad_channels.cc5, sample_time);

        adcs.adc3.convert(&ad_channels.op3, sample_time);

        adcs.adc5.convert(&ad_channels.op4, sample_time);
        adcs.adc5.convert(&ad_channels.op5, sample_time);
        //adc5.convert(&op5, sample_time);
    }
}
