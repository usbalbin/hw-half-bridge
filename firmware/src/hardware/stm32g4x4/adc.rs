use fugit::{HertzU32, NanosDurationU32};
use micromath::F32;
use stm32g4xx_hal::{
    self as hal,
    adc::{
        self,
        config::{Resolution, SampleTime},
        Adc, AdcClaim, AdcCommonExt,
    },
    delay::SystDelay,
    gpio::{
        self,
        gpioa::{PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7},
        gpiob::{PB0, PB11, PB14},
        gpioc::{PC0, PC2, PC3, PC4, PC5},
        gpiof::{PF0, PF1},
    },
    rcc::Rcc,
    stasis::Entitlement,
    stm32,
};

use crate::hardware::{F_PLL_P, F_SYS};

pub const ADC_CLOCK_MODE: adc::config::ClockMode = adc::config::ClockMode::AdcHclkDiv4;

const F_ADC: HertzU32 = match ADC_CLOCK_MODE {
    adc::config::ClockMode::AdcKerCk { prescaler, src } => {
        let src = match src {
            adc::config::ClockSource::SystemClock => F_SYS,
            adc::config::ClockSource::PllP => F_PLL_P,
        };

        let prescaler = match prescaler {
            adc::config::Prescaler::Div_1 => 1,
            adc::config::Prescaler::Div_2 => 2,
            adc::config::Prescaler::Div_4 => 4,
            adc::config::Prescaler::Div_6 => 6,
            adc::config::Prescaler::Div_8 => 8,
            adc::config::Prescaler::Div_10 => 10,
            adc::config::Prescaler::Div_12 => 12,
            adc::config::Prescaler::Div_16 => 16,
            adc::config::Prescaler::Div_32 => 32,
            adc::config::Prescaler::Div_64 => 64,
            adc::config::Prescaler::Div_128 => 128,
            adc::config::Prescaler::Div_256 => 256,
        };

        HertzU32::Hz(src.to_Hz() / prescaler)
    }
    adc::config::ClockMode::AdcHclkDiv1 => F_SYS,
    adc::config::ClockMode::AdcHclkDiv2 => HertzU32::Hz(F_SYS.to_Hz() / 2),
    adc::config::ClockMode::AdcHclkDiv4 => HertzU32::Hz(F_SYS.to_Hz() / 4),
};

pub struct Adcs {
    pub adc1: Adc<stm32::ADC1, adc::Configured>,

    #[cfg(feature = "hw_triggered_adc2")]
    pub adc2: adc::DynamicAdc<stm32::ADC2>,

    #[cfg(not(feature = "hw_triggered_adc2"))]
    pub adc2: Adc<stm32::ADC2, adc::Configured>,
    #[allow(dead_code)]
    pub adc3: Adc<stm32::ADC3, adc::Configured>,
    #[allow(dead_code)]
    pub adc4: Adc<stm32::ADC4, adc::Configured>,
    #[allow(dead_code)]
    pub adc5: Adc<stm32::ADC5, adc::Configured>,
}

impl Adcs {
    pub(crate) fn init(
        ad_channels: &AdcChannels,
        adc2_trigger: impl Into<adc::config::ExternalTrigger12>,
        adc12_common: stm32::ADC12_COMMON,
        adc345_common: stm32::ADC345_COMMON,
        adc1: stm32::ADC1,
        adc2: stm32::ADC2,
        adc3: stm32::ADC3,
        adc4: stm32::ADC4,
        adc5: stm32::ADC5,
        delay: &mut SystDelay,
        rcc: &mut Rcc,
    ) -> Self {
        defmt::info!("Initializing ADCs...");

        let cfg = ADC_CLOCK_MODE;

        defmt::assert!(F_ADC.to_MHz() <= 52);

        let adc12_common = adc12_common.claim(cfg, rcc);
        let adc345_common = adc345_common.claim(cfg, rcc);

        let adc1 =
            adc12_common.claim_and_configure(adc1, hal::adc::config::AdcConfig::default(), delay);

        #[cfg(not(feature = "hw_triggered_adc2"))]
        let adc2 =
            adc12_common.claim_and_configure(adc2, hal::adc::config::AdcConfig::default(), delay);

        #[cfg(feature = "hw_triggered_adc2")]
        let adc2 = {
            let cfg = hal::adc::config::AdcConfig::<adc::config::ExternalTrigger12>::default()
                .external_trigger(adc::config::TriggerMode::RisingEdge, adc2_trigger.into())
                .end_of_conversion_interrupt(adc::config::Eoc::Sequence)
                .continuous(adc::config::Continuous::Single)
                .subgroup_len(adc::config::SubGroupLength::One);

            let adc = adc12_common.claim_and_configure(adc2, cfg, delay);

            let mut adc = adc.into_dynamic_adc();
            adc.reset_sequence();
            adc.configure_channel(
                &ad_channels.fb_a,
                adc::config::Sequence::One,
                adc::config::SampleTime::Cycles_12_5,
            );

            adc
        };

        let adc3 =
            adc345_common.claim_and_configure(adc3, hal::adc::config::AdcConfig::default(), delay);

        let adc4 =
            adc345_common.claim_and_configure(adc4, hal::adc::config::AdcConfig::default(), delay);

        let adc5 =
            adc345_common.claim_and_configure(adc5, hal::adc::config::AdcConfig::default(), delay);

        Adcs {
            adc1,
            adc2,
            adc3,
            adc4,
            adc5,
        }
    }

    pub fn read(&mut self, ad_channels: &AdcChannels) {
        let sample_time = hal::adc::config::SampleTime::Cycles_12_5;
        //let fast_sample_time = hal::adc::config::SampleTime::Cycles_6_5; // Should be fine for current signals since come from the current amplifiers with ~20R @ 1MHz

        //adc1.convert(&op1_comp1_b_cc4_pin_fb_a, sample_time);
        self.adc1.convert(&ad_channels.ntc_1, sample_time);
        self.adc1.convert(&ad_channels.ntc_2, sample_time);
        self.adc1.convert(&ad_channels.ntc_3, sample_time);
        self.adc1.convert(&ad_channels.ntc_4, sample_time);
        self.adc1.convert(&ad_channels.ntc_5, sample_time);
        self.adc1.convert(&ad_channels.adc12_in8_pot, sample_time);

        #[cfg(feature = "cs-op")]
        self.adc1.convert(&ad_channels.cc4, sample_time);

        #[cfg(not(feature = "fb_a-op"))]
        self.adc2.convert(&ad_channels.fb_a, sample_time);
        //adc2.convert(&op1_comp1_b_cc4_pin_fb_a, sample_time);
        //adc2.convert(&op2_pin_fb_b, sample_time);
        self.adc2.convert(&ad_channels.ntc_1, sample_time);
        self.adc2.convert(&ad_channels.ntc_2, sample_time);
        //adc2.convert(&ntc_3, sample_time);
        //adc2.convert(&ntc_4, sample_time);
        self.adc2.convert(&ad_channels.ntc_5, sample_time);
        self.adc2.convert(&ad_channels.fb_c, sample_time);
        self.adc2.convert(&ad_channels.adc12_in8_pot, sample_time);
        self.adc2.convert(&ad_channels.fb1_lo, sample_time);
        self.adc2.convert(&ad_channels.fb1_hi, sample_time);
        self.adc2.convert(&ad_channels.fb_d, sample_time);
        self.adc2.convert(&ad_channels.fb_b, sample_time);
        self.adc2.convert(&ad_channels.fb_a, sample_time);

        #[cfg(feature = "cs-op")]
        self.adc2.convert(&ad_channels.cc5, sample_time);

        #[cfg(feature = "cs-op")]
        self.adc2.convert(&ad_channels.cc1, sample_time);
        //self.adc2.convert(&ad_channels.pwm_led8_adc2_in12, sample_time);

        //self.adc2.convert(&ad_channels.cc1a, sample_time); // Use OP2 or OP3 instead
        //self.adc2.convert(&ad_channels.cc1b, sample_time);
        //self.adc2.convert(&ad_channels.cc3, sample_time); // Use OP2 or OP5 insead

        #[cfg(feature = "cs-op")]
        self.adc3.convert(&ad_channels.cc1, sample_time);

        #[cfg(feature = "cs-op")]
        self.adc5.convert(&ad_channels.cc2, sample_time);
        #[cfg(feature = "cs-op")]
        self.adc5.convert(&ad_channels.cc3, sample_time);
        //self.adc5.convert(&op5, sample_time);
    }

    pub fn adc_to_voltage(x: u16) -> f32 {
        const X: f32 = 3.3 / 4095.0;
        f32::from(x) * X
    }

    pub fn voltage_to_adc(v: f32) -> u16 {
        const X: f32 = 4095.0 / 3.3;
        (v * X).clamp(u16::MIN as _, u16::MAX as _) as u16
    }

    pub fn degrees_c_to_adc(t: f32) -> u16 {
        use micromath::F32Ext;

        let t: f32 = t + 273.15; // To kelvin
        let r_pull_up = 10_000.0;
        let r_ntc_25c = 10_000.0;
        let beta = 4100.0;
        let t_ref = 273.15 + 25.0;
        let vcc = 3.3;

        let r_ntc = r_ntc_25c * (beta * (1.0 / t - 1.0 / t_ref)).exp();

        let v_adc = (r_ntc * vcc) / (r_pull_up + r_ntc);

        Self::voltage_to_adc(v_adc)
    }

    pub fn adc_to_degreec_c(x: u16) -> f32 {
        let r_pull_up = 10_000.0;
        let r_ntc_25c = 10_000.0;
        let beta = 4100.0;
        let t_ref = 273.15 + 25.0;
        let vcc = 3.3;

        let v_adc = Self::adc_to_voltage(x);
        let r_ntc = (r_pull_up * v_adc) / (vcc - v_adc);

        //let r_ntc / r_ntc_25c = e.pow(beta * (t1_inv - t0_inv));

        // ln e^x = x;

        //let beta * (1.0/t1 - 1.0/t0) = ln(r_ntc / r_ntc_25c);
        let t = 1.0 / ((F32::ln(F32(r_ntc / r_ntc_25c)).0 / beta) + 1.0 / t_ref);

        t - 273.15
    }

    pub fn adc_to_ma_buck(x: u16, zero: u16) -> f32 {
        let v_adc = Self::adc_to_voltage(x);
        let v_zero = Self::adc_to_voltage(zero);

        let v_relative = v_adc - v_zero;

        let volts_per_amp = -0.066; // Negative in buck direction
        let amp = v_relative / volts_per_amp;

        amp * 1000.0
    }
}

pub struct AdcChannels {
    //op1_comp1_b_cc4_pin_fb_a: PA1<gpio::Analog>,
    pub ntc_1: PC0<gpio::Analog>, //ok
    pub ntc_2: PC3<gpio::Analog>, //ok
    pub ntc_3: PA2<gpio::Analog>, //ok
    pub ntc_4: PF0<gpio::Analog>, //ok
    pub ntc_5: PA0<gpio::Analog>, //ok
    pub adc12_in8_pot: PC2<gpio::Analog>,
    pub adc1_in4_pot2_pwm_led5: PA3<gpio::Analog>,

    #[cfg(not(feature = "cs-op"))]
    pub cc1: Entitlement<PB0<gpio::Analog>>,
    //cc1b: PC1<gpio::Analog>, // No op available on this pin unless the signal is routed to cc5 by mounting R28
    #[cfg(not(feature = "cs-op"))]
    pub cc2: Entitlement<PB11<gpio::Analog>>,
    #[cfg(not(feature = "cs-op"))]
    pub cc3: Entitlement<PB14<gpio::Analog>>,
    #[cfg(not(feature = "cs-op"))]
    pub cc4: Entitlement<PA1<gpio::Analog>>,
    #[cfg(not(feature = "cs-op"))]
    pub cc5: Entitlement<PA7<gpio::Analog>>,

    #[cfg(feature = "cs-op")]
    pub cc1: opamp::Follower<opamp::Opamp3, Entitlement<PB0<gpio::Analog>>, opamp::InternalOutput>,
    #[cfg(feature = "cs-op")]
    pub cc2: opamp::Follower<opamp::Opamp4, Entitlement<PB11<gpio::Analog>>, opamp::InternalOutput>,
    #[cfg(feature = "cs-op")]
    pub cc3: opamp::Follower<opamp::Opamp5, Entitlement<PB14<gpio::Analog>>, opamp::InternalOutput>,
    #[cfg(feature = "cs-op")]
    pub cc4: opamp::Follower<opamp::Opamp1, Entitlement<PA1<gpio::Analog>>, opamp::InternalOutput>,
    #[cfg(feature = "cs-op")]
    pub cc5: opamp::Follower<opamp::Opamp2, Entitlement<PA7<gpio::Analog>>, opamp::InternalOutput>,

    //op12_comp2_cc5_pin_b: PA7<gpio::Analog>,
    pub fb1_lo: PA4<gpio::Analog>,
    pub fb1_hi: PA5<gpio::Analog>,

    #[cfg(not(feature = "fb_a-op"))]
    pub fb_a: PC5<gpio::Analog>, // Replaces pwm_led5 pot2
    pub fb_b: PF1<gpio::Analog>,
    pub fb_c: PA6<gpio::Analog>,
    pub fb_d: PC4<gpio::Analog>,

    #[cfg(feature = "fb_a-op")]
    pub fb_a: opamp1::Follower<PA3<gpio::Analog>>, // Replaces pwm_led7
                                                   //pwm_led8_adc2_in12: PB2<gpio::Analog>,// already used
}

pub const fn sampling_time(sample_time: SampleTime, res: Resolution) -> NanosDurationU32 {
    // All these should have an additional 0.5 cycle.
    // However the same thing for the `res` so we just add 1 in the end
    let cycles_sampl = match sample_time {
        SampleTime::Cycles_2_5 => 2,
        SampleTime::Cycles_6_5 => 6,
        SampleTime::Cycles_12_5 => 12,
        SampleTime::Cycles_24_5 => 24,
        SampleTime::Cycles_47_5 => 47,
        SampleTime::Cycles_92_5 => 92,
        SampleTime::Cycles_247_5 => 247,
        SampleTime::Cycles_640_5 => 640,
    };

    let cycles_sar = match res {
        Resolution::Twelve => 12,
        Resolution::Ten => 10,
        Resolution::Eight => 8,
        Resolution::Six => 6,
    };

    let cycles: u64 = cycles_sampl + cycles_sar + 1;

    NanosDurationU32::nanos((cycles * 1_000_000_000 / F_ADC.raw() as u64) as u32)
}
