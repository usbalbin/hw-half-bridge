#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use embassy_stm32::{bind_interrupts, dac::{self, DacChannel}, dma};
use full_control::control_2p2z::{
    self, DacSettings, Topology, TransferFunction, TwoPoleTwoZeroParams,
};
use test_app as _; // global logger + panicking-behavior + memory layout
use embassy_stm32::peripherals::DMA1_CH1;
bind_interrupts!(struct Irqs {
    DMA1_CHANNEL1 => dma::InterruptHandler<DMA1_CH1>;
});

const PARAMS: control_2p2z::Parameters = control_2p2z::Parameters {
    v_out: TARGET as f64,
    v_diode: 0.0,
    c_out: 47e-6,
    f_sw: 500e3,
    l_inductor: 2e-6,
    r_esr_out_cap: 5e-3,
    current_sense_gain: 66e-3,
    safety_factor: 2.0,
    i_load: 1.0,
    phase_margin: control_2p2z::PhaseMargin::Manual {
        phase_margin: 75.0f64.to_radians(),
    },
    cycles_per_tick: 1,
};

const TF_AND_DAC: (TransferFunction, DacSettings) =
    PARAMS.to_transfer_function(3.3, Topology::Buck);
const DAC_SLOPE: f64 = TF_AND_DAC.1.dac_slope;
const CONTROL_PARAMS: TwoPoleTwoZeroParams<f32> = TF_AND_DAC.0.to_2p2z();
const TARGET: f32 = 1.0;

/// Load current (A)
const LOAD_PROFILE: [f32; 12] = [1e-3, 5.0, 1e-3, 50e-3, 1e-3, 1e-3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

struct MyDac(DacChannel<'static, embassy_stm32::mode::Blocking>);

impl load::Dac for MyDac {
    const V_MAX: f32 = 3.3;

    fn set_voltage(&mut self, voltage: f32) {
        self.0.set(dac::Value::Bit12Right(
            (voltage as f32 * (4095.0 / Self::V_MAX)) as u16,
        ));
    }

    fn set_waveform(&mut self, _values: &[u16]) {
        todo!()
    }
}

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    dispatchers = [SPI4],
    device = embassy_stm32,
)]
mod app {
    use super::*;
    use embassy_stm32::{
        Config, adc::{self, Adc, AdcChannel, AdcConfig, InjectedAdc, SampleTime}, dac::Dac, gpio::{Output, Speed}, hrtim::{
            self, HrControltExt, HrPwmBuilderExt as _, Parts,
            stm32_hrtim::{
                DacResetOnCounterReset, DacStepOnCmp2, HrCountingDirection, HrPwmAdvExt,
                HrTimerMode, Polarity, PreloadSource, compare_register::HrCompareRegister,
                timer_eev_cfg::EevCfgs,
            },
        }, peripherals::ADC1, rcc::{Pll, PllMul, PllPDiv, PllPreDiv, PllRDiv, PllSource, Sysclk, mux::Adcsel}, triggers
    };
    use full_control::control_2p2z::TwoPoleTwoZero;
    use load::Load;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        adc: InjectedAdc<'static, ADC1, 1>,
        controller: TwoPoleTwoZero<f32>,
        //load: Load<MyDac, embassy_stm32::gpio::Output<'static>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let config = {
            let mut config = Config::default();
            config.rcc.hsi = true;
            config.rcc.pll = Some(Pll {
                source: PllSource::HSI,
                divp: Some(PllPDiv::DIV7),
                divq: None,
                divr: Some(PllRDiv::DIV2),
                prediv: PllPreDiv::DIV4,
                mul: PllMul::MUL85,
            });
            config.rcc.sys = Sysclk::PLL1_R;
            config.rcc.mux.adc12sel = Adcsel::PLL1_P;
            config
        };
        let p = embassy_stm32::init(config);

        let cfg = AdcConfig::default();
        let adc = Adc::new(p.ADC1, cfg);

        let v = p.PA0.degrade_adc();

        let a_li = hrtim::Pin {
            speed: Speed::Medium,
            pin: p.PA8,
        };
        let a_hi = hrtim::Pin {
            speed: Speed::Medium,
            pin: p.PA9,
        };

        let prescaler = hrtim::Pscl1;

        let Parts { control, tima, .. } = p.HRTIM1.hr_control();
        let (control, ..) = control.wait_for_calibration();
        let mut control = control.constrain();

        let mut timer = tima
            .pwm_advanced(a_li, a_hi)
            .prescaler(prescaler)
            .period(5440) // 170MHz * 32 / 1 / 5440 = 1MHz
            .out1_polarity(Polarity::ActiveHigh)
            .out2_polarity(Polarity::ActiveHigh)
            .preload(PreloadSource::OnCounterReset)
            .timer_mode(HrTimerMode::Continuous)
            .counting_direction(HrCountingDirection::Up)
            .eev_cfg(EevCfgs::default())
            .dac_trigger_cfg(DacResetOnCounterReset, DacStepOnCmp2)
            //.repetition_counter(repetition_counter)
            //.enable_repetition_interrupt()
            .finalize(&mut control);

        timer.cr4.set_duty(2048); // Set ADC sample point
        control.adc_trigger2.enable_source(&timer.cr4);

        let adc = adc.setup_injected_conversions(
            [(v, SampleTime::CYCLES47_5)],
            triggers::HRTIM_ADC_TRG2,
            adc::Exten::RISING_EDGE,
            true
        );

        let controller = CONTROL_PARAMS.to_controller(0.0, 4095.0);

        let load_dac2 = DacChannel::new_blocking(p.DAC2, p.PA6);
        let load_dac = MyDac(load_dac2);

        let mut load = load::Load::new(
            load_dac,
            Output::new(p.PA5, embassy_stm32::gpio::Level::Low, Speed::Low),
        );
        load.set_range(load::Range::High);

        //let ref_dac = DacChannel::new_triggered(p.DAC1, dma, trigger, irq, p.PA4);

        task0::spawn(load).ok();

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                adc,
                controller,
                //load,
            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }

    // TODO: Add tasks
    #[task(priority = 1)]
    async fn task0(_cx: task0::Context, mut load: Load<MyDac, Output<'static>>) {
        loop {
            for i in LOAD_PROFILE {
                load.set_current_manual_range(i);
                defmt::println!("Load: {}mA", i * 1000.0);
                embassy_time::Timer::after_millis(1).await;
            }
        }
    }

    #[task(binds = ADC1_2, local = [adc, controller, i: usize = 0])]
    fn task1(cx: task1::Context) {
        //let x = cx.local.adc.read_injected_samples()[0];
        //let error = TARGET - x as f32;
        //cx.local.controller.update(error);

        let i = cx.local.i;
        //cx.local.load.set_current_auto_range(LOAD_PROFILE[(*i / 4000) % LOAD_PROFILE.len()]);
        *i += 1;
    }
}
