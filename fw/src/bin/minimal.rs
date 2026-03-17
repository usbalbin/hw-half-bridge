#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

use embassy_stm32::comp;
use embassy_stm32::hrtim::Pscl1;
use embassy_stm32::hrtim::stm32_hrtim::capture::HrCaptCh1;
use embassy_stm32::hrtim::stm32_hrtim::control::AdcTriggerPostscaler;
use embassy_stm32::hrtim::stm32_hrtim::output::HrOut;
use embassy_stm32::hrtim::stm32_hrtim::pac::HRTIM_TIMA;
use embassy_stm32::hrtim::stm32_hrtim::timer::{Ch1, Ch2};
use embassy_stm32::hrtim::stm32_hrtim::{DacResetOnCounterReset, DacStepOnCmp2, NoDacTrigger};
use embassy_stm32::peripherals::{COMP1, DMA1_CH1, DMA1_CH2};
use embassy_stm32::time::Hertz;
use embassy_stm32::{
    bind_interrupts,
    dac::{self, DacChannel},
    dma,
};
use fugit::NanosDurationU32;
use full_control::control_2p2z::{
    self, DacSettings, Parameters, PhaseMargin, Topology, TransferFunction, TwoPoleTwoZeroParams,
};
use test_app as _; // global logger + panicking-behavior + memory layout
bind_interrupts!(struct Irqs {
    DMA1_CHANNEL1 => dma::InterruptHandler<DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<DMA1_CH2>;
    COMP1_2_3 => comp::InterruptHandler<COMP1>;
});
const F_SYS: fugit::HertzU32 = fugit::HertzU32::MHz(170);
// ── Physical circuit constants (same as buck-boost-test.rs) ─────────────────
const F_SW: f64 = 500e3; // 500 kHz
const T_PERIOD: f64 = 1.0 / F_SW;
const F_HR: u64 = F_SYS.raw() as u64 * 32;
const PERIOD_TICKS: u16 = (F_HR / F_SW as u64) as u16;
const V_IN: f64 = 24.0; // nominal
const V_IN_MIN: f64 = 15.0; // worst-case for slope compensation (highest D, largest required S_e)
const V_IN_MAX: f64 = 28.0; // worst-case for on-time (shortest, fewest slope steps available)
const V_TARGET: f64 = 14.4;
const V_TARGET_INIT: f64 = 5.0;
const C_OUT: f64 = 10e-6; // ~10 µF
const L_INDUCTOR: f64 = 2e-6; // 2 µH
const R_ESR: f64 = 10e-3; // 10 mΩ
const CS_GAIN: f64 = 0.066; // ACS37030: 66 mV/A
const R_LOAD: f64 = 2.9; // 14.4V / 5A max charge current
/// Maximum trip-current the controller can command (A).
/// Must exceed the peak inductor current PLUS the slope-compensation offset
/// at the trip point (≈ |slope| × t_on ≈ 2.6 A at D = 0.56).  Set higher than
/// the physical inductor-current limit to provide DAC headroom.
const MAX_CURRENT: f64 = 12.0;

// ── STM32G474 ADC / DAC ────────────────────────────────────────────────────
const V_REF: f64 = 3.3;
const ADC_MAX: f64 = 4095.0; // 12-bit
const LSB: f64 = V_REF / ADC_MAX; // ~0.806 mV per code
const ADC_POST_SCALER: AdcTriggerPostscaler = AdcTriggerPostscaler::Div4;

// ── Feedback resistor divider (VOUT sensing, PC5) ───────────────────────────
const R_FB_HI: f64 = 20_000.0; // 20 kΩ
const R_FB_LO: f64 = 1_000.0; // 1 kΩ
const DIVIDER_RATIO: f64 = R_FB_LO / (R_FB_HI + R_FB_LO); // ≈ 0.04808
// 5.0 V × 0.04808 ≈ 0.240 V → TARGET_CODE ≈ 298

// ── VIN sensing resistor divider (PC4) ──────────────────────────────────────
const R_VIN_HI: f64 = 20_000.0; // 20 kΩ
const R_VIN_LO: f64 = 1_000.0; // 1 kΩ
const VIN_SCALE: f64 = (R_VIN_HI + R_VIN_LO) / R_VIN_LO; // = 21.0  (ADC code → V_in)

// ── Controller target and DAC limits in codes ───────────────────────────────
const TARGET_CODE_INIT: f64 = vout_to_code(V_TARGET_INIT as f32) as f64;
const DAC_MAX_CODE: f64 = MAX_CURRENT * CS_GAIN / LSB; // ≈ 819

const fn vout_to_code(vout: f32) -> u16 {
    (vout as f64 * DIVIDER_RATIO / LSB) as u16
}

// ── 2P2Z compensator design ────────────────────────────────────────────────
const CTRL_PARAMS: Parameters = Parameters {
    v_out: V_TARGET,
    c_out: C_OUT,
    f_sw: F_SW,
    l_inductor: L_INDUCTOR,
    r_esr_out_cap: R_ESR,
    current_sense_gain: CS_GAIN,
    i_load: V_TARGET / R_LOAD,
    v_diode: 0.0,
    phase_margin: PhaseMargin::Manual {
        phase_margin: 75.0f64.to_radians(),
    },
    safety_factor: 5.0,
    cycles_per_tick: ADC_POST_SCALER as usize + 1,
};

// Transfer function and DAC settings at the Buck operating point.
// Use V_IN_MIN: at minimum input voltage D is highest, requiring the largest slope compensation.
// Over-compensation at higher Vin is benign; under-compensation at low Vin risks subharmonic oscillation.
const TF_DAC: (TransferFunction, DacSettings) =
    CTRL_PARAMS.to_transfer_function(V_IN_MIN, Topology::Buck);

// Physical-domain weights (error in Volts, output in current-sense Volts)
const WEIGHTS_PHYS: TwoPoleTwoZeroParams<f32> = TF_DAC.0.to_2p2z();

// Code-domain weights: b-coefficients scaled by 1/divider_ratio.
// The a-coefficients are unchanged (they multiply past outputs already in codes).
const WEIGHTS_CODE: TwoPoleTwoZeroParams<f32> = TwoPoleTwoZeroParams {
    a1: WEIGHTS_PHYS.a1,
    a2: WEIGHTS_PHYS.a2,
    b0: (WEIGHTS_PHYS.b0 as f64 / DIVIDER_RATIO) as f32,
    b1: (WEIGHTS_PHYS.b1 as f64 / DIVIDER_RATIO) as f32,
    b2: (WEIGHTS_PHYS.b2 as f64 / DIVIDER_RATIO) as f32,
};

/// Minimum DAC steps that must occur during the HRTIM on-time.
///
/// Step smoothness vs. slope accuracy: each step changes the current trip
/// threshold by only ~7–15 mA while the inductor current ripple is ~4 A —
/// so any value ≥ 4 is electrically irrelevant.  8 is a conservative default.
const MIN_SLOPE_STEPS_DURING_ON_TIME: u16 = 8;

/// Approximate on-time in HR ticks (Buck: D = V_target / V_in).
/// Use V_IN_MAX: at maximum input voltage the on-time is shortest, bounding the slope-step search
/// so that the selected CR2 always fits within the available on-time across the full Vin range.
const ON_TIME_TICKS: u16 = (V_TARGET / V_IN_MAX * PERIOD_TICKS as f64) as u16;

/// Brute-force search over CR2 ∈ [97, on_time_ticks/min_steps] for the
/// (CR2, INCDATA_reg) pair that minimises **over**-compensation.
///
/// Rounding direction: **ceiling** (always round up).
/// Rationale: over-compensation (S_e > designed) is benign — the converter
/// transitions gradually toward voltage mode.  Under-compensation risks
/// subharmonic oscillations at D > 50%.
///
/// Slope identity: achieved = (INCDATA_reg / 16) × f_hr / CR2  [DAC codes/s]
const fn best_slope_params(
    target_slope_codes_per_s: f64,
    f_hr: u64,
    on_time_ticks: u16,
    min_steps: u16,
) -> (u16, u16) {
    // dac_slope is negative (down-slope in V/s); take the magnitude.
    let slope = if target_slope_codes_per_s < 0.0 {
        -target_slope_codes_per_s
    } else {
        target_slope_codes_per_s
    };
    // Ideal INCDATA_reg (12.4 fp) per HR tick
    let target_per_tick = slope * 16.0 / f_hr as f64;
    // Upper bound: CR2 ≤ on_time / min_steps keeps min_steps steps per on-time
    let cr2_max = on_time_ticks / min_steps;

    let mut best_cr2: u16 = 0;
    let mut best_step: u16 = 0;
    let mut best_over_err: f64 = 1.0; // 100%; any real value beats this

    let mut cr2: u16 = 97; // HRTIM CMP must be strictly > 0x60 = 96
    while cr2 <= cr2_max {
        let ideal = target_per_tick * cr2 as f64;
        // Ceiling via integer arithmetic (.round()/.ceil() require std on no_std targets)
        let floor = ideal as u16;
        let step_reg = if (floor as f64) < ideal { floor + 1 } else { floor };
        if step_reg > 0 {
            let over_err = (step_reg as f64 - ideal) / ideal; // ≥ 0 for ceiling
            if best_step == 0 || over_err < best_over_err {
                best_over_err = over_err;
                best_cr2 = cr2;
                best_step = step_reg;
            }
        }
        cr2 += 1;
    }
    assert!(best_step > 0);
    (best_cr2, best_step)
}

const SLOPE_PARAMS: (u16, u16) = best_slope_params(
    TF_DAC.1.dac_slope / LSB,
    F_HR,
    ON_TIME_TICKS,
    MIN_SLOPE_STEPS_DURING_ON_TIME,
);

/// HRTIM CMP2 ticks between consecutive DAC sawtooth steps.
/// Chosen by brute-force to minimise over-compensation with at least
/// MIN_SLOPE_STEPS_DURING_ON_TIME steps during the switch on-time.
pub const HR_TICKS_PER_DAC_INC: u16 = SLOPE_PARAMS.0;

/// DAC sawtooth step value in 12.4 fixed-point format (`set_sawtooth_step_value`).
/// This is the compile-time value for V_IN_MIN; the runtime value lives in DAC_STEP_LIVE.
pub const DAC_STEP_SIZE: u16 = SLOPE_PARAMS.1;

/// Multiply a DAC slope magnitude (V/s) by this constant to get the INCDATA register
/// value for the fixed HR_TICKS_PER_DAC_INC (CR2).
///
/// INCDATA = |S_e| / LSB  ×  16 / F_HR  ×  CR2
///
/// Used by `task1` to update the slope compensation ramp from measured Vin without
/// re-running the full `best_slope_params` search.
const SLOPE_TO_INCDATA: f32 =
    (16.0 * HR_TICKS_PER_DAC_INC as f64 / (LSB * F_HR as f64)) as f32;

/// Load current (A) – 12 steps × 1 ms each
const LOAD_PROFILE: [f32; 12] = [
    1e-3, 1.0, 1e-3, 0.5, 1e-3, 50e-3, 1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
];

/// Control-loop rate (Hz): switching frequency divided by the ADC postscaler.
const CTRL_HZ: usize = F_SW as usize / (ADC_POST_SCALER as usize + 1); // 125 000 Hz (Div4 = ÷4)
/// Total duration of each load-profile step in milliseconds.
const STEP_MS: usize = 1000;
/// Capture window in milliseconds at the start of each step (must be < STEP_MS).
const CAPTURE_MS: usize = 20;
const _: () = assert!(CAPTURE_MS < STEP_MS, "CAPTURE_MS must be less than STEP_MS");
/// Number of V_out samples captured per load-step transition.
/// = CTRL_HZ × CAPTURE_MS / 1000
const CAPTURE_LEN: usize = CTRL_HZ * CAPTURE_MS / 1000;

/// Monotonically-increasing write index managed by the control-loop ISR.
///   usize::MAX  → idle / consumed
///   0 …< CAPTURE_LEN → capture in progress
///   >= CAPTURE_LEN   → capture complete, safe to read
///
/// `#[no_mangle]` keeps the symbol name predictable so host tools can find it by name.
#[unsafe(no_mangle)]
static CAPTURE_IDX: core::sync::atomic::AtomicUsize =
    core::sync::atomic::AtomicUsize::new(usize::MAX);

/// Every V_out ADC code captured during one complete load-profile cycle.
///
/// Written by `control_loop` (priority 15) when `CAPTURE_IDX < CAPTURE_LEN`.
/// Read by `task0` (priority 1) after `CAPTURE_IDX >= CAPTURE_LEN`.
///
/// `#[no_mangle]` keeps the symbol name predictable so it can be read back with:
///   addr=$(nm target/thumbv7em-none-eabihf/debug/minimal | awk '/CAPTURE_BUF/{print "0x"$1}')
///   probe-rs read b16 $addr 500 --chip stm32g474retx
#[unsafe(no_mangle)]
static mut CAPTURE_BUF: [u16; CAPTURE_LEN] = [0; CAPTURE_LEN];

pub const DEADTIME: NanosDurationU32 = NanosDurationU32::nanos(32);

pub const DEADTIME_TICKS: u16 =
    (DEADTIME.ticks() as u32 * 8 * F_SYS.to_MHz() as u32).div_ceil(1000) as u16;

pub const DEADTIME_FALLING_TICKS: u16 = DEADTIME_TICKS;
pub const DEADTIME_RISING_TICKS: u16 = DEADTIME_TICKS;

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

type Out1 = HrOut<HRTIM_TIMA, Pscl1, Ch1, NoDacTrigger, NoDacTrigger>;
type Out2 = HrOut<HRTIM_TIMA, Pscl1, Ch2, NoDacTrigger, NoDacTrigger>;
type Capture1 = HrCaptCh1<HRTIM_TIMA, Pscl1>;

// TODO(7) Configure the `rtic::app` macro
#[rtic::app(
    dispatchers = [SPI4, SPI3],
    device = embassy_stm32,
)]
mod app {
    use core::{
        mem,
        sync::atomic::{AtomicU16, Ordering},
    };

    use super::*;
    use embassy_stm32::{
        Config, Peri,
        adc::{self, Adc, AdcChannel, AdcConfig, InjectedAdc, SampleTime},
        comp::{self, Comp},
        dac::{Dac, SOFTWARE},
        gpio::{Input, Output, Pull, Speed},
        hrtim::{
            self, HrControltExt, HrPwmBuilderExt as _, Parts,
            resonant_converter::DeadtimeConfig,
            stm32_hrtim::{
                self, DacResetOnCounterReset, DacStepOnCmp2, HrCountingDirection, HrPwmAdvExt,
                HrTimerMode, HrtimPrescaler, Polarity, PreloadSource,
                capture::HrCapture,
                compare_register::HrCompareRegister,
                deadtime::DeadtimePrescaler,
                external_event::{self, ToExternalEventSource},
                output::HrOutput,
                timer::{HrSlaveTimerCpt, HrTimer},
                timer_eev_cfg::{EevCfg, EevCfgs, EventFilter},
            },
        },
        mode::{self, Blocking},
        peripherals::{self, ADC1, ADC2, PC4, PC5},
        rcc::{Pll, PllMul, PllPDiv, PllPreDiv, PllRDiv, PllSource, Sysclk, mux::Adcsel},
        triggers,
    };
    use embassy_time::{Duration, Ticker};
    use full_control::control_2p2z::TwoPoleTwoZero;
    use load::Load;

    // Shared resources go here
    #[shared]
    struct Shared {
        target: AtomicU16,
        /// Most-recent raw Vin ADC code, written by control_loop and read by task1.
        vin_codes: AtomicU16,
        /// Live DAC sawtooth step value (INCDATA 12.4 fp), written by task1 from
        /// measured Vin and read by control_loop each tick.
        dac_step_live: AtomicU16,
    }

    // Local resources go here
    #[local]
    struct Local {
        adc: InjectedAdc<'static, ADC2, 2>,
        ref_dac: DacChannel<'static, mode::Blocking>,
        controller: TwoPoleTwoZero<f32>,

        nucleo_user_button: Input<'static>,

        target_setting: probe_plotter::Setting<f32>,
        max_load: probe_plotter::Setting<f32>,

        load_i_metric: probe_plotter::Metric<f32>,

        vin_metric: probe_plotter::Metric<u16>,
        vout_metric: probe_plotter::Metric<u16>,

        current_metric: probe_plotter::Metric<u16>,
        temp_metric: probe_plotter::Metric<u16>,

        runtime_metric: probe_plotter::Metric<u32>,
        capt_metric: probe_plotter::Metric<u16>,

        capture_ch1: Capture1,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // Enable DWT cycle counter so control_loop can measure its own execution time.
        let mut core = cx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();

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
        let adc = Adc::new(p.ADC2, cfg);

        let vout = p.PC5.degrade_adc();
        let vin = p.PC4.degrade_adc();

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
        let (mut control, _, eev_inputs) = control
            .set_adc2_trigger_psc(ADC_POST_SCALER) // apply the Div4 postscaler so ADC fires at CTRL_HZ = 125 kHz
            .wait_for_calibration();
        //let eev1 = eev_inputs.eev_input6;

        let mut ref_dac = DacChannel::new_sawtooth_internal::<_, dac::Ch1>(
            p.DAC1,
            triggers::HRTIM_DAC_RESET_TRG1,
            triggers::HRTIM_DAC_STEP_TRG1,
            //p.PA4,
        );
        ref_dac.set_sawtooth_reset_value(2048);
        ref_dac.set_sawtooth_step_direction(dac::StepDirection::Increment);
        ref_dac.set_sawtooth_step_value(DAC_STEP_SIZE);
        let mut cfg = comp::Config::default();
        cfg.inverting_input = comp::InvertingInput::Dac2; // Dac2 is DAC1CH1 for COMP1
        let comp1 = Comp::new(p.COMP1, p.PA1, Irqs, cfg);
        mem::forget(comp1);
        let eev6 = unsafe { external_event::SourceBuilder::<6, false>::new(0b10) }
            .edge_or_polarity(external_event::EdgeOrPolarity::Polarity(
                Polarity::ActiveLow,
            ))
            .filter(external_event::EevSamplingFilter::None)
            .finalize(&mut control);
        let mut control = control.constrain();
        let mut timer = tima
            .pwm_advanced(a_li, a_hi)
            .prescaler(prescaler)
            .period(PERIOD_TICKS) // 170MHz * 32 / 1 / 5440 = 1MHz
            .out1_polarity(Polarity::ActiveHigh)
            .out2_polarity(Polarity::ActiveHigh)
            .preload(PreloadSource::OnCounterReset)
            .deadtime(
                DeadtimeConfig::default()
                    .prescaler(DeadtimePrescaler::ThrtimDiv8)
                    .deadtime_falling_value(super::DEADTIME_FALLING_TICKS)
                    .deadtime_rising_value(super::DEADTIME_RISING_TICKS),
            )
            .timer_mode(HrTimerMode::Continuous)
            .counting_direction(HrCountingDirection::Up)
            .eev_cfg(
                EevCfgs::default().eev6(EevCfg::default().filter(EventFilter::BlankingResetToCmp3)),
            )
            .dac_trigger_cfg(DacResetOnCounterReset, DacStepOnCmp2)
            //.repetition_counter(repetition_counter)
            //.enable_repetition_interrupt()
            .finalize(&mut control);

        timer.cr1.set_duty(PERIOD_TICKS - 128); // Set max duty
        timer.cr2.set_duty(HR_TICKS_PER_DAC_INC); // Set DAC increment interval
        timer.cr3.set_duty(200); // Set end of COMP blanking
        timer.cr4.set_duty(2048); // Set ADC sample point
        control.adc_trigger2.enable_source(&timer.cr4);

        timer.timer.capture_ch1().add_event(&eev6);

        timer.out1.enable_set_event(&timer.timer);
        timer.out1.enable_rst_event(&timer.cr1);
        timer.out1.enable_rst_event(&eev6);

        timer.timer.start(&mut control.control);

        let adc = adc.setup_injected_conversions(
            [(vout, SampleTime::CYCLES47_5), (vin, SampleTime::CYCLES47_5)],
            triggers::HRTIM_ADC_TRG2,
            adc::Exten::RISING_EDGE,
            true,
        );

        let controller = WEIGHTS_CODE.to_controller(0.0, DAC_MAX_CODE as f32);

        let load_dac2 = DacChannel::new_blocking(p.DAC2, p.PA6);
        let load_dac = MyDac(load_dac2);

        let mut load = load::Load::new(
            load_dac,
            Output::new(p.PA5, embassy_stm32::gpio::Level::Low, Speed::Low),
        );
        load.set_range(load::Range::High);

        task0::spawn(load).ok();
        task1::spawn((timer.out1, timer.out2) /*p.PC4, p.PC5*/).ok();

        let capture_ch1 = timer.timer.split_capture().ch1;

        (
            Shared {
                target: AtomicU16::new(TARGET_CODE_INIT as u16),
                vin_codes: AtomicU16::new(0),
                dac_step_live: AtomicU16::new(DAC_STEP_SIZE),
            },
            Local {
                adc,
                ref_dac,
                controller,
                nucleo_user_button: Input::new(p.PC13, Pull::None),
                
                target_setting: probe_plotter::make_setting!(TARGET: f32 = 5.0, 0.0..=14.5, 0.1).unwrap(),
                max_load: probe_plotter::make_setting!(MAX_LOAD: f32 = 4.0, 0..=10, 0.1).unwrap(),

                temp_metric: probe_plotter::make_metric!(
                    TEMP: u16 = 0,
                    "(1.0 / ((ln((10000 * (TEMP * 3.3 / 4095.0)) / (3.3 - (TEMP * 3.3 / 4095.0)) / 10000) / 4100) + 1.0 / (273.15 + 25))) - 273.15"
                ).unwrap(),
                vin_metric: probe_plotter::make_metric!(VIN: u16 = 0, "(3.3 * VIN / 4095) * (20000 + 1000) / 1000").unwrap(),
                vout_metric: probe_plotter::make_metric!(VOUT: u16 = 0, "(3.3 * VOUT / 4095) * (20000 + 1000) / 1000").unwrap(),
                current_metric: probe_plotter::make_metric!(
                    CURRENT: u16 = 0,
                    "((CURRENT * 3.3 / 4095.0) - (3.3 / 2)) / -0.066"// Negate to get positive current in buck direction
                ).unwrap(),
                runtime_metric: probe_plotter::make_metric!(RUNTIME_US: u32 = 0, "RUNTIME_US / 170").unwrap(),
                load_i_metric: probe_plotter::make_metric!(LOAD_I: f32 = 0.0, "LOAD_I").unwrap(),
                capt_metric: probe_plotter::make_metric!(DUTY: u16 = 0, "100 * DUTY / (5440 * 2)").unwrap(),
                capture_ch1,
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
    #[task(priority = 1, local = [load_i_metric, max_load])]
    async fn task0(ctx: task0::Context, mut load: Load<MyDac, Output<'static>>) {
        loop {
            for (step, &c) in LOAD_PROFILE.iter().enumerate() {
                let c = c * ctx.local.max_load.get();
                // Apply the load step, then immediately arm the capture so the
                // ISR fills CAPTURE_BUF with the transient response.
                load.set_current_manual_range(c);
                ctx.local.load_i_metric.set(c);
                CAPTURE_IDX.store(0, Ordering::Release);

                // Wait for CAPTURE_LEN samples (~CAPTURE_MS ms) to arrive.
                loop {
                    if CAPTURE_IDX.load(Ordering::Acquire) >= CAPTURE_LEN {
                        break;
                    }
                    embassy_time::Timer::after_millis(1).await;
                }

                // Log the capture so it can be compared with the simulation.
                // Alternatively, read directly from target memory (no RTT pressure):
                //   addr=$(nm target/thumbv7em-none-eabihf/dev/minimal | awk '/CAPTURE_BUF/{print "0x"$1}')
                //   probe-rs read b16 $addr 500
                defmt::info!(
                    "step {} load={} mA ({} samples @ {} Hz):",
                    step as u32,
                    (c * 1000.0) as i32,
                    CAPTURE_LEN as u32,
                    CTRL_HZ as u32,
                );
                // SAFETY: CAPTURE_IDX >= CAPTURE_LEN (confirmed with Acquire) means
                // the ISR has finished writing; no further writes until we reset the index.
                let buf: &[u16; CAPTURE_LEN] = unsafe { &*(&raw const CAPTURE_BUF) };
                for chunk in buf.chunks(32) {
                    defmt::info!("{:?}", chunk);
                }

                // 22.762512 [INFO ] [300, 301, 299, 301, 301, 302, 300, 301, 297, 298, 296, 296, 292, 293, 291, 294, 293, 296, 295, 298, 297, 299, 299, 302, 300, 302, 301, 302, 298, 299, 298, 298] (minimal src/bin/minimal.rs:435)
                // 22.762573 [INFO ] [295, 297, 296, 298, 297, 299, 298, 299, 296, 298, 296, 298, 297, 299, 299, 301, 299, 299, 296, 298, 296, 299, 297, 300, 298, 301, 299, 300, 295, 297, 296, 299] (minimal src/bin/minimal.rs:435)
                // 22.762603 [INFO ] [298, 44, 10753, 11265, 11009, 44, 1, 40, 257, 296, 10752, 256, 10497, 1, 38, 1, 9985, 10497, 10241, 299, 298, 300, 299, 301, 301, 302, 9513, 9729, 9217, 9985, 9729, 10497] (minimal src/bin/minimal.rs:435)
                // 22.763793 [INFO ] [300, 298, 301, 299] (minimal src/bin/minimal.rs:435)

                // Mark the buffer as consumed and wait out the rest of the step.
                CAPTURE_IDX.store(usize::MAX, Ordering::Relaxed);
                embassy_time::Timer::after_millis((STEP_MS - CAPTURE_MS) as u64).await;
            }
        }
    }

    #[task(priority = 2, local = [i: usize = 0, nucleo_user_button, target_setting, vin_metric], shared = [&target, &vin_codes, &dac_step_live])]
    async fn task1(
        ctx: task1::Context,
        mut out: (Out1, Out2),
        //mut adc2: Adc<'static, ADC2>,
        //vin: Peri<'static, PC4>,
        //vout: Peri<'static, PC5>,
    ) {
        let mut ticker = Ticker::every(Duration::from_millis(1));
        let mut btn_ms_pressed = 0u32;
        let mut is_wait_for_btn_release = true;

        loop {
            let target = vout_to_code(ctx.local.target_setting.get());
            ctx.shared.target.store(target, Ordering::Relaxed);

            let is_btn_pressed = ctx.local.nucleo_user_button.is_high();
            let status = out.0.get_state();
            if is_btn_pressed && !is_wait_for_btn_release {
                match status {
                    stm32_hrtim::output::State::Idle => {
                        if btn_ms_pressed >= 2000 {
                            out.0.enable();
                            out.1.enable();
                            defmt::info!("Enabled by user");
                            btn_ms_pressed = 0;
                            is_wait_for_btn_release = true;
                        }
                    }
                    stm32_hrtim::output::State::Running => {
                        if btn_ms_pressed >= 50 {
                            out.0.disable();
                            out.1.disable();
                            defmt::info!("Disabled by user");
                            btn_ms_pressed = 0;
                            is_wait_for_btn_release = true;
                        }
                    }
                    stm32_hrtim::output::State::Fault => todo!(),
                }
                btn_ms_pressed = btn_ms_pressed.saturating_add(1);
            } else if !is_btn_pressed {
                is_wait_for_btn_release = false;
                btn_ms_pressed = 0;
            }

            // // NTC: Small value is hot
            // if t < *ctx.local.max_temp_adc {
            //     ctx.local.half_bridge.disable();
            //     defmt::error!("Disabled due to overheat");
            // }
            // ctx.local.half_bridge.clear_repetition_interrupt();
            // ctx.local.runtime_metric.set(start.elapsed());

            // Slope feed-forward: recompute DAC step value from the latest Vin reading.
            // Runs at 1 ms cadence — Vin changes slowly so this is more than sufficient.
            // Uses f32 throughout; the STM32G474 FPU handles single-precision natively.
            {
                let raw_vin = ctx.shared.vin_codes.load(Ordering::Relaxed);
                ctx.local.vin_metric.set(raw_vin);
                if raw_vin > 0 {
                    let v_out = ctx.local.target_setting.get(); // actual target, f32 volts
                    let vin_v = (raw_vin as f32 * LSB as f32 * VIN_SCALE as f32)
                        .clamp(V_IN_MIN as f32, V_IN_MAX as f32);
                    let d_prime = 1.0_f32 - v_out / vin_v;
                    if d_prime > 0.01 {
                        // m_c = (1 + π/2) / (π × D')
                        let m_c = (1.0_f32 + core::f32::consts::FRAC_PI_2)
                            / (core::f32::consts::PI * d_prime);
                        // S_n = (V_in − V_out) × CS_GAIN / L  [V/s at the DAC sense point]
                        let s_n = (vin_v - v_out) * CS_GAIN as f32 / L_INDUCTOR as f32;
                        // S_e = (m_c − 1) × S_n  (magnitude, ≥ 0)
                        let s_e = ((m_c - 1.0_f32) * s_n).max(0.0_f32);
                        // INCDATA = ceil(S_e × SLOPE_TO_INCDATA)
                        let incdata_f = s_e * SLOPE_TO_INCDATA;
                        let floor = (incdata_f + 0.5) as u16;
                        //let incdata = if (floor as f32) < incdata_f { floor + 1 } else { floor };
                        ctx.shared.dac_step_live.store(floor.max(1), Ordering::Relaxed);
                    }
                }
            }

            ticker.next().await;
        }
    }

    #[task(binds = ADC1_2, local = [adc, controller, ref_dac, vout_metric, capture_ch1, capt_metric, runtime_metric, i: usize = 0], shared = [&target, &vin_codes, &dac_step_live], priority = 15)]
    fn control_loop(ctx: control_loop::Context) {
        let t0 = cortex_m::peripheral::DWT::cycle_count();

        let samples = ctx.local.adc.read_injected_samples();
        let vout = samples[0];
        ctx.shared.vin_codes.store(samples[1], Ordering::Relaxed);

        let target = ctx.shared.target.load(Ordering::Relaxed);
        let error = target as i32 - vout as i32;
        let ctrl = ctx.local.controller.update(error as f32) as u16;
        let dac_code = 2048u16.saturating_sub(ctrl);
        ctx.local.ref_dac.set_sawtooth_reset_value(dac_code);
        ctx.local.ref_dac.set_sawtooth_step_value(ctx.shared.dac_step_live.load(Ordering::Relaxed));
        ctx.local.vout_metric.set(vout);

        // Store every sample when a capture is in progress.
        // Ordering::Relaxed load is fine: we only need atomicity, not ordering, for
        // the index check.  The Release store of idx+1 pairs with the Acquire load
        // in task0 to guarantee CAPTURE_BUF writes are visible once the index reaches
        // CAPTURE_LEN.
        let idx = CAPTURE_IDX.load(Ordering::Relaxed);
        if idx < CAPTURE_LEN {
            // SAFETY: only this ISR writes to CAPTURE_BUF while idx < CAPTURE_LEN.
            // task0 reads only after confirming idx >= CAPTURE_LEN with Acquire ordering.
            unsafe {
                (*(&raw mut CAPTURE_BUF))[idx] = vout;
            }
            CAPTURE_IDX.store(idx + 1, Ordering::Release);
        }

        {
            let (capt_ticks, _) = ctx.local.capture_ch1.get_last();
            ctx.local.capt_metric.set(capt_ticks);
        }

        // Measure ISR execution time in cycles; stored as µs via the metric formula (÷170).
        {
            let cycles = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(t0);
            ctx.local.runtime_metric.set(cycles);
        }
        *ctx.local.i += 1;
    }
}
