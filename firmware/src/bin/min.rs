#![no_std]
#![no_main]
#![allow(clippy::uninlined_format_args)]

use half_bridge::{
    self as _,
    hardware::{SYS_PLL_M_DIV, SYS_PLL_N_MUL, SYS_PLL_P_DIV, SYS_PLL_R_DIV, SYS_PLL_SOURCE},
}; // global logger + panicking-behavior + memory layout

use defmt::info;

/// Example showcasing the use of the HRTIM peripheral to trigger the ADC at various points of the switch cycle off HRTIM_TIMA
use cortex_m_rt::entry;
use stm32_hrtim::{
    compare_register::HrCompareRegister, output::HrOutput, timer::HrTimer, HrParts, HrPwmAdvExt,
    Pscl1,
};
use stm32g4xx_hal::{
    adc::{self, temperature::Temperature, AdcClaim, AdcCommonExt, Vref},
    delay::SYSTDelayExt,
    dma::{self, channel::DMAExt, config::DmaConfig, TransferExt},
    gpio::GpioExt,
    hrtim::{HrControltExt, HrPwmBuilderExt},
    pwr::{self, PwrExt},
    rcc::{self, RccExt},
    stm32::{CorePeripherals, Peripherals},
};

//use SYS_PLL_SOURCE
//pub const SYS_PLL_N_MUL
///pub const SYS_PLL_M_DIV
//pub const SYS_PLL_R_DIV

#[entry]
fn main() -> ! {
    const VREF: f32 = 3.3;

    info!("start");

    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().expect("cannot take core peripherals");

    info!("rcc");
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

    // This would lead to HrTim running at 170MHz * 32 = 5.44GHz...

    let mut delay = cp.SYST.delay(&rcc.clocks);

    let dma::channel::Channels { ch1: dma1ch1, .. } = dp.DMA1.split(&rcc);
    let config = DmaConfig::default()
        .transfer_complete_interrupt(true)
        .circular_buffer(true)
        .memory_increment(true);

    info!("Setup Gpio");
    let gpioa = dp.GPIOA.split(&mut rcc);
    let pa0 = gpioa.pa0.into_analog();

    let pin_a = gpioa.pa8;
    let pin_b = gpioa.pa9;

    // ...with a prescaler of 1 this gives us a HrTimer with a tick rate of 5.44GHz
    // With max the max period set, this would be 960MHz/2^16 ~= 15kHz...
    let prescaler = Pscl1;

    //        .               .
    //        .  50%          .
    //         ------          ------
    //out1    |      |        |      |
    //        |      |        |      |
    // --------      ----------      --------
    //        .    ^     ^
    //        .    |     |
    //AD samlp    pa0   temp
    let period = 0xFFFF;
    let (hr_control, ..) = dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();
    let mut hr_control = hr_control.constrain();
    let HrParts {
        mut timer,
        mut cr1,
        mut cr3,
        mut cr4,
        out: (mut out1, mut out2),
        ..
    } = dp
        .HRTIM_TIMA
        .pwm_advanced((pin_a, pin_b))
        .prescaler(prescaler)
        .period(period)
        .finalize(&mut hr_control);

    cr1.set_duty(period / 2);
    cr3.set_duty(period / 3);
    cr4.set_duty((2 * u32::from(period) / 3) as u16);

    hr_control.adc_trigger1.enable_source(&cr3);
    hr_control.adc_trigger1.enable_source(&cr4);

    out1.enable_rst_event(&cr1); // Set low on compare match with cr1
    out2.enable_rst_event(&cr1);

    out1.enable_set_event(&timer); // Set high at new period
    out2.enable_set_event(&timer);

    info!("Setup Adc1");
    let mut adc12_common = dp.ADC12_COMMON.claim(
        adc::config::ClockMode::AdcKerCk {
            prescaler: adc::config::Prescaler::Div_1,
            src: adc::config::ClockSource::PllP,
        },
        &mut rcc,
    );
    let mut adc = adc12_common.claim(dp.ADC1, &mut delay);

    adc.set_external_trigger((
        adc::config::TriggerMode::RisingEdge,
        (&hr_control.adc_trigger1).into(),
    ));
    adc12_common.enable_temperature();
    adc.set_continuous(adc::config::Continuous::Discontinuous);
    adc.reset_sequence();
    adc.configure_channel(
        &pa0,
        adc::config::Sequence::One,
        adc::config::SampleTime::Cycles_640_5,
    );
    adc.configure_channel(
        &Temperature,
        adc::config::Sequence::Two,
        adc::config::SampleTime::Cycles_640_5,
    );

    info!("Setup DMA");
    let first_buffer = cortex_m::singleton!(: [u16; 10] = [0; 10]).unwrap();

    let mut transfer = dma1ch1.into_circ_peripheral_to_memory_transfer(
        adc.enable_dma(adc::config::Dma::Continuous),
        &mut first_buffer[..],
        config,
    );

    transfer.start(|adc| adc.start_conversion());

    out1.enable();
    out2.enable();

    timer.start(&mut hr_control.control);

    loop {
        let mut b = [0_u16; 4];
        let r = transfer.read_exact(&mut b);

        info!("read: {}", r);
        assert!(r == b.len());

        let millivolts = Vref::sample_to_millivolts((b[0] + b[2]) / 2);
        info!("pa3: {}mV", millivolts);
        let temp = Temperature::temperature_to_degrees_centigrade(
            (b[1] + b[3]) / 2,
            VREF,
            adc::config::Resolution::Twelve,
        );
        info!("temp: {}℃C", temp);
    }
}
