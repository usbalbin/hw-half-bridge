use stm32_hrtim::{control::HrTimCalibrated, external_event::{EevInputs, ExternalEventSource}, stm32};
use stm32g4xx_hal::{
    comparator::{self, split, ComparatorExt, ComparatorSplit}, gpio::{self, gpioa::{PA1, PA7}, gpiob::{PB0, PB11, PB14}}, stasis::Entitlement, rcc::Rcc
};

use crate::hardware::stm32g4x4::I_FILTER;

use super::dacs::DacTokens;

pub struct Eevs {
    pub cc1: ExternalEventSource<7, false>,
    //cc1b: ExternalEventSource<5, false>,
    pub cc2: ExternalEventSource<8, false>,

    pub cc3: ExternalEventSource<10, false>,
    pub cc4: ExternalEventSource<6, false>,
    pub cc5: ExternalEventSource<1, false>,
}

impl Eevs {
    pub(crate) fn init(
        dacs: DacTokens,
        comp: stm32::COMP,
        cc1_pin: Entitlement<PB0<gpio::Analog>>,
        //cc1b_pin: Entitlement<PC1<gpio::Analog>>,
        cc2_pin: Entitlement<PB11<gpio::Analog>>,
        cc3_pin: Entitlement<PB14<gpio::Analog>>,
        cc4_pin: Entitlement<PA1<gpio::Analog>>,
        cc5_pin: Entitlement<PA7<gpio::Analog>>,
        eev_inputs: EevInputs,
        rcc: &mut Rcc,
        ctrl: &mut HrTimCalibrated,
    ) -> Eevs {
        defmt::info!("Initializing Comparators...");

        macro_rules! init_comp {
        ($comp:expr, $pos_in:expr, $neg_in:expr, $eev_input:expr, $rcc:expr, $hr_control:expr, $($filter:expr)*) => {{
            use stm32_hrtim::external_event::ToExternalEventSource;
            let comp = $comp
                .comparator(
                    $pos_in,
                    $neg_in,
                    comparator::Config::default(),
                    &$rcc.clocks,
                )
                .enable()
                .lock();

            #[allow(unused_mut)]
            let mut eev = $eev_input
                .bind(&comp)
                .edge_or_polarity(stm32_hrtim::external_event::EdgeOrPolarity::Polarity(
                    stm32_hrtim::Polarity::ActiveHigh,
                ));
            $(eev = eev.filter($filter);)*

            eev.finalize($hr_control)
        }};
    }

        let (comp1, comp2, comp3, comp4, _comp5, comp6, comp7) = comp.split(rcc);

        // filt=eev6 // fast=eev4,
        let comp1_cc4 = init_comp!(
            comp1,
            cc4_pin, // ok
            dacs.cc4,
            eev_inputs.eev_input6,
            rcc,
            ctrl,
            I_FILTER
        );

        // fast=eev1 // filt=eev6
        let comp2_cc5 = init_comp!(
            comp2,
            cc5_pin, // ok
            dacs.cc5,
            eev_inputs.eev_input1,
            rcc,
            ctrl, /* No filter */
        ); // <-- Same DAC as comp4

        // fast=eev5, // filt=eev8
        /*let comp3_cc1b = init_comp!(
            comp3,
            cc1b_pin, // ok
            dacs.cc1b_cc4,
            eev_inputs.eev_input5,
            rcc,
            ctrl, /* No filter */
        );*/ // Same DAC as comp1

        // filt=eev7 // fast=eev2, filt=eev9
        let comp4_cc1a = init_comp!(
            comp4,
            cc1_pin, // ok, TODO: Is ADC3 good enough?
            dacs.cc1,
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
            cc2_pin, // ok
            dacs.cc2,
            eev_inputs.eev_input8,
            rcc,
            ctrl,
            I_FILTER
        );

        // filt=eev10, //fast=eev5
        let comp7_cc3 = init_comp!(
            comp7,
            cc3_pin, // ok
            dacs.cc3,
            eev_inputs.eev_input10,
            rcc,
            ctrl,
            I_FILTER
        );

        let cc1 = comp4_cc1a;
        //let cc1b = comp3_cc1b;
        let cc2 = comp6_cc2;

        let cc3 = comp7_cc3;
        let cc4 = comp1_cc4;
        let cc5 = comp2_cc5; // WARNING uses same DAC as cc1, so will have wonky slope compensation, adjust phase order to minimize bad stuff

        Eevs {
            cc1,
            //cc1b,
            cc2,
            cc3,
            cc4,
            cc5,
        }
    }
}
