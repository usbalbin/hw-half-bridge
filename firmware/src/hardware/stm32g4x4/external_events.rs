use stm32_hrtim::{
    control::HrTimCalibrated,
    external_event::{EevInputs, ExternalEventSource},
};
use stm32g4xx_hal::{
    comparator::{self, ComparatorExt, ComparatorSplit},
    gpio::{
        self,
        gpioa::{PA1, PA7},
        gpiob::{PB0, PB11, PB14},
    },
    hrtim::external_event::EevInputExt,
    rcc::Rcc,
    stasis::Entitlement,
    stm32,
};

use crate::hardware::stm32g4x4::I_FILTER;

use super::dacs::DacTokens;

pub struct Eevs {
    #[cfg(feature = "hv1")]
    pub cc1_filt: ExternalEventSource<7, false>, //Comp4
    //cc1b: ExternalEventSource<5, false>,
    #[cfg(feature = "hv2")]
    pub cc2_filt: ExternalEventSource<8, false>, //Comp6
    #[cfg(feature = "hv3")]
    pub cc3_filt: ExternalEventSource<10, false>, //Comp7
    #[cfg(feature = "hv4")]
    pub cc4_filt: ExternalEventSource<6, false>, //Comp1

    #[cfg(feature = "hv1")]
    pub cc1_fast: ExternalEventSource<2, true>, //Comp4
    #[cfg(feature = "hv2")]
    pub cc2_fast: ExternalEventSource<3, true>, //Comp6
    #[cfg(feature = "hv3")]
    pub cc3_fast: ExternalEventSource<5, true>, //Comp7
    #[cfg(feature = "hv4")]
    pub cc4_fast: ExternalEventSource<4, true>, //Comp1
    #[cfg(feature = "hv5")]
    pub cc5_fast: ExternalEventSource<1, true>, //Comp2
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
            ($comp:expr, $pos_in:expr, $neg_in:expr, $rcc:expr) => {{
                $comp
                    .comparator(
                        $pos_in,
                        $neg_in,
                        comparator::Config::default().output_inverted(),
                        &$rcc.clocks,
                    )
                    .enable()
                    .lock()
            }};
        }

        macro_rules! init_eev {
            ($comp:expr, $eev_input:expr, $hr_control:expr, [$($fast:ident)*], $($filter:expr)*) => {{
                use stm32_hrtim::external_event::ToExternalEventSource;
                let eev = $eev_input
                    .bind(&$comp)
                    .edge_or_polarity(stm32_hrtim::external_event::EdgeOrPolarity::Polarity(
                        stm32_hrtim::Polarity::ActiveLow,
                    ));
                $(let eev = eev.filter($filter);)*
                $(
                    let $fast = ();
                    let _ = $fast;
                    let eev = eev.fast();
                )*

                eev.finalize($hr_control)
            }};
        }

        let (comp1, comp2, _comp3, comp4, _comp5, comp6, comp7) = comp.split(rcc);

        #[cfg(feature = "hv4")]
        let comp1 = init_comp!(comp1, cc4_pin, dacs.cc4, rcc);
        #[cfg(feature = "hv5")]
        let comp2 = init_comp!(comp2, cc5_pin, dacs.cc5, rcc);
        #[cfg(feature = "hv1")]
        let comp4 = init_comp!(comp4, cc1_pin, dacs.cc1, rcc);
        #[cfg(feature = "hv2")]
        let comp6 = init_comp!(comp6, cc2_pin, dacs.cc2, rcc);
        #[cfg(feature = "hv3")]
        let comp7 = init_comp!(comp7, cc3_pin, dacs.cc3, rcc);

        // filt=eev6 // fast=eev4,
        #[cfg(feature = "hv4")]
        let comp1_cc4 = init_eev!(comp1, eev_inputs.eev_input6, ctrl, [], I_FILTER);

        // fast=eev5, // filt=eev8
        /*let comp3_cc1b = init_comp!(
            comp3,
            eev_inputs.eev_input5,
            ctrl, /* No filter */
        );*/ // Same DAC as comp1

        // filt=eev7 // fast=eev2, filt=eev9
        #[cfg(feature = "hv1")]
        let comp4_cc1a = init_eev!(comp4, eev_inputs.eev_input7, ctrl, [], I_FILTER); // Same DAC as comp2

        // filt=eev9 // fast=eev5
        //let comp5 = init_comp!(comp5, comp5_pin, dac4ch1, eev_input9, rcc, hr_control); // <-- No available in_pos, DAC same as comp7

        // filt=eev8, // fast=eev3,
        #[cfg(feature = "hv2")]
        let comp6_cc2 = init_eev!(comp6, eev_inputs.eev_input8, ctrl, [], I_FILTER);

        // filt=eev10, //fast=eev5
        #[cfg(feature = "hv3")]
        let comp7_cc3 = init_eev!(comp7, eev_inputs.eev_input10, ctrl, [], I_FILTER);

        #[cfg(feature = "hv1")]
        let cc1_filt = comp4_cc1a;
        //let cc1b = comp3_cc1b;
        #[cfg(feature = "hv2")]
        let cc2_filt = comp6_cc2;

        #[cfg(feature = "hv3")]
        let cc3_filt = comp7_cc3;
        #[cfg(feature = "hv4")]
        let cc4_filt = comp1_cc4;

        // --------------------------------------------------------

        #[cfg(feature = "hv1")]
        let cc1_fast = init_eev!(
            comp4,
            eev_inputs.eev_input2,
            ctrl,
            [fast], // fast mode
                    /* No filter */
        );

        #[cfg(feature = "hv2")]
        let cc2_fast = init_eev!(
            comp6,
            eev_inputs.eev_input3,
            ctrl,
            [fast], // fast mode
                    /* No filter */
        );

        #[cfg(feature = "hv3")]
        let cc3_fast = init_eev!(
            comp7,
            eev_inputs.eev_input5,
            ctrl,
            [fast], // fast mode
                    /* No filter */
        );

        #[cfg(feature = "hv4")]
        let cc4_fast = init_eev!(
            comp1,
            eev_inputs.eev_input4,
            ctrl,
            [fast], // fast mode
                    /* No filter */
        );

        #[cfg(feature = "hv5")]
        let cc5_fast = init_eev!(
            comp2,
            eev_inputs.eev_input1,
            ctrl,
            [fast], // fast mode
                    /* No filter */
        ); // <-- Same DAC as comp4

        Eevs {
            #[cfg(feature = "hv1")]
            cc1_filt,
            //cc1b,
            #[cfg(feature = "hv2")]
            cc2_filt,
            #[cfg(feature = "hv3")]
            cc3_filt,
            #[cfg(feature = "hv4")]
            cc4_filt,

            #[cfg(feature = "hv1")]
            cc1_fast,
            #[cfg(feature = "hv2")]
            cc2_fast,
            #[cfg(feature = "hv3")]
            cc3_fast,
            #[cfg(feature = "hv4")]
            cc4_fast,
            #[cfg(feature = "hv5")]
            cc5_fast, // WARNING uses same DAC as cc1, so will have wonky slope compensation, adjust phase order to minimize bad stuff
        }
    }
}
