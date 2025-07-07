use stm32g4xx_hal::{
    dac::{self, Dac3Ch1, Dac3Ch2, Dac4Ch1, Dac4Ch2, DacExt, DacOut},
    rcc::Rcc,
    stasis::{Entitlement, Freeze, Frozen},
    stm32,
};

use super::timers::Timers;

pub type DacHb1 = Dac3Ch2<{ dac::M_INT_SIG }, dac::SawtoothGenerator>;
pub type DacHb2 = Dac4Ch2<{ dac::M_INT_SIG }, dac::SawtoothGenerator>;
pub type DacHb3 = Dac4Ch1<{ dac::M_INT_SIG }, dac::SawtoothGenerator>;
pub type DacHb4 = Dac3Ch1<{ dac::M_INT_SIG }, dac::SawtoothGenerator>;
pub type DacHb5 = DacHb1;

impl Dacs {
    pub(crate) fn init(
        _dac1: stm32::DAC1,
        _dac2: stm32::DAC2,
        dac3: stm32::DAC3,
        dac4: stm32::DAC4,
        timers: &Timers,
        rcc: &mut Rcc,
    ) -> (Dacs, DacTokens) {
        defmt::info!("Initializing DACs...");
        // DAC1 and DAC2 might be too slow to be useful for generating the sawtooth shape required for
        // slope compensation
        /*
        let dac_ampl = 0;

        let (dac1ch1, dac1ch2) = {
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
        let dir = dac::CountingDirection::Increment;
        let step_size = 1; // todo
        let dac_cfg = dac::SawtoothConfig::with_slope(dir, step_size);

        #[cfg(any(feature = "hv1", feature = "hv4"))]
        let (dac3ch1, dac3ch2) = dac3.constrain((dac::Dac3IntSig1, dac::Dac3IntSig2), rcc);

        #[cfg(feature = "hv4")]
        let dac3ch1 = dac3ch1.enable_sawtooth_generator(
            dac_cfg
                .inc_trigger(&timers.timer4b.cr2)
                .reset_trigger(&timers.timer4b.timer),
            rcc,
        );
        #[cfg(feature = "hv1")]
        let dac3ch2 = dac3ch2.enable_sawtooth_generator(
            dac_cfg
                .inc_trigger(&timers.timer1.cr2)
                .reset_trigger(&timers.timer1.timer),
            rcc,
        );

        #[cfg(any(feature = "hv2", feature = "hv3"))]
        let (dac4ch1, dac4ch2) = dac4.constrain((dac::Dac4IntSig1, dac::Dac4IntSig2), rcc);

        #[cfg(feature = "hv3")]
        let dac4ch1 = dac4ch1.enable_sawtooth_generator(
            dac_cfg
                .inc_trigger(&timers.timer3.cr2)
                .reset_trigger(&timers.timer3.timer),
            rcc,
        );
        #[cfg(feature = "hv2")]
        let dac4ch2 = dac4ch2.enable_sawtooth_generator(
            dac_cfg
                .inc_trigger(&timers.timer2.cr2)
                .reset_trigger(&timers.timer2.timer),
            rcc,
        );

        #[cfg(any(feature = "hv1", feature = "hv5"))]
        let (cc1_cc5, [cc1_ot, cc5_ot]) = dac3ch2.freeze();
        #[cfg(feature = "hv2")]
        let (cc2, [cc2_ot]) = dac4ch2.freeze();
        #[cfg(feature = "hv3")]
        let (cc3, [cc3_ot]) = dac4ch1.freeze();
        #[cfg(feature = "hv4")]
        let (cc4, [cc4_ot]) = dac3ch1.freeze();

        (
            Dacs {
                #[cfg(any(feature = "hv1", feature = "hv5"))]
                cc1_cc5,
                #[cfg(feature = "hv2")]
                cc2,
                #[cfg(feature = "hv3")]
                cc3,
                #[cfg(feature = "hv4")]
                cc4,
            },
            DacTokens {
                #[cfg(feature = "hv1")]
                cc1: cc1_ot,
                #[cfg(feature = "hv2")]
                cc2: cc2_ot,
                #[cfg(feature = "hv3")]
                cc3: cc3_ot,
                #[cfg(feature = "hv4")]
                cc4: cc4_ot,
                #[cfg(feature = "hv5")]
                cc5: cc5_ot,
            },
        )
    }

    pub fn set_all_currents(&mut self, currents: [u16; 4]) {
        #[cfg(any(feature = "hv1", feature = "hv5"))]
        self.cc1_cc5.set_value(currents[0]);
        #[cfg(feature = "hv2")]
        self.cc2.set_value(currents[1]);
        #[cfg(feature = "hv3")]
        self.cc3.set_value(currents[2]);
        #[cfg(feature = "hv4")]
        self.cc4.set_value(currents[3]);
    }
}

pub struct Dacs {
    #[cfg(any(feature = "hv1", feature = "hv5"))]
    pub cc1_cc5: Frozen<DacHb1, 2>,
    #[cfg(feature = "hv2")]
    pub cc2: Frozen<DacHb2, 1>,
    #[cfg(feature = "hv3")]
    pub cc3: Frozen<DacHb3, 1>,
    #[cfg(feature = "hv4")]
    pub cc4: Frozen<DacHb4, 1>,
}

pub(crate) struct DacTokens {
    #[cfg(feature = "hv1")]
    pub(crate) cc1: Entitlement<DacHb1>,
    #[cfg(feature = "hv2")]
    pub(crate) cc2: Entitlement<DacHb2>,
    #[cfg(feature = "hv3")]
    pub(crate) cc3: Entitlement<DacHb3>,
    #[cfg(feature = "hv4")]
    pub(crate) cc4: Entitlement<DacHb4>,
    #[cfg(feature = "hv5")]
    pub(crate) cc5: Entitlement<DacHb5>,
}
