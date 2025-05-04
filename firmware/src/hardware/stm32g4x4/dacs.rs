use stm32_hrtim::stm32;
use stm32g4xx_hal::{
    dac::{self, Dac3Ch1, Dac3Ch2, Dac4Ch1, Dac4Ch2, DacExt}, rcc::Rcc, stasis::{Entitlement, Freeze, Frozen}
};

pub type DacHb1 = Dac3Ch2<dac::M_INT_SIG, dac::SawtoothGenerator>;
pub type DacHb2 = Dac4Ch2<dac::M_INT_SIG, dac::SawtoothGenerator>;
pub type DacHb3 = Dac4Ch1<dac::M_INT_SIG, dac::SawtoothGenerator>;
pub type DacHb4 = Dac3Ch1<dac::M_INT_SIG, dac::SawtoothGenerator>;
pub type DacHb5 = DacHb1;

impl Dacs {
    pub(crate) fn init(
        _dac1: stm32::DAC1,
        _dac2: stm32::DAC2,
        dac3: stm32::DAC3,
        dac4: stm32::DAC4,
        inc_trigger_hb1: Entitlement<HrCr2<TimHb1, super::Prescaler>>,
        inc_trigger_hb2: Entitlement<HrCr2<TimHb2, super::Prescaler>>,
        inc_trigger_hb3: Entitlement<HrCr2<TimHb3, super::Prescaler>>,
        inc_trigger_hb4: Entitlement<HrCr2<TimHb4, super::Prescaler>>,
        rcc: &mut Rcc,
    ) -> (Dacs, DacTokens) {
        defmt::info!("Initializing DACs...");

        let dac_ampl = 0;
        // DAC1 and DAC2 might be too slow to be useful for generating the sawtooth shape required for
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
        let dir = dac::CountingDirection::Increment;
        let step_size = ();
        let dac_cfg = dac::SawtoothConfig::with_slope(dir, step_size);

        let (dac3ch1, dac3ch2) = {
            let (ch1, ch2) = dac3.constrain((dac::Dac3IntSig1, dac::Dac3IntSig2), rcc);
            (
                ch1.enable_generator(dac_cfg.inc_trigger(inc_trigger_hb4).reset_trigger(reset_trigger), &mut rcc),
                ch2.enable_generator(dac_cfg.inc_trigger(inc_trigger_hb1).reset_trigger(reset_trigger), &mut rcc),
            )
        };

        let (dac4ch1, dac4ch2) = {
            let (ch1, ch2) = dac4.constrain((dac::Dac4IntSig1, dac::Dac4IntSig2), rcc);
            (
                ch1.enable_sawtooth_generator(dac_cfg.inc_trigger(inc_trigger_hb3).reset_trigger(reset_trigger), &mut rcc),
                ch2.enable_sawtooth_generator(dac_cfg.inc_trigger(inc_trigger_hb2).reset_trigger(reset_trigger), &mut rcc),
            )
        };

        let (cc1_cc5, [cc1_ot, cc5_ot]) = dac3ch2.freeze();
        let (cc4, [cc4_ot]) = dac3ch1.freeze();
        let (cc2, [cc2_ot]) = dac4ch2.freeze();
        let (cc3, [cc3_ot]) = dac4ch1.freeze();

        (
            Dacs {
                cc1_cc5,
                cc4,
                cc2,
                cc3,
            },
            DacTokens {
                cc4: cc4_ot,
                cc1: cc1_ot,
                cc5: cc5_ot,
                cc3: cc3_ot,
                cc2: cc2_ot,
            },
        )
    }
}

pub struct Dacs {
    pub cc1_cc5: Frozen<DacHb1, 2>,
    pub cc2: Frozen<DacHb2, 1>,
    pub cc3: Frozen<DacHb3, 1>,
    pub cc4: Frozen<DacHb4, 1>,
}

pub(crate) struct DacTokens {
    pub(crate) cc1: Entitlement<DacHb1>,
    pub(crate) cc2: Entitlement<DacHb2>,
    pub(crate) cc3: Entitlement<DacHb3>,
    pub(crate) cc4: Entitlement<DacHb4>,
    pub(crate) cc5: Entitlement<DacHb5>,
}
