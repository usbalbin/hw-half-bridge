use stm32_hrtim::stm32;
use stm32g4xx_hal::{
    dac::{self, Dac3Ch1, Dac3Ch2, Dac4Ch1, Dac4Ch2, DacExt},
    observable::{Observable, ObservationToken, Observed}, rcc::Rcc,
};

impl Dacs {
    pub(crate) fn init(
        _dac1: stm32::DAC1,
        _dac2: stm32::DAC2,
        dac3: stm32::DAC3,
        dac4: stm32::DAC4,
        rcc: &mut Rcc,
    ) -> (Dacs, DacTokens) {
        defmt::info!("Initializing DACs...");

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

        let (cc1_cc5, [cc1_ot, cc5_ot]) = dac3ch2.observe();
        let (cc4, [cc4_ot]) = dac3ch1.observe();
        let (cc2, [cc2_ot]) = dac4ch2.observe();
        let (cc3, [cc3_ot]) = dac4ch1.observe();

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
    pub cc4: Observed<Dac3Ch1<0b11, dac::WaveGenerator>, 1>,
    pub cc1_cc5: Observed<Dac3Ch2<0b11, dac::WaveGenerator>, 2>,

    pub cc3: Observed<Dac4Ch1<0b11, dac::WaveGenerator>, 1>,
    pub cc2: Observed<Dac4Ch2<0b11, dac::WaveGenerator>, 1>,
}

pub(crate) struct DacTokens {
    pub(crate) cc4: ObservationToken<Dac3Ch1<0b11, dac::WaveGenerator>>,
    pub(crate) cc1: ObservationToken<Dac3Ch2<0b11, dac::WaveGenerator>>,
    pub(crate) cc5: ObservationToken<Dac3Ch2<0b11, dac::WaveGenerator>>,

    pub(crate) cc3: ObservationToken<Dac4Ch1<0b11, dac::WaveGenerator>>,
    pub(crate) cc2: ObservationToken<Dac4Ch2<0b11, dac::WaveGenerator>>,
}
