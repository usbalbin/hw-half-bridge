#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate stm32g4xx_hal as hal;

use cortex_m_rt::entry;
use defmt::println;

#[entry]
fn main() -> ! {
    defmt::println!("Hello, STM32G4!");
    
    //let x = half_bridge::control_2p2z::smlad(u32::MAX, u32::MAX, u32::MAX);

    half_bridge::control_2p2z::test();

    println!("All done");

    #[allow(clippy::empty_loop)]
    loop {}
}
