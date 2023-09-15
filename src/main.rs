//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
#![deny(unsafe_code)]
#![deny(warnings)]

use defmt::*;
use defmt_rtt as _;
use hal::{entry, gpio::FunctionPio0, prelude::_rphal_pio_PIOExt};
use panic_probe as _;
use rp2040_hal as hal;

mod lights;
mod receiver;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
// use sparkfun_pro_micro_rp2040 as bsp;

use hal::{clocks::Clock, pac, watchdog::Watchdog};

use crate::{
    lights::{initialize_lights, FrontLeds, Leds, RearLeds},
    receiver::initialize_receiver,
};

#[allow(unsafe_code)]
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    defmt::info!("{}", clocks.system_clock.freq().to_Hz());

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let receiver = initialize_receiver(
        pac.TIMER,
        &mut pac.RESETS,
        &clocks,
        pac.PWM,
        pins.gpio3,
        pins.gpio5,
        pins.gpio4,
    );

    let pin = pins
        .gpio8
        .into_push_pull_output_in_state(hal::gpio::PinState::Low)
        .into_function::<FunctionPio0>()
        .into_dyn_pin();

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let mut tx = initialize_lights(&mut pio, sm0, &clocks, pin);

    loop {
        let leds = Leds {
            front_right: FrontLeds {
                yellow: 0,
                low_beam: 0,
                high_beam: 0,
            },
            front_left: FrontLeds {
                yellow: 42,
                low_beam: 0,
                high_beam: 0,
            },
            rear_left: RearLeds {
                yellow: 42,
                white: 0,
                red: 0,
            },
            rear_right: RearLeds {
                yellow: 0,
                white: 0,
                red: 0,
            },
        };

        leds.write(&mut tx);

        println!(
            "{} {} {}",
            receiver.steering(),
            receiver.throttle(),
            receiver.has_watchdog_expired()
        );

        delay.delay_ms(500);

        let leds = Leds {
            front_right: FrontLeds {
                yellow: 0,
                low_beam: 0,
                high_beam: 0,
            },
            front_left: FrontLeds {
                yellow: 0,
                low_beam: 0,
                high_beam: 0,
            },
            rear_left: RearLeds {
                yellow: 0,
                white: 0,
                red: 0,
            },
            rear_right: RearLeds {
                yellow: 0,
                white: 0,
                red: 0,
            },
        };

        leds.write(&mut tx);

        delay.delay_ms(500);
    }
}

// End of file
