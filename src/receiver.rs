use core::{cell::RefCell, sync::atomic::AtomicU16};

use critical_section::Mutex;
use fugit::MillisDurationU64;
use rp2040_hal::{
    clocks::ClocksManager,
    gpio::{
        bank0::{Gpio3, Gpio4, Gpio5},
        FunctionNull, FunctionSioInput,
        Interrupt::EdgeLow,
        Pin, PullDown, PullNone,
    },
    pac,
    pac::{interrupt, PWM, RESETS, TIMER},
    pwm::{InputHighRunning, Pwm1, Pwm2, Slice, Slices},
    timer::Instant,
    Timer,
};

struct Globals {
    steering_pin: Pin<Gpio3, FunctionSioInput, PullNone>,
    steering_pwm: Slice<Pwm1, InputHighRunning>,
    throttle_pin: Pin<Gpio5, FunctionSioInput, PullNone>,
    throttle_pwm: Slice<Pwm2, InputHighRunning>,
    update_pin: Pin<Gpio4, FunctionSioInput, PullNone>,
}

static STEERING: AtomicU16 = AtomicU16::new(0);
static THROTTLE: AtomicU16 = AtomicU16::new(0);

struct TimerPair {
    timer: Option<Timer>,
    last_update: Instant,
}

impl TimerPair {
    const fn default() -> Self {
        Self {
            timer: None,
            last_update: Instant::from_ticks(0),
        }
    }
}

trait TimerWatchdog {
    fn has_watchdog_expired(&self) -> bool;
}

impl TimerWatchdog for Mutex<RefCell<TimerPair>> {
    fn has_watchdog_expired(&self) -> bool {
        critical_section::with(|cs| {
            let pair = self.borrow(cs).borrow();
            if let Some(timer) = &pair.timer {
                let current = timer.get_counter();
                let delta = current - pair.last_update;
                delta > MillisDurationU64::millis(100u64)
            } else {
                true
            }
        })
    }
}

static LAST_UPDATE: Mutex<RefCell<TimerPair>> = Mutex::new(RefCell::new(TimerPair::default()));

static GLOBAL_PINS: Mutex<RefCell<Option<Globals>>> = Mutex::new(RefCell::new(None));

#[interrupt]
fn IO_IRQ_BANK0() {
    static mut GLOBALS: Option<Globals> = None;

    if GLOBALS.is_none() {
        critical_section::with(|cs| {
            *GLOBALS = GLOBAL_PINS.borrow(cs).take();
        });
    }

    if let Some(globals) = GLOBALS {
        if globals.steering_pin.interrupt_status(EdgeLow) {
            let count = globals.steering_pwm.get_counter();
            globals.steering_pwm.set_counter(0);
            globals.steering_pin.clear_interrupt(EdgeLow);
            STEERING.store(count, core::sync::atomic::Ordering::Release)
        }

        if globals.throttle_pin.interrupt_status(EdgeLow) {
            let count = globals.throttle_pwm.get_counter();
            globals.throttle_pwm.set_counter(0);
            globals.throttle_pin.clear_interrupt(EdgeLow);
            THROTTLE.store(count, core::sync::atomic::Ordering::Release)
        }

        if globals.update_pin.interrupt_status(EdgeLow) {
            critical_section::with(|cs| {
                let mut pair = LAST_UPDATE.borrow(cs).borrow_mut();
                if let Some(timer) = &pair.timer {
                    pair.last_update = timer.get_counter();
                }
            });

            globals.update_pin.clear_interrupt(EdgeLow);
        }
    }
}

pub struct Receiver {}

impl Receiver {
    pub fn has_watchdog_expired(&self) -> bool {
        LAST_UPDATE.has_watchdog_expired()
    }

    pub fn steering(&self) -> u16 {
        STEERING.load(core::sync::atomic::Ordering::Acquire)
    }

    pub fn throttle(&self) -> u16 {
        THROTTLE.load(core::sync::atomic::Ordering::Acquire)
    }
}

pub fn initialize_receiver(
    timer: TIMER,
    resets: &mut RESETS,
    clocks: &ClocksManager,
    pwm: PWM,
    steering_pin: Pin<Gpio3, FunctionNull, PullDown>,
    throttle_pin: Pin<Gpio5, FunctionNull, PullDown>,
    update_pin: Pin<Gpio4, FunctionNull, PullDown>,
) -> Receiver {
    let timer = rp2040_hal::Timer::new(timer, resets, clocks);

    let slices = Slices::new(pwm, resets);
    let mut steering_pwm = slices.pwm1.into_mode::<InputHighRunning>();
    steering_pwm.set_div_int(125);
    #[allow(unsafe_code)] // Workaround to HAL issue. Safe because we only read from here
    let steering_pin = unsafe {
        steering_pwm
            .input_from(steering_pin.into_floating_input())
            .into_unchecked::<FunctionSioInput, PullNone>()
    };

    let mut throttle_pwm = slices.pwm2.into_mode::<InputHighRunning>();
    throttle_pwm.set_div_int(125);
    #[allow(unsafe_code)] // Workaround to HAL issue. Safe because we only read from here
    let throttle_pin = unsafe {
        throttle_pwm
            .input_from(throttle_pin.into_floating_input())
            .into_unchecked::<FunctionSioInput, PullNone>()
    };

    let update_pin = update_pin.into_floating_input();

    steering_pwm.enable();
    throttle_pwm.enable();

    steering_pin.set_interrupt_enabled(EdgeLow, true);
    throttle_pin.set_interrupt_enabled(EdgeLow, true);
    update_pin.set_interrupt_enabled(EdgeLow, true);

    critical_section::with(|cs| {
        LAST_UPDATE.borrow(cs).replace(TimerPair {
            timer: Some(timer),
            last_update: Instant::from_ticks(0),
        });

        GLOBAL_PINS.borrow(cs).replace(Some(Globals {
            steering_pin,
            steering_pwm,
            throttle_pin,
            throttle_pwm,
            update_pin,
        }))
    });

    #[allow(unsafe_code)] // We've computed that our interrupt enabling is safe
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    Receiver {}
}
