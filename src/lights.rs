use rp2040_hal::{
    clocks::ClocksManager,
    gpio::{DynPinId, FunctionPio0, Pin, PullDown},
    pac::PIO0,
    pio::{PIOBuilder, PinDir, Tx, UninitStateMachine, PIO, SM0},
    Clock,
};

pub fn initialize_lights(
    pio: &mut PIO<PIO0>,
    sm: UninitStateMachine<(PIO0, SM0)>,
    clocks: &ClocksManager,
    pin: Pin<DynPinId, FunctionPio0, PullDown>,
) -> Tx<(PIO0, SM0)> {
    let program = pio_proc::pio_asm!(
        ".define public t1 8", // High time at start
        ".define public t2 6", // Delta
        ".define public t3 8", // Low time at end
        ".side_set 1",
        "new_data:"
        "pull       side 0 [0]",
        "mov x osr  side 0 [0]",
        "jmp !x do_stop side 0 [0]",
        "out y, 1       side 0 [2]",
        "jmp check_bit side 0 [0]",
        "bitloop:",
        "out y, 1       side 0 [t3 -1]",
        "check_bit:",
        "jmp !y do_zero side 1 [t1 -1]",
        "do_one:",
        "jmp !osre bitloop    side 1 [t2 -1]",
        ".wrap",
        "do_zero:",
        "jmp !osre bitloop    side 0 [t2 - 1]",
        ".wrap_target",
        "jmp new_data       side 0 [0]",
        "do_stop:",
        "pull       side 0 [0]",
        "mov x osr  side 0 [0]",
        "keep_looping:",
        "jmp x-- keep_looping   side 0 [7]", // TODO
        "nop   side 1 [7]", // TODO
        "jmp new_data       side 0 [0]", // TODO
    );
    let installed = pio.install(&program.program).unwrap();

    let frequency = 871000;
    let cycles_per_bit =
        (program.public_defines.t1 + program.public_defines.t2 + program.public_defines.t3) as u32;
    let frequency_per_bit = frequency * cycles_per_bit;

    let int_part = clocks.system_clock.freq().to_Hz() / frequency_per_bit;
    let remainder = clocks.system_clock.freq().to_Hz() % frequency_per_bit;
    let fract_part = (remainder * 256) / frequency_per_bit;

    let (mut sm, _, tx) = PIOBuilder::from_program(installed)
        .side_set_pin_base(pin.id().num)
        .out_shift_direction(rp2040_hal::pio::ShiftDirection::Right)
        .autopull(false)
        .pull_threshold(24)
        .clock_divisor_fixed_point(int_part as u16, fract_part as u8)
        .buffers(rp2040_hal::pio::Buffers::OnlyTx)
        .build(sm);

    sm.set_pindirs([(pin.id().num, PinDir::Output)]);

    sm.start();

    tx
}

#[derive(Clone, Copy, Debug)]
pub struct FrontLeds {
    pub yellow: u8,
    pub low_beam: u8,
    pub high_beam: u8,
}

impl From<FrontLeds> for u32 {
    fn from(value: FrontLeds) -> Self {
        let mut ret = 0xFF000000u32;
        ret |= (value.high_beam as u32) << 16;
        ret |= (value.low_beam as u32) << 8;
        ret | (value.yellow as u32)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct RearLeds {
    pub yellow: u8,
    pub white: u8,
    pub red: u8,
}

impl From<RearLeds> for u32 {
    fn from(value: RearLeds) -> Self {
        let mut ret = 0xFF000000u32;
        ret |= (value.red as u32) << 16;
        ret |= (value.white as u32) << 8;
        ret | (value.yellow as u32)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Leds {
    pub front_right: FrontLeds,
    pub front_left: FrontLeds,
    pub rear_right: RearLeds,
    pub rear_left: RearLeds,
}

impl Leds {
    pub fn write(&self, tx: &mut Tx<(PIO0, SM0)>) {
        critical_section::with(|_cs| {
            tx.write(self.front_left.into());
            tx.write(self.front_right.into());
            tx.write(self.rear_right.into());
            tx.write(self.rear_left.into());
            tx.write(0xFF000000u32);
            tx.write(0);
            tx.write(42);
        });
    }
}
