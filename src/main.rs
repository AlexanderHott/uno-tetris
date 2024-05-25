#![no_std]
#![no_main]

use arduino_hal::{
    delay_ms,
    port::{mode::Output, Pin},
};
use panic_halt as _;
use ufmt::derive::uDebug;

mod serial {
    use arduino_hal::{hal::usart::Usart0, DefaultClock};
    use core::cell::RefCell;

    pub struct Console(pub RefCell<Option<Usart0<DefaultClock>>>);
    /// SAFETY: There is only one "thread", so `Send`ing it isn't possible.
    unsafe impl Send for Console {}
    unsafe impl Sync for Console {}

    pub static CONSOLE: Console = Console(RefCell::new(None));

    pub fn init(usart: Usart0<DefaultClock>) {
        *CONSOLE.0.borrow_mut() = Some(usart);
    }

    #[macro_export]
    macro_rules! print {
        ($($t:tt)*) => {
            if let Some(serial) = serial::CONSOLE.0.borrow_mut().as_mut() {
                let _ = ufmt::uwrite!(serial, $($t)*);
            }
        };
    }

    #[macro_export]
    macro_rules! println {
        ($($t:tt)*) => {
            if let Some(serial) = serial::CONSOLE.0.borrow_mut().as_mut() {
                let _ = ufmt::uwriteln!(serial, $($t)*);
            }
        };
    }
}

const NUM_DEVICES: usize = 4;

#[derive(Copy, Clone, uDebug)]
enum Command {
    DecodeMode(bool),
    Intensity(u8), // TODO max 0xF i think
    ScanLimit(u8),
    Shutdown(bool),
    DisplayTest(bool),
}

impl Command {
    /// #define OP_DECODEMODE 9    ///< MAX72xx opcode for DECODE MODE
    /// #define OP_INTENSITY 10    ///< MAX72xx opcode for SET INTENSITY
    /// #define OP_SCANLIMIT 11    ///< MAX72xx opcode for SCAN LIMIT
    /// #define OP_SHUTDOWN 12     ///< MAX72xx opcode for SHUT DOWN
    /// #define OP_DISPLAYTEST 15  ///< MAX72xx opcode for DISPLAY TEST
    fn as_u8s(&self) -> (u8, u8) {
        match &self {
            Command::DecodeMode(b) => (0x9, if *b { 0xff } else { 0 }),
            Command::Intensity(n) => (0xA, *n),
            Command::ScanLimit(n) => (0xB, *n),
            Command::Shutdown(b) => (0xC, if *b { 0 } else { 1 }),
            Command::DisplayTest(b) => (0xF, (*b) as u8),
        }
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let ser = arduino_hal::default_serial!(dp, pins, 57600);

    serial::init(ser);
    println!("Serial init");

    let mut din = pins.d13.into_output().downgrade();
    let mut cs = pins.d12.into_output().downgrade();
    let mut clk = pins.d11.into_output().downgrade();
    cs.set_high();

    delay_ms(1);

    control(Command::DisplayTest(false), &mut din, &mut cs, &mut clk);
    control(Command::ScanLimit(7), &mut din, &mut cs, &mut clk);
    control(Command::Intensity(0), &mut din, &mut cs, &mut clk);
    control(Command::DecodeMode(false), &mut din, &mut cs, &mut clk);
    control(Command::Shutdown(false), &mut din, &mut cs, &mut clk);

    /*
     * For examples (and inspiration), head to
     *
     *     https://github.com/Rahix/avr-hal/tree/main/examples
     *
     * NOTE: Not all examples were ported to all boards!  There is a good chance though, that code
     * for a different board can be adapted for yours.  The Arduino Uno currently has the most
     * examples available.
     */

    loop {
        for row in 0..8 {
            set_row(0, 1, row, u8::MAX, &mut din, &mut cs, &mut clk);
            delay_ms(500);
            set_row(0, 1, row, 0u8, &mut din, &mut cs, &mut clk);
        }
    }
}

fn control(command: Command, din: &mut Pin<Output>, cs: &mut Pin<Output>, clk: &mut Pin<Output>) {
    let mut spi_data = [0u8; 2 * NUM_DEVICES];

    print!("{:?}: ", command);
    for dev_idx in 0..NUM_DEVICES {
        let (opcode, param) = command.as_u8s();
        spi_data[2 * dev_idx] = opcode;
        spi_data[2 * dev_idx + 1] = param;
        print!("{:?} {:?} ", opcode, param);
    }
    println!("");

    cs.set_low();

    for data in spi_data {
        shift_out(din, clk, data);
    }

    cs.set_high();
}

fn shift_out(data_pin: &mut Pin<Output>, clock_pin: &mut Pin<Output>, mut value: u8) {
    for _ in 0..u8::BITS {
        if (value & 128) != 0 {
            data_pin.set_high();
        } else {
            data_pin.set_low();
        }
        value <<= 1;

        clock_pin.set_high();
        clock_pin.set_low();
    }
}

/// device indexes are inclusive
fn set_row(
    idx_dev_frst: usize,
    idx_dev_last: usize,
    row: u8,
    value: u8,
    data_pin: &mut Pin<Output>,
    chip_select: &mut Pin<Output>,
    clock_pin: &mut Pin<Output>,
) {
    let mut spi_data = [0u8; 2 * NUM_DEVICES as usize];

    for dev_idx in idx_dev_frst..idx_dev_last + 1 {
        spi_data[2 * dev_idx] = row + 1;
        spi_data[2 * dev_idx + 1] = value;
    }

    chip_select.set_low();

    for data in spi_data {
        shift_out(data_pin, clock_pin, data);
    }

    chip_select.set_high();
}
