#![no_std]
#![no_main]

use arduino_hal::delay_ms;
use panic_halt as _;

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
            if let Some(serial) = crate::serial::CONSOLE.0.borrow_mut().as_mut() {
                let _ = ufmt::uwrite!(serial, $($t)*);
            }
        };
    }

    #[macro_export]
    macro_rules! println {
        ($($t:tt)*) => {
            if let Some(serial) = crate::serial::CONSOLE.0.borrow_mut().as_mut() {
                let _ = ufmt::uwriteln!(serial, $($t)*);
            }
        };
    }
}

mod max7219 {
    use crate::println;
    use arduino_hal::port::{mode::Output, Pin};

    pub const NUM_DEVICES: usize = 4;
    const MATRIX_SIZE: usize = 8;

    #[derive(Copy, Clone)]
    enum Config {
        /// TODO: No idea what this is
        DecodeMode(bool),
        /// How bright the LEDs are. 0x0..=0xF
        Intensity(u8),
        /// How many LEDs turn on when addressing. Leave at MAX=8
        ScanLimit(u8),
        /// TODO: Shutdown mode.
        Shutdown(bool),
        /// Dispaly test mode turns on all the LEDs as a "display test".
        DisplayTest(bool),
    }

    pub struct Max7219 {
        /// Data in pin
        din: Pin<Output>,
        /// Chip select pin
        cs: Pin<Output>,
        /// Clock pin
        clk: Pin<Output>,
        /// Data to be transferred to the MAX7219 chip.
        spi_data: [u8; NUM_DEVICES * 2],
    }

    impl Max7219 {
        pub fn new(din: Pin<Output>, cs: Pin<Output>, clk: Pin<Output>) -> Self {
            assert!(NUM_DEVICES > 0);
            Max7219 {
                din,
                cs,
                clk,
                spi_data: [0; NUM_DEVICES * 2],
            }
        }

        /// device indexes are inclusive
        pub fn set_row(&mut self, idx_dev_frst: usize, idx_dev_last: usize, row: u8, value: u8) {
            for dev_idx in idx_dev_frst..idx_dev_last + 1 {
                // Matrix is 1 indexed
                self.spi_data[2 * dev_idx + 0] = row + 1;
                self.spi_data[2 * dev_idx + 1] = value;
            }

            self.bitbang_write()
        }

        /// Initialize the matrix to be controlled. Must be called before you are able to turn on
        /// LEDS.
        pub fn init(&mut self) {
            self.cs.set_high();

            self.display_test_mode(false);
            self.scan_limit(7);
            self.intensity(0);
            self.decode_mode(false);
            self.shutdown_mode(false);
        }

        pub fn decode_mode(&mut self, enabled: bool) {
            self.configure(Config::DecodeMode(enabled));
        }
        pub fn intensity(&mut self, intensity: u8) {
            assert!(intensity <= 0xF, "0xF is the max intensity");
            self.configure(Config::Intensity(intensity));
        }
        pub fn scan_limit(&mut self, scan_limit: u8) {
            assert!(
                scan_limit as usize <= MATRIX_SIZE,
                "The max scan limit is the number of LEDs on the device."
            );
            self.configure(Config::ScanLimit(scan_limit));
        }
        pub fn shutdown_mode(&mut self, enabled: bool) {
            self.configure(Config::Shutdown(enabled));
        }
        /// Forces all LEDs on.
        pub fn display_test_mode(&mut self, enabled: bool) {
            self.configure(Config::DisplayTest(enabled));
        }

        /// Implemented as per the spec sheet. Applies to all connected modules.
        fn configure(&mut self, config: Config) {
            let (opcode, option) = match config {
                Config::DecodeMode(b) => (0x9, if b { 0xff } else { 0 }),
                Config::Intensity(n) => (0xA, n),
                Config::ScanLimit(n) => (0xB, n),
                Config::Shutdown(b) => (0xC, if b { 0 } else { 1 }),
                Config::DisplayTest(b) => (0xF, b as u8),
            };
            for i in 0..NUM_DEVICES {
                self.spi_data[i * 2 + 0] = opcode;
                self.spi_data[i * 2 + 1] = option;
            }

            self.bitbang_write();
        }

        /// Writes the data in `spi_data` via `shift_out`.
        fn bitbang_write(&mut self) {
            self.cs.set_low();
            for data in self.spi_data {
                self.shift_out(data);
            }
            self.cs.set_high();
        }

        /// MSB shifted out first
        pub fn shift_out(&mut self, mut value: u8) {
            for _ in 0..u8::BITS {
                if (value & 128) != 0 {
                    self.din.set_high();
                } else {
                    self.din.set_low();
                }
                value <<= 1;

                self.clk.set_high();
                self.clk.set_low();
            }
        }
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let ser = arduino_hal::default_serial!(dp, pins, 57600);

    serial::init(ser);

    let din = pins.d13.into_output().downgrade();
    let cs = pins.d12.into_output().downgrade();
    let clk = pins.d11.into_output().downgrade();
    let mut matrix = max7219::Max7219::new(din, cs, clk);
    matrix.init();

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
        for i in 0..max7219::NUM_DEVICES {
            for row in 0..8 {
                matrix.set_row(i, i, row, u8::MAX);
                delay_ms(100);
                matrix.set_row(i, i, row, 0);
            }
        }
    }
}
