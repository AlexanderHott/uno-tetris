#![no_std]
#![no_main]
#![feature(generic_const_exprs)]

use arduino_hal::delay_ms;
use panic_halt as _;
use tetris::Tetris;

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

    // pub const NUM_DEVICES: usize = 4;
    // const MATRIX_SIZE: usize = 8;
    //
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

    pub struct Max7219<const MATRIX_SIZE: usize, const DEVICE_COUNT: usize> {
        /// Data in pin
        din: Pin<Output>,
        /// Chip select pin
        cs: Pin<Output>,
        /// Clock pin
        clk: Pin<Output>,
        /// Data to be transferred to the MAX7219 chip.
        spi_data: [u16; DEVICE_COUNT],
    }

    impl<const MATRIX_SIZE: usize, const DEVICE_COUNT: usize> Max7219<MATRIX_SIZE, DEVICE_COUNT> {
        pub fn new(din: Pin<Output>, cs: Pin<Output>, clk: Pin<Output>) -> Self {
            Max7219 {
                din,
                cs,
                clk,
                spi_data: [0; DEVICE_COUNT],
            }
        }

        /// device indexes are inclusive
        pub fn set_row(&mut self, idx_dev_frst: usize, idx_dev_last: usize, row: u8, value: u8) {
            self.spi_data = [0; DEVICE_COUNT];
            for dev_idx in idx_dev_frst..idx_dev_last + 1 {
                // Matrix is 1 indexed
                let val = (((row + 1) as u16) << 8) + value as u16; // TODO:  get rid of u8, u8 ->
                                                                    // u16 somehow
                self.spi_data[dev_idx] = val;
            }

            self.bitbang_write()
        }

        pub fn clear(&mut self) {
            for row in 0..8 {
                self.set_row(0, DEVICE_COUNT - 1, row, 0);
            }
        }

        /// Set the entire board
        pub fn set_board(&mut self, led_states: &[u8; DEVICE_COUNT * MATRIX_SIZE]) {
            for (i, led_state) in led_states.into_iter().enumerate() {
                let device_idx = i % DEVICE_COUNT;
                let row: u16 = (i as u16 / 4) + 1;

                self.spi_data[device_idx] = (row << 8) + (*led_state) as u16;

                if i % DEVICE_COUNT == DEVICE_COUNT - 1 {
                    self.bitbang_write();
                }
            }
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
            for i in 0..DEVICE_COUNT {
                let val = ((opcode as u16) << 8) + option as u16;
                self.spi_data[i] = val;
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
        pub fn shift_out(&mut self, mut value: u16) {
            for _ in 0..u16::BITS {
                if (value & 0b1000_0000_0000_0000) != 0 {
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

mod tetris {
    use crate::println;

    const BLOCKS_PER_SHAPE: usize = 4;

    /// Tetris shapes.
    /// https://en.wikipedia.org/wiki/Tetromino
    #[derive(Copy, Clone)]
    enum Shape {
        /// XXXXXX
        ///   XX
        T { x: usize, y: usize }, // TODO: add rotation
        /// XX
        /// XX
        /// XXXX
        L { x: usize, y: usize },
        /// XX
        /// XX
        /// XX
        /// XX
        I { x: usize, y: usize },
    }

    impl Shape {
        /// Shapes are always positioned at the top-left corner, so all relative offsets are
        /// positive. Returns an array of (x,y) absolute (not relative) points in the 8x32
        /// coordinate system.
        fn get_absolute_points(&self) -> [(usize, usize); BLOCKS_PER_SHAPE] {
            // TODO: maybe change to abs cus it can be done here;
            match self {
                Shape::T { x, y } => [
                    (x + 0, y + 0),
                    (x + 1, y + 0),
                    (x + 2, y + 0),
                    (x + 1, y + 1),
                ],
                Shape::L { x, y } => [
                    (x + 0, y + 0),
                    (x + 0, y + 1),
                    (x + 0, y + 2),
                    (x + 1, y + 2),
                ],
                Shape::I { x, y } => [
                    (x + 0, y + 0),
                    (x + 0, y + 1),
                    (x + 0, y + 2),
                    (x + 0, y + 3),
                ],
            }
        }

        fn x(&self) -> usize {
            *match self {
                Shape::T { x, .. } => x,
                Shape::L { x, .. } => x,
                Shape::I { x, .. } => x,
            }
        }

        fn set_x(&mut self, new_x: usize) {
            match self {
                Shape::T { ref mut x, .. } => *x = new_x,
                Shape::L { ref mut x, .. } => *x = new_x,
                Shape::I { ref mut x, .. } => *x = new_x,
            };
        }

        fn y(&self) -> usize {
            *match self {
                Shape::T { y, .. } => y,
                Shape::L { y, .. } => y,
                Shape::I { y, .. } => y,
            }
        }

        fn set_y(&mut self, new_y: usize) {
            match self {
                Shape::T { ref mut y, .. } => *y = new_y,
                Shape::L { ref mut y, .. } => *y = new_y,
                Shape::I { ref mut y, .. } => *y = new_y,
            };
        }
    }

    pub struct Tetris {
        board: Board,
    }

    impl Tetris {
        pub fn new() -> Self {
            println!("new tetris");
            Tetris {
                board: Board::new(),
            }
        }

        pub fn render_board(&mut self) -> [u8; 32] {
            let moved = self.board.try_move_current_shape_down();
            self.board.render();

            if !moved {
                self.board.clear_full_rows();
                // self.board.replace_current_shape();
            }

            self.board.matrix
        }
    }

    struct Board {
        shapes: [Option<Shape>; 100],
        current_shape: Shape,
        pub matrix: [u8; 32],
    }

    impl Board {
        pub fn new() -> Self {
            let mut shapes = [None; 100];
            for i in 0..7 {
                shapes[i] = Some(Shape::I { x: i, y: 28 })
            }
            Board {
                shapes,
                current_shape: Shape::I { x: 7, y: 26 },
                matrix: [0u8; 32],
            }
        }

        pub fn render(&mut self) -> [u8; 32] {
            self.matrix = [0u8; 32];
            // already placed shapes
            for shape_opt in self.shapes {
                if let Some(shape) = shape_opt {
                    for (x, y) in shape.get_absolute_points() {
                        self.set_bit(x, y)
                    }
                }
            }
            // current shape
            for (x, y) in self.current_shape.get_absolute_points() {
                self.set_bit(x, y)
            }
            // TODO: blinking shadow of current shape

            self.matrix
        }

        fn set_bit(&mut self, x: usize, y: usize) {
            assert!(x < 8);
            assert!(y < 32); // TODO make 32 = 8 * dev count

            self.matrix[y] |= 0b1000_0000 >> x;
        }

        fn bit_at(&self, x: usize, y: usize) -> bool {
            self.matrix[y] & (0b1000_0000 >> x) != 0
        }

        /// Tries to move the current active shape down. Returns
        /// `true` if it moved.
        pub fn try_move_current_shape_down(&mut self) -> bool {
            if self.can_move_down(&self.current_shape) {
                self.current_shape.set_y(self.current_shape.y() + 1);
                return true;
            }
            return false;
        }

        // TODO: get bounding box
        fn can_move_down(&self, shape: &Shape) -> bool {
            for (x, y) in shape.get_absolute_points() {
                if y >= 31 || self.bit_at(x, y + 1) {
                    return false;
                }
            }
            return true;
        }

        /// Clears any full rows and returns how many rows were cleared.
        /// ## ##    ## <- c
        ///             <- b
        fn clear_full_rows(&mut self) -> usize {
            // TODO: maybe can start looking forward from the base of the last placed shape to save
            // loop iterations.

            println!("clearing rows");
            let mut base_idx = 31;
            let mut curr_idx = 31;
            loop {
                println!("  clearing row {}: {}", curr_idx, self.matrix[curr_idx]);
                if self.matrix[curr_idx] == u8::MAX {
                    self.matrix[curr_idx] = 0;
                } else {
                    self.matrix[base_idx] = self.matrix[curr_idx];
                    base_idx -= 1;
                }
                curr_idx -= 1;

                if curr_idx == 0 {
                    break base_idx; // cleared = base - curr, but curr = 0
                }
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
    // TODO: hard code 8 as a constant. It isn't really generic
    let mut matrix: max7219::Max7219<8, 4> = max7219::Max7219::new(din, cs, clk);
    matrix.init();
    let mut tetris = Tetris::new();

    loop {
        let screen = tetris.render_board();
        let screen = rotate_bits_left(screen);
        matrix.clear();
        matrix.set_board(&screen);
        delay_ms(1000);

        // let cats = [
        //     0b00100010, 0b01010101, 0b01011101, 0b10000000, 0b10100100, 0b10000000, 0b01000001,
        //     0b00111110, 0b00100010, 0b01010101, 0b01011101, 0b10000000, 0b10100100, 0b10000000,
        //     0b01000001, 0b00111110, 0b00100010, 0b01010101, 0b01011101, 0b10000000, 0b10100100,
        //     0b10000000, 0b01000001, 0b00111110, 0b00100010, 0b01010101, 0b01011101, 0b10000000,
        //     0b10100100, 0b10000000, 0b01000001, 0b00111110,
        // ];
        // let cats_rotated = rotate_bits_left(cats);
        // matrix.set_board(&cats_rotated);
        //
        // delay_ms(1000);
        // matrix.set_board(&[
        //     0b00100010, 0b00100010, 0b00100010, 0b00100010, 0b01010101, 0b01010101, 0b01010101,
        //     0b01010101, 0b01011101, 0b01011101, 0b01011101, 0b01011101, 0b10000000, 0b10000000,
        //     0b10000000, 0b10000000, 0b10100100, 0b10100100, 0b10100100, 0b10100100, 0b10000000,
        //     0b10000000, 0b10000000, 0b10000000, 0b01000001, 0b01000001, 0b01000001, 0b01000001,
        //     0b00111110, 0b00111110, 0b00111110, 0b00111110,
        // ]);
        // delay_ms(1000);

        // matrix.set_board(&[
        //   0b00100010, 0b00100010, 0b00100010, 0b00100010,
        //   0b01010101, 0b01010101, 0b01010101, 0b01010101,
        //   0b01011101, 0b01011101, 0b01011101, 0b01011101,
        //   0b10000000, 0b10000000, 0b10000000, 0b10000000,
        //   0b10100100, 0b10100100, 0b10100100, 0b10100100,
        //   0b10000000, 0b10000000, 0b10000000, 0b10000000,
        //   0b01000001, 0b01000001, 0b01000001, 0b01000001,
        //   0b00111110, 0b00111110, 0b00111110, 0b00111110,
        // ]);
        // delay_ms(1000);
        // matrix.set_board(&[
        //     0b01000100, 0b01000100, 0b01000100, 0b01000100, 0b10101010, 0b10101010, 0b10101010,
        //     0b10101010, 0b10111010, 0b10111010, 0b10111010, 0b10111010, 0b00000001, 0b00000001,
        //     0b00000001, 0b00000001, 0b01001001, 0b01001001, 0b01001001, 0b01001001, 0b00000001,
        //     0b00000001, 0b00000001, 0b00000001, 0b10000010, 0b10000010, 0b10000010, 0b10000010,
        //     0b01111100, 0b01111100, 0b01111100, 0b01111100,
        // ]);
        // delay_ms(1000);
    }
}

/// Rotates a 32x8 bit matrix left (CCW) into a 8x32 bit matrix.
/// It works by turning the `u8`s into bits, rotating the matrix, and reinterpreting the bits as
/// `u8`s
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    00000000,
///    01000000,
///    11100000,
///
///
///    00000000, 00000000, 00000000, 00000000,
///    00000000, 00000000, 00000000, 00000000,
///    00000000, 00000000, 00000000, 00000000,
///    00000000, 00000000, 00000000, 00000000,
///    00000000, 00000000, 00000000, 00000000,
///    00000000, 00000000, 00000000, 00000001,
///    00000000, 00000000, 00000000, 00000011,
///    00000000, 00000000, 00000000, 00000001
pub fn rotate_bits_left(arr: [u8; 32]) -> [u8; 32] {
    let mut result = [0u8; 32];

    // chunk_i is the current (vertical) chunk we are looking at.
    // result_idx % 4 = chunk_i
    for (chunk_i, chunk) in arr.chunks(8).into_iter().enumerate() {
        // bit_pos is the current vertical slice of bits we are looking at, starting with 0 on the
        // right.
        for bit_pos in 0..8 {
            let mut x = 0;
            for (shift_by, num) in chunk.iter().enumerate() {
                // the first number in the array is the MSB of the resulting output number
                // so we need to shift it all the way to the right. We could reverse
                // `chunk.iter()`, but that might not be as clear.
                let shift_by = 7 - shift_by;
                if num & (1 << bit_pos) != 0 {
                    x += 1 << shift_by
                }
            }
            // instead of storing results in the order 0, 1, 2, 3...
            // we store them in 0, 4, 8...1, 5, 9... because that is
            // the order we calculate them in.
            let result_idx = bit_pos * 4 + chunk_i;
            result[result_idx] = x;
        }
    }

    result
}
