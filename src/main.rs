#![no_std]
#![no_main]
#![feature(generic_const_exprs)]
#![feature(abi_avr_interrupt)]

use core::num::NonZeroU32;

use panic_halt as _;
use tetris::Tetris;
use timer::millis;

mod serial {
    use arduino_hal::{hal::usart::Usart0, DefaultClock};
    use avr_device::interrupt::{self, Mutex};
    use core::cell::RefCell;

    type Console = Mutex<RefCell<Option<Usart0<DefaultClock>>>>;

    pub static CONSOLE: Console = Mutex::new(RefCell::new(None));

    pub fn init(usart: Usart0<DefaultClock>) {
        // interrupt free critical section
        interrupt::free(|cs| {
            let mut rc = CONSOLE.borrow(cs).borrow_mut();
            *rc = Some(usart);
        })
    }

    #[macro_export]
    macro_rules! print {
        ($($arg:tt)*) => {
            avr_device::interrupt::free(|cs| {
                let mut console = crate::serial::CONSOLE.borrow(cs).borrow_mut();
                if let Some(usart) = console.as_mut() {
                    let _ = ufmt::uwrite!(usart, $($arg)*);
                }
            })
        };
    }

    #[macro_export]
    macro_rules! println {
        ($($arg:tt)*) => {
            avr_device::interrupt::free(|cs| {
                let mut console = crate::serial::CONSOLE.borrow(cs).borrow_mut();
                if let Some(usart) = console.as_mut() {
                    let _ = ufmt::uwriteln!(usart, $($arg)*);
                }
            })
        };
    }
}

mod timer {
    use arduino_hal::pac::TC0;
    use avr_device::interrupt::{self, Mutex};
    use core::cell::Cell;

    const PRESCALE_FACTOR: u32 = 1024;
    const TIMER_COUNTS: u32 = 125;
    // const PRESCALE_FACTOR: u32 = 64;
    // const TIMER_COUNTS: u32 = 250;

    /// | Prescale Factory | Timer Counts (MAX + 1) | Overflow Interval |
    /// | ---              | ---                    | ---               |
    /// | 64               | 250                    | 1 ms              |
    /// | 256	           | 125                    | 2 ms              |
    /// | 256	           | 250                    | 4 ms              |
    /// | 1024	           | 125                    | 8 ms              |
    /// | 1024	           | 250                    | 16 ms             |
    ///
    /// Finer resolution leads to higher CPU usage.
    const MILLIS_INCREMENT: u32 = PRESCALE_FACTOR * TIMER_COUNTS / 16_000 /*clock freq*/;

    static MILLIS_COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

    /// Interrupt sub-routine for the Clear Timer on Compare (CTC) hardware clock.
    #[avr_device::interrupt(atmega328p)]
    fn TIMER0_COMPA() {
        // interrupt free critical section
        interrupt::free(|cs| {
            let counter_cell = MILLIS_COUNTER.borrow(cs);
            let counter = counter_cell.get();
            counter_cell.set(counter + MILLIS_INCREMENT);
        })
    }

    /// Initialize a
    pub fn init(tc0: TC0) {
        // Enable Clear Timer on Compare (CTC) to have the timer interrupt on a custom interval.
        tc0.tccr0a.write(|w| w.wgm0().ctc());
        // Set the custom "counts" value to the Output Compare Register (`ocr0a`)
        tc0.ocr0a.write(|w| w.bits(TIMER_COUNTS as u8));
        tc0.tccr0b.write(|w| match PRESCALE_FACTOR {
            8 => w.cs0().prescale_8(),
            64 => w.cs0().prescale_64(),
            256 => w.cs0().prescale_256(),
            1024 => w.cs0().prescale_1024(),
            _ => panic!("invalid TIMER_COUNTS value. Must be 8, 64, 256, 1024"),
        });
        tc0.timsk0.write(|w| w.ocie0a().set_bit());

        interrupt::free(|cs| {
            MILLIS_COUNTER.borrow(cs).set(0);
        });
    }

    /// Milliseconds since chip start. About 8ms resolution. Overflows in ~50 days.
    pub fn millis() -> u32 {
        // https://github.com/arduino/ArduinoCore-avr/blob/321fca0bac806bdd36af8afbc13587f4b67eb5f1/cores/arduino/wiring.c#L65
        interrupt::free(|cs| MILLIS_COUNTER.borrow(cs).get())
    }
}

mod rand {
    /// Random number generator based on
    /// https://en.wikipedia.org/wiki/Xorshift
    pub struct Rand {
        state: u32,
    }

    impl Rand {
        pub fn new() -> Self {
            Rand { state: 1 }
        }

        pub fn from_seed(seed: core::num::NonZeroU32) -> Self {
            Rand { state: seed.get() }
        }

        pub fn rand(&mut self) -> u32 {
            let mut x = self.state;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            self.state = x;

            self.state
        }

        pub fn randrange(&mut self, lower: u32, upper: u32) -> u32 {
            let range = upper - lower;
            let random = self.rand() % range;

            return random + lower;
        }

        pub fn shuffle<'a, T>(&'_ mut self, array: &'a mut [T]) -> &'a mut [T] {
            let n = array.len() as u32;
            for i in 0..n - 2 {
                let j = self.randrange(i, n) as usize;
                array.swap(i as usize, j);
            }
            array
        }
    }
}

mod max7219 {
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
            // TODO: self.spi_data ^= self.spi_data;
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
            self.intensity(0x0);
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
        fn absolute_points(&self) -> [(usize, usize); BLOCKS_PER_SHAPE] {
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

        fn relative_points(&self, x: usize, y: usize) -> [(usize, usize); BLOCKS_PER_SHAPE] {
            match self {
                Shape::T { .. } => [
                    (x + 0, y + 0),
                    (x + 1, y + 0),
                    (x + 2, y + 0),
                    (x + 1, y + 1),
                ],
                Shape::L { .. } => [
                    (x + 0, y + 0),
                    (x + 0, y + 1),
                    (x + 0, y + 2),
                    (x + 1, y + 2),
                ],
                Shape::I { .. } => [
                    (x + 0, y + 0),
                    (x + 0, y + 1),
                    (x + 0, y + 2),
                    (x + 0, y + 3),
                ],
            }
        }

        fn height(&self) -> usize {
            match self {
                Shape::T { .. } => 2,
                Shape::L { .. } => 3,
                Shape::I { .. } => 4,
            }
        }

        fn width(&self) -> usize {
            match self {
                Shape::T { .. } => 3,
                Shape::L { .. } => 2,
                Shape::I { .. } => 1,
            }
        }

        /// returns a u8 where all the 1's represent columns where the shape is in
        fn column_mask(&self) -> u8 {
            match self {
                Shape::T { x, .. } => 0b11100000 >> x,
                Shape::L { x, .. } => 0b11000000 >> x,
                Shape::I { x, .. } => 0b10000000 >> x,
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
        current_shape: Shape,
        shadow_shape: Option<Shape>,
        board: BitBoard,
    }

    impl Tetris {
        pub fn new() -> Self {
            println!("new tetris");
            Tetris {
                current_shape: Shape::I { x: 7, y: 0 },
                shadow_shape: None,
                board: BitBoard::new(),
            }
        }

        /// Clears the current_shape and shadow_shape from the screen so that collision detection
        /// can work correctly.
        fn clear_current_shapes(&mut self) {
            for (x, y) in self.current_shape.absolute_points() {
                self.board.set_bit(x, y, false);
            }
            if let Some(shadow_shape) = self.shadow_shape {
                for (x, y) in shadow_shape.absolute_points() {
                    self.board.set_bit(x, y, false);
                }
            }
        }

        pub fn move_current_shape_left(&mut self) {
            let x = self.current_shape.x();
            if x == 0 {
                return;
            }
            self.clear_current_shapes();
            self.current_shape.set_x(x - 1);
            self.shadow_shape = None;
        }

        pub fn move_current_shape_right(&mut self) {
            let x = self.current_shape.x() + self.current_shape.width() - 1;
            if x == 7 {
                return;
            }
            self.clear_current_shapes();
            self.current_shape.set_x(self.current_shape.x() + 1);
            self.shadow_shape = None;
        }

        pub fn fast_place_current_shape(&mut self) {
            self.clear_current_shapes();
            self.current_shape
                .set_y(self.shadow_shape.expect("should exist").y());
            self.shadow_shape = None
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

        // TODO: make random without replacement (deck of cards)
        fn replace_current_shape(&mut self) {
            // self.current_shape = Shape::T { x: 0, y: 0 };
            self.current_shape = Shape::I { x: 0, y: 0 };
            self.shadow_shape = None;
        }

        // TODO: get bounding box
        fn can_move_down(&self, shape: &Shape) -> bool {
            for (x, y) in shape.absolute_points() {
                if y >= 31 || self.board.bit_at(x, y + 1) {
                    return false;
                }
            }
            return true;
        }

        fn update_shadow(&mut self) {
            let column_mask = self.current_shape.column_mask();
            let shadow_height = self.board.highest_free_row(self.current_shape, column_mask);
            let mut shadow_shape = self.current_shape.clone();
            shadow_shape.set_y(shadow_height);
            self.shadow_shape = Some(shadow_shape)
        }

        pub fn render_board(
            &mut self,
            should_move_current_shape_down: bool,
            render_shadow: bool,
        ) -> [u8; 32] {
            if self.shadow_shape.is_none() {
                self.update_shadow();
            }

            self.clear_current_shapes();

            let mut did_move = false;
            if should_move_current_shape_down {
                did_move = self.try_move_current_shape_down();
            }

            // add current piece back
            for (x, y) in self.current_shape.absolute_points() {
                self.board.set_bit(x, y, true);
            }
            if render_shadow {
                for (x, y) in self
                    .shadow_shape
                    .expect("should be some after .update_shadow()")
                    .absolute_points()
                {
                    self.board.set_bit(x, y, true);
                }
            }

            if should_move_current_shape_down {
                if !did_move {
                    self.board.clear_full_rows();
                    // TODO: check if we can replace the shape, otherwise end the game
                    self.replace_current_shape();
                }
            }

            self.board.bitboard
        }
    }

    /// Bitboard with top left as (0,0).
    struct BitBoard {
        pub bitboard: [u8; 32],
    }

    impl BitBoard {
        pub fn new() -> Self {
            BitBoard {
                // matrix: [0u8; 32],
                bitboard: [
                    0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8,
                    0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0b11110000, 0b11111110,
                    0b11111110, 0b11111110, 0b11111110,
                ],
            }
        }

        fn highest_free_row(&self, shape: Shape, column_mask: u8) -> usize {
            // TODO: recalc bounds based on bottom height of shape.
            // we shouldn't even be doing this if we are unable to place the shape
            // which is the game-over condition
            for i in 1..32 {
                if i + shape.height() - 1 >= 32 {
                    return i - 1;
                }
                for (x, y) in shape.relative_points(shape.x(), i) {
                    if self.bit_at(x, y) {
                        return i - 1;
                    }
                }
            }
            return 31 - shape.height() + 1;
        }

        fn set_bit(&mut self, x: usize, y: usize, on: bool) {
            assert!(x < 8);
            assert!(y < 32); // TODO make 32 = 8 * dev count

            if on {
                self.bitboard[y] |= 0b1000_0000 >> x;
            } else {
                self.bitboard[y] &= !(0b1000_0000 >> x);
            }
        }

        fn bit_at(&self, x: usize, y: usize) -> bool {
            self.bitboard[y] & (0b1000_0000 >> x) != 0
        }

        /// Clears any full rows and returns how many rows were cleared.
        /// ## ##    ## <- c
        ///             <- b
        fn clear_full_rows(&mut self) -> usize {
            // TODO: maybe can start looking forward from the base of the last placed shape to save
            // loop iterations.

            let mut base_idx = 31;
            let mut curr_idx = 31;
            loop {
                if self.bitboard[curr_idx] == u8::MAX {
                    self.bitboard[curr_idx] = 0;
                } else {
                    self.bitboard[base_idx] = self.bitboard[curr_idx];
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

    crate::timer::init(dp.TC0);
    // must enable interrupts for the timer to work
    // SAFETY: This is not called from inside a critical section.
    unsafe { avr_device::interrupt::enable() };

    let pins = arduino_hal::pins!(dp);
    let ser = arduino_hal::default_serial!(dp, pins, 57600);

    serial::init(ser);

    let din = pins.d13.into_output().downgrade();
    let cs = pins.d12.into_output().downgrade();
    let clk = pins.d11.into_output().downgrade();

    let right_btn = pins.d2.into_pull_up_input().downgrade();
    let up_btn = pins.d3.into_pull_up_input().downgrade();
    let down_btn = pins.d4.into_pull_up_input().downgrade();
    let left_btn = pins.d5.into_pull_up_input().downgrade();
    let mut right_btn_pressed = false;
    let mut up_btn_pressed = false;
    let mut down_btn_pressed = false;
    let mut left_btn_pressed = false;

    // TODO: hard code 8 as a constant. It isn't really generic
    let mut matrix: max7219::Max7219<8, 4> = max7219::Max7219::new(din, cs, clk);
    matrix.init();
    // matrix.intensity(0xF);
    let mut tetris = Tetris::new();

    const MOVE_DOWN_TIME: u32 = 1000;
    const BLINK_TIME: u32 = 100;

    // get random seed from 10 analog samples
    // a0 should be disconnected
    let mut seed = 0;
    let mut adc = arduino_hal::Adc::new(dp.ADC, Default::default());
    let a0 = pins.a0.into_analog_input(&mut adc);
    for i in 1..10 {
        let an = a0.analog_read(&mut adc);
        seed += i * an;
    }
    let seed = match NonZeroU32::new(seed as u32) {
        Some(s) => s,
        None => NonZeroU32::new(seed as u32 + 1).expect("shoudn't be zero after an +1"),
    };

    let mut rand = rand::Rand::from_seed(seed);

    loop {
        // we handle the main game loop of tetris because it requires us to poll for new frames
        // and render them on the LED matrix

        // pseudo code
        // while not game over {
        //   get user input
        //   do action based on user input
        //   if time since last moved down - now > 1s {
        //      move current piece down
        //   }
        //   ask tetris for frame
        //   render new frame
        // }

        let game_over = false;
        let mut last_shape_move = millis();
        let mut render_shadow = false;
        let mut last_shadow_blink = millis();
        while !game_over {
            // handle user input
            if right_btn.is_high() && !right_btn_pressed {
                right_btn_pressed = true;
                tetris.move_current_shape_right();
            } else if right_btn.is_low() {
                right_btn_pressed = false;
            }
            if up_btn.is_high() && !up_btn_pressed {
                up_btn_pressed = true;
            } else if up_btn.is_low() {
                up_btn_pressed = false;
            }
            if down_btn.is_high() && !down_btn_pressed {
                down_btn_pressed = true;
                tetris.fast_place_current_shape()
            } else if down_btn.is_low() {
                down_btn_pressed = false;
            }
            if left_btn.is_high() && !left_btn_pressed {
                left_btn_pressed = true;
                tetris.move_current_shape_left();
            } else if left_btn.is_low() {
                left_btn_pressed = false;
            }
            // TODO check user input
            // TODO match user_input {}

            let now = millis();
            let should_move_current_shape_down = now - last_shape_move > MOVE_DOWN_TIME;

            if should_move_current_shape_down {
                last_shape_move = now;
            }
            if now - last_shadow_blink > BLINK_TIME {
                render_shadow = !render_shadow;
                last_shadow_blink = now;
            }

            let screen = tetris.render_board(should_move_current_shape_down, render_shadow);
            let screen = rotate_bits_left(screen);
            matrix.clear();
            matrix.set_board(&screen);
        }
    }
}
///
/// Rotates a 32x8 bit matrix left (CCW) into a 8x32 bit matrix.
/// It works by turning the `u8`s into bits, rotating the matrix, and reinterpreting the bits as
/// `u8`s.
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
