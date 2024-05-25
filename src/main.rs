#![no_std]
#![no_main]

use arduino_hal::{
    delay_ms, delay_us, hal::port::{PD0, PD1}, pac::USART0, port::{
        mode::{Input, Output},
        Pin,
    }, Usart
};
use panic_halt as _;

const NUM_DEVICES: usize = 4;
/// #define SPI_OFFSET(i, x) (((LAST_BUFFER - (i)) * 2) + (x))
const LAST_BUFFER: usize = NUM_DEVICES - 1;
const fn SPI_OFFSET(i: usize, x: usize) -> usize {
    (LAST_BUFFER - i) * 2 + x
}

#[derive(Copy, Clone)]
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
            Command::Shutdown(b) => (0xC, if *b {0} else {1}),
            Command::DisplayTest(b) => (0xF, (*b) as u8),
        }
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    ufmt::uwriteln!(&mut serial, "Hello from Arduino!\r").unwrap();

    let mut din = pins.d13.into_output().downgrade();
    let mut cs = pins.d12.into_output().downgrade();
    let mut clk = pins.d11.into_output().downgrade();
    cs.set_high();

    delay_ms(1);

    control(
        Command::DisplayTest(false),
        &mut din, &mut cs, &mut clk, &mut serial);
    control(
        Command::ScanLimit(7),
        &mut din, &mut cs, &mut clk, &mut serial,);
    control(
        Command::Intensity(0),
        &mut din, &mut cs, &mut clk, &mut serial,);
    control(
        Command::DecodeMode(false),
        &mut din, &mut cs, &mut clk, &mut serial,);
    control(
        Command::Shutdown(false),
        &mut din, &mut cs, &mut clk, &mut serial,);


    /*
     * For examples (and inspiration), head to
     *
     *     https://github.com/Rahix/avr-hal/tree/main/examples
     *
     * NOTE: Not all examples were ported to all boards!  There is a good chance though, that code
     * for a different board can be adapted for yours.  The Arduino Uno currently has the most
     * examples available.
     */

    set_row(0, 0, 0xFF, &mut din, &mut cs, &mut clk);

    loop {
        for row in 0..8 {
            set_row(0, row, u8::MAX, &mut din, &mut cs, &mut clk);
            delay_ms(500);
            set_row(0, row, 0u8, &mut din, &mut cs, &mut clk);
        }
    }
}

fn control(
    command: Command,
    data_pin: &mut Pin<Output>,
    chip_select: &mut Pin<Output>,
    clock_pin: &mut Pin<Output>,
    serial: &mut Usart<USART0, Pin<Input, PD0>, Pin<Output, PD1>>,
) {
    let mut spi_data = [0u8; 2 * NUM_DEVICES];

    for device_idx in 0..NUM_DEVICES {
        let (opcode, param) = command.as_u8s();
        spi_data[SPI_OFFSET(device_idx, 0)] = opcode;
        spi_data[SPI_OFFSET(device_idx, 1)] = param;
        ufmt::uwrite!(serial, "{} {} ", opcode, param).unwrap();
    }
    ufmt::uwriteln!(serial, "").unwrap();

    chip_select.set_low();

    for data in spi_data {
        shift_out(data_pin, clock_pin, data, Some(serial));
    }

    chip_select.set_high();
}

// void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val)
// {
// 	uint8_t i;
//
// 	for (i = 0; i < 8; i++)  {
// 		if (bitOrder == LSBFIRST) {
// 			digitalWrite(dataPin, val & 1);
// 			val >>= 1;
// 		} else {
// 			digitalWrite(dataPin, (val & 128) != 0);
// 			val <<= 1;
// 		}
//
// 		digitalWrite(clockPin, HIGH);
// 		digitalWrite(clockPin, LOW);
// 	}
// }
fn shift_out(
    data_pin: &mut Pin<Output>,
    clock_pin: &mut Pin<Output>,
    mut value: u8,
    mut serialopt: Option<&mut Usart<USART0, Pin<Input, PD0>, Pin<Output, PD1>>>,
) {
    
    for _ in 0..u8::BITS {
        if (value & 128) != 0 {
            // if let Some(ref mut serial) = serialopt {
            //     ufmt::uwrite!(serial, "{}", 1).unwrap();
            // }
            data_pin.set_high();
        } else {
            // if let Some(ref mut serial) = serialopt {
            // ufmt::uwrite!(serial, ".", ).unwrap();
            // }
            data_pin.set_low();
        }
        value <<= 1;

        clock_pin.set_high();
        clock_pin.set_low();
    }

    // if let Some(serial) = serialopt {
    //     ufmt::uwriteln!(serial, "",).unwrap();
    // }
}

// void setRow(uint8_t device, uint8_t row, uint8_t value) {
//   uint8_t* spiData = (uint8_t*)malloc(SPI_DATA_SIZE);
//
//   for (int i = 0; i < MAX_DEVICES; ++i) {
//     if (i == device) {
//       spiData[SPI_OFFSET(i, 0)] = OP_DIGIT0 + row;
//       spiData[SPI_OFFSET(i, 1)] = value;
//     }
//   }
//
//
//   digitalWrite(CS_PIN, LOW);
//   for (uint16_t i = 0; i < SPI_DATA_SIZE; i++) {
//     shiftOut(DATA_PIN, CLK_PIN, MSBFIRST, spiData[i]);
//   }
//   digitalWrite(CS_PIN, HIGH);
//
//   free(spiData);
// }
fn set_row(
    device_idx: u8,
    row: u8,
    value: u8,
    data_pin: &mut Pin<Output>,
    chip_select: &mut Pin<Output>,
    clock_pin: &mut Pin<Output>,
) {
    let mut spi_data = [0u8; 2 * NUM_DEVICES as usize];


    for device_idx in 0..NUM_DEVICES {
        spi_data[SPI_OFFSET(device_idx, 0)] = row + 1;
        spi_data[SPI_OFFSET(device_idx, 1)] = value;
    }

    for i in (0..2 * NUM_DEVICES as usize).step_by(2) {
        if i / 2 == device_idx.into() {
            let high = (row + 1) as u8;
            let low = value;
            spi_data[i] = high;
            spi_data[i + 1] = low;
        }
    }

    chip_select.set_low();

    for data in spi_data {
        shift_out(data_pin, clock_pin, data, None);
    }

    chip_select.set_high();
}
