# uno-tetris

## TODO

- [ ] control LED matrix from rust via bitbang
  - [x] set entire screen at once
  - [ ] text + scrolling
- [ ] get tetris working
  - [x] main game logic of moving piece down and clearing rows
  - [ ] move piece left and right
  - [ ] rotate pieces
  - [ ] score counting
  - [ ] current piece shadow

optimizations

- [ ] matrix diffing engine for fewer updates
- [ ] control LED matrix with SPI
- [ ] replace `&[T]` with `Arc<T> / Rc<T>`

---

## Resources

- Embedded rust book: https://docs.rust-embedded.org/book/intro/index.html
- Arduino Uno std lib source: https://github.com/arduino/ArduinoCore-avr
- Making a timer in rust: https://blog.rahix.de/005-avr-hal-millis/

---

Rust project for the _Arduino Uno_.

## Build Instructions

1. Install prerequisites as described in the [`avr-hal` README] (`avr-gcc`, `avr-libc`, `avrdude`, [`ravedude`]).

2. Run `cargo build` to build the firmware.

3. Run `cargo run` to flash the firmware to a connected board. If `ravedude`
   fails to detect your board, check its documentation at
   <https://crates.io/crates/ravedude>.

4. `ravedude` will open a console session after flashing where you can interact
   with the UART console of your board.

[`avr-hal` README]: https://github.com/Rahix/avr-hal#readme
[`ravedude`]: https://crates.io/crates/ravedude

## License

Licensed under either of

- Apache License, Version 2.0
  ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license
  ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.
