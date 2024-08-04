# `gd32c1x3-hal`

> [HAL] for the GD32C1x3 family of microcontrollers

[hal]: https://crates.io/crates/embedded-hal

[![Continuous integration](https://github.com/guineawheek/gd32c1x3-hal/workflows/Continuous%20integration/badge.svg)](https://github.com/guineawheek/gd32c1x3-hal)
[![crates.io](https://img.shields.io/crates/v/gd32c1x3-hal.svg)](https://crates.io/crates/gd32c1x3-hal)
[![Released API docs](https://docs.rs/gd32c1x3-hal/badge.svg)](https://docs.rs/gd32c1x3-hal)

## Quick start guide

Embedded Rust development requires a bit more setup than ordinary development.

You will also need a debug probe, for example an
[ST-Link V3 mini](https://www.st.com/en/development-tools/stlink-v3mini.html) for programming and
debugging. (There are many different STLink probes out there, all of them _should_ work fine with
the instructions given here, other JTAG or SWD debug probes will work as well but will need
different software or configuration).

### Installing software

To program your microcontroller, you need to install:

- [openocd](http://openocd.org/) or [stlink](https://github.com/stlink-org/stlink)
- `gdb-multiarch` (on some platforms you may need to use `gdb-arm-none-eabi` instead, make sure to
  update `.cargo/config` to reflect this change)

Finally, you need to install arm target support for the Rust compiler. To do so, run

```
rustup target install thumbv7em-none-eabihf
```

### Setting up your project

Create a new Rust project as you usually do with `cargo init`. The hello world
of embedded development is usually to blink an LED and code to do so is
available in [examples/blinky.rs](examples/blinky.rs). Copy that file to the
`main.rs` of your project.

You also need to add some dependencies to your `Cargo.toml`:

```toml
[dependencies]
embedded-hal = "1.0.0"
nb = "1.1.0"
cortex-m = { version = "0.7.7", features = ["critical-section-single-core"] }
cortex-m-rt = "0.7.3"
# Panic behaviour, see https://crates.io/keywords/panic-impl for alternatives
panic-halt = "0.2.0"
gd32c1x3-hal = { version = "0.1.0", features = ["rt", "gd32c103"] }
```

If you build your project now, you should get a single error:
`error: language item required, but not found: eh_personality`. This unhelpful error message is
fixed by compiling for the right target.

We also need to tell Rust how to link our executable, and how to lay out the
result in memory. To accomplish all this, copy [.cargo/config](.cargo/config) and
[memory.x](memory.x) from the gd32c1x3-hal repo to your project.

```bash
cargo build
```

If everything went well, your project should have built without errors.

### Programming the microcontroller

It is now time to actually run the code on the hardware. To do so plug your
debug probe into your board and start `openocd` using

```bash
openocd -f interface/stlink-v3.cfg -f target/stm32f1x.cfg
```

If you are not using an stlink V3, change the interface accordingly.
For more information, see the [embeddonomicon].

If all went well, it should detect your microcontroller and say
`Info : stm32f1x.cpu: hardware has 6 breakpoints, 4 watchpoints`. Keep it running in the background.

We will use gdb for uploading the compiled binary to the microcontroller and
for debugging. Cargo will automatically start `gdb` thanks to the
[.cargo/config](.cargo/config) you added earlier. `gdb` also needs to be told
to connect to openocd which is done by copying [.gdbinit](.gdbinit) to the root
of your project.

You may also need to tell `gdb` that it is safe to load `.gdbinit` from the
working directory.

- Linux
  ```bash
  echo "set auto-load safe-path $(pwd)" >> ~/.gdbinit
  ```
- Windows
  ```batch
  echo set auto-load safe-path %CD% >> %USERPROFILE%\.gdbinit
  ```

If everything was successful, cargo should compile your project, start GDB, load your program and
give you a prompt. If you type `continue` in the GDB prompt, your program should start and an LED
attached to PC13 should start blinking.

### Going further

From here on, you can start adding more code to your project to make it do something more
interesting. For crate documentation, see [docs.rs/gd32c1x3-hal](https://docs.rs/gd32c1x3-hal).
There are also a lot more [examples](examples) available. If something is unclear in the docs or
examples, please, open an issue and we will try to improve it.

## Selecting a microcontroller

This crate supports multiple microcontrollers in the gd32c1x3 family. Which specific microcontroller
you want to build for has to be specified with a feature, for example `gd32c103`.

If no microcontroller is specified, the crate will not compile.

### Supported Microcontrollers

- `gd32c103xx` (e.g. GD32C103TB, GD32C103CB, GD32C103RB, GD32C103VB)
- `gd32c113xx` (e.g. GD32C113TB, GD32C113CB, GD32C113RB, GD32C113VB)

## Trying out the examples

You may need to give `cargo` permission to call `gdb` from the working directory.

- Linux
  ```bash
  echo "set auto-load safe-path $(pwd)" >> ~/.gdbinit
  ```
- Windows
  ```batch
  echo set auto-load safe-path %CD% >> %USERPROFILE%\.gdbinit
  ```

Compile, load, and launch the hardware debugger.

```bash
$ rustup target add thumbv7em-none-eabihf

# on another terminal
$ openocd -f interface/$INTERFACE.cfg -f target/stm32f1x.cfg

# flash and debug the "Hello, world" example. Change gd32c103 to match your hardware
$ cargo run --features gd32c103 --example hello
```

`$INTERFACE` should be set based on your debugging hardware. If you are using
an stlink V2, use `stlink-v2.cfg`. For more information, see the
[embeddonomicon].

[embeddonomicon]: https://rust-embedded.github.io/book/start/hardware.html

## Using as a Dependency

When using this crate as a dependency in your project, the microcontroller can
be specified as part of the `Cargo.toml` definition.

```toml
gd32c1x3-hal = { version = "0.1.0", features = ["gd32c103", "rt"] }
```

## Documentation

The documentation can be found at [docs.rs](https://docs.rs/gd32c1x3-hal/).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the
work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.

See the [contributing guide](CONTRIBUTING.md) for more details.