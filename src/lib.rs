//! HAL for the STM32f446 family of microcontrollers
//!
//! This is an implementation of the [`embedded-hal`] traits for the STM32f446 family of
//! microcontrollers.
//!
//! [`embedded-hal`]: https://github.com/japaric/embedded-hal
//!
//! # Usage
//!
//! To build applications (binary crates) using this crate follow the [cortex-m-quickstart]
//! instructions and add this crate as a dependency in step number 5 and make sure you enable the
//! "rt" Cargo feature of this crate.
//!
//! [cortex-m-quickstart]: https://docs.rs/cortex-m-quickstart/~0.2.3
//!
//! # Examples
//!
//! Examples of *using* these abstractions can be found in the documentation of the [`f3`] crate.
//!
//! [`f3`]: https://docs.rs/f3/~0.5.1

#![deny(missing_docs)]
//#![deny(warnings)]
#![feature(never_type)]
#![no_std]

extern crate cast;
extern crate cortex_m;
extern crate embedded_hal as hal;
extern crate nb;
pub extern crate stm32f4;
extern crate void;
pub use stm32f4::stm32f446;
pub use stm32f4::*;

pub mod delay;

//pub mod dma;
pub mod flash;
pub mod gpio;
//pub mod i2c;
//pub mod i2s;
pub mod prelude;
pub mod rcc;
pub mod serial;
//pub mod spi;
pub mod time;
pub mod timer;
