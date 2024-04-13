#![cfg_attr(not(test), no_std)]

mod emulated_stepper;
pub mod target_queue;

pub use emulated_stepper::{Callbacks, Direction, EmulatedStepper, PidTimeIterator};

pub(crate) type Instant = fugit::Instant<u32, 1, 1>;
pub(crate) type Duration = fugit::Duration<u32, 1, 1>;
