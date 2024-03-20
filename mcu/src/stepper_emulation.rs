use crate::commands::CommandContext;
use anchor::*;

#[derive(Debug, defmt::Format, Eq, PartialEq, PartialOrd, Ord)]
enum Direction {
    Forward,
    Backward,
}

#[derive(Debug, defmt::Format)]
struct State {
    last_step: u64,
    position: u32,
    direction: Direction,
}

#[derive(Debug, defmt::Format, Copy, Clone)]
struct Move {
    interval: u32,
    count: u16,
    add: u16,
}

impl Move {
    fn total_time(&self) -> u64 {
        self.time_after_steps(self.count)
    }

    fn time_after_steps(&self, steps: u16) -> u64 {
        if steps == 0 {
            return 0;
        }
        ((self.add as u32) * (steps as u32 - 1) * (steps as u32) / 2) as u64
            + (steps as u64) * (self.interval as u64)
    }

    fn steps_before_time(&self, target: u64) -> u16 {
        let mut l = 0;
        let mut r = self.count;
        while l <= r {
            let m = (r - l) / 2 + l;
            let v = self.time_after_steps(m);
            if v == target {
                return m;
            } else if v >= target {
                r = m - 1;
            } else {
                l = m + 1;
            }
        }
        l - 1
    }

    fn advance(&self, steps: u16) -> Move {
        let steps = steps.clamp(0, self.count);
        Move {
            interval: self
                .interval
                .wrapping_add(((self.add as i32) * (steps as i32)) as u32),
            count: self.count - steps,
            add: self.add,
        }
    }
}

impl State {
    /// Advances the state by the given move, up to maximum time u32
    fn advance(&mut self, cmd: &Move, max_time: u64) -> Option<Move> {
        let available_time = max_time.wrapping_sub(self.last_step);
        if available_time & (1 << 63) != 0 {
            // max_time is in the past, return the entire move
            return Some(*cmd);
        }

        let total_time = cmd.total_time();
        if total_time < available_time {
            // Apply the full move
            self.last_step = self.last_step + total_time;
            self.step(cmd.count as u32);
            return None;
        }

        let steps_before = cmd.steps_before_time(available_time);
        if steps_before == 0 {
            return Some(*cmd); // Fast path: nothing can be applied
        }
        // Slow path: apply the time and number of steps before `steps_before` and return the
        // remaining move.
        self.last_step += cmd.time_after_steps(steps_before);
        self.step(steps_before as u32);
        Some(cmd.advance(steps_before))
    }

    fn step(&mut self, count: u32) {
        self.position = if self.direction == Direction::Forward {
            self.position.wrapping_add(count)
        } else {
            self.position.wrapping_sub(count)
        }
    }
}

#[derive(Debug, defmt::Format)]
struct EmulatedStepper {}

#[klipper_command]
pub fn config_stepper(
    _context: &mut CommandContext,
    _oid: u8,
    _step_pin: u8,
    _dir_pin: u8,
    _invert_step: u8,
    _step_pulse_ticks: u32,
) {
    // TODO:
}

#[klipper_command]
pub fn queue_step(_context: &mut CommandContext, _oid: u8, _interval: u32, _count: u16, _add: i16) {
    // TODO:
}

#[klipper_command]
pub fn set_next_step_dir(_context: &mut CommandContext, _oid: u8, _dir: u8) {
    // TODO:
}

#[klipper_command]
pub fn reset_step_clock(_context: &mut CommandContext, _oid: u8, _clock: u8) {
    // TODO:
}

#[klipper_command]
pub fn stepper_get_position(_context: &mut CommandContext, _oid: u8) {
    // TODO:
}

#[klipper_command]
pub fn stepper_stop_on_trigger(_context: &mut CommandContext, _oid: u8, _trsync_oid: u8) {
    // TODO:
}

#[klipper_command]
pub fn config_digital_out(
    _context: &mut CommandContext,
    _oid: u8,
    _pin: u8,
    _value: u8,
    _default_value: u8,
    _max_duration: u32,
) {
    // TODO:
}

#[klipper_command]
pub fn queue_digital_out(_context: &mut CommandContext, _oid: u8, _clock: u8, _on_ticks: u32) {
    // TODO:
}

#[klipper_command]
pub fn update_digital_out(_context: &mut CommandContext, _oid: u8, _value: u8) {
    // TODO:
}
