#![cfg_attr(not(test), no_std)]

use heapless::Deque;

pub type Instant = fugit::Instant<u64, 1, 96_000_000>;
pub type Duration = fugit::Duration<u64, 1, 96_000_000>;

#[derive(Debug, Copy, Clone, Eq, PartialEq, PartialOrd, Ord)]
pub enum Direction {
    Forward,
    Backward,
}

#[derive(Debug)]
struct State {
    last_step: Instant,
    position: u32,
}

#[derive(Debug, Copy, Clone)]
#[repr(C)]
struct Move {
    interval: u32,
    count: u16,
    add: i16,
    direction: Direction,
}

impl Move {
    fn total_time(&self) -> Duration {
        self.time_after_steps(self.count)
    }

    fn time_after_steps(&self, steps: u16) -> Duration {
        if steps == 0 {
            return Duration::from_ticks(0);
        }
        let base = (steps as u64) * (self.interval as u64);
        let accel = (self.add as i32) * (steps as i32 - 1) * (steps as i32) / 2;
        Duration::from_ticks(base.wrapping_add(accel as u64))
    }

    fn steps_before_time(&self, target: Duration) -> u16 {
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
            direction: self.direction,
        }
    }
}

impl State {
    /// Advances the state by the given move, up to maximum time u32
    fn advance(&mut self, cmd: &Move, up_to_time: Instant) -> AdvanceResult {
        let available_time = match up_to_time.checked_duration_since(self.last_step) {
            Some(t) => t,
            None => return AdvanceResult::FutureMove,
        };

        let total_time = cmd.total_time();
        if total_time < available_time {
            // Apply the full move
            self.last_step += total_time;
            self.step(cmd.direction, cmd.count as u32);
            return AdvanceResult::Consumed;
        }

        let steps_before = cmd.steps_before_time(available_time);
        if steps_before == 0 {
            return AdvanceResult::Partial(*cmd); // Fast path: nothing can be applied
        }
        // Slow path: apply the time and number of steps before `steps_before` and return the
        // remaining move.
        self.last_step += cmd.time_after_steps(steps_before);
        self.step(cmd.direction, steps_before as u32);
        AdvanceResult::Partial(cmd.advance(steps_before))
    }

    fn step(&mut self, direction: Direction, count: u32) {
        self.position = if direction == Direction::Forward {
            self.position.wrapping_add(count)
        } else {
            self.position.wrapping_sub(count)
        }
    }
}

#[derive(Debug, Copy, Clone)]
enum AdvanceResult {
    FutureMove,
    Consumed,
    Partial(Move),
}

pub trait PidTimeIterator {
    fn next(&mut self) -> Instant;
    fn advance(&mut self) -> Instant;
}

pub trait Callbacks {
    fn append(&mut self, time: Instant, value: u32);
    fn update_last(&mut self, time: Instant, value: u32);
    fn can_append(&self) -> bool;
}

#[derive(Debug)]
pub struct EmulatedStepper<T> {
    queue: heapless::Deque<Move, 32>,
    current_move: Option<Move>,
    next_direction: Direction,
    state: State,
    target_time: T,
    callback_state: CallbackState,
}

#[derive(Debug)]
struct CallbackState {
    last_append: (Instant, u32),
    incomplete: bool,
}

impl CallbackState {
    fn append(&mut self, next_time: Instant, position: u32, callbacks: &mut impl Callbacks) {
        callbacks.append(next_time, position);
        self.last_append = (next_time, position);
    }

    fn update(&mut self, position: u32, callbacks: &mut impl Callbacks) {
        let pos = position;
        // if pos != self.last_append.1 {
        callbacks.update_last(self.last_append.0, pos);
        // }
    }

    fn emit(&mut self, next_time: Instant, position: u32, callbacks: &mut impl Callbacks) {
        if self.incomplete {
            self.update(position, callbacks);
        } else {
            self.append(next_time, position, callbacks);
        }
    }

    fn can_append(&self, callbacks: &impl Callbacks) -> bool {
        callbacks.can_append()
    }
}

impl<T: PidTimeIterator> EmulatedStepper<T> {
    pub fn new(target_time: T) -> Self {
        Self {
            queue: Deque::new(),
            current_move: None,
            next_direction: Direction::Forward,
            state: State {
                last_step: Instant::from_ticks(0),
                position: 0,
            },
            target_time,
            callback_state: CallbackState {
                last_append: (Instant::from_ticks(0), 0),
                incomplete: true,
            },
        }
    }

    pub fn reset_clock(&mut self, time: Instant) {
        self.state.last_step = time;
    }

    pub fn advance(&mut self, callbacks: &mut impl Callbacks) {
        while self.callback_state.can_append(callbacks) {
            let cmd = match self.current_move.as_mut() {
                None => match self.queue.pop_front() {
                    None => return, // Nothing to do
                    Some(m) => {
                        self.current_move = Some(m);
                        self.current_move.as_mut().unwrap()
                    }
                },
                Some(m) => m,
            };

            // Get next PID tick
            let mut next_time = self.target_time.next();
            while cmd.count != 0 && self.callback_state.can_append(callbacks) {
                // Apply current command up to the next tick
                match self.state.advance(&cmd, next_time) {
                    // Command was fully consumed, last_step was left <= next_time
                    AdvanceResult::Consumed => {
                        self.callback_state
                            .emit(next_time, self.state.position, callbacks);
                        self.callback_state.incomplete = true;
                        cmd.count = 0;
                        break;
                    }
                    AdvanceResult::Partial(new_cmd) => {
                        // Force advance to next PID tick
                        self.callback_state
                            .emit(next_time, self.state.position, callbacks);
                        self.callback_state.incomplete = false;
                        next_time = self.target_time.advance();
                        *cmd = new_cmd;
                    }
                    AdvanceResult::FutureMove => {
                        next_time = self.target_time.advance();
                        self.callback_state.incomplete = false;
                    }
                };
            }
            if cmd.count == 0 {
                self.current_move = None
            }
        }
    }

    pub fn queue_move(&mut self, interval: u32, count: u16, add: i16) -> bool {
        let cmd = Move {
            interval,
            count,
            add,
            direction: self.next_direction,
        };
        if self.queue.push_back(cmd).is_err() {
            return false;
        }
        true
    }

    pub fn set_next_dir(&mut self, dir: Direction) {
        self.next_direction = dir;
    }

    pub fn last_step(&self) -> Instant {
        self.state.last_step
    }

    pub fn has_moves(&self) -> bool {
        self.current_move.is_some() || !self.queue.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestPidTimeIterator(Instant, Duration, Instant);

    impl TestPidTimeIterator {
        fn set_time(&mut self, t: Instant) {
            self.2 = t;
        }
    }

    impl PidTimeIterator for TestPidTimeIterator {
        fn advance(&mut self) -> Instant {
            self.0 += self.1;
            self.0
        }

        fn next(&mut self) -> Instant {
            loop {
                if self.0 >= self.2 {
                    return self.0;
                } else {
                    self.advance();
                }
            }
        }
    }

    pub fn count_ticks(interval: u32, count: u16, add: i16) -> u64 {
        let mut total = 0u64;
        let mut interval = interval;
        for _ in 0..count {
            total += interval as u64;
            interval = interval.wrapping_add(add as i32 as u32);
        }
        total
    }

    #[test]
    fn total_ticks() {
        let cmd_up = Move {
            interval: 1000,
            count: 100,
            add: 1,
            direction: Direction::Forward,
        };
        assert_eq!(
            cmd_up.total_time().ticks(),
            count_ticks(cmd_up.interval, cmd_up.count, cmd_up.add)
        );
        let cmd_down = Move {
            interval: cmd_up.advance(cmd_up.count).interval,
            count: cmd_up.count,
            add: -cmd_up.add,
            direction: Direction::Forward,
        };
        assert_eq!(
            cmd_down.total_time().ticks(),
            count_ticks(cmd_down.interval, cmd_down.count, cmd_down.add)
        );
    }

    #[test]
    fn test_it() {
        struct Cbs {
            buf: Vec<(Instant, u32)>,
        }

        impl Callbacks for Cbs {
            fn update_last(&mut self, time: Instant, value: u32) {
                if let Some((t, v)) = self.buf.last_mut() {
                    if time != *t {
                        panic!("Wrong time");
                    }
                    *v = value;
                }
            }

            fn append(&mut self, time: Instant, value: u32) {
                self.buf.push((time, value));
            }

            fn can_append(&self) -> bool {
                self.buf.len() < 100
            }
        }

        let mut cbs = Cbs { buf: vec![] };

        let mut s = EmulatedStepper::new(TestPidTimeIterator(
            Instant::from_ticks(0),
            Duration::from_ticks(12000),
            Instant::from_ticks(0),
        ));

        s.target_time.set_time(Instant::from_ticks(6000));
        s.target_time.set_time(Instant::from_ticks(0));

        let first_step = Instant::from_ticks(6000);
        s.reset_clock(first_step);
        s.queue_move(1066, 10000, 0);
        s.queue_move(1066, 100, 10);
        s.set_next_dir(Direction::Backward);
        s.queue_move(1066 + 11066, 100, -10);
        while s.has_moves() {
            s.advance(&mut cbs);
            cbs.buf.clear();
        }
        assert_eq!(s.state.position, 10000);
        assert_eq!(
            (s.state.last_step - first_step).ticks(),
            count_ticks(1066, 10000, 0)
                + count_ticks(1066, 100, 10)
                + count_ticks(1066 + 11066, 100, -10)
        );
    }

    #[test]
    fn bench_test() {
        #[derive(Debug)]
        struct Cbs {
            buf: Vec<(Instant, u32)>,
        }

        impl Callbacks for Cbs {
            fn update_last(&mut self, time: Instant, value: u32) {
                if let Some((t, v)) = self.buf.last_mut() {
                    if time != *t {
                        panic!("Wrong time");
                    }
                    *v = value;
                }
            }

            fn append(&mut self, time: Instant, value: u32) {
                self.buf.push((time, value));
            }

            fn can_append(&self) -> bool {
                true
            }
        }

        let mut s = EmulatedStepper::new(TestPidTimeIterator(
            Instant::from_ticks(0),
            Duration::from_ticks(12000),
            Instant::from_ticks(0),
        ));

        s.target_time.set_time(Instant::from_ticks(0));

        let first_step = Instant::from_ticks(100000);
        s.reset_clock(first_step);
        s.queue_move(100, 60000, 0);
        let mut cbs = Cbs { buf: vec![] };
        s.advance(&mut cbs);
        println!("{cbs:?}");
        assert!(!s.has_moves());
        assert_eq!(s.state.position, 60000);
        assert_eq!(
            (s.state.last_step - first_step).ticks(),
            count_ticks(100, 60000, 0)
        );
    }
}
