use crate::{emulated_stepper::Callbacks, Instant};
use core::cell::RefCell;
use heapless::Deque;

pub struct TargetQueueInner<const N: usize> {
    queue: Deque<(Instant, u32), N>,
    last_value: u32,
}

pub type TargetQueueInnerTypeCell<const N: usize> = RefCell<TargetQueueInner<N>>;

pub struct ControlOutput {
    pub position: i32,
    pub position_1: Option<(u32, i32)>,
    pub position_2: Option<(u32, i32)>,
}

impl ControlOutput {
    fn single(position: i32) -> Self {
        Self {
            position,
            position_1: None,
            position_2: None,
        }
    }
}

pub trait Mutex {
    type Inner<T>;
    fn new<T>(val: T) -> Self::Inner<T>;
    fn lock<T, R>(inner: &Self::Inner<T>, f: impl FnOnce(&T) -> R) -> R;
}

pub struct TargetQueue<M: Mutex, const N: usize> {
    inner: M::Inner<TargetQueueInnerTypeCell<N>>,
}

impl<M: Mutex, const N: usize> Default for TargetQueue<M, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<M: Mutex, const N: usize> TargetQueue<M, N> {
    pub fn new() -> Self {
        Self {
            inner: M::new(RefCell::new(TargetQueueInner {
                queue: Deque::new(),
                last_value: 0,
            })),
        }
    }

    fn can_append(&self) -> bool {
        !M::lock(&self.inner, |q| q.borrow().queue.is_full())
    }

    fn append(&self, time: Instant, value: u32) {
        M::lock(&self.inner, |q| {
            let mut q = q.borrow_mut();

            q.queue.push_back((time, value)).unwrap();
            q.last_value = value;
        });
    }

    fn update_last(&self, _time: Instant, value: u32) {
        M::lock(&self.inner, |q| {
            let mut q = q.borrow_mut();
            if let Some((_, v)) = q.queue.back_mut() {
                *v = value;
            }
            q.last_value = value;
        });
    }

    pub fn get_for_control(&self, time: u32) -> ControlOutput {
        let time = Instant::from_ticks(time);
        M::lock(&self.inner, |q| {
            let mut inner = q.borrow_mut();
            let last = inner.last_value as i32;
            let q = &mut inner.queue;

            // Remove from front such that the next item will be read now

            while let Some((t, _)) = q.front() {
                if *t >= time {
                    break;
                }
                q.pop_front();
            }
            if q.is_empty() {
                return ControlOutput::single(last);
            }
            let mut iter = q.iter();
            let v0 = iter.next().copied();
            let v0 = match v0 {
                Some((t0, v0)) if t0 == time => v0,
                _ => return ControlOutput::single(last),
            };
            let v1 = iter.next().copied();
            let v2 = iter.next().copied();
            ControlOutput {
                position: v0 as i32,
                position_1: v1.map(|(t, v)| (t.ticks(), v as i32)),
                position_2: v2.map(|(t, v)| (t.ticks(), v as i32)),
            }
        })
    }
}

impl<M: Mutex, const N: usize> Callbacks for &TargetQueue<M, N> {
    fn append(&mut self, time: Instant, value: u32) {
        TargetQueue::append(self, time, value)
    }

    fn update_last(&mut self, time: Instant, value: u32) {
        TargetQueue::update_last(self, time, value)
    }

    fn can_append(&self) -> bool {
        TargetQueue::can_append(self)
    }
}
