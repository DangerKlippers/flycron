use std::collections::VecDeque;

use control_law::ControllerTelemetry;
use dimensioned::{f64prefixes::*, si::*, Dimensioned};
use lib_klipper::{
    gcode::{parse_gcode, GCodeCommand, GCodeOperation},
    glam::DVec4,
    planner::{Planner, PlanningMove, PlanningOperation, PrinterLimits},
};
use serde::Serialize;

#[derive(Debug)]
struct Plant {
    // Model internal parameters
    range: (Meter<f64>, Meter<f64>),
    encoder_scale: PerMeter<f64>,        // counts/mm
    mass: Kilogram<f64>,                 // moving mass
    gravity_accel: MeterPerSecond2<f64>, // Acceleration by gravity

    // Inputs
    throttle: f64,

    // Internal state
    position: Meter<f64>,
    velocity: MeterPerSecond<f64>,
}

impl Plant {
    fn tick(&mut self, dt: Second<f64>, t: &mut Telemetry) {
        let accel = self.throttle_accel() - self.gravity_accel;
        self.velocity += accel * dt;
        // Collision detection
        if (self.position <= self.range.0 && self.velocity < 0.0 * MPS)
            || (self.position >= self.range.1 && self.velocity > 0.0 * MPS)
        {
            self.velocity = 0.0 * MPS;
        }
        self.position += self.velocity * dt;
        if self.position >= self.range.1 {
            self.position = self.range.1;
        } else if self.position <= self.range.0 {
            self.position = self.range.0;
        }

        t.position = self.position;
        t.velocity = self.velocity;
        t.accel = accel;
        t.throttle = self.throttle;
    }

    fn throttle_accel(&self) -> MeterPerSecond2<f64> {
        let a = 427.0;
        let b = -22.1;
        let c = 3.56;

        let x = self.throttle;
        let grams_force = a * x * x + b * x + c;
        grams_force * MILLI * KG * self.gravity_accel / self.mass
    }

    fn current_position(&self) -> i32 {
        (self.position * self.encoder_scale).round() as i32
    }

    fn set_input_throttle(&mut self, throttle: f64) {
        self.throttle = throttle.clamp(0.0, 1.0);
    }
}

impl Default for Plant {
    fn default() -> Self {
        Plant {
            range: (0.0 * M, 100.0 * MILLI * M),
            encoder_scale: 600.0 * KILO * PM,
            mass: 100.0 * MILLI * KG,
            gravity_accel: 9.80665 * MPS2,

            throttle: 0.0,

            position: 0.0 * M,
            velocity: 0.0 * MPS,
        }
    }
}

// #[derive(Debug)]
// pub struct Move {
//     start_time: Second<f64>,
//     start: Meter<f64>,
//     end: Meter<f64>,
//     velocity: MeterPerSecond<f64>,
//     accel: MeterPerSecond2<f64>,
// }
//
// impl Move {
//     fn new(
//         start_time: Second<f64>,
//         start: Meter<f64>,
//         end: Meter<f64>,
//         velocity: MeterPerSecond<f64>,
//         accel: MeterPerSecond2<f64>,
//     ) -> Self {
//         Self {
//             start_time,
//             start,
//             end,
//             velocity,
//             accel,
//         }
//     }
// }

#[derive(Debug)]
pub struct ZMoveQueue {
    location: Meter<f64>,
    velocity: MeterPerSecond<f64>,
    acceleration: MeterPerSecond2<f64>,
    queue: VecDeque<(Second<f64>, PlanningMove)>,
    planner: Planner,
    cur_time: Second<f64>,
    next_time: Second<f64>,
}

impl ZMoveQueue {
    pub fn new(location: Meter<f64>) -> Self {
        let limits = PrinterLimits {
            ..Default::default()
        };
        Self {
            location,
            velocity: 0.0 * MPS,
            acceleration: 0.0 * MPS2,
            queue: VecDeque::new(),
            planner: Planner::from_limits(limits),
            cur_time: 0.0 * S,
            next_time: 0.0 * S,
        }
    }

    pub fn add_move(
        &mut self,
        delay: Option<Second<f64>>,
        target: Meter<f64>,
        velocity: MeterPerSecond<f64>,
        accel: MeterPerSecond2<f64>,
    ) {
        let accel = accel / MILLI;
        self.planner.process_cmd(
            &parse_gcode(&format!(
                "set_velocity_limit accel={accel} accel_to_decel={accel}"
            ))
            .unwrap(),
        );
        if let Some(delay) = delay {
            let delay = delay / MILLI;
            self.planner
                .process_cmd(&parse_gcode(&format!("G4 P{delay}")).unwrap());
        }
        self.planner.process_cmd(&GCodeCommand {
            op: GCodeOperation::Move {
                x: None,
                y: None,
                z: Some(*(target / MILLI).value_unsafe()),
                e: None,
                f: Some(*(velocity / MILLI * MIN).value_unsafe()),
            },
            comment: None,
        });
    }

    fn flush(&mut self) {
        self.planner.finalize();
        for o in self.planner.iter() {
            match o {
                PlanningOperation::Fill => {}
                PlanningOperation::Delay(d) => {
                    if self.next_time < self.cur_time {
                        self.next_time = self.cur_time;
                    }
                    self.next_time += d.duration().as_secs_f64() * S;
                }
                PlanningOperation::Move(m) => {
                    self.queue.push_back((self.next_time, m));
                    self.next_time += m.total_time() * S;
                }
            }
        }
    }

    fn retire(&mut self) {
        loop {
            match self.queue.front() {
                None => return,
                Some((s, m)) if *s + m.total_time() * S > self.cur_time => return,
                Some(_) => {
                    self.queue.pop_front();
                }
            }
        }
    }

    pub fn eval_at(&self, time: Second<f64>) -> Meter<f64> {
        for (s, m) in &self.queue {
            let t = time - *s;
            if t < 0.0 * S {
                return self.location;
            }
            match eval_move(m, t) {
                None => {}
                Some(p) => return p.z * MILLI * M,
            }
        }
        self.location
    }

    pub fn execute(&mut self, lookahead: Second<f64>) {
        self.retire();
        self.location = self.eval_at(self.cur_time);
        let look1 = self.eval_at(self.cur_time + lookahead);
        let look2 = self.eval_at(self.cur_time + 2.0 * lookahead);
        let v2 = (look2 - look1) / lookahead;
        self.velocity = (look1 - self.location) / lookahead;
        self.acceleration = (v2 - self.velocity) / lookahead;
    }

    pub fn update(&mut self, target_time: Second<f64>, lookahead: Second<f64>) -> Meter<f64> {
        self.cur_time = target_time;
        self.flush();
        self.execute(lookahead);
        self.location
    }
}

fn eval_move(mov: &PlanningMove, t: Second<f64>) -> Option<DVec4> {
    let t = t.value_unsafe;

    let atime = mov.accel_time();
    let ctime = mov.cruise_time();
    let dtime = mov.decel_time();

    if t < 0.0 {
        return None;
    };
    let ta = t.min(atime);
    let adist = mov.start_v * ta + 0.5 * mov.acceleration * ta * ta;
    if t <= atime {
        return Some(mov.start + mov.rate * adist);
    }
    let tc = (t - ta).min(ctime);
    let cdist = mov.cruise_v * tc;
    if t <= atime + ctime {
        return Some(mov.start + mov.rate * (adist + cdist));
    }
    let td = (t - ta - tc).min(dtime);
    if t <= atime + ctime + dtime {
        let ddist = mov.cruise_v * ta - 0.5 * mov.acceleration * td * td;
        return Some(mov.start + mov.rate * (adist + cdist + ddist));
    }
    None
}

#[derive(Debug)]
struct Simulator {
    ticks_per_second: Hertz<f64>,
    controller_ticks_per_second: Hertz<f64>,

    move_queue: ZMoveQueue,
    controller: control_law::Controller,
    system: Plant,
    time: Second<f64>,
    next_controller_update: Second<f64>,

    ticks: usize,
    controller_updates: usize,
}

#[derive(Debug, Default)]
struct Telemetry {
    controlled: bool,

    time: Second<f64>,
    setpoint: Meter<f64>,
    position: Meter<f64>,
    velocity: MeterPerSecond<f64>,
    accel: MeterPerSecond2<f64>,
    throttle: f64,

    controller: ControllerTelemetry,
}

impl Telemetry {
    fn new(time: Second<f64>) -> Self {
        Self {
            time,
            ..Default::default()
        }
    }
}

impl Simulator {
    fn run(&mut self, t: Second<f64>, mut cb: impl FnMut(&Telemetry)) {
        let end = self.time + t;
        let dt = 1.0 / self.ticks_per_second;
        while self.time < end {
            let t = self.run_round(dt);
            if t.controlled {
                cb(&t);
            }
        }
    }

    fn run_round(&mut self, dt: Second<f64>) -> Telemetry {
        let mut t = Telemetry::new(self.time);
        self.move_queue
            .update(self.time, 1.0 / self.controller_ticks_per_second);
        t.setpoint = self.move_queue.location;
        if self.next_controller_update <= self.time {
            t.controlled = true;
            let output = self.controller.update(
                (self.move_queue.location * self.system.encoder_scale).round() as i32,
                self.move_queue.velocity.value_unsafe as f32,
                self.move_queue.acceleration.value_unsafe as f32,
                self.system.current_position(),
            );
            self.system.set_input_throttle(output.output as f64);
            self.next_controller_update += 1.0 / self.controller_ticks_per_second;
            self.controller_updates += 1;

            t.controller = output.telemetry;
        }

        self.system.tick(dt, &mut t);

        self.time += dt;
        self.ticks += 1;

        t
    }
}

fn main() {
    let mut sim = Simulator {
        ticks_per_second: 128000.0 * HZ,
        controller_ticks_per_second: 8000.0 * HZ,

        move_queue: ZMoveQueue::new(0.0 * MILLI * M),
        controller: Default::default(),
        system: Default::default(),
        time: 0.0 * S,
        next_controller_update: 0.0 * S,

        ticks: 0,
        controller_updates: 0,
    };

    sim.controller.update_gains(
        0,
        &control_law::PidGains {
            p: 0.3,
            p_max: 1.0,
            i: 0.0,
            i_max: 1.0,
            d: 0.0,
            d_max: 1.0,
        },
    );
    sim.controller.update_gains(
        1,
        &control_law::PidGains {
            p: 0.4,
            p_max: 1.0,
            i: 0.0,
            i_max: 1.0,
            d: 0.0,
            d_max: 1.0,
        },
    );

    let mut writer = csv::Writer::from_path("/tmp/sim.csv").expect("CSV writer creation failed");

    sim.move_queue.add_move(
        Some(2.0 * S),
        60.0 * MILLI * M,
        30.0 * MILLI * MPS,
        2000.0 * MILLI * MPS2,
    );
    sim.move_queue.add_move(
        Some(2.0 * S),
        65.0 * MILLI * M,
        30.0 * MILLI * MPS,
        2000.0 * MILLI * MPS2,
    );

    sim.move_queue.add_move(
        Some(2.0 * S),
        65.2 * MILLI * M,
        30.0 * MILLI * MPS,
        2000.0 * MILLI * MPS2,
    );

    sim.move_queue.flush();
    // println!("{sim:#?}");

    sim.run(10.00 * S, |t| {
        writer.serialize(t).expect("Could not write row");
    });

    sim.system.mass = 110.0 * MILLI * KG;
    sim.run(10.00 * S, |t| {
        writer.serialize(t).expect("Could not write row");
    });

    // println!("{sim:#?}");
}

impl Serialize for Telemetry {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut s = serializer.serialize_struct("Telemetry", 11)?;
        s.serialize_field("time", &self.time.value_unsafe())?;
        s.serialize_field("setpoint", &(self.setpoint / MILLI).value_unsafe())?;
        s.serialize_field("position", &(self.position / MILLI).value_unsafe())?;
        s.serialize_field("velocity", &(self.velocity / MILLI).value_unsafe())?;
        s.serialize_field("accel", &(self.accel).value_unsafe())?;
        s.serialize_field("throttle", &self.throttle)?;
        s.serialize_field("pid_pos_p", &self.controller.pos_p)?;
        s.serialize_field("pid_pos_i", &self.controller.pos_i)?;
        s.serialize_field("pid_pos_d", &self.controller.pos_d)?;
        s.serialize_field("pid_vel_p", &self.controller.vel_p)?;
        s.serialize_field("pid_vel_i", &self.controller.vel_i)?;
        s.serialize_field("pid_vel_d", &self.controller.vel_d)?;
        s.end()
    }
}
