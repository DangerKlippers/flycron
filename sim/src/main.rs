use dimensioned::{f64prefixes::*, si::*};
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

#[derive(Debug)]
struct Simulator {
    ticks_per_second: Hertz<f64>,
    controller_ticks_per_second: Hertz<f64>,

    setpoint: Meter<f64>,
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

    pid_p: f32,
    pid_i: f32,
    pid_d: f32,
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
        t.setpoint = self.setpoint;
        if self.next_controller_update <= self.time {
            t.controlled = true;
            let output = self.controller.update(
                (self.setpoint * self.system.encoder_scale).round() as i32,
                self.system.current_position(),
            );
            self.system.set_input_throttle(output.output as f64);
            self.next_controller_update += 1.0 / self.controller_ticks_per_second;
            self.controller_updates += 1;

            t.pid_p = output.telemetry.p;
            t.pid_i = output.telemetry.i;
            t.pid_d = output.telemetry.d;
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

        setpoint: 50.0 * MILLI * M,
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
            p: 0.00008,
            p_max: 1.0,
            i: 0.00000002,
            i_max: 1.0,
            d: 0.02,
            d_max: 1.0,
        },
    );

    println!("{sim:#?}");

    let mut writer = csv::Writer::from_path("/tmp/sim.csv").expect("CSV writer creation failed");

    sim.run(10.00 * S, |t| {
        writer.serialize(&t).expect("Could not write row");
    });

    sim.system.mass = 110.0 * MILLI * KG;
    sim.run(10.00 * S, |t| {
        writer.serialize(&t).expect("Could not write row");
    });

    println!("{sim:#?}");
}

impl Serialize for Telemetry {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use dimensioned::Dimensioned;
        use serde::ser::SerializeStruct;
        let mut s = serializer.serialize_struct("Telemetry", 8)?;
        s.serialize_field("time", &self.time.value_unsafe())?;
        s.serialize_field("setpoint", &(self.setpoint / MILLI).value_unsafe())?;
        s.serialize_field("position", &(self.position / MILLI).value_unsafe())?;
        s.serialize_field("velocity", &(self.velocity / MILLI).value_unsafe())?;
        s.serialize_field("accel", &(self.accel).value_unsafe())?;
        s.serialize_field("throttle", &self.throttle)?;
        s.serialize_field("pid_p", &self.pid_p)?;
        s.serialize_field("pid_i", &self.pid_i)?;
        s.serialize_field("pid_d", &self.pid_d)?;
        s.end()
    }
}
