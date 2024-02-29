#![no_std]

use micromath::F32Ext;
use pid::Pid;

pub struct Controller {
    pid0: Pid<f32>,
}

impl core::default::Default for Controller {
    fn default() -> Self {
        Self {
            pid0: Pid::new(0.0f32, 1.0),
        }
    }
}

#[derive(Default)]
pub struct PidGains {
    pub p: f32,
    pub p_max: f32,
    pub i: f32,
    pub i_max: f32,
    pub d: f32,
    pub d_max: f32,
}

impl Controller {
    pub fn update_gains(&mut self, loop_idx: usize, gains: &PidGains) {
        let pid = match loop_idx {
            0 => &mut self.pid0,
            _ => return,
        };
        pid.p(gains.p, gains.p_max)
            .i(gains.i, gains.i_max)
            .d(gains.d, gains.d_max);
    }

    pub fn update(&mut self, target_position: i32, current_position: i32) -> f32 {
        self.pid0.setpoint(target_position as f32);
        let desired_thrust = self
            .pid0
            .next_control_output(current_position as f32)
            .output;
        self.thrust_to_throttle(desired_thrust, 100.0)
    }

    pub fn thrust_to_throttle(&self, target_thrust: f32, z_height: f32) -> f32 {
        self.thrust_throttle_raw(target_thrust * self.ground_effect_scale(z_height))
    }

    pub fn ground_effect_scale(&self, _z_height: f32) -> f32 {
        1.0
    }

    pub fn thrust_throttle_raw(&self, target_thrust: f32) -> f32 {
        let a = 427.0;
        let b = -22.1;
        let c = 3.56 - target_thrust;
        let discriminant: f32 = b * b - 4.0 * a * c;
        if discriminant > 0.0 {
            let sqrt_discriminant = discriminant.sqrt();
            let x1 = (-b + sqrt_discriminant) / (2.0 * a);
            let x2 = (-b - sqrt_discriminant) / (2.0 * a);
            x1.max(x2) // take the positive one
        } else if discriminant == 0.0 {
            -b / (2.0 * a)
        } else {
            0.0
        }
    }
}