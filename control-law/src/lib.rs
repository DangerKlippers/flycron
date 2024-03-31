#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
use micromath::F32Ext;
use pid::Pid;

#[derive(Debug)]
pub struct Controller {
    pid_pos: Pid<f32>,
    pid_vel: Pid<f32>,
}

impl core::default::Default for Controller {
    fn default() -> Self {
        Self {
            pid_pos: Pid::new(0.0f32, 1.0),
            pid_vel: Pid::new(0.0f32, 1.0),
        }
    }
}

#[derive(Default, Debug)]
pub struct PidGains {
    pub p: f32,
    pub p_max: f32,
    pub i: f32,
    pub i_max: f32,
    pub d: f32,
    pub d_max: f32,
}

#[derive(Debug)]
pub struct Output {
    pub output: f32,
    #[cfg(feature = "telemetry")]
    pub telemetry: ControllerTelemetry,
}

#[cfg(feature = "telemetry")]
#[derive(Debug, Default)]
pub struct ControllerTelemetry {
    pub pos_p: f32,
    pub pos_i: f32,
    pub pos_d: f32,
    pub vel_p: f32,
    pub vel_i: f32,
    pub vel_d: f32,
}

impl Controller {
    pub fn update_gains(&mut self, loop_idx: usize, gains: &PidGains) {
        let pid = match loop_idx {
            0 => &mut self.pid_pos,
            1 => &mut self.pid_vel,
            _ => return,
        };
        pid.p(gains.p, gains.p_max)
            .i(gains.i, gains.i_max)
            .d(gains.d, gains.d_max);
    }

    pub fn update(
        &mut self,
        target_position: i32,
        target_velocity: f32,
        target_acceleration: f32,
        current_position: i32,
    ) -> Output {
        self.pid_vel.p(0.2, 1.0);
        self.pid_pos.setpoint(target_position as f32);
        let pid_pos_out = self.pid_pos.next_control_output(current_position as f32);

        self.pid_vel.setpoint(pid_pos_out.output + target_velocity);
        // TODO: Observe velocity
        let pid_vel_out = self.pid_vel.next_control_output(0.0);

        // self.pid_vel.setpoint(target_velocity);
        // self.pid0.setpoint(target_position as f32);
        // let pid_output = self.pid0.next_control_output(current_position as f32);

        let thrust_target = pid_vel_out.output + target_acceleration * 0.0;

        let output = self.thrust_to_throttle(thrust_target, 100.0);

        #[cfg(feature = "telemetry")]
        let telemetry = ControllerTelemetry {
            pos_p: pid_pos_out.p,
            pos_i: pid_pos_out.i,
            pos_d: pid_pos_out.d,
            vel_p: pid_vel_out.p,
            vel_i: pid_vel_out.i,
            vel_d: pid_vel_out.d,
        };

        Output {
            output,
            #[cfg(feature = "telemetry")]
            telemetry,
        }
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
        let c = 3.56 - target_thrust * 400.0;
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
