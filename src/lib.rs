#![no_std]
extern crate libc;

extern "C" {
  fn spark_max_new(device_id: c_int, motor_type: c_int) -> *mut c_void;
  fn spark_max_set(ctx: *mut c_void, speed: c_double);
  fn spark_max_set_voltage(ctx: *mut c_void, voltage: c_int);
  fn spark_max_get(ctx: *mut c_void) -> c_double;
  fn spark_max_set_inverted(ctx: *mut c_void, inverted: bool);
  fn spark_max_is_inverted(ctx: *mut c_void) -> c_bool;
  fn spark_max_disable(ctx: *mut c_void);
  fn spark_max_stop_motor(ctx: *mut c_void);
  fn spark_max_get_encoder(
    ctx: *mut c_void,
    encoder_type: c_int,
    counts_per_rev: c_int,
  ) -> *mut c_void;
  fn spark_max_get_alternate_encoder(
    ctx: *mut c_void,
    encoder_type: c_int,
    counts_per_rev: c_int,
  ) -> *mut c_void;
  fn spark_max_get_absolute_encoder(ctx: *mut c_void, encoder_type: c_int)
    -> *mut c_void;
  fn spark_max_get_analog(ctx: *mut c_void, mode: c_int) -> *mut c_void;
  fn spark_max_get_pid_controller(ctx: *mut c_void) -> *mut c_void;
}

pub struct SparkMax<MT = MotorType> {
  ctx: *mut c_void,
}
impl<MT> SparkMax<MT> {
  pub fn new(device_id: u8) -> Self {
    Self {
      ctx: unsafe { spark_max_new(device_id, MT.into()) },
    }
  }

  pub fn set_speed(&self, speed: f32) {
    unsafe {
      spark_max_set(self.ctx, speed.into());
    }
  }
  pub fn set_voltage(&self, voltage: i32) {
    unsafe {
      spark_max_set_voltage(self.ctx, voltage.into());
    }
  }

  pub fn set_inverted(&self, inverted: bool) {
    unsafe {
      spark_max_set_inverted(self.ctx, inverted);
    }
  }
  pub fn is_inverted(&self) -> bool { unsafe { spark_max_is_inverted(self.ctx) } }

  pub fn disable(&self) {
    unsafe {
      spark_max_disable(self.ctx);
    }
  }
  pub fn stop_motor(&self) {
    unsafe {
      spark_max_stop_motor(self.ctx);
    }
  }
}

pub enum MotorType {
  Brushed = 0,
  Brushless = 1,
}
