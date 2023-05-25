#include "include/CANSparkMax.h"
#include "include/CANSparkMaxLowLevel.h"
#include "include/CANPIDController.h"
#include "include/REVLibError.h"
#include "include/SparkMaxRelativeEncoder.h"
#include "include/SparkMaxAlternateEncoder.h"
#include "include/SparkMaxAbsoluteEncoder.h"
#include "include/SparkMaxAnalogSensor.h"
#include "include/SparkMaxLimitSwitch.h"
#include <cstdint>

/*
 * motor_type:
 *  0 => kBrushed
 *  1 => kBrushless
 */
void *can_spark_max_new(int device_id, int motor_type)
{
  rev::CANSparkMaxLowLevel::MotorType mtype;

  // XXX: Other cases aren't handled
  switch (motor_type) {
    case 0:
      mtype = rev::CANSparkMaxLowLevel::MotorType::kBrushed;
    case 1:
      mtype = rev::CANSparkMaxLowLevel::MotorType::kBrushless;
  }

  return new rev::CANSparkMax(device_id, mtype);
}

void can_spark_max_set(rev::CANSparkMax ctx, double speed) {
  ctx.Set(speed);
}

void can_spark_max_set_voltage(rev::CANSparkMax ctx, int voltage) {
  ctx.SetVoltage(voltage);
}

double can_spark_max_get(rev::CANSparkMax ctx) {
  return ctx.Get();
}

void spark_max_set_inverted(rev::CANSparkMax ctx, bool inverted) {
  ctx.SetInverted(inverted);
}

bool spark_max_is_inverted(rev::CANSparkMax ctx) {
  return ctx.GetInverted();
}

void spark_max_disable(rev::CANSparkMax ctx) {
  ctx.Disable();
}

void spark_max_stop_motor(rev::CANSparkMax ctx) {
  ctx.StopMotor();
}

/*
 * encoder_type:
 *  0 => kNoSensor
 *  1 => kHallSensor
 *  2 => kQuadrature
 */
rev::SparkMaxRelativeEncoder spark_max_get_encoder(rev::CANSparkMax ctx, int encoder_type, int counts_per_rev)
{
  rev::SparkMaxRelativeEncoder::Type _type;

  switch (encoder_type) {
    case 0:
      _type = rev::SparkMaxRelativeEncoder::Type::kNoSensor;
    case 1:
      _type = rev::SparkMaxRelativeEncoder::Type::kHallSensor;
    case 2:
      _type = rev::SparkMaxRelativeEncoder::Type::kQuadrature;
  }

  return ctx.GetEncoder(_type, counts_per_rev);
}

/*
 * encoder_type:
 *  0 => kQuadrature
 */
rev::SparkMaxAlternateEncoder spark_max_get_alternate_encoder(rev::CANSparkMax ctx, int encoder_type, int counts_per_rev)
{
  rev::SparkMaxAlternateEncoder::Type _type;

  switch (encoder_type) {
    case 0:
      _type = rev::SparkMaxAlternateEncoder::Type::kQuadrature;
  }

  return ctx.GetAlternateEncoder(_type, counts_per_rev);
}

/*
 * encoder_type:
 *  0 => kDutyCycle
 */
rev::SparkMaxAbsoluteEncoder spark_max_get_absolute_encoder(rev::CANSparkMax ctx, int encoder_type)
{
  rev::SparkMaxAbsoluteEncoder::Type _type;

  switch (encoder_type) {
    case 0:
      _type = rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle;
  }

  return ctx.GetAbsoluteEncoder(_type);
}

/*
 * mode:
 *  0 => kAbsolute
 *  1 => kRelative
 */
rev::SparkMaxAnalogSensor spark_max_get_analog(rev::CANSparkMax ctx, int mode)
{
  rev::SparkMaxAnalogSensor::Mode _mode;

  switch (mode) {
    case 0:
      _mode = rev::SparkMaxAnalogSensor::Mode::kAbsolute;
    case 1:
      _mode = rev::SparkMaxAnalogSensor::Mode::kRelative;
  }

  return ctx.GetAnalog(_mode);
}

rev::SparkMaxPIDController spark_max_get_pid_controller(rev::CANSparkMax ctx) {
  return ctx.GetPIDController();
}

/*
 * switch_type:
 *  0 => kNormallyOpen
 *  1 => kNormallyClosed
 */
rev::SparkMaxLimitSwitch spark_max_get_forward_limit_switch(rev::CANSparkMax ctx, int switch_type)
{
  rev::SparkMaxLimitSwitch::Type _type;

  switch(switch_type) {
    case 0:
      _type = rev::SparkMaxLimitSwitch::Type::kNormallyOpen;
    case 1:
      _type = rev::SparkMaxLimitSwitch::Type::kNormallyClosed;
  }

  return ctx.GetForwardLimitSwitch(_type);
}

/*
 * switch_type:
 *  0 => kNormallyOpen
 *  1 => kNormallyClosed
 */
rev::SparkMaxLimitSwitch spark_max_get_reverse_limit_switch(rev::CANSparkMax ctx, int switch_type)
{
  rev::SparkMaxLimitSwitch::Type _type;

  switch(switch_type) {
    case 0:
      _type = rev::SparkMaxLimitSwitch::Type::kNormallyOpen;
    case 1:
      _type = rev::SparkMaxLimitSwitch::Type::kNormallyClosed;
  }

  return ctx.GetReverseLimitSwitch(_type);
}

/* XXX: HANDLE THE ERROR */
void spark_max_set_smart_current_limit(
  rev::CANSparkMax ctx, int stall_limit, int free_limit, int limit_rpm
) {
  ctx.SetSmartCurrentLimit(stall_limit, free_limit, limit_rpm);
}

/* XXX: HANDLE THE ERROR */
void spark_max_set_secondary_current_limit(rev::CANSparkMax ctx, double limit, int limit_cycles)
{
  ctx.SetSecondaryCurrentLimit(limit, limit_cycles);
}

/*
 * idle_mode:
 *  0 => kCoast
 *  1 => kBrake
 */
/* XXX: HANDLE THE ERROR */
void spark_max_set_idle_mode(rev::CANSparkMax ctx, int idle_mode)
{
  rev::CANSparkMax::IdleMode _mode;

  switch (idle_mode) {
    case 0:
      _mode = rev::CANSparkMax::IdleMode::kCoast;
    case 1:
      _mode = rev::CANSparkMax::IdleMode::kBrake;
  }

  ctx.SetIdleMode(_mode);
}

/*
 * idle_mode:
 *  0 => kCoast
 *  1 => kBrake
 */
int spark_max_get_idle_mode(rev::CANSparkMax ctx)
{
  rev::CANSparkMax::IdleMode idle_mode = ctx.GetIdleMode();
  int _mode;

  switch (idle_mode) {
    case rev::CANSparkMax::IdleMode::kCoast:
      _mode = 0;
    case rev::CANSparkMax::IdleMode::kBrake:
      _mode = 1;
  }

  return _mode;
}

/* XXX: HANDLE THE ERROR */
void spark_max_enable_voltage_compensation(rev::CANSparkMax ctx, double nominal_voltage) {
  ctx.EnableVoltageCompensation(nominal_voltage);
}

/* XXX: HANDLE THE ERROR */
void spark_max_disable_voltage_compensation(rev::CANSparkMax ctx) {
  ctx.DisableVoltageCompensation();
}

double spark_max_get_voltage_compensation_nominal_voltage(rev::CANSparkMax ctx) {
  return ctx.GetVoltageCompensationNominalVoltage();
}

/* XXX: HANDLE THE ERROR */
void spark_max_set_open_loop_ramp_rate(rev::CANSparkMax ctx, double rate) {
  ctx.SetOpenLoopRampRate(rate);
}

/* XXX: HANDLE THE ERROR */
void spark_max_set_closed_loop_ramp_rate(rev::CANSparkMax ctx, double rate) {
  ctx.SetClosedLoopRampRate(rate);
}

/* XXX: HANDLE THE ERROR */
double spark_max_get_open_loop_ramp_rate(rev::CANSparkMax ctx) {
  return ctx.GetOpenLoopRampRate();
}

/* XXX: HANDLE THE ERROR */
double spark_max_get_closed_loop_ramp_rate(rev::CANSparkMax ctx) {
  return ctx.GetClosedLoopRampRate();
}

/***********************
 * FOLLOWER STUFF HERE *
 ***********************/

uint16_t spark_max_get_faults(rev::CANSparkMax ctx) {
  return ctx.GetFaults();
}

uint16_t spark_max_get_sticky_faults(rev::CANSparkMax ctx) {
  return ctx.GetStickyFaults();
}

/********************
 * FAULT STUFF HERE *
 ********************/

double spark_max_get_bus_voltage(rev::CANSparkMax ctx) {
  return ctx.GetBusVoltage();
}

double spark_max_get_applied_output(rev::CANSparkMax ctx) {
  return ctx.GetAppliedOutput();
}

double spark_max_get_output_current(rev::CANSparkMax ctx) {
  return ctx.GetOutputCurrent();
}

double spark_max_get_motor_temperature(rev::CANSparkMax ctx) {
  return ctx.GetMotorTemperature();
}
