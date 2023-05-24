#include "include/CANSparkMax.h"
#include "include/CANSparkMaxLowLevel.h"
#include "include/CANPIDController.h"
#include "include/REVLibError.h"
#include "include/SparkMaxRelativeEncoder.h"
#include "include/SparkMaxAlternateEncoder.h"
#include "include/SparkMaxAbsoluteEncoder.h"
#include "include/SparkMaxAnalogSensor.h"

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

/********************************
 * LIMIT SWITCH STUFF GOES HERE *
 ********************************/

