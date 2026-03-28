//
// Created by clemens on 26.07.24.
//

#ifndef WHEEL_SPEED_CONTROLLER_HPP
#define WHEEL_SPEED_CONTROLLER_HPP

class WheelSpeedController {
 public:
  struct Gains {
    float feedforward = 1.5f;
    float kp = 0.35f;
    float ki = 1.5f;
  };

  explicit WheelSpeedController(Gains gains) : gains_(gains) {
  }

  void SetGains(Gains gains) {
    gains_ = gains;
  }

  void Reset() {
    target_speed_ = 0;
    measured_speed_ = 0;
    integral_ = 0;
    duty_ = 0;
  }

  void SetTargetSpeed(float speed) {
    target_speed_ = speed;
  }

  void SetMeasuredSpeed(float speed) {
    measured_speed_ = speed;
  }

  float Update(float dt) {
    float error = target_speed_ - measured_speed_;
    float next_integral = integral_;
    if (dt > 0.0f) {
      next_integral += error * dt;
    }

    float unsaturated = gains_.feedforward * target_speed_ + gains_.kp * error + gains_.ki * next_integral;

    // Only integrate while unsaturated, or when the error drives the controller back out of saturation.
    if ((unsaturated <= 1.0f && unsaturated >= -1.0f) || (unsaturated > 1.0f && error < 0.0f) ||
        (unsaturated < -1.0f && error > 0.0f)) {
      integral_ = next_integral;
      unsaturated = gains_.feedforward * target_speed_ + gains_.kp * error + gains_.ki * integral_;
    }

    duty_ = ClampUnit(unsaturated);
    return duty_;
  }

  float target_speed() const {
    return target_speed_;
  }

  float measured_speed() const {
    return measured_speed_;
  }

  float duty() const {
    return duty_;
  }

 private:
  static float ClampUnit(float value) {
    if (value >= 1.0f) {
      return 1.0f;
    }
    if (value <= -1.0f) {
      return -1.0f;
    }
    return value;
  }

  Gains gains_;
  float target_speed_ = 0;
  float measured_speed_ = 0;
  float integral_ = 0;
  float duty_ = 0;
};

#endif  // WHEEL_SPEED_CONTROLLER_HPP
