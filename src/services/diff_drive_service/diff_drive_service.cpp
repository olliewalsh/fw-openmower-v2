//
// Created by clemens on 26.07.24.
//

#include "diff_drive_service.hpp"

#include <ulog.h>

#include <cmath>
#include <drivers/motor/motor_driver.hpp>
#include <services.hpp>
#include <xbot-service/portable/system.hpp>

using namespace xbot::driver::motor;

WheelSpeedController::Gains DiffDriveService::GetConfiguredWheelSpeedControllerGains() const {
  WheelSpeedController::Gains gains{
      static_cast<float>(WheelSpeedFeedforward.value),
      static_cast<float>(WheelSpeedKp.value),
      static_cast<float>(WheelSpeedKi.value),
  };

  if (gains.feedforward <= 0.0f) {
    gains.feedforward = kDefaultWheelSpeedControllerGains.feedforward;
  }
  if (gains.kp < 0.0f) {
    gains.kp = kDefaultWheelSpeedControllerGains.kp;
  }
  if (gains.ki < 0.0f) {
    gains.ki = kDefaultWheelSpeedControllerGains.ki;
  }

  return gains;
}

float DiffDriveService::GetMaxDuty() const {
  const auto val = static_cast<float>(MaxDuty.value);
  if (val > 0.0f && val <= 1.0f) {
    return val;
  }
  return 0.95f;
}

float DiffDriveService::GetNominalWheelSpeedLimit() const {
  const auto gains = GetConfiguredWheelSpeedControllerGains();
  return GetMaxDuty() / gains.feedforward;
}

void DiffDriveService::UpdateControllerGains() {
  const auto gains = GetConfiguredWheelSpeedControllerGains();
  left_wheel_controller_.SetGains(gains);
  right_wheel_controller_.SetGains(gains);
  const float max_duty = GetMaxDuty();
  left_wheel_controller_.SetMaxDuty(max_duty);
  right_wheel_controller_.SetMaxDuty(max_duty);
}

void DiffDriveService::UpdateDutyFromMeasuredSpeeds(float dt) {
  UpdateControllerGains();
  const float nominal_limit = GetNominalWheelSpeedLimit();
  const float abs_l = std::fabs(desired_speed_l_);
  const float abs_r = std::fabs(desired_speed_r_);
  const float peak = abs_l > abs_r ? abs_l : abs_r;
  const float scale = (nominal_limit > 0.0f && peak > nominal_limit) ? nominal_limit / peak : 1.0f;
  left_wheel_controller_.SetTargetSpeed(desired_speed_l_ * scale);
  right_wheel_controller_.SetTargetSpeed(desired_speed_r_ * scale);
  left_wheel_controller_.Update(dt);
  right_wheel_controller_.Update(dt);
}

void DiffDriveService::OnEmergencyChangedEvent() {
  bool emergency = emergency_service.GetEmergencyReasons() != 0;
  if (!emergency) {
    // only set speed to 0 if the emergency happens, not if it's cleared
    return;
  }
  chMtxLock(&state_mutex_);
  desired_speed_l_ = 0;
  desired_speed_r_ = 0;
  left_wheel_controller_.Reset();
  right_wheel_controller_.Reset();
  // Instantly send the 0 duty cycle
  SetDuty();
  chMtxUnlock(&state_mutex_);
}
void DiffDriveService::SetDrivers(MotorDriver* left_driver, MotorDriver* right_driver) {
  left_esc_driver_ = left_driver;
  right_esc_driver_ = right_driver;
}

bool DiffDriveService::OnStart() {
  // Check, if configuration is valid, if not retry
  if (WheelDistance.value == 0) {
    ULOG_ARG_ERROR(&service_id_, "WheelDistance was 0, cannot start service!");
    return false;
  }

  if (WheelTicksPerMeter.value == 0.0) {
    ULOG_ARG_ERROR(&service_id_, "WheelTicksPerMeter was 0, cannot start service!");
    return false;
  }

  UpdateControllerGains();
  desired_speed_l_ = 0;
  desired_speed_r_ = 0;
  left_wheel_controller_.Reset();
  right_wheel_controller_.Reset();
  last_ticks_valid = false;

  // Kick off the request-response cycle
  left_esc_driver_->RequestStatus();
  right_esc_driver_->RequestStatus();
  return true;
}

void DiffDriveService::OnCreate() {
  chDbgAssert(left_esc_driver_ != nullptr, "Left Motor Driver cannot be null!");
  chDbgAssert(right_esc_driver_ != nullptr, "Right Motor Driver cannot be null!");

  // Register callbacks
  left_esc_driver_->SetStateCallback(
      etl::delegate<void(const MotorDriver::ESCState&)>::create<DiffDriveService, &DiffDriveService::LeftESCCallback>(
          *this));
  right_esc_driver_->SetStateCallback(
      etl::delegate<void(const MotorDriver::ESCState&)>::create<DiffDriveService, &DiffDriveService::RightESCCallback>(
          *this));

  left_esc_driver_->Start();
  right_esc_driver_->Start();
}

void DiffDriveService::OnStop() {
  desired_speed_l_ = 0;
  desired_speed_r_ = 0;
  left_wheel_controller_.Reset();
  right_wheel_controller_.Reset();
  last_ticks_valid = false;
}

void DiffDriveService::tick() {
  chMtxLock(&state_mutex_);

  // Check, if we recently received duty. If not, set to zero for safety
  if (xbot::service::system::getTimeMicros() - last_duty_received_micros_ > 1'000'000) {
    // it's ok to set it here, because we know that duty_set_ is false (we're in a timeout after all)
    desired_speed_l_ = 0;
    desired_speed_r_ = 0;
    left_wheel_controller_.Reset();
    right_wheel_controller_.Reset();
  }

  if (!duty_sent_) {
    SetDuty();
  }

  // Check, if we have received ESC status updates recently. If not, send a disconnected message
  // and request status to kick off the request-response cycle.
  if (xbot::service::system::getTimeMicros() - last_valid_esc_state_micros_ > 1'000'000) {
    StartTransaction();
    if (!left_esc_state_valid_) {
      SendLeftESCStatus(static_cast<uint8_t>(MotorDriver::ESCState::ESCStatus::ESC_STATUS_DISCONNECTED));
    }
    if (!right_esc_state_valid_) {
      SendRightESCStatus(static_cast<uint8_t>(MotorDriver::ESCState::ESCStatus::ESC_STATUS_DISCONNECTED));
    }
    CommitTransaction();
    left_esc_driver_->RequestStatus();
    right_esc_driver_->RequestStatus();
  }

  duty_sent_ = false;
  chMtxUnlock(&state_mutex_);
}

void DiffDriveService::SetDuty() {
  // Get the current emergency state
  bool emergency = emergency_service.GetEmergencyReasons() != 0;
  if (emergency) {
    left_esc_driver_->SetDuty(0);
    right_esc_driver_->SetDuty(0);
  } else {
    float left_duty = left_wheel_controller_.duty();
    // The right motor is installed with opposite polarity.
    float right_duty = -right_wheel_controller_.duty();
    // Scale both duties proportionally if either exceeds the limit,
    // preserving the left/right ratio so the turn radius is maintained.
    const float max_duty = GetMaxDuty();
    const float abs_l = std::fabs(left_duty);
    const float abs_r = std::fabs(right_duty);
    const float peak = abs_l > abs_r ? abs_l : abs_r;
    if (peak > max_duty) {
      const float scale = max_duty / peak;
      left_duty *= scale;
      right_duty *= scale;
    }
    left_esc_driver_->SetDuty(left_duty);
    right_esc_driver_->SetDuty(right_duty);
  }
  duty_sent_ = true;
}

void DiffDriveService::LeftESCCallback(const MotorDriver::ESCState& state) {
  bool request_next = false;
  chMtxLock(&state_mutex_);
  left_esc_state_ = state;
  left_esc_state_valid_ = true;
  if (right_esc_state_valid_) {
    ProcessStatusUpdate();
    request_next = true;
  }
  chMtxUnlock(&state_mutex_);
  if (request_next) {
    left_esc_driver_->RequestStatus();
    right_esc_driver_->RequestStatus();
  }
}

void DiffDriveService::RightESCCallback(const MotorDriver::ESCState& state) {
  bool request_next = false;
  chMtxLock(&state_mutex_);
  right_esc_state_ = state;
  right_esc_state_valid_ = true;
  if (left_esc_state_valid_) {
    ProcessStatusUpdate();
    request_next = true;
  }
  chMtxUnlock(&state_mutex_);
  if (request_next) {
    left_esc_driver_->RequestStatus();
    right_esc_driver_->RequestStatus();
  }
}

void DiffDriveService::ProcessStatusUpdate() {
  uint32_t micros = xbot::service::system::getTimeMicros();
  last_valid_esc_state_micros_ = micros;
  StartTransaction();
  SendLeftESCTemperature(left_esc_state_.temperature_pcb);
  SendLeftESCCurrent(left_esc_state_.current_input);
  SendLeftESCStatus(static_cast<uint8_t>(left_esc_state_.status));

  SendRightESCTemperature(right_esc_state_.temperature_pcb);
  SendRightESCCurrent(right_esc_state_.current_input);
  SendRightESCStatus(static_cast<uint8_t>(right_esc_state_.status));

  // Calculate the twist according to wheel ticks
  if (last_ticks_valid) {
    float dt = static_cast<float>(micros - last_ticks_micros_) / 1'000'000.0f;
    const float wheel_ticks_per_meter = static_cast<float>(WheelTicksPerMeter.value);
    const float wheel_distance = static_cast<float>(WheelDistance.value);
    if (dt > 0.0f && wheel_ticks_per_meter > 0.0f && wheel_distance > 0.0f) {
      int32_t d_left = static_cast<int32_t>(left_esc_state_.tacho - last_ticks_left);
      int32_t d_right = static_cast<int32_t>(right_esc_state_.tacho - last_ticks_right);
      left_wheel_controller_.SetMeasuredSpeed(static_cast<float>(d_left) / (dt * wheel_ticks_per_meter));
      right_wheel_controller_.SetMeasuredSpeed(-static_cast<float>(d_right) / (dt * wheel_ticks_per_meter));
      float vx = 0.5f * (left_wheel_controller_.measured_speed() + right_wheel_controller_.measured_speed());
      float vr = (right_wheel_controller_.measured_speed() - left_wheel_controller_.measured_speed()) / wheel_distance;
      UpdateDutyFromMeasuredSpeeds(dt);
      double data[6]{};
      data[0] = vx;
      data[5] = vr;
      SendActualTwist(data, 6);
      uint32_t ticks[2];
      ticks[0] = left_esc_state_.tacho;
      ticks[1] = right_esc_state_.tacho;
      SendWheelTicks(ticks, 2);
    }
  }
  last_ticks_valid = true;
  last_ticks_left = left_esc_state_.tacho;
  last_ticks_right = right_esc_state_.tacho;
  last_ticks_micros_ = micros;

  right_esc_state_valid_ = left_esc_state_valid_ = false;

  CommitTransaction();
}

void DiffDriveService::OnControlTwistChanged(const double* new_value, uint32_t length) {
  if (length != 6) return;
  chMtxLock(&state_mutex_);
  last_duty_received_micros_ = xbot::service::system::getTimeMicros();
  // we can only do forward and rotation around one axis
  const auto linear = static_cast<float>(new_value[0]);
  const auto angular = static_cast<float>(new_value[5]);

  desired_speed_r_ = linear + 0.5f * static_cast<float>(WheelDistance.value) * angular;
  desired_speed_l_ = linear - 0.5f * static_cast<float>(WheelDistance.value) * angular;
  UpdateDutyFromMeasuredSpeeds(0.0f);

  // Limit comms frequency to once per tick()
  if (!duty_sent_) {
    SetDuty();
  }
  chMtxUnlock(&state_mutex_);
}
