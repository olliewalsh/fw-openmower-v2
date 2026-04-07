//
// Created by clemens on 31.07.24.
//

#include "mower_service.hpp"

#include <cmath>
#include <xbot-service/portable/system.hpp>

#include "services.hpp"

namespace {
#ifdef ROBOT_PLATFORM_Sabo
constexpr float kSaboMowerTargetErpm = 10500.0f;
#endif
}  // namespace

void MowerService::OnCreate() {
  chDbgAssert(mower_driver_ != nullptr, "Mower Motor Driver cannot be null!");
  mower_driver_->SetStateCallback(
      etl::delegate<void(const MotorDriver::ESCState&)>::create<MowerService, &MowerService::ESCCallback>(*this));
  mower_driver_->Start();
}

bool MowerService::OnStart() {
  mower_command_ = 0;
  return true;
}

void MowerService::OnStop() {
  mower_command_ = 0;
}

void MowerService::tick() {
  chMtxLock(&mtx);

  // Check, if we recently received a mower enable command. If not, set to zero for safety.
  if (xbot::service::system::getTimeMicros() - last_duty_received_micros_ > 10'000'000) {
    // It's ok to set it here, because we know that duty_sent_ is false (we're in a timeout after all).
    mower_command_ = 0;
  }

  if (!duty_sent_) {
    // Send the latest motor command, if we haven't in the meantime
    // (e.g. due to new value or emergency)
    ApplyMotorCommand();
  }

  mower_driver_->RequestStatus();

  // TODO: actually detect some rain
  bool rain_detected = false;

  StartTransaction();
  SendRainDetected(rain_detected);

  // Check, if we have received ESC status updates recently. If not, send a disconnected message
  if (xbot::service::system::getTimeMicros() - last_valid_esc_state_micros_ > 1'000'000 || !esc_state_valid_) {
    // No recent update received (or none at all)
    mower_command_ = 0;
    SendMowerStatus(static_cast<uint8_t>(MotorDriver::ESCState::ESCStatus::ESC_STATUS_DISCONNECTED));
  } else {
    // We got recent data, send it
    StartTransaction();
    SendMowerESCTemperature(esc_state_.temperature_pcb);
    SendMowerMotorCurrent(esc_state_.current_input);
    SendMowerStatus(static_cast<uint8_t>(esc_state_.status));
    SendMowerMotorTemperature(esc_state_.temperature_motor);
    SendMowerRunning(std::fabs(esc_state_.rpm) > 0);
    SendMowerMotorRPM(esc_state_.rpm);
  }
  CommitTransaction();

  duty_sent_ = false;
  chMtxUnlock(&mtx);
}

void MowerService::ESCCallback(const MotorDriver::ESCState& state) {
  chMtxLock(&state_mutex_);
  esc_state_ = state;
  esc_state_valid_ = true;
  last_valid_esc_state_micros_ = xbot::service::system::getTimeMicros();
  chMtxUnlock(&state_mutex_);
}

void MowerService::ApplyMotorCommand() {
  // Get the current emergency state
  bool emergency = emergency_service.GetEmergencyReasons() != 0;
#ifdef ROBOT_PLATFORM_Sabo
  const float command = emergency ? 0.0f : mower_command_;
  if (mower_driver_->SupportsSpeedControl()) {
    mower_driver_->SetSpeed(command);
  } else {
    mower_driver_->SetDuty(0);
  }
#else
  if (emergency) {
    mower_driver_->SetDuty(0);
  } else {
    mower_driver_->SetDuty(mower_command_);
  }
#endif
  duty_sent_ = true;
}

void MowerService::OnMowerEnabledChanged(const uint8_t& new_value) {
  chMtxLock(&mtx);
  last_duty_received_micros_ = xbot::service::system::getTimeMicros();
  if (new_value) {
    mower_command_ =
#ifdef ROBOT_PLATFORM_Sabo
        kSaboMowerTargetErpm;
#else
        1.0f;
#endif
  } else {
    mower_command_ = 0;
  }
  if (!duty_sent_) {
    ApplyMotorCommand();
  }
  chMtxUnlock(&mtx);
}

void MowerService::SetDriver(MotorDriver* motor_driver) {
  mower_driver_ = motor_driver;
}
void MowerService::OnEmergencyChangedEvent() {
  bool emergency = emergency_service.GetEmergencyReasons() != 0;
  if (!emergency) {
    // only set speed to 0 if the emergency happens, not if it's cleared
    return;
  }
  chMtxLock(&mtx);
  mower_command_ = 0;
  // Instantly send the stop command.
  ApplyMotorCommand();
  chMtxUnlock(&mtx);
}
