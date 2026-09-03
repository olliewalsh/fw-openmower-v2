//
// Created by clemens on 26.07.24.
//

#ifndef DIFF_DRIVE_SERVICE_HPP
#define DIFF_DRIVE_SERVICE_HPP

#include <drivers/gps/nmea_gps_driver.h>
#include <etl/atomic.h>

#include <DiffDriveServiceBase.hpp>
#include <drivers/motor/motor_driver.hpp>
#include <globals.hpp>
#include <xbot-service/portable/socket.hpp>

#include "wheel_speed_controller.hpp"

using namespace xbot::service;
using namespace xbot::driver::motor;

class DiffDriveService : public DiffDriveServiceBase {
 private:
  THD_WORKING_AREA(wa, 1024){};
  MotorDriver *left_esc_driver_ = nullptr;
  MotorDriver *right_esc_driver_ = nullptr;

  MotorDriver::ESCState left_esc_state_{};
  MotorDriver::ESCState right_esc_state_{};
  bool left_esc_state_valid_ = false;
  bool right_esc_state_valid_ = false;
  uint32_t last_valid_esc_state_micros_ = 0;
  static constexpr uint8_t ESC_LEFT = 1 << 0;
  static constexpr uint8_t ESC_RIGHT = 1 << 1;
  etl::atomic<uint8_t> escs_connected_{0};
  uint32_t last_duty_received_micros_ = 0;

  uint32_t last_ticks_left = 0;
  uint32_t last_ticks_right = 0;
  bool last_ticks_valid = false;
  uint32_t last_ticks_micros_ = 0;
  static constexpr uint8_t kSpeedWindowSamples = 3;
  int32_t speed_window_left_ticks_[kSpeedWindowSamples]{};
  int32_t speed_window_right_ticks_[kSpeedWindowSamples]{};
  float speed_window_dt_[kSpeedWindowSamples]{};
  int32_t speed_window_left_sum_ = 0;
  int32_t speed_window_right_sum_ = 0;
  float speed_window_dt_sum_ = 0.0f;
  uint8_t speed_window_index_ = 0;
  uint8_t speed_window_count_ = 0;
  bool pure_rotation_commanded_ = false;
  float desired_speed_l_ = 0;
  float desired_speed_r_ = 0;

  static constexpr WheelSpeedController::Gains kDefaultWheelSpeedControllerGains{1.2f, 0.35f, 0.8f, 0.05f};
  WheelSpeedController left_wheel_controller_{kDefaultWheelSpeedControllerGains};
  WheelSpeedController right_wheel_controller_{kDefaultWheelSpeedControllerGains};
  bool duty_sent_ = false;

  WheelSpeedController::Gains GetConfiguredWheelSpeedControllerGains() const;
  float GetMaxDuty() const;
  float GetNominalWheelSpeedLimit() const;
  void UpdateControllerGains();
  void ResetSpeedMeasurementWindow();
  void UpdateDutyFromMeasuredSpeeds(float dt);

 public:
  explicit DiffDriveService(uint16_t service_id) : DiffDriveServiceBase(service_id, wa, sizeof(wa)) {
  }

  void OnEmergencyChangedEvent();

  void SetDrivers(MotorDriver *left_driver, MotorDriver *right_driver);

  bool IsHealthy() override {
    return IsRunning() && (escs_connected_.load() == (ESC_LEFT | ESC_RIGHT));
  }

 protected:
  bool OnStart() override;
  void OnCreate() override;
  void OnStop() override;

 private:
  void tick();
  ServiceSchedule tick_schedule_{*this, 40'000,
                                 XBOT_FUNCTION_FOR_METHOD(DiffDriveService, &DiffDriveService::tick, this)};

  void SetDuty();

  void LeftESCCallback(const MotorDriver::ESCState &state);
  void RightESCCallback(const MotorDriver::ESCState &state);
  void ProcessStatusUpdate();

 protected:
  void OnControlTwistChanged(const double *new_value, uint32_t length) override;
};

#endif  // DIFF_DRIVE_SERVICE_HPP
