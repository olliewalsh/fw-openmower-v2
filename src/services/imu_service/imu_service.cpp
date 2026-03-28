//
// Created by clemens on 31.07.24.
//

#include "imu_service.hpp"

#include <etl/to_string.h>
#include <lsm6ds3tr-c_reg.h>
#include <ulog.h>

#include <cmath>
#include <xbot-service/portable/system.hpp>

#include "services.hpp"

namespace {
constexpr float kDefaultCollisionAccelThreshold = 8.0f;
constexpr float kDefaultCollisionJerkThreshold = 120.0f;
constexpr float kDefaultCollisionGravityFilterHz = 1.5f;
constexpr float kDefaultCollisionSignalFilterHz = 25.0f;
constexpr float kDefaultCollisionWheelCurrentThreshold = 0.3f;
constexpr float kDefaultCollisionActualLinearSpeedThreshold = 0.03f;
constexpr float kDefaultCollisionActualAngularSpeedThreshold = 0.15f;
constexpr float kDefaultCollisionActualSpeedDropThreshold = 0.06f;
constexpr uint16_t kDefaultCollisionConsecutiveSamples = 2;
}  // namespace

static SPIConfig spi_config = {
    false,
    false,
    nullptr,
    nullptr,
    LINE_IMU_CS,
    SPI_CFG1_MBR_DIV32 | SPI_CFG1_DSIZE_0 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_2,
    SPI_CFG2_COMM_FULL_DUPLEX | SPI_CFG2_CPOL | SPI_CFG2_CPHA,
};

static stmdev_ctx_t dev_ctx{};

static constexpr auto write_reg_lambda = [](void *, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  spiSelect(&SPID_IMU);
  spiSend(&SPID_IMU, 1, &reg);
  spiSend(&SPID_IMU, len, bufp);
  spiUnselect(&SPID_IMU);
  return (int32_t)0;
};

static constexpr auto read_reg_lambda = [](void *, uint8_t reg, uint8_t *bufp, uint16_t len) {
  reg |= 0x80;

  spiSelect(&SPID_IMU);
  spiSend(&SPID_IMU, 1, &reg);
  spiReceive(&SPID_IMU, len, bufp);
  spiUnselect(&SPID_IMU);
  return (int32_t)0;
};

void ImuService::OnCreate() {
  error_message = "";
  // Acquire Bus and never let it go, there's only the one IMU connected to it.
  spiAcquireBus(&SPID_IMU);
  spiStart(&SPID_IMU, &spi_config);

  dev_ctx.write_reg = write_reg_lambda;
  dev_ctx.read_reg = read_reg_lambda;
  for (int i = 0; i < 100; i++) {
    uint8_t whoamI = 0;
    lsm6ds3tr_c_device_id_get(&dev_ctx, &whoamI);

    if (whoamI == 0x6a || whoamI == 0x6c) {
      imu_found = true;
      error_message = "None";
      break;
    } else {
      imu_found = false;
      error_message = "IMU Not Found. Whoami=0x";
      etl::format_spec hex_spec{};
      hex_spec.base(16);
      etl::to_string(whoamI, error_message, hex_spec, true);
      chThdSleep(TIME_MS2I(100));
    }
  }

  if (!imu_found) {
    return;
  }

  /* Restore default configuration */
  lsm6ds3tr_c_reset_set(&dev_ctx, PROPERTY_ENABLE);

  uint8_t rst = 0;
  int rst_count = 0;
  do {
    lsm6ds3tr_c_reset_get(&dev_ctx, &rst);
    rst_count++;
  } while (rst);

  /* Enable Block Data Update */
  lsm6ds3tr_c_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_833Hz);
  lsm6ds3tr_c_gy_data_rate_set(&dev_ctx, LSM6DS3TR_C_GY_ODR_833Hz);
  /* Set full scale */
  lsm6ds3tr_c_xl_full_scale_set(&dev_ctx, LSM6DS3TR_C_2g);
  lsm6ds3tr_c_gy_full_scale_set(&dev_ctx, LSM6DS3TR_C_2000dps);
  /* Configure filtering chain(No aux interface) */
  /* Accelerometer - analog filter */
  lsm6ds3tr_c_xl_filter_analog_set(&dev_ctx, LSM6DS3TR_C_XL_ANA_BW_400Hz);
  /* Accelerometer - LPF1 path ( LPF2 not used )*/
  // lsm6ds3tr_c_xl_lp1_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_LP1_ODR_DIV_4);
  /* Accelerometer - LPF1 + LPF2 path */
  lsm6ds3tr_c_xl_lp2_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_LOW_NOISE_LP_ODR_DIV_100);
  ULOG_ARG_INFO(&service_id_, "IMU configured successfully");
}

bool ImuService::OnStart() {
  // Validate and parse axis remapping
  for (int i = 0; i < 3; ++i) {
    int8_t val = AxisRemap.value[i];

    if (val < -3 || val == 0 || val > 3) {
      ULOG_ARG_ERROR(&service_id_, "Invalid axis remap value: %d", val);
      return false;
    }
    axis_remap_sign_[i] = (val > 0) ? 1 : -1;
    axis_remap_idx_[i] = abs(val) - 1;
  }

  collision_active_ = false;
  gravity_initialized_ = false;
  publish_axes_this_tick_ = false;
  collision_trigger_count_ = 0;
  last_actual_speed_ = 0.0f;
  memset(gravity_estimate_, 0x00, sizeof(gravity_estimate_));
  memset(linear_acceleration_, 0x00, sizeof(linear_acceleration_));
  memset(filtered_linear_acceleration_, 0x00, sizeof(filtered_linear_acceleration_));
  memset(previous_filtered_linear_acceleration_, 0x00, sizeof(previous_filtered_linear_acceleration_));
  memset(collision_debug_, 0x00, sizeof(collision_debug_));
  SetCollisionEmergency(false);

  return true;
}

uint16_t ImuService::GetEmergencyReasons() {
  const uint16_t emergency_reasons = emergency_service.GetEmergencyReasons();
  if (collision_active_ && (emergency_reasons & EmergencyReason::COLLISION) != 0 &&
      (emergency_reasons & EmergencyReason::LATCH) == 0) {
    collision_active_ = false;
    collision_trigger_count_ = 0;
  }

  return collision_active_ ? static_cast<uint16_t>(EmergencyReason::COLLISION | EmergencyReason::LATCH) : 0;
}

void ImuService::tick() {
  if (!imu_found) {
    static uint32_t last_log = 0;
    uint32_t now = xbot::service::system::getTimeMicros();
    if (now - last_log > 1'000'000) {
      ULOG_ARG_ERROR(&service_id_, error_message.c_str());
      last_log = now;
    }
    return;
  }
  lsm6ds3tr_c_reg_t reg;
  lsm6ds3tr_c_status_reg_get(&dev_ctx, &reg.status_reg);

  if (reg.status_reg.xlda) {
    /* Read acceleration data */
    memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
    lsm6ds3tr_c_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
    axes[0] = axis_remap_sign_[0] * lsm6ds3tr_c_from_fs2g_to_mg(data_raw_acceleration[axis_remap_idx_[0]]) * 0.00980665;
    axes[1] = axis_remap_sign_[1] * lsm6ds3tr_c_from_fs2g_to_mg(data_raw_acceleration[axis_remap_idx_[1]]) * 0.00980665;
    axes[2] = axis_remap_sign_[2] * lsm6ds3tr_c_from_fs2g_to_mg(data_raw_acceleration[axis_remap_idx_[2]]) * 0.00980665;
  }

  if (reg.status_reg.gda) {
    /* Read angular rate data */
    memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
    lsm6ds3tr_c_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
    axes[3] = axis_remap_sign_[0] * M_PI *
              lsm6ds3tr_c_from_fs2000dps_to_mdps(data_raw_angular_rate[axis_remap_idx_[0]]) / 180000.0;
    axes[4] = axis_remap_sign_[1] * M_PI *
              lsm6ds3tr_c_from_fs2000dps_to_mdps(data_raw_angular_rate[axis_remap_idx_[1]]) / 180000.0;
    axes[5] = axis_remap_sign_[2] * M_PI *
              lsm6ds3tr_c_from_fs2000dps_to_mdps(data_raw_angular_rate[axis_remap_idx_[2]]) / 180000.0;
  }

  /*if (reg.status_reg.tda) {
    // Read temperature data
    memset(&data_raw_temperature, 0x00, sizeof(int16_t));
    lsm6ds3tr_c_temperature_raw_get(&dev_ctx, &data_raw_temperature);
    temperature_degC = lsm6ds3tr_c_from_lsb_to_celsius(
                         data_raw_temperature );
  }*/

  UpdateCollisionDetection(xbot::service::system::getTimeMicros());
  publish_axes_this_tick_ = !publish_axes_this_tick_;
  if (publish_axes_this_tick_) {
    SendAxes(axes, 9);
    SendCollisionDebug(collision_debug_, 10);
  }
}

void ImuService::UpdateCollisionDetection(uint32_t now_micros) {
  (void)now_micros;
  if (DisableCollisionDetection.value != 0) {
    return;
  }

  constexpr double dt = 0.005;
  constexpr double two_pi = 2.0 * M_PI;
  const double cutoff_hz =
      CollisionGravityFilterHz.value > 0.0f ? CollisionGravityFilterHz.value : kDefaultCollisionGravityFilterHz;
  const double tau = 1.0 / (two_pi * cutoff_hz);
  const double alpha = dt / (tau + dt);
  const double signal_cutoff_hz =
      CollisionSignalFilterHz.value > 0.0f ? CollisionSignalFilterHz.value : kDefaultCollisionSignalFilterHz;
  const double signal_tau = 1.0 / (two_pi * signal_cutoff_hz);
  const double signal_alpha = dt / (signal_tau + dt);
  const double jerk_threshold =
      CollisionJerkThreshold.value > 0.0f ? CollisionJerkThreshold.value : kDefaultCollisionJerkThreshold;
  const float wheel_current_threshold = CollisionWheelCurrentThreshold.value > 0.0f
                                            ? CollisionWheelCurrentThreshold.value
                                            : kDefaultCollisionWheelCurrentThreshold;
  const float actual_linear_speed_threshold = CollisionActualLinearSpeedThreshold.value > 0.0f
                                                  ? CollisionActualLinearSpeedThreshold.value
                                                  : kDefaultCollisionActualLinearSpeedThreshold;
  const float actual_angular_speed_threshold = CollisionActualAngularSpeedThreshold.value > 0.0f
                                                   ? CollisionActualAngularSpeedThreshold.value
                                                   : kDefaultCollisionActualAngularSpeedThreshold;
  const float actual_speed_drop_threshold = CollisionActualSpeedDropThreshold.value > 0.0f
                                                ? CollisionActualSpeedDropThreshold.value
                                                : kDefaultCollisionActualSpeedDropThreshold;
  const uint16_t consecutive_samples =
      CollisionConsecutiveSamples.value > 0 ? CollisionConsecutiveSamples.value : kDefaultCollisionConsecutiveSamples;

  if (!gravity_initialized_) {
    for (size_t i = 0; i < 3; ++i) {
      gravity_estimate_[i] = axes[i];
      linear_acceleration_[i] = 0.0;
      filtered_linear_acceleration_[i] = 0.0;
      previous_filtered_linear_acceleration_[i] = 0.0;
    }
    gravity_initialized_ = true;
    SetCollisionEmergency(false);
    return;
  }

  for (size_t i = 0; i < 3; ++i) {
    gravity_estimate_[i] += alpha * (axes[i] - gravity_estimate_[i]);
    linear_acceleration_[i] = axes[i] - gravity_estimate_[i];
    filtered_linear_acceleration_[i] += signal_alpha * (linear_acceleration_[i] - filtered_linear_acceleration_[i]);
  }

  float avg_abs_current = 0.0f;
  float actual_linear_velocity = 0.0f;
  float actual_angular_velocity = 0.0f;
  float commanded_linear_velocity = 0.0f;
  bool esc_state_valid = false;
  diff_drive.GetCollisionMetrics(avg_abs_current, actual_linear_velocity, actual_angular_velocity,
                                 commanded_linear_velocity, esc_state_valid);
  double gyro_sq = 0.0;
  double jerk_sq = 0.0;
  for (size_t i = 0; i < 3; ++i) {
    const double jerk = (filtered_linear_acceleration_[i] - previous_filtered_linear_acceleration_[i]) / dt;
    if (i != 2) {
      gyro_sq += axes[3 + i] * axes[3 + i];
      jerk_sq += jerk * jerk;
    }
    previous_filtered_linear_acceleration_[i] = filtered_linear_acceleration_[i];
  }

  const double gyro_mag = std::sqrt(gyro_sq);
  const double jerk_mag = std::sqrt(jerk_sq);
  const float actual_speed =
      std::sqrt(actual_linear_velocity * actual_linear_velocity + actual_angular_velocity * actual_angular_velocity);

  const bool motion_armed = esc_state_valid && (std::fabs(actual_linear_velocity) >= actual_linear_speed_threshold ||
                                                std::fabs(actual_angular_velocity) >= actual_angular_speed_threshold);
  const bool imu_trigger = jerk_mag >= jerk_threshold;
  const bool current_spike = esc_state_valid && avg_abs_current >= wheel_current_threshold;
  const float speed_drop_value = last_actual_speed_ - actual_speed;
  const bool linear_command_active = std::fabs(commanded_linear_velocity) > 0.001f;
  const bool speed_drop = esc_state_valid && last_actual_speed_ >= actual_linear_speed_threshold &&
                          speed_drop_value >= actual_speed_drop_threshold && linear_command_active;
  const bool drive_corroborated = current_spike || speed_drop;
  const bool collision_candidate = motion_armed && imu_trigger && drive_corroborated;

  if (collision_candidate) {
    collision_trigger_count_ =
        collision_trigger_count_ < consecutive_samples ? collision_trigger_count_ + 1 : consecutive_samples;
  } else {
    collision_trigger_count_ = 0;
  }

  if (!collision_active_ && collision_trigger_count_ >= consecutive_samples) {
    SetCollisionEmergency(true);
  }

  collision_debug_[0] = filtered_linear_acceleration_[0];
  collision_debug_[1] = filtered_linear_acceleration_[1];
  collision_debug_[2] = filtered_linear_acceleration_[2];
  collision_debug_[3] = jerk_mag;
  collision_debug_[4] = gyro_mag;
  collision_debug_[5] = actual_linear_velocity;
  collision_debug_[6] = actual_angular_velocity;
  collision_debug_[7] = avg_abs_current;
  collision_debug_[8] = speed_drop_value;
  collision_debug_[9] = collision_candidate ? 1.0 : 0.0;

  last_actual_speed_ = actual_speed;
}

void ImuService::SetCollisionEmergency(bool active) {
  if (collision_active_ == active) {
    return;
  }

  collision_active_ = active;
}
