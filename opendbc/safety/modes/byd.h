#pragma once

#include "opendbc/safety/declarations.h"

// Message addresses
#define BYD_STEERING_TORQUE  0x1FCU  // 508 - steering angle feedback
#define BYD_WHEEL_SPEED      0x1F0U  // 496 - wheel speeds
#define BYD_PEDAL            0x342U  // 834 - brake/gas pedal
#define BYD_ACC_HUD          0x32DU  // 813 - ACC HUD on cam bus
#define BYD_LKAS             0x1E2U  // 482 - LKAS command

// CAN bus numbers
#define BYD_MAIN 0
#define BYD_CAM  2

// Steering limits for angle control
// angle_deg_to_can = 10 (DBC factor is 0.1, so 1 degree = 10 CAN units)
static const AngleSteeringLimits BYD_STEERING_LIMITS = {
  .max_angle = 2200,  // 220 degrees in CAN units (220 * 10)
  .angle_deg_to_can = 10,
  .angle_rate_up_lookup = {
    {0., 5., 15.},
    {50., 16., 3.}  // deg/s at each speed breakpoint, * 10 for CAN units
  },
  .angle_rate_down_lookup = {
    {0., 5., 15.},
    {50., 35., 5.}
  },
};

static void byd_rx_hook(const CANPacket_t *msg) {
  // Messages on main bus (bus 0)
  if (msg->bus == BYD_MAIN) {
    // Steering angle feedback from STEERING_TORQUE
    // ANGLE: start_bit=16, size=16, little endian signed, factor=0.1
    if (msg->addr == BYD_STEERING_TORQUE) {
      int angle_meas_new = (msg->data[3] << 8) | msg->data[2];
      angle_meas_new = to_signed(angle_meas_new, 16);
      update_sample(&angle_meas, angle_meas_new);
    }

    // Vehicle speed from WHEEL_SPEED
    // All speeds: 12-bit, little endian, factor=0.1 (km/h)
    // FL: bits 0-11, FR: bits 16-27, RL: bits 28-39, RR: bits 40-51
    if (msg->addr == BYD_WHEEL_SPEED) {
      uint32_t val_lo = GET_BYTES(msg, 0, 4);
      uint32_t val_hi = GET_BYTES(msg, 4, 4);
      uint32_t speed_fl = val_lo & 0xFFFU;
      uint32_t speed_fr = (val_lo >> 16) & 0xFFFU;
      uint32_t speed_rl = (val_lo >> 28) | ((val_hi & 0xFFU) << 4);
      uint32_t speed_rr = (val_hi >> 8) & 0xFFFU;
      // Average all wheel speeds, convert from 0.1 km/h to m/s
      float speed_kph = (speed_fl + speed_fr + speed_rl + speed_rr) / 4.0f * 0.1f;
      vehicle_moving = speed_kph > 0.5f;
      UPDATE_VEHICLE_SPEED(speed_kph * KPH_TO_MS);
    }

    // Brake and gas pedal from PEDAL
    // AcceleratorPedal: byte 0, factor 0.01
    // BrakePedal: byte 1, factor 0.01
    if (msg->addr == BYD_PEDAL) {
      gas_pressed = msg->data[0] > 3U;    // > 3% threshold
      brake_pressed = msg->data[1] > 3U;  // > 3% threshold
    }
  }

  // ACC state from camera bus (bus 2)
  // CRUISE_STATE: start_bit=44, size=4 -> byte 5, bits 4-7
  // CRUISE_STATE == 3 means ACC engaged
  if ((msg->bus == BYD_CAM) && (msg->addr == BYD_ACC_HUD)) {
    int cruise_state = (msg->data[5] >> 4) & 0xFU;
    bool cruise_engaged = (cruise_state == 3);
    pcm_cruise_check(cruise_engaged);
  }
}

static bool byd_tx_hook(const CANPacket_t *msg) {
  // TODO: enable for production
  // MVP: disable tx checks for initial testing
  SAFETY_UNUSED(msg);
  return true;

#if 0
  bool tx = true;
  bool violation = false;

  // LKAS steering command check
  if (msg->addr == BYD_LKAS) {
    // LKAS_Output: start_bit=24, size=16, little endian signed, factor=0.1
    // Bytes 3-4, signed 16-bit
    int desired_angle = (msg->data[4] << 8) | msg->data[3];
    desired_angle = to_signed(desired_angle, 16);

    // LKAS_ACTIVE: start_bit=21, size=1, big endian -> byte 2, bit 5
    bool lka_active = (msg->data[2] >> 5) & 1U;

    if (steer_angle_cmd_checks(desired_angle, lka_active, BYD_STEERING_LIMITS)) {
      violation = true;
    }
  }

  if (violation) {
    tx = false;
  }

  return tx;
#endif
}

static safety_config byd_init(uint16_t param) {
  SAFETY_UNUSED(param);

  static const CanMsg BYD_TX_MSGS[] = {
    {BYD_LKAS, BYD_MAIN, 8, .check_relay = true},
  };

  static RxCheck byd_rx_checks[] = {
    // Steering angle sensor (main bus, 50Hz)
    {.msg = {{BYD_STEERING_TORQUE, BYD_MAIN, 8, 50U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    // Wheel speeds (main bus, 50Hz)
    {.msg = {{BYD_WHEEL_SPEED, BYD_MAIN, 8, 50U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    // Pedal status (main bus, 50Hz)
    {.msg = {{BYD_PEDAL, BYD_MAIN, 8, 50U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    // ACC HUD (camera bus, 10Hz)
    {.msg = {{BYD_ACC_HUD, BYD_CAM, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  return BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
}

static uint32_t byd_compute_checksum(const CANPacket_t *msg) {
  int len = GET_LEN(msg);
  if (len <= 0) return 0;

  uint8_t byte_key = 0xAF;
  int sum_first = 0;   // Sum for upper 4-bit nibbles
  int sum_second = 0;  // Sum for lower 4-bit nibbles

  // Iterate through data bytes, skipping the last byte (checksum location)
  for (int i = 0; i < (len - 1); i++) {
    sum_first  += (msg->data[i] >> 4);      // Extract high nibble
    sum_second += (msg->data[i] & 0x0FU);   // Extract low nibble
  }

  // Extract remainder from low nibble sum BEFORE mixing key
  uint8_t remainder = (uint8_t)(sum_second >> 4);

  // Mix in the byte_key (Cross-mixing: key_low to sum_first, key_high to sum_second)
  sum_first  += (byte_key & 0x0FU);
  sum_second += (byte_key >> 4);

  // Perform the transformation: (-sum + 9) & 0xF
  uint8_t first_part  = (uint8_t)((0x9 - sum_first) & 0x0FU);
  uint8_t second_part = (uint8_t)((0x9 - sum_second) & 0x0FU);

  // Final Assembly
  // High nibble gets the remainder correction and offset 5
  uint8_t res_high = (uint8_t)((first_part + (5 - remainder)) & 0x0FU);
  uint8_t res_low  = second_part;

  return (uint32_t)((res_high << 4) | res_low);
}

static uint32_t byd_get_checksum(const CANPacket_t *msg) {
  // Return the last byte of the message payload
  int checksum_byte = GET_LEN(msg) - 1;
  if (checksum_byte < 0) return 0;

  return (uint8_t)(msg->data[checksum_byte]);
}

static uint8_t byd_get_counter(const CANPacket_t *msg) {
  // Counter is at different positions depending on message
  // Most BYD messages: COUNTER at bits 52-55 (byte 6, bits 4-7)
  if ((msg->addr == BYD_STEERING_TORQUE) || (msg->addr == BYD_WHEEL_SPEED) ||
      (msg->addr == BYD_PEDAL) || (msg->addr == BYD_LKAS)) {
    return (msg->data[6] >> 4) & 0xFU;
  }
  return 0;
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .get_checksum = byd_get_checksum,
  .compute_checksum = byd_compute_checksum,
  .get_counter = byd_get_counter,
};
