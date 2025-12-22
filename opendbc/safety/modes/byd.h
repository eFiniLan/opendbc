#pragma once

#include "opendbc/safety/declarations.h"

#define BYD_LKAS          0x1E2U
#define BYD_LKAS_HUD      0x316U

// CAN bus numbers
#define BYD_MAIN 0
#define BYD_CAM  2

// GCOV_EXCL_START
// Unreachable by design (doesn't define any rx msgs)
static void byd_rx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
}
// GCOV_EXCL_STOP

// GCOV_EXCL_START
static bool byd_tx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  return true;
}
// GCOV_EXCL_STOP

static safety_config byd_init(uint16_t param) {
  // static const CanMsg BYD_TX_MSGS[] = {{BYD_LKAS, 0, 8, .check_relay = true}, {BYD_LKAS_HUD, 0, 8, .check_relay = true}};

  // static RxCheck byd_rx_checks[] = {
  //   {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},

  // }

  // return BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
  // Enables passthrough mode where relay is open and bus 0 gets forwarded to bus 2 and vice versa
  const uint16_t ALLOUTPUT_PARAM_PASSTHROUGH = 1;
  controls_allowed = true;
  bool alloutput_passthrough = GET_FLAG(param, ALLOUTPUT_PARAM_PASSTHROUGH);
  return (safety_config){NULL, 0, NULL, 0, !alloutput_passthrough}; // NOLINT(readability/braces)

}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
};
