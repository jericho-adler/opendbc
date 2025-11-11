#pragma once

#include "opendbc/safety/declarations.h"

// Volvo CMA platform CAN message addresses
#define VOLVO_LCA_STEER           0x58U    // TX from VCU1 to PSCM, LCA steering command (0x58)
#define VOLVO_BUS1_SPEED          0x70U   // RX from BCM, vehicle speed (BUS1_SPEED)
#define VOLVO_BCM2                0x69U   // RX from BCM, brake pedal, cruise state
#define VOLVO_SAS                 0x55U    // RX from SAS, steering angle sensor
#define VOLVO_PSCM                0x16U    // RX from PSCM, driver steering input
#define VOLVO_GEAR_POSITION       0x80U   // RX from transmission, gear position
#define VOLVO_ECM_1               0x250U   // RX from ECM, accelerator pedal position (0x250)

// CAN bus definitions for Volvo CMA platform
// Using same naming as carstate.py for consistency: main, pt, party
#define VOLVO_MAIN_BUS    0U  // Bus.main - VCU1 car side
#define VOLVO_PT_BUS      1U  // Bus.pt - VCU1 ECM side (where ECM is)
#define VOLVO_PARTY_BUS   2U  // Bus.party - VCU PSCM/BCM2 side (BCM2, SAS, EGSM, PSCM, where LCA is sent to)

static void volvo_rx_hook(const CANPacket_t *msg) {
  // Basic vehicle state monitoring - very relaxed implementation

  // Main bus (bus 0) messages
  if (msg->bus == VOLVO_MAIN_BUS) {
    // Gear position comes from main bus
    if (msg->addr == VOLVO_GEAR_POSITION) {
      // Signal: GEAR_POSITION (0: Park, 1: Reverse, 2: Neutral, 3: Drive)
      // This is used by carstate.py for gear shifter state
    }
  }

  // PT bus (bus 1) messages
  if (msg->bus == VOLVO_PT_BUS) {
    if (msg->addr == VOLVO_ECM_1) {
      // Gas pedal position - ACCELERATOR_PEDAL_POS
      // DBC: SG_ ACCELERATOR_PEDAL_POS : 31|8@0+ (1,0) [0|255]
      // carstate.py: > 20+1 (20 baseline + 1 tolerance)
      uint8_t gas_pedal_position = msg->data[3];
      gas_pressed = gas_pedal_position > 20+1; // Match carstate.py tolerance
    }

    // Update vehicle speed from BUS1_SPEED
    if (msg->addr == VOLVO_BUS1_SPEED) {
      // Signal: BUS1_SPEED (0.015625 m/s per bit)
      // DBC: SG_ BUS1_SPEED : 23|16@0+ (0.015625,0) [0|65535] "m/s" XXX
      uint16_t speed_raw = ((msg->data[2] & 0xFFU) << 8) | msg->data[3];
      vehicle_moving = speed_raw > 6; // > 0.09375 m/s (approx 0.1 m/s)
      UPDATE_VEHICLE_SPEED(speed_raw * 0.015625);
    }
  }

  // Party bus (bus 2) messages - BCM2, SAS, PSCM, EGSM
  if (msg->bus == VOLVO_PARTY_BUS) {

    // Update brake pedal and cruise state from BCM2
    if (msg->addr == VOLVO_BCM2) {
      // DBC: SG_ BRAKE_PEDAL_PRESSED_A : 47|1@0+ (-1,1) - inverted in DBC, so we invert raw bit
      // DBC: SG_ BRAKE_PEDAL_PRESSED_B : 46|1@0+ (1,0) - not inverted
      // carstate.py reads brake from cp_party (Bus.party)
      bool brake_a = !((msg->data[5] >> 7) & 1U); // Raw bit, active low (DBC inverts it)
      bool brake_b = (msg->data[5] >> 6) & 1U; // Raw bit, active high
      brake_pressed = brake_a || brake_b;

      // DBC: SG_ CRUISE_OR_PILOT_ASSIST_ENGAGED : 12|1@0+ (1,0)
      // carstate.py reads cruise state from cp (Bus.main) - but BCM2 is on party bus
      bool cruise_engaged = (msg->data[1] >> 4) & 1U;
      pcm_cruise_check(cruise_engaged);
    }

    // Update steering angle from SAS
    if (msg->addr == VOLVO_SAS) {
      // DBC: SG_ SAS_ANGLE_SENSOR : 6|15@0- (-0.05596,0)
      // carstate.py uses SAS (not PSCM) for steering angle
      // Bit position 6, 15 bits, signed, little endian
      int angle_raw = ((msg->data[0] & 0x7FU) << 8) | msg->data[1];
      if (msg->data[0] & 0x80U) {
        angle_raw = -angle_raw;
      }
      update_sample(&angle_meas, angle_raw);
    }

    // Update driver steering input from PSCM
    if (msg->addr == VOLVO_PSCM) {
      // DBC: SG_ DRIVER_INPUT_DEVIATION : 47|8@0- (1,0)
      // carstate.py uses abs(DRIVER_INPUT_DEVIATION) for steering torque and pressed detection
      // Bit position 47, 8 bits, signed
      int driver_input = msg->data[5];
      update_sample(&torque_driver, driver_input);
    }
  }
}

static bool volvo_tx_hook(const CANPacket_t *msg) {
  bool tx = true;

  // Very relaxed safety policy - only basic frame ID checks
  if (msg->addr == VOLVO_LCA_STEER) {
    // LCA message flows: VCU1 (main bus) -> PSCM (party bus)
    // We're acting as VCU1, so we send LCA message to party bus (bus 2)
    if (msg->bus != VOLVO_PARTY_BUS) {
      tx = false;  // Wrong bus
    }
  }

  return tx;
}

static safety_config volvo_init(uint16_t param) {
  SAFETY_UNUSED(param);

  // Define allowed TX messages - very permissive
  static const CanMsg VOLVO_TX_MSGS[] = {
    {VOLVO_LCA_STEER, VOLVO_PARTY_BUS, 8, .check_relay = true},  // LCA steering command to party bus
  };

  // Define RX checks - temporarily set to 1 Hz for development
  // TODO: Update to actual frequencies once CAN bus rates are confirmed
  static RxCheck volvo_rx_checks[] = {
    //{.msg = {{VOLVO_GEAR_POSITION, VOLVO_MAIN_BUS, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // TODO: 40 Hz
    {.msg = {{VOLVO_BUS1_SPEED, VOLVO_PT_BUS, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // TODO: 100 Hz
    //{.msg = {{VOLVO_BCM2, VOLVO_PARTY_BUS, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // TODO: 50 Hz
    {.msg = {{VOLVO_SAS, VOLVO_PARTY_BUS, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // TODO: 100 Hz
    {.msg = {{VOLVO_PSCM, VOLVO_PARTY_BUS, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // TODO: 100 Hz
    {.msg = {{VOLVO_ECM_1, VOLVO_PT_BUS, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // TODO: 17 Hz
  };

  return BUILD_SAFETY_CFG(volvo_rx_checks, VOLVO_TX_MSGS);
}

const safety_hooks volvo_hooks = {
  .init = volvo_init,
  .rx = volvo_rx_hook,
  .tx = volvo_tx_hook,
  // No custom fwd hook - stock LCA always blocked by .check_relay = true
};