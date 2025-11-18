from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo.volvocan import create_lca_steering, create_pscm_message, create_lca_3_message, create_lca_2_message, create_speed_1_message, create_speed_2_message, create_speed_3_message
from opendbc.car.volvo.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.party])
    self.apply_torque_last = 0

    # LCA_3 timer state (218 kHz timers)
    self.lca_3_timer_1 = 0
    self.lca_3_timer_2 = 0
    self.lca_3_timer_1_initialized = False
    self.lca_3_timer_2_initialized = False

  def update(self, CC, CS, now_nanos):
    CS.CC_frame = self.frame
    can_sends = []
    actuators = CC.actuators

    # lateral control - torque-based steering
    # NOTE: LCA message is sent every frame (even when inactive) to replace stock LCA
    # Stock LCA is permanently blocked by panda safety, so we must always send
    if self.frame % CarControllerParams.STEER_STEP == 0: # 100 Hz
      # Convert normalized torque to raw torque value
      apply_torque = int(round(actuators.torque * CarControllerParams.STEER_MAX))

      # Apply driver torque limits
      # apply_torque = apply_driver_steer_torque_limits(apply_torque, self.apply_torque_last,
      #                                                CS.out.steeringTorque, CarControllerParams)

      # Disable torque when not active
      if not CC.latActive:
        apply_torque = 0

      # LCA - 0x58 - 100 Hz
      can_sends.append(create_lca_steering(self.packer, CC.latActive, apply_torque, CS.msg_lca))
      self.apply_torque_last = apply_torque

      # Check if PA hands-on-wheel spoof toggle is enabled (bit 7 of alternativeExperience)
      spoof_pa_hands_enabled = bool(self.CP.alternativeExperience & 128)
      spoof_pa_hands = CS.pilot_assist_engaged and spoof_pa_hands_enabled
      # PSCM - 0x16 - 100 Hz
      can_sends.append(create_pscm_message(self.packer, CC.latActive, CS.msg_pscm, self.frame, spoof_pa_hands))

    # SPEED messages - 0x60, 0x67, 0x68 - 50 Hz
    # Forward speed messages to bus 2 before LCA_2
    if self.frame % 2 == 0: # 50 Hz
      can_sends.append(create_speed_3_message(self.packer, CS.msg_speed_3))
      can_sends.append(create_speed_1_message(self.packer, CS.msg_speed_1))
      can_sends.append(create_speed_2_message(self.packer, CS.msg_speed_2))

    # LCA_2 - 0x69 - 50 Hz
    # Spoof PILOT_ASSIST_ENGAGED to keep PSCM accepting LCA commands
    if self.frame % 2 == 0: # 50 Hz
      can_sends.append(create_lca_2_message(self.packer, CC.latActive, CS.msg_lca_2))
      #self.lca_2_counter_1_prev = int(CS.msg_lca_2['COUNTER_1'])
      pass

    # LCA_3 - 0x57 - avg 66.66 Hz
    #if (self.frame * 67) % 100 < 67: # if (self.frame % 3) < 2:
    # 0x57 at ~66.67 Hz: send on 2 out of every 3 frames
    # Pattern: send on frame % 3 == 0 or 2, skip when frame % 3 == 1
    if self.frame % 3 != 1:  # → 2/3 * 100 Hz = 66.67 Hz
      can_sends.append(create_lca_3_message(self.packer, CC.latActive, apply_torque, CS.msg_lca_3))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    self.frame += 1
    return new_actuators, can_sends
