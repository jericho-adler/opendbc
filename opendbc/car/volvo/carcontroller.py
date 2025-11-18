from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo.volvocan import create_lca_steering, create_pscm_message, create_lca_3_message, create_lca_2_message, create_speed_1_message, create_speed_2_message, create_speed_3_message, create_0x1a_message, create_gear_position_message, create_egsm_message
from opendbc.car.volvo.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.party])
    self.apply_torque_last = 0

    self.gear_acc = 60

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
      # EGSM - 0x45 - 100 Hz
      #can_sends.append(create_egsm_message(self.packer, CS.msg_egsm))

    # LCA_3 - 0x57 - avg 66.66 Hz
    #if (self.frame * 67) % 100 < 67: # if (self.frame % 3) < 2:
    # 0x57 at ~66.67 Hz: send on 2 out of every 3 frames
    # Pattern: send on frame % 3 == 0 or 2, skip when frame % 3 == 1
    if self.frame % 3 != 1:  # → 2/3 * 100 Hz = 66.67 Hz
      can_sends.append(create_lca_3_message(self.packer, CC.latActive, apply_torque, CS.msg_lca_3))
      #can_sends.append(create_0x1a_message(self.packer, CS.msg_0x1a))
      pass

    # SPEED messages - 0x60, 0x67, 0x68 - 50 Hz
    if self.frame % 2 == 0: # 50 Hz
      #can_sends.append(create_speed_3_message(self.packer, CS.msg_speed_3))
      #can_sends.append(create_speed_1_message(self.packer, CS.msg_speed_1))
      #can_sends.append(create_speed_2_message(self.packer, CS.msg_speed_2))
      pass

    # LCA_2 - 0x69 - 50 Hz
    # Spoof PILOT_ASSIST_ENGAGED to keep PSCM accepting LCA commands
    if self.frame % 2 == 0: # 50 Hz
      can_sends.append(create_lca_2_message(self.packer, CC.latActive, CS.msg_lca_2))
      pass

    # GEAR_POSITION - 0x80 - 40 Hz
    #self.gear_acc += 40 # Bresenham-style approach
    #if self.gear_acc >= 100:
    #    self.gear_acc -= 100
    if self.frame % 5 == 0 or self.frame % 5 == 2:  # 2/5 * 100 Hz = 40 Hz
      can_sends.append(create_gear_position_message(self.packer, CS.msg_gear_position))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    self.frame += 1
    return new_actuators, can_sends
