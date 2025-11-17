from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo.volvocan import create_lca_steering, create_pscm_message, create_lca_3_control, create_lca_2_message
from opendbc.car.volvo.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.party])
    self.apply_torque_last = 0

    # LCA_3 timer state (218 kHz timers)
    self.lca_3_timer_1 = 0
    self.lca_3_timer_2 = 0
    self.lca_3_timer_initialized = False
    self.lca_3_timer_1_prev = 0

    self.lca_2_counter_1_prev = 0

  def update(self, CC, CS, now_nanos):
    CS.CC_frame = self.frame
    can_sends = []
    actuators = CC.actuators

    # lateral control - torque-based steering
    # NOTE: LCA message is sent every frame (even when inactive) to replace stock LCA
    # Stock LCA is permanently blocked by panda safety, so we must always send
    if self.frame % CarControllerParams.STEER_STEP == 0:
      # Convert normalized torque to raw torque value
      apply_torque = int(round(actuators.torque * CarControllerParams.STEER_MAX))

      # Apply driver torque limits
      # apply_torque = apply_driver_steer_torque_limits(apply_torque, self.apply_torque_last,
      #                                                CS.out.steeringTorque, CarControllerParams)

      # Disable torque when not active
      if not CC.latActive:
        apply_torque = 0

      can_sends.append(create_lca_steering(self.packer, CC.latActive, apply_torque, CS.msg_lca))
      self.apply_torque_last = apply_torque

      # Check if PA hands-on-wheel spoof toggle is enabled (bit 1 of alternativeExperience)
      spoof_pa_hands_enabled = bool(self.CP.alternativeExperience & 2)
      spoof_pa_hands = CS.pilot_assist_engaged and spoof_pa_hands_enabled
      can_sends.append(create_pscm_message(self.packer, CC.latActive, CS.msg_pscm, self.frame, spoof_pa_hands))

    # LCA_2 message at 50 Hz (send every other frame = 50 Hz)
    # Spoof PILOT_ASSIST_ENGAGED to keep PSCM accepting LCA commands
    #if self.frame % 2 == 0:
    if int(CS.msg_lca_2['COUNTER_1']) != int(self.lca_2_counter_1_prev): # PSCM is really strict on timing, send message as soon as the counter changes
      can_sends.append(create_lca_2_message(self.packer, CC.latActive, CS.msg_lca_2))
      self.lca_2_counter_1_prev = int(CS.msg_lca_2['COUNTER_1'])
      pass

    # LCA_3 message at 67 Hz
    """
    if (self.frame * 67) % 100 < 67: # if (self.frame % 3) < 2:
      # Initialize timers from RX when both values are valid (> 0)
      if not self.lca_3_timer_initialized:
        timer_1_rx = CS.msg_lca_3['TIMER_1']
        timer_2_rx = CS.msg_lca_3['TIMER_2']
        if True: # if timer_1_rx > 0 and timer_2_rx > 0:
          self.lca_3_timer_1 = int(timer_1_rx)
          self.lca_3_timer_2 = int(timer_2_rx)
          self.lca_3_timer_initialized = True

      # Only send message after timers are initialized
      if self.lca_3_timer_initialized:
        # Send message with current timer values
        can_sends.append(create_lca_3_control(self.packer, CC.latActive, apply_torque, CS.msg_lca_3, self.lca_3_timer_1, self.lca_3_timer_2))

        # Increment timers for next message
        # At 67 Hz, each message is ~15ms apart
        # 218000 Hz * 0.015s ≈ 3270 counts per message
        TIMER_INCREMENT = 3270
        self.lca_3_timer_1 = (self.lca_3_timer_1 + TIMER_INCREMENT) & 0xFFFF  # 16-bit wraparound
        self.lca_3_timer_2 = (self.lca_3_timer_2 + TIMER_INCREMENT) & 0xFFFF  # 16-bit wraparound
    """
    # 67 Hz - send message as soon as the timer changes
    if int(CS.msg_lca_3['TIMER_1']) != int(self.lca_3_timer_1_prev):
      self.lca_3_timer_1_prev = int(CS.msg_lca_3['TIMER_1'])
      can_sends.append(create_lca_3_control(self.packer, CC.latActive, apply_torque, CS.msg_lca_3, 0, 0))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    self.frame += 1
    return new_actuators, can_sends
