from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo.volvocan import create_lca_steering, create_pscm_message, create_vcu1_pscm_control, create_vcu1_message
from opendbc.car.volvo.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.party])
    self.apply_torque_last = 0

    # VCU1_PSCM_CONTROL timer state (218 kHz timers)
    self.vcu1_pscm_control_timer_1 = 0
    self.vcu1_pscm_control_timer_2 = 0
    self.vcu1_pscm_control_timer_initialized = False

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

      can_sends.append(create_pscm_message(self.packer, CC.latActive, CS.msg_pscm, self.frame, CS.pilot_assist_engaged))

    # VCU1 message at 50 Hz (send every other frame = 50 Hz)
    # Spoof PILOT_ASSIST_ENGAGED to keep PSCM accepting LCA commands
    if self.frame % 2 == 0:
      can_sends.append(create_vcu1_message(self.packer, CC.latActive, CS.msg_vcu1))
      pass

    # VCU1_PSCM_CONTROL message at 67 Hz (send 2 out of every 3 frames = 66.67 Hz)
    if (self.frame % 3) < 2:
      # Initialize timers from RX when both values are valid (> 0)
      if not self.vcu1_pscm_control_timer_initialized:
        timer_1_rx = CS.msg_vcu1_pscm_control['TIMER_1']
        timer_2_rx = CS.msg_vcu1_pscm_control['TIMER_2']
        if True: # if timer_1_rx > 0 and timer_2_rx > 0:
          self.vcu1_pscm_control_timer_1 = int(timer_1_rx)
          self.vcu1_pscm_control_timer_2 = int(timer_2_rx)
          self.vcu1_pscm_control_timer_initialized = True

      # Only send message after timers are initialized
      if self.vcu1_pscm_control_timer_initialized:
        # Send message with current timer values
        can_sends.append(create_vcu1_pscm_control(self.packer, CC.latActive, apply_torque, CS.msg_vcu1_pscm_control,
                                                  self.vcu1_pscm_control_timer_1, self.vcu1_pscm_control_timer_2))

        # Increment timers for next message
        # At 67 Hz, each message is ~15ms apart
        # 218000 Hz * 0.015s ≈ 3270 counts per message
        TIMER_INCREMENT = 3270
        self.vcu1_pscm_control_timer_1 = (self.vcu1_pscm_control_timer_1 + TIMER_INCREMENT) & 0xFFFF  # 16-bit wraparound
        self.vcu1_pscm_control_timer_2 = (self.vcu1_pscm_control_timer_2 + TIMER_INCREMENT) & 0xFFFF  # 16-bit wraparound

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    self.frame += 1
    return new_actuators, can_sends
