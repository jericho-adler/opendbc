from opendbc.car import structs, Bus
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volvo.values import DBC, CarControllerParams
from opendbc.car.interfaces import CarStateBase

GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.cruise_enabled_prev = False
    self.cruise_last_disabled_frame = 0
    self.cruise_double_tap_active = False
    self.CC_frame = 0 # CarController frame
  def update(self, can_parsers) -> structs.CarState:
    cp_main = can_parsers[Bus.main]
    cp_pt = can_parsers[Bus.pt]
    cp_party = can_parsers[Bus.party]
    ret = structs.CarState()

    # car speed
    # Basic vehicle state from BUS1_SPEED on PT bus
    ret.vEgoRaw = cp_pt.vl["BUS1_SPEED"]["BUS1_SPEED"]
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw <= 0.09375

    # gas
    ret.gasPressed = cp_pt.vl["ECM_1"]["ACCELERATOR_PEDAL_POS"] > 20+1 # 20 baseline + 1 tolerance

    # brake
    #ret.brakePressed = bool(cp_main.vl["LCA_2"]["BRAKE_PEDAL_PRESSED_A"] or cp_main.vl["LCA_2"]["BRAKE_PEDAL_PRESSED_B"])
    # BRAKE_PEDAL_PRESSED_A goes active when user starts pressing brake pedal, but no brake light is on yet due to tolerance
    # BRAKE_PEDAL_PRESSED_B goes active when when the brake pedal is pressed above minimum threshold, brake light is on
    ret.brakePressed = cp_main.vl["LCA_2"]["BRAKE_PEDAL_PRESSED_B"] == 1
    ret.parkingBrake = False # TODO: add parking brake

    # steering wheel
    ret.steeringAngleDeg = -cp_party.vl['PSCM']['PSCM_ANGLE_SENSOR'] # openpilot expects a negative value for a right turn
    #ret.steeringAngleDeg = cp_party.vl['SAS']['SAS_ANGLE_SENSOR']

    # For torque-based control, we need steering torque feedback
    # TODO: Find actual steering torque signals in the DBC or reverse engineer them
    ret.steeringTorque = -cp_party.vl['DRIVER_INPUT']['STEERING_DRIVER_INPUT']  # Driver torque (car right turn is negative, openpilot right turn is positive)
    #ret.steeringTorqueEps = 0  # EPS torque - placeholder until signal is found
    ret.steeringPressed = abs(cp_party.vl['DRIVER_INPUT']['STEERING_DRIVER_INPUT']) > 2

    # EPS status - placeholder until actual signal is found
    self.eps_active = True  # Assume EPS is active for now

    # cruise - double-tap detection (on-off-on within 500ms/50 frames / 1000ms/100 frames)
    cruise_raw = cp_pt.vl["BUS1_CRUISE_CONTROL"]["CRUISE_CONTROL_ENABLED"] == 1

    # Detect on-off-on double-tap pattern
    if cruise_raw and not self.cruise_enabled_prev:
      # Just turned ON - check if we turned OFF recently (within 100 frames = 1000ms)
      if self.CC_frame - self.cruise_last_disabled_frame <= 100:
        self.cruise_double_tap_active = True
    elif not cruise_raw and self.cruise_enabled_prev:
      # Just turned OFF
      self.cruise_last_disabled_frame = self.CC_frame
      self.cruise_double_tap_active = False

    ret.cruiseState.enabled = cruise_raw and self.cruise_double_tap_active
    self.cruise_enabled_prev = cruise_raw

    #ret.cruiseState.enabled = cruise_raw and not ret.gasPressed # No more double-tap detection, uncomment if needed
    ret.cruiseState.available = True  # TODO: Determine actual availability
    ret.cruiseState.speed = 0  # TODO: Find cruise set speed (not required for lateral control)
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.standstill = ret.standstill # False # Todo: Find cruise control standstill signal

    # gear
    gearPosition = cp_main.vl['GEAR_POSITION']['GEAR_POSITION'] # 0: Parked; 1: R; 2: N; 3: D
    if gearPosition == 0:
      ret.gearShifter = GearShifter.park
    elif gearPosition == 1:
      ret.gearShifter = GearShifter.reverse
    elif gearPosition == 2:
      ret.gearShifter = GearShifter.neutral
    elif gearPosition == 3:
      ret.gearShifter = GearShifter.drive

    # blinkers TODO FlexRay
    ret.leftBlinker = False
    ret.rightBlinker = False

    # lock info TODO FlexRay
    ret.doorOpen = False # TODO: add door open
    ret.seatbeltUnlatched = False # TODO: add seatbelt unlatched

    # Store entire message dictionaries
    self.msg_pscm = cp_party.vl['PSCM']
    self.msg_lca = cp_main.vl['LCA']
    self.msg_lca_2 = cp_main.vl['LCA_2']
    self.msg_lca_3 = cp_main.vl['LCA_3']
    self.pilot_assist_engaged = cp_main.vl['LCA_2']['PILOT_ASSIST_ENGAGED'] == 1

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.main], [], 0),
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
      Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], 2),
    }
