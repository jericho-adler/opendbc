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
    self.msg_pscm = {
      'PSCM_ANGLE_SENSOR': 0,
      'BIT_0': 0,
      'BYTE_2': 0,
      'BYTE_3': 0,
      'BYTE_4': 0,
      'DRIVER_INPUT_DEVIATION': 0,
      'BYTE_6': 0,
      'BYTE_7': 0,
    }
    self.msg_driver_input = {
      'BYTE_0': 0,
      'BYTE_1': 0,
      'BYTE_2': 0,
      'STEERING_DRIVER_RATE_OF_CHANGE': 0,
      'BYTE_4': 0,
      'BYTE_5': 0,
      'STEERING_DRIVER_INPUT': 0,
      'BYTE_7': 0,
    }
    self.msg_lca = {
      'NEW_SIGNAL_3': 0,
      'NEW_SIGNAL_8': 0,
      'LCA_ENABLE_INV': 0,
      'NEW_SIGNAL_2': 0,
      'NEW_SIGNAL_1': 0,
      'LCA_STEER_LOOSELY_1': 0,
      'LCA_STEER_ACTIVE_INCOHERENT': 0,
      'LCA_STEER_ACTIVE': 0,
      'NEW_SIGNAL_7': 0,
      'NEW_SIGNAL_9': 0,
      'LCA_STEER_LOOSELY_2': 0,
      'NEW_SIGNAL_4': 0,
      'CURVE_RIGHT': 0,
      'NEW_SIGNAL_5': 0,
      'LCA_STEER': 0,
      'NEW_SIGNAL_10': 0,
      'NEW_SIGNAL_6': 0,
    }
  def update(self, can_parsers) -> structs.CarState:
    cp_main = can_parsers[Bus.main]
    cp_pt = can_parsers[Bus.pt]
    cp_party = can_parsers[Bus.party]
    ret = structs.CarState()

    # car speed
    # Basic vehicle state from BUS1_SPEED on PT bus
    ret.vEgoRaw = cp_pt.vl["BUS1_SPEED"]["BUS1_SPEED"]
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    # gas
    ret.gasPressed = cp_pt.vl["ECM_1"]["ACCELERATOR_PEDAL_POS"] > 20+1 # 20 baseline + 1 tolerance

    # brake
    ret.brakePressed = bool(cp_main.vl["VCU1"]["BRAKE_PEDAL_PRESSED_A"] or cp_main.vl["VCU1"]["BRAKE_PEDAL_PRESSED_B"])
    # BRAKE_PEDAL_PRESSED_A goes active when user starts pressing brake pedal, but no brake light is on yet due to tolerance
    # BRAKE_PEDAL_PRESSED_B goes active when when the brake pedal is pressed above minimum threshold, brake light is on
    ret.parkingBrake = False # TODO: add parking brake

    # steering wheel
    #ret.steeringAngleDeg = cp_party.vl['PSCM']['PSCM_ANGLE_SENSOR']
    ret.steeringAngleDeg = cp_party.vl['SAS']['SAS_ANGLE_SENSOR']

    # For torque-based control, we need steering torque feedback
    # TODO: Find actual steering torque signals in the DBC or reverse engineer them
    ret.steeringTorque = abs(cp_party.vl['PSCM']['DRIVER_INPUT_DEVIATION'])  # Driver torque
    #ret.steeringTorqueEps = 0  # EPS torque - placeholder until signal is found
    ret.steeringPressed = abs(cp_party.vl['PSCM']['DRIVER_INPUT_DEVIATION']) > CarControllerParams.STEER_DRIVER_ALLOWANCE

    # EPS status - placeholder until actual signal is found
    self.eps_active = True  # Assume EPS is active for now

    # cruise
    # Cruise control / Pilot Assist status from BCM2
    ret.cruiseState.enabled = cp_main.vl["VCU1"]["CRUISE_OR_PILOT_ASSIST_ENGAGED"] == 1
    ret.cruiseState.available = True  # TODO: Determine actual availability
    ret.cruiseState.speed = 0  # TODO: Find cruise set speed (not required for lateral control)
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.standstill = ret.standstill # False # Todo: Find cruise control standstill signal

    # gear TODO
    #if bool(cp_cam.vl['Dat_BSI']['P103_Com_bRevGear']):
    #  ret.gearShifter = GearShifter.reverse
    #else:
    #  ret.gearShifter = GearShifter.drive
    gearPosition = cp_main.vl['GEAR_POSITION']['GEAR_POSITION'] # 0: Parked; 1: R; 2: N; 3: D (using bus 2)
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

    self.msg_pscm['PSCM_ANGLE_SENSOR'] = cp_party.vl['PSCM']['PSCM_ANGLE_SENSOR']
    self.msg_pscm['BIT_0'] = cp_party.vl['PSCM']['BIT_0']
    self.msg_pscm['BYTE_2'] = cp_party.vl['PSCM']['BYTE_2']
    self.msg_pscm['BYTE_3'] = cp_party.vl['PSCM']['BYTE_3']
    self.msg_pscm['BYTE_4'] = cp_party.vl['PSCM']['BYTE_4']
    self.msg_pscm['DRIVER_INPUT_DEVIATION'] = cp_party.vl['PSCM']['DRIVER_INPUT_DEVIATION']
    self.msg_pscm['BYTE_6'] = cp_party.vl['PSCM']['BYTE_6']
    self.msg_pscm['BYTE_7'] = cp_party.vl['PSCM']['BYTE_7']

    self.msg_driver_input['BYTE_0'] = cp_party.vl['DRIVER_INPUT']['BYTE_0']
    self.msg_driver_input['BYTE_1'] = cp_party.vl['DRIVER_INPUT']['BYTE_1']
    self.msg_driver_input['BYTE_2'] = cp_party.vl['DRIVER_INPUT']['BYTE_2']
    self.msg_driver_input['STEERING_DRIVER_RATE_OF_CHANGE'] = cp_party.vl['DRIVER_INPUT']['STEERING_DRIVER_RATE_OF_CHANGE']
    self.msg_driver_input['BYTE_4'] = cp_party.vl['DRIVER_INPUT']['BYTE_4']
    self.msg_driver_input['BYTE_5'] = cp_party.vl['DRIVER_INPUT']['BYTE_5']
    self.msg_driver_input['STEERING_DRIVER_INPUT'] = cp_party.vl['DRIVER_INPUT']['STEERING_DRIVER_INPUT']
    self.msg_driver_input['BYTE_7'] = cp_party.vl['DRIVER_INPUT']['BYTE_7']

    self.msg_lca['NEW_SIGNAL_3'] = cp_main.vl['LCA']['NEW_SIGNAL_3']
    self.msg_lca['NEW_SIGNAL_8'] = cp_main.vl['LCA']['NEW_SIGNAL_8']
    self.msg_lca['LCA_ENABLE_INV'] = cp_main.vl['LCA']['LCA_ENABLE_INV']
    self.msg_lca['NEW_SIGNAL_2'] = cp_main.vl['LCA']['NEW_SIGNAL_2']
    self.msg_lca['NEW_SIGNAL_1'] = cp_main.vl['LCA']['NEW_SIGNAL_1']
    self.msg_lca['LCA_STEER_LOOSELY_1'] = cp_main.vl['LCA']['LCA_STEER_LOOSELY_1']
    self.msg_lca['LCA_STEER_ACTIVE_INCOHERENT'] = cp_main.vl['LCA']['LCA_STEER_ACTIVE_INCOHERENT']
    self.msg_lca['LCA_STEER_ACTIVE'] = cp_main.vl['LCA']['LCA_STEER_ACTIVE']
    self.msg_lca['NEW_SIGNAL_7'] = cp_main.vl['LCA']['NEW_SIGNAL_7']
    self.msg_lca['NEW_SIGNAL_9'] = cp_main.vl['LCA']['NEW_SIGNAL_9']
    self.msg_lca['LCA_STEER_LOOSELY_2'] = cp_main.vl['LCA']['LCA_STEER_LOOSELY_2']
    self.msg_lca['NEW_SIGNAL_4'] = cp_main.vl['LCA']['NEW_SIGNAL_4']
    self.msg_lca['CURVE_RIGHT'] = cp_main.vl['LCA']['CURVE_RIGHT']
    self.msg_lca['NEW_SIGNAL_5'] = cp_main.vl['LCA']['NEW_SIGNAL_5']
    self.msg_lca['LCA_STEER'] = cp_main.vl['LCA']['LCA_STEER']
    self.msg_lca['NEW_SIGNAL_10'] = cp_main.vl['LCA']['NEW_SIGNAL_10']
    self.msg_lca['NEW_SIGNAL_6'] = cp_main.vl['LCA']['NEW_SIGNAL_6']

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.main: CANParser(DBC[CP.carFingerprint][Bus.main], [], 0),
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
      Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], 2),
    }
