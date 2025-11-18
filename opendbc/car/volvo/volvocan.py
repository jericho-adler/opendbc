import random
from opendbc.car.volvo.helpers import checksum_lca_2_message, checksum_2_0x69_message
from opendbc.car.carlog import carlog

def create_lca_steering(packer, lat_active: bool, apply_torque: int, msg_lca: dict):
  """
  Create LCA (Lane Centering Assist) steering command for Volvo CMA platform.
  Uses torque-based control via the LCA_STEER signal.

  NOTE: This message must be sent continuously (even when inactive) because
  stock LCA is permanently blocked by panda safety. When lat_active=False,
  we send a safe/inactive LCA message to maintain PSCM communication.

  Args:
    packer: CAN packer instance
    lat_active: Whether lateral control is active
    apply_torque: Steering torque to apply (-255 to 255)
    msg_lca: Dictionary containing LCA message values
  """
  if not lat_active:
    return packer.make_can_msg('LCA', 2, msg_lca)

  baseline_loosely_1 = 102 # Standard straight road or light right turn
  baseline_loosely_2 = 154 # Standard straight road or light right turn

  # In openpilot, a positive actuators.torque value corresponds to a LEFT turn.
  # In Volvo, a positive LCA_STEER value corresponds to a LEFT turn.
  lca_steer = apply_torque

  loosely_1 = baseline_loosely_1
  loosely_2 = baseline_loosely_2

  curve_right = 0
  if lca_steer < 0:
    curve_right = 63
  elif lca_steer > 0:
    curve_right = 0

  values = {
    'NEW_SIGNAL_3': 2,
    'LCA_ENABLE_INV': 0 if lat_active else 1,
    'NEW_SIGNAL_1': 3,
    'LCA_STEER_LOOSELY_1': loosely_1 if lat_active else 0,
    'LCA_STEER_ACTIVE_INCOHERENT': 1 if lat_active else 0,
    'LCA_STEER_ACTIVE': 3 if lat_active else 0,
    'NEW_SIGNAL_7': 7,
    'LCA_STEER_LOOSELY_2': loosely_2 if lat_active else 0,
    'NEW_SIGNAL_4': 39 if lat_active else 251, # Stock LCA increased from 35 to 39 steppedly when steering request was overriden by openpilot that couldn't steer enough
    'CURVE_RIGHT': curve_right,
    'NEW_SIGNAL_5': 3,
    'LCA_STEER': apply_torque if lat_active else 0,
    'NEW_SIGNAL_6': 15, # ?
  }

  """if not lat_active:
    values = {
      'NEW_SIGNAL_3': 0,
      'LCA_ENABLE_INV': 1,
      'NEW_SIGNAL_1': 3,
      'LCA_STEER_LOOSELY_1': 0,
      'LCA_STEER_ACTIVE_INCOHERENT': 0,
      'LCA_STEER_ACTIVE': 0,
      'NEW_SIGNAL_7': 7,
      'LCA_STEER_LOOSELY_2': 0,
      'NEW_SIGNAL_4': 251,
      'CURVE_RIGHT': 0,
      'NEW_SIGNAL_5': 3,
      'LCA_STEER': 0,
      'NEW_SIGNAL_6': 15,
    }"""

  return packer.make_can_msg('LCA', 2, values)

def create_pscm_message(packer, lat_active: bool, msg_pscm: dict, frame: int, spoof_pa_hands_on_wheel: bool):
  values = {
    'PSCM_ANGLE_SENSOR': msg_pscm['PSCM_ANGLE_SENSOR'],
    'BIT_0': msg_pscm['BIT_0'],
    'HANDS_ON_STEERING_WHEEL_A': msg_pscm['HANDS_ON_STEERING_WHEEL_A'],
    'HANDS_ON_STEERING_WHEEL_B': msg_pscm['HANDS_ON_STEERING_WHEEL_B'],
    'BYTE_4': msg_pscm['BYTE_4'],
    'DRIVER_INPUT_DEVIATION': msg_pscm['DRIVER_INPUT_DEVIATION'],
    'BYTE_6': msg_pscm['BYTE_6'],
    'BYTE_7': msg_pscm['BYTE_7'],
  }

  # Spoof hands on wheel when:
  # - lat_active (openpilot is steering), OR
  # - spoof_pa_hands_on_wheel (Pilot Assist is engaged AND toggle enabled)
  if lat_active or spoof_pa_hands_on_wheel:
    #values['DRIVER_INPUT_DEVIATION'] = -1 # Spoof hands on steering wheel
    values['DRIVER_INPUT_DEVIATION'] = 1 if frame % 2 == 0 else 0
    values['HANDS_ON_STEERING_WHEEL_B'] = 186 if frame % 2 == 0 else 154 # msg_pscm['HANDS_ON_STEERING_WHEEL_B']
    values['HANDS_ON_STEERING_WHEEL_A'] = 195 if frame % 2 == 0 else 249 # msg_pscm['HANDS_ON_STEERING_WHEEL_A']

  return packer.make_can_msg('PSCM', 0, values)

def create_lca_3_message(packer, lat_active: bool, apply_torque: int, msg_lca_3: dict):
  """
  Create LCA_3 message for Volvo CMA platform.
  This message enables PSCM to accept LCA commands.

  Args:
    packer: CAN packer instance
    lat_active: Whether lateral control is active
    msg_lca_3: Dictionary containing LCA_3 message values
  """
  signal_9 = 128 if lat_active else msg_lca_3['NEW_SIGNAL_9']
  if lat_active:
    if apply_torque > 0: # Left turn
      signal_9 = 255
    elif apply_torque < 0: # Right turn
      signal_9 = 0
  values = {
    'NEW_SIGNAL_3': 0 if lat_active else msg_lca_3['NEW_SIGNAL_3'],
    'LCA_ACCEPT_COMMANDS_RELATED': 15 if lat_active else msg_lca_3['LCA_ACCEPT_COMMANDS_RELATED'],
    'NEW_SIGNAL_2': 0 if lat_active else msg_lca_3['NEW_SIGNAL_2'],
    'NEW_SIGNAL_5': 30 if lat_active else msg_lca_3['NEW_SIGNAL_5'],
    'LCA_ACCEPT_COMMANDS_INV': 0 if lat_active else msg_lca_3['LCA_ACCEPT_COMMANDS_INV'],
    'NEW_SIGNAL_4': 3 if lat_active else msg_lca_3['NEW_SIGNAL_4'],
    'SPEED_A': msg_lca_3['SPEED_A'],
    'SPEED_B': msg_lca_3['SPEED_B'],
    'NEW_SIGNAL_8': 1 if lat_active else msg_lca_3['NEW_SIGNAL_8'],
    'COUNTER_1': msg_lca_3['COUNTER_1'],
    'NEW_SIGNAL_7': 3 if lat_active else msg_lca_3['NEW_SIGNAL_7'],
    'NEW_SIGNAL_9': signal_9,
  }

  return packer.make_can_msg('LCA_3', 2, values)

def diff_dicts(a, b):
  only_in_a = a.keys() - b.keys()
  only_in_b = b.keys() - a.keys()
  in_both = a.keys() & b.keys()

  changed = {k: (a[k], b[k]) for k in in_both if a[k] != b[k]}

  return {
    "only_in_a": {k: a[k] for k in only_in_a},
    "only_in_b": {k: b[k] for k in only_in_b},
    "changed": changed,
  }

def create_lca_2_message(packer, lat_active: bool, msg_lca_2: dict):
  """
  Create LCA_2 message to spoof PILOT_ASSIST_ENGAGED when openpilot is active.

  When lat_active=True, we set PILOT_ASSIST_ENGAGED=1 to make PSCM accept LCA commands,
  even if the driver has disabled stock Pilot Assist.

  Args:
    packer: CAN packer instance
    lat_active: Whether lateral control is active
    msg_lca_2: Dictionary containing LCA_2 message values from car
  """
  #if not lat_active:
  #  return packer.make_can_msg('LCA_2', 2, msg_lca_2)

  values = {
    'BYTE_0': 24 if lat_active else msg_lca_2['BYTE_0'], # 24 always
    'COUNTER_1': msg_lca_2['COUNTER_1'], # Byte 1 Low Nibble [5:8] - 4-bit counter that increments by +2 (modulo 16)
    'PILOT_ASSIST_ENGAGED': 1 if lat_active else msg_lca_2['PILOT_ASSIST_ENGAGED'], # Byte 1 [4]
    'BYTE_1_MSBS_3': msg_lca_2['BYTE_1_MSBS_3'], # Byte 1 [0:3]
    'CHECKSUM_2': msg_lca_2['CHECKSUM_2'], # Checksum on bytes 0 and 1
    'NEW_SIGNAL_2': 0 if lat_active else msg_lca_2['NEW_SIGNAL_2'],
    'COUNTER_2': msg_lca_2['COUNTER_2'], # Byte 5 Low Nibble - 4-bit counter that increments by +4 (modulo 16)
    'NEW_SIGNAL_3': 3 if lat_active else msg_lca_2['NEW_SIGNAL_3'],
    'BRAKE_PEDAL_PRESSED_B': msg_lca_2['BRAKE_PEDAL_PRESSED_B'],
    'BRAKE_PEDAL_PRESSED_A': msg_lca_2['BRAKE_PEDAL_PRESSED_A'],
    'CHECKSUM': msg_lca_2['CHECKSUM'], # Byte 6 is a checksum based on Bytes 1, 2, and 5 only
    'BYTE_7': 0 if lat_active else msg_lca_2['BYTE_7'],
  }

  # Reconstruct bytes 1, 2, and 5 from signal values for checksum calculation
  b1 = ((int(values['COUNTER_1']) & 0x0F) |
        ((int(values['PILOT_ASSIST_ENGAGED']) & 0x01) << 4) |
        ((int(values['BYTE_1_MSBS_3']) & 0x07) << 5))
  b2 = int(values['CHECKSUM_2']) & 0xFF
  # Note: BRAKE_PEDAL_PRESSED_A has scale=-1, offset=1 in DBC, so we need to invert:
  # raw = (physical - offset) / scale = (physical - 1) / -1
  brake_pedal_a_raw = int((values['BRAKE_PEDAL_PRESSED_A'] - 1) / -1)
  b5 = ((int(values['COUNTER_2']) & 0x0F) |
        ((int(values['NEW_SIGNAL_3']) & 0x03) << 4) |
        ((int(values['BRAKE_PEDAL_PRESSED_B']) & 0x01) << 6) |
        ((brake_pedal_a_raw & 0x01) << 7))

  b0 = int(values['BYTE_0']) # Used for checksum 1 and 2

  values['CHECKSUM'] = checksum_lca_2_message(b0, b5)

  # Only validate when not active and message is valid (BYTE_0 should be 24, not 0)
  if not lat_active:
    #assert values['CHECKSUM'] == msg_lca_2['CHECKSUM']
    if values['CHECKSUM'] != msg_lca_2['CHECKSUM']:
      carlog.warning("[volvocan.py] LCA_2 CHECKSUM mismatch")
      print(f"b0={b0}, b1={b1}, b2={b2}, b5={b5}, calculated={values['CHECKSUM']}, expected={msg_lca_2['CHECKSUM']}")
      #assert False

  # Checksum 2
  b1 = (int(values['BYTE_1_MSBS_3']) & 0b111) << 5 | (int(values['PILOT_ASSIST_ENGAGED']) & 0b1) << 4 | (int(values['COUNTER_1']) & 0b1111)
  checksum_2 = checksum_2_0x69_message(b0, b1)
  values['CHECKSUM_2'] = checksum_2
  if not lat_active:
    if values['CHECKSUM_2'] != msg_lca_2['CHECKSUM_2']:
      carlog.warning("[volvocan.py] LCA_2 CHECKSUM_2 mismatch")
      print(f"b0={b0}, b1={b1}, calculated={values['CHECKSUM_2']}, expected={msg_lca_2['CHECKSUM_2']}")
      #assert False
  return packer.make_can_msg('LCA_2', 2, values)

def create_speed_1_message(packer, msg_speed_1: dict):
  """
  Forward SPEED_1 message (0x67) by copying all bytes.

  Args:
    packer: CAN packer instance
    msg_speed_1: Dictionary containing SPEED_1 message values from car
  """
  values = {
    'ALL_BYTES': msg_speed_1['ALL_BYTES'],
  }

  return packer.make_can_msg('SPEED_1', 2, values)

def create_speed_2_message(packer, msg_speed_2: dict):
  """
  Forward SPEED_2 message (0x68) by copying all bytes.

  Args:
    packer: CAN packer instance
    msg_speed_2: Dictionary containing SPEED_2 message values from car
  """
  values = {
    'ALL_BYTES': msg_speed_2['ALL_BYTES'],
  }

  return packer.make_can_msg('SPEED_2', 2, values)

def create_speed_3_message(packer, msg_speed_3: dict):
  """
  Forward SPEED_3 message (0x60) by copying all bytes.

  Args:
    packer: CAN packer instance
    msg_speed_3: Dictionary containing SPEED_3 message values from car
  """
  values = {
    'ALL_BYTES': msg_speed_3['ALL_BYTES'],
  }

  return packer.make_can_msg('SPEED_3', 2, values)