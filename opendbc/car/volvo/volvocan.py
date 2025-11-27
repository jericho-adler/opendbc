import random
from opendbc.car.volvo.helpers import checksum_lca_2_message, checksum_2_0x69_message, checksum_1_pscm_related_message, checksum_2_pscm_related_message, checksum_lca_4_message, checksum_lca_5_message
from opendbc.car.volvo.lca_encoder import LCATargetAngleEncoder
from opendbc.car.carlog import carlog


def torque_to_lca5_bytes(torque_16bit: int) -> tuple[int, int]:
  """
  Convert 16-bit signed torque value to LCA_5 message bytes.

  Encoding scheme discovered from stock Volvo CMA system:
  - Left (positive): LCA_TURN_BITS increments from 128, STEER counts 0-255
  - Right (negative): LCA_TURN_BITS decrements from 255, STEER counts 255-0
  - Neutral: LCA_TURN_BITS=186 (0xBA), STEER=0

  Args:
    torque_16bit: Signed torque value (-1791 to +1791)

  Returns:
    Tuple of (lca_turn_bits, lca_5_steer) both 0-255

  Examples:
    torque_to_lca5_bytes(0)    → (186, 0)   # Neutral
    torque_to_lca5_bytes(255)  → (128, 255) # Small left
    torque_to_lca5_bytes(256)  → (129, 0)   # Transition
    torque_to_lca5_bytes(1791) → (134, 255) # Max left
    torque_to_lca5_bytes(-255) → (255, 0)   # Small right
    torque_to_lca5_bytes(-256) → (254, 255) # Transition
    torque_to_lca5_bytes(-1791)→ (249, 0)   # Max right
  """
  from opendbc.car.volvo.values import CarControllerParams as CCP

  # Clamp to safe limits
  torque_16bit = max(CCP.LCA_TORQUE_MIN, min(CCP.LCA_TORQUE_MAX, torque_16bit))

  if torque_16bit == 0:
    # Neutral/inactive
    return (CCP.LCA_TURN_INACTIVE, 0)

  elif torque_16bit > 0:
    # Left turn (positive torque)
    # Formula: torque = (LCA_TURN_BITS - 128) × 256 + STEER
    # Solve for LCA_TURN_BITS and STEER:
    high_byte = torque_16bit // 256  # How many full 256-step increments
    low_byte = torque_16bit % 256     # Remainder within current 256 range

    lca_turn_bits = CCP.LCA_TURN_LEFT_MIN + high_byte
    lca_5_steer = low_byte

    # Clamp to maximum
    if lca_turn_bits > CCP.LCA_TURN_LEFT_MAX:
      lca_turn_bits = CCP.LCA_TURN_LEFT_MAX
      lca_5_steer = 255

    return (lca_turn_bits, lca_5_steer)

  else:
    # Right turn (negative torque)
    # Formula: torque = -(255 - LCA_TURN_BITS) × 256 - (255 - STEER)
    # Working with absolute value for clarity
    abs_torque = abs(torque_16bit)

    high_byte = abs_torque // 256
    low_byte = abs_torque % 256

    # For right turn: LCA_TURN_BITS decrements from 255
    lca_turn_bits = CCP.LCA_TURN_RIGHT_MAX - high_byte
    # STEER counts down from 255
    lca_5_steer = 255 - low_byte

    # Clamp to minimum
    if lca_turn_bits < CCP.LCA_TURN_RIGHT_MIN:
      lca_turn_bits = CCP.LCA_TURN_RIGHT_MIN
      lca_5_steer = 0

    return (lca_turn_bits, lca_5_steer)


def lca5_bytes_to_torque(lca_turn_bits: int, lca_5_steer: int) -> int:
  """
  Convert LCA_5 message bytes back to 16-bit signed torque value.

  This is the inverse of torque_to_lca5_bytes() and useful for:
  - Testing/validation
  - Debugging
  - Reading stock LCA commands

  Args:
    lca_turn_bits: LCA_TURN_BITS value (0-255)
    lca_5_steer: LCA_5_STEER value (0-255)

  Returns:
    Signed torque value (-1791 to +1791)

  Examples:
    lca5_bytes_to_torque(186, 0)   → 0     # Neutral
    lca5_bytes_to_torque(128, 255) → 255   # Small left
    lca5_bytes_to_torque(129, 0)   → 256   # Transition
    lca5_bytes_to_torque(134, 255) → 1791  # Max left
    lca5_bytes_to_torque(255, 0)   → -255  # Small right
    lca5_bytes_to_torque(254, 255) → -256  # Transition
    lca5_bytes_to_torque(249, 0)   → -1791 # Max right
  """
  from opendbc.car.volvo.values import CarControllerParams as CCP

  # Check for neutral/inactive
  if lca_turn_bits == CCP.LCA_TURN_INACTIVE:
    return 0

  # Check if left turn range
  if CCP.LCA_TURN_LEFT_MIN <= lca_turn_bits <= CCP.LCA_TURN_LEFT_MAX:
    # Left turn: torque = (LCA_TURN_BITS - 128) × 256 + STEER
    return (lca_turn_bits - CCP.LCA_TURN_LEFT_MIN) * 256 + lca_5_steer

  # Check if right turn range
  elif CCP.LCA_TURN_RIGHT_MIN <= lca_turn_bits <= CCP.LCA_TURN_RIGHT_MAX:
    # Right turn: torque = -(255 - LCA_TURN_BITS) × 256 - (255 - STEER)
    return -((CCP.LCA_TURN_RIGHT_MAX - lca_turn_bits) * 256 + (255 - lca_5_steer))

  else:
    # Unknown/invalid range - treat as neutral
    return 0


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
    apply_torque: Steering torque to apply (-127 to +127)
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

  loosely_1_original = msg_lca['LCA_STEER_LOOSELY_1']
  loosely_2_original = msg_lca['LCA_STEER_LOOSELY_2']

  curve_right = 0
  if lca_steer < 0:
    curve_right = 63
  elif lca_steer > 0:
    curve_right = 0
    #loosely_1 = 152
    #loosely_2 = 230

  if loosely_1_original != 0 or loosely_2_original != 0: # TODO Temporary
    #loosely_1 = loosely_1_original
    #loosely_2 = loosely_2_original
    pass

  # LCA_STEER encoding depends on direction (similar to LCA_5_STEER):
  # - Left turn (apply_torque > 0): zero point is 0, use absolute value
  # - Right turn (apply_torque < 0): zero point is 255, use 255 - abs(value)
  if lat_active:
    if apply_torque < 0:  # Right turn
      lca_steer_value = 255 - abs(apply_torque)
    else:  # Left turn or neutral
      lca_steer_value = abs(apply_torque)
  else:
    lca_steer_value = 0

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
    'LCA_STEER': lca_steer_value,
    'NEW_SIGNAL_6': 1, #15, # ?
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

def create_lca_3_message(packer, lat_active: bool, apply_torque: int, msg_lca_3: dict, counter_value: int):
  """
  Create LCA_3 message for Volvo CMA platform.
  This message enables PSCM to accept LCA commands.

  Args:
    packer: CAN packer instance
    lat_active: Whether lateral control is active
    msg_lca_3: Dictionary containing LCA_3 message values
    counter_value: Counter value to use (from pattern or stock)
  """
  signal_9 = 128 if lat_active else msg_lca_3['NEW_SIGNAL_9']
  if lat_active:
    if apply_torque > 0: # Left turn
      signal_9 = 255
    elif apply_torque < 0: # Right turn
      signal_9 = 0
  signal_9 = msg_lca_3['NEW_SIGNAL_9'] # TODO: Remove
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
    'COUNTER_1': counter_value,
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

def create_lca_2_message(packer, lat_active: bool, msg_lca_2: dict, counter_1: int, counter_2: int):
  """
  Create LCA_2 message to spoof PILOT_ASSIST_ENGAGED when openpilot is active.

  When lat_active=True, we set PILOT_ASSIST_ENGAGED=1 to make PSCM accept LCA commands,
  even if the driver has disabled stock Pilot Assist.

  Args:
    packer: CAN packer instance
    lat_active: Whether lateral control is active
    msg_lca_2: Dictionary containing LCA_2 message values from car
    counter_1: Managed COUNTER_1 value (increments by +2 mod 16)
    counter_2: Managed COUNTER_2 value (increments by +4 mod 16)
  """
  #return packer.make_can_msg('LCA_2', 2, msg_lca_2)
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
    'CHECKSUM_1': msg_lca_2['CHECKSUM_1'], # Byte 6 is a checksum based on Bytes 1, 2, and 5 only
    'BYTE_7': 0 if lat_active else msg_lca_2['BYTE_7'],
  }

  def build_bytes(values: dict) -> list[int]:
    b0 = int(values['BYTE_0']) # Used for checksum 1 and 2
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
    b3 = None
    b4 = None
    return [b0, b1, b2, b3, b4, b5]

  built_bytes = build_bytes(values)
  b0 = built_bytes[0]
  b1 = built_bytes[1]
  b2 = built_bytes[2]
  b5 = built_bytes[5]

  values['CHECKSUM_1'] = checksum_lca_2_message(b0, b5)

  # Only validate when not active and message is valid (BYTE_0 should be 24, not 0)
  if not lat_active:
    #assert values['CHECKSUM_1'] == msg_lca_2['CHECKSUM_1']
    if values['CHECKSUM_1'] != msg_lca_2['CHECKSUM_1']:
      carlog.warning("[volvocan.py] LCA_2 CHECKSUM mismatch")
      print(f"b0={b0}, b1={b1}, b2={b2}, b5={b5}, calculated={values['CHECKSUM_1']}, expected={msg_lca_2['CHECKSUM_1']}")
      #assert False

  # Checksum 2
  #b1 = (int(values['BYTE_1_MSBS_3']) & 0b111) << 5 | (int(values['PILOT_ASSIST_ENGAGED']) & 0b1) << 4 | (int(values['COUNTER_1']) & 0b1111)
  checksum_2 = checksum_2_0x69_message(b0, b1)
  values['CHECKSUM_2'] = checksum_2
  if not lat_active:
    if values['CHECKSUM_2'] != msg_lca_2['CHECKSUM_2']:
      carlog.warning("[volvocan.py] LCA_2 CHECKSUM_2 mismatch")
      print(f"b0={b0}, b1={b1}, calculated={values['CHECKSUM_2']}, expected={msg_lca_2['CHECKSUM_2']}")
      #assert False
  values['COUNTER_1'] = counter_1
  values['COUNTER_2'] = counter_2
  built_bytes = build_bytes(values)
  b0 = built_bytes[0]
  b1 = built_bytes[1]
  b2 = built_bytes[2]
  b5 = built_bytes[5]
  values['CHECKSUM_1'] = checksum_lca_2_message(b0, b5)
  values['CHECKSUM_2'] = checksum_2_0x69_message(b0, b1)
  return packer.make_can_msg('LCA_2', 2, values)

def create_lca_5_message(packer, lat_active: bool, target_angle_deg: float, msg_lca_5: dict, counter: int):
  """
  Create LCA_5 message (0x67) with angle-based steering control.

  Args:
    packer: CAN packer instance
    lat_active: Whether lateral control is active
    target_angle_deg: Target steering angle in degrees (positive = left, negative = right)
    msg_lca_5: Stock LCA_5 values from car
    counter: Counter value (0-15, increments by 4)

  Returns:
    CAN message for LCA_5 on bus 2
  """
  # Use angle encoder to convert target angle to LCA_5 bytes
  if lat_active:
    lca_turn_bits, lca_5_steer = LCATargetAngleEncoder.encode(target_angle_deg)
    #lca_turn_bits = 125
    #lca_5_steer = 255
  else:
    # When not active, use inactive encoding
    lca_turn_bits, lca_5_steer = LCATargetAngleEncoder.encode_inactive()

  # Build values dictionary (wheel speeds and counter unchanged)
  values = {
    'WHEEL_SPEED_1': msg_lca_5['WHEEL_SPEED_1'],
    'NEW_SIGNAL_4': msg_lca_5['NEW_SIGNAL_4'],
    'NEW_SIGNAL_1': msg_lca_5['NEW_SIGNAL_1'],
    'COUNTER': counter,
    'WHEEL_SPEED_2': msg_lca_5['WHEEL_SPEED_2'],
    'NEW_SIGNAL_5': msg_lca_5['NEW_SIGNAL_5'],
    'LCA_TURN_BITS': lca_turn_bits,  # Byte 6 - angle encoding
    'LCA_5_STEER': lca_5_steer,      # Byte 7 - angle encoding
  }

  # Build bytes for checksum calculation (unchanged)
  def build_bytes(vals: dict) -> list[int]:
    # Byte 0: WHEEL_SPEED_1[6:0] (bits 6-0) + NEW_SIGNAL_4 (bit 7)
    byte0 = (int(vals['WHEEL_SPEED_1']) & 0x7F) | ((int(vals['NEW_SIGNAL_4']) & 0x01) << 7)
    # Byte 1: WHEEL_SPEED_1[14:7] (upper 8 bits of 15-bit value)
    byte1 = (int(vals['WHEEL_SPEED_1']) >> 7) & 0xFF
    # Byte 3: NEW_SIGNAL_1 (bits 3-0) + COUNTER (bits 7-4)
    byte3 = (int(vals['NEW_SIGNAL_1']) & 0x0F) | ((int(vals['COUNTER']) & 0x0F) << 4)
    # Byte 4: WHEEL_SPEED_2[6:0] (bits 6-0) + NEW_SIGNAL_5 (bit 7)
    byte4 = (int(vals['WHEEL_SPEED_2']) & 0x7F) | ((int(vals['NEW_SIGNAL_5']) & 0x01) << 7)
    # Byte 5: WHEEL_SPEED_2[14:7] (upper 8 bits of 15-bit value)
    byte5 = (int(vals['WHEEL_SPEED_2']) >> 7) & 0xFF
    return [byte0, byte1, byte3, byte4, byte5]

  # Calculate checksum
  built = build_bytes(values)
  values['CHECKSUM'] = checksum_lca_5_message(built[0], built[1], built[2], built[3], built[4])

  return packer.make_can_msg('LCA_5', 2, values)

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

def create_0x1a_message(packer, msg_0x1a: dict):
  """
  Forward 0x1A message by copying all bytes.

  Args:
    packer: CAN packer instance
    msg_0x1a: Dictionary containing 0x1A message values from car
  """
  values = {
    'ALL_BYTES': msg_0x1a['ALL_BYTES'],
  }
  return packer.make_can_msg('NEW_MSG_1A', 2, values)

def create_gear_position_message(packer, msg_gear_position: dict):
  """
  Forward GEAR_POSITION message by copying all bytes.

  Args:
    packer: CAN packer instance
    msg_gear_position: Dictionary containing GEAR_POSITION message values from car
  """
  values = {
    'ALL_BYTES': msg_gear_position['ALL_BYTES'],
  }
  return packer.make_can_msg('GEAR_POSITION', 2, values)

def create_egsm_message(packer, msg_egsm: dict):
  """
  Forward EGSM message by copying all bytes.

  Args:
    packer: CAN packer instance
    msg_egsm: Dictionary containing EGSM message values from car
  """
  values = {
    'ALL_BYTES': msg_egsm['ALL_BYTES'],
  }
  return packer.make_can_msg('EGSM', 0, values)

def create_pscm_related_message(packer, lat_active: bool, stock_lca_engaged: bool, msg_pscm_related: dict, sig1_counter: int):
  # BO_ 23 PSCM_RELATED: 8 XXX
  # SG_ CHECKSUM : 7|8@0+ (1,0) [0|255] "" XXX
  # SG_ LCA_ENABLED_ECHO : 11|4@0+ (1,0) [0|15] "" XXX
  # SG_ SIG1_BYTE_1_HI_NIBBLE : 15|4@0+ (1,0) [0|15] "" XXX
  # SG_ SIG1_REPLICA_BYTE_2_LO_NIBLE : 19|4@0+ (1,0) [0|15] "" XXX
  # SG_ NEW_SIGNAL_2 : 23|4@0+ (1,0) [0|15] "" XXX
  # SG_ BYTE_3 : 31|8@0+ (1,0) [0|255] "" XXX
  # SG_ BYTE_4 : 39|8@0+ (1,0) [0|255] "" XXX
  # SG_ BYTE_5 : 47|8@0+ (1,0) [0|255] "" XXX
  # SG_ BYTE_6 : 55|8@0+ (1,0) [0|255] "" XXX
  # SG_ BYTE_7 : 63|8@0+ (1,0) [0|255] "" XXX
  values = dict(msg_pscm_related)

  # Update SIG1 counter (same value in both locations for redundancy)
  values['SIG1_BYTE_1_HI_NIBBLE'] = sig1_counter
  values['SIG1_REPLICA_BYTE_2_LO_NIBLE'] = sig1_counter

  b0 = int(values['CHECKSUM_1'])
  b1 = int(values['SIG1_BYTE_1_HI_NIBBLE']) << 4 | int(values['LCA_ENABLED_ECHO'])
  b2 = int(values['NEW_SIGNAL_2']) << 4 | int(values['SIG1_REPLICA_BYTE_2_LO_NIBLE'])
  b3 = int(values['CHECKSUM_2'])
  b4 = int(values['BYTE_4'])
  b5 = int(values['BYTE_5'])
  b6 = int(values['BYTE_6'])
  b7 = int(values['BYTE_7'])
  values['CHECKSUM_1'] = checksum_1_pscm_related_message(b1, b2)
  values['CHECKSUM_2'] = checksum_2_pscm_related_message(b2)
  #assert values['CHECKSUM_1'] == msg_pscm_related['CHECKSUM_1']
  #assert values['CHECKSUM_2'] == msg_pscm_related['CHECKSUM_2']
  if lat_active and not stock_lca_engaged:
    values['LCA_ENABLED_ECHO'] = 0
    b1 = int(values['SIG1_BYTE_1_HI_NIBBLE']) << 4 | int(values['LCA_ENABLED_ECHO'])
    values['CHECKSUM_1'] = checksum_1_pscm_related_message(b1, b2)
  return packer.make_can_msg('PSCM_RELATED', 0, values)

def create_lca_4_message(packer, lat_active: bool, msg_lca_4: dict, lca_steer: int):
  """
  Create LCA_4 (0x90) message to maintain Pilot Assist state when openpilot is active.

  Critical: LCA_ENABLE (byte 1 bits 0-1) must be held at 3 (both bits=1) when lat_active.
  When PA turns off, these bits start varying (become counters). We need to keep them
  stable at 3 to fool PSCM into thinking PA is still on, allowing LCA commands to be accepted.

  Based on analysis from route_analysis/pilot_assist_off/BASELINE_FILTERED_FINDINGS.md:
  - Message 0x090 byte 1 bits 0-1 are PA state signals
  - During PA ON: bits are stable at 3 (binary 11)
  - During PA OFF: bits start varying (counters)
  - PSCM uses this to determine whether to accept LCA steering commands

  Args:
    packer: CAN packer instance
    lat_active: Whether lateral control is active
    msg_lca_4: Dictionary containing LCA_4 message values from car

  Returns:
    CAN message for LCA_4 on bus 2
  """
  if not lat_active:
    # When not active, just relay stock message unchanged
    return packer.make_can_msg('LCA_4', 2, msg_lca_4)

  # When lat_active, force LCA_ENABLE to 3 (PA ON state)
  values = {
    'BYTE_1': msg_lca_4['BYTE_1'],
    'LCA_ENABLE': 3,  # Force bits 0-1 to 1 (value=3 means both bits set)
    'BYTE_1_FLAGS': msg_lca_4['BYTE_1_FLAGS'],
    'BYTE_1_NIBBLE_HI': msg_lca_4['BYTE_1_NIBBLE_HI'],
    'BYTE_2': msg_lca_4['BYTE_2'],
    'BYTE_3': msg_lca_4['BYTE_3'],
    'BYTE_4': msg_lca_4['BYTE_4'],
    'BYTE_5': msg_lca_4['BYTE_5'], # TODO
    'BYTE_6': msg_lca_4['BYTE_6'],
    'BYTE_7_NIBBLE_LO': msg_lca_4['BYTE_7_NIBBLE_LO'],
    'BYTE_7_NIBBLE_HI': msg_lca_4['BYTE_7_NIBBLE_HI'],
  }

  # TODO: Add checksum calculation when checksum function is implemented
  # If message has a checksum signal, it would be calculated here like:
  # values['CHECKSUM'] = checksum_lca_4_message(...)

  # TODO: Add checksum validation when not active (once checksum is known)
  # if not lat_active and 'CHECKSUM' in msg_lca_4:
  #   if values['CHECKSUM'] != msg_lca_4['CHECKSUM']:
  #     carlog.warning("[volvocan.py] LCA_4 CHECKSUM mismatch")

  return packer.make_can_msg('LCA_4', 2, values)