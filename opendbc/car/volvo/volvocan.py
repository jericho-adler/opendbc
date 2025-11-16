import random

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

def create_pscm_message(packer, lat_active: bool, msg_pscm: dict, frame: int, pilot_assist_engaged: bool):
  values = {
    'PSCM_ANGLE_SENSOR': msg_pscm['PSCM_ANGLE_SENSOR'],
    'BIT_0': msg_pscm['BIT_0'],
    'BYTE_2': msg_pscm['BYTE_2'],
    'BYTE_3': msg_pscm['BYTE_3'],
    'BYTE_4': msg_pscm['BYTE_4'],
    'DRIVER_INPUT_DEVIATION': msg_pscm['DRIVER_INPUT_DEVIATION'],
    'BYTE_6': msg_pscm['BYTE_6'],
    'BYTE_7': msg_pscm['BYTE_7'],
  }
  if lat_active or pilot_assist_engaged:
    #values['DRIVER_INPUT_DEVIATION'] = -1 # Spoof hands on steering wheel
    values['DRIVER_INPUT_DEVIATION'] = 1 if frame % 2 == 0 else 0
    values['BYTE_3'] = 186 if frame % 2 == 0 else 154 # msg_pscm['BYTE_3']
    values['BYTE_2'] = 195 if frame % 2 == 0 else 249 # msg_pscm['BYTE_2']

  return packer.make_can_msg('PSCM', 0, values)

def create_vcu1_pscm_control(packer, lat_active: bool, apply_torque: int, msg_vcu1_pscm_control: dict,
                            timer_1: int, timer_2: int):
  """
  Create VCU1_PSCM_CONTROL message for Volvo CMA platform.
  This message enables PSCM to accept LCA commands.

  Args:
    packer: CAN packer instance
    lat_active: Whether lateral control is active
    msg_vcu1_pscm_control: Dictionary containing VCU1_PSCM_CONTROL message values
    timer_1: 16-bit timer value (218 kHz, increments by ~3270 per message)
    timer_2: 16-bit timer value (218 kHz, increments by ~3270 per message)
  """
  signal_9 = 128 if lat_active else msg_vcu1_pscm_control['NEW_SIGNAL_9']
  if lat_active:
    if apply_torque > 0: # Left turn
      signal_9 = 255
    elif apply_torque < 0: # Right turn
      signal_9 = 0
  values = {
    'NEW_SIGNAL_3': 0 if lat_active else msg_vcu1_pscm_control['NEW_SIGNAL_3'],
    'LCA_ACCEPT_COMMANDS_RELATED': 15 if lat_active else msg_vcu1_pscm_control['LCA_ACCEPT_COMMANDS_RELATED'],
    'NEW_SIGNAL_2': 0 if lat_active else msg_vcu1_pscm_control['NEW_SIGNAL_2'],
    'NEW_SIGNAL_5': 30 if lat_active else msg_vcu1_pscm_control['NEW_SIGNAL_5'],
    'LCA_ACCEPT_COMMANDS_INV': 0 if lat_active else msg_vcu1_pscm_control['LCA_ACCEPT_COMMANDS_INV'],
    'NEW_SIGNAL_4': 3 if lat_active else msg_vcu1_pscm_control['NEW_SIGNAL_4'],
    'TIMER_1': timer_1,
    'TIMER_2': timer_2,
    'NEW_SIGNAL_8': 1 if lat_active else msg_vcu1_pscm_control['NEW_SIGNAL_8'],
    'COUNTER_1': msg_vcu1_pscm_control['COUNTER_1'],
    'NEW_SIGNAL_7': 3 if lat_active else msg_vcu1_pscm_control['NEW_SIGNAL_7'],
    'NEW_SIGNAL_9': signal_9,
  }

  return packer.make_can_msg('VCU1_PSCM_CONTROL', 2, values)

def checksum_vcu1_message(b1: int, b2: int, b5: int) -> int:
  """
  Compute the 8-bit checksum from Byte1, Byte2 and Byte5.

  b1, b2, b5: integers 0..255
  returns: checksum byte 0..255
  """

  # Masks for each checksum bit (bit 0 = LSB)
  M1 = [0x42, 0x00, 0x00, 0x00,
        0x42, 0x00, 0x00, 0x00]

  M2 = [0x05, 0x00, 0x00, 0x00,
        0x05, 0x00, 0x00, 0x00]

  M5 = [0x83, 0x86, 0xCF, 0xCD,
        0x09, 0x02, 0x44, 0x89]

  def parity8(x: int) -> int:
    """Return 1 if x has an odd number of bits set, else 0."""
    x ^= x >> 4
    x ^= x >> 2
    x ^= x >> 1
    return x & 1

  c = 0
  for bit in range(8):
    p = (
      parity8(b1 & M1[bit]) ^
      parity8(b2 & M2[bit]) ^
      parity8(b5 & M5[bit])
    )
    c |= (p << bit)

  return c & 0xFF

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

def create_vcu1_message(packer, lat_active: bool, msg_vcu1: dict):
  """
  Create VCU1 message to spoof PILOT_ASSIST_ENGAGED when openpilot is active.

  When lat_active=True, we set PILOT_ASSIST_ENGAGED=1 to make PSCM accept LCA commands,
  even if the driver has disabled stock Pilot Assist.

  Args:
    packer: CAN packer instance
    lat_active: Whether lateral control is active
    msg_vcu1: Dictionary containing VCU1 message values from car
  """
  values = {
    'BYTE_0': 24 if lat_active else msg_vcu1['BYTE_0'], # 24 always
    'COUNTER_1': msg_vcu1['COUNTER_1'], # Byte 1 Low Nibble [5:8] - 4-bit counter that increments by +2 (modulo 16)
    'PILOT_ASSIST_ENGAGED': 1 if lat_active else msg_vcu1['PILOT_ASSIST_ENGAGED'], # Byte 1 [4]
    'BYTE_1_MSBS_3': msg_vcu1['BYTE_1_MSBS_3'], # Byte 1 [0:3]
    'BYTE_2': msg_vcu1['BYTE_2'], # No idea what this is
    'NEW_SIGNAL_2': 0 if lat_active else msg_vcu1['NEW_SIGNAL_2'],
    'COUNTER_2': msg_vcu1['COUNTER_2'], # Byte 5 Low Nibble - 4-bit counter that increments by +4 (modulo 16)
    'NEW_SIGNAL_3': 3 if lat_active else msg_vcu1['NEW_SIGNAL_3'],
    'BRAKE_PEDAL_PRESSED_B': msg_vcu1['BRAKE_PEDAL_PRESSED_B'],
    'BRAKE_PEDAL_PRESSED_A': msg_vcu1['BRAKE_PEDAL_PRESSED_A'],
    'CHECKSUM': msg_vcu1['CHECKSUM'], # Byte 6 is a checksum based on Bytes 1, 2, and 5 only
    'BYTE_7': 0 if lat_active else msg_vcu1['BYTE_7'],
  }

  # Reconstruct bytes 1, 2, and 5 from signal values for checksum calculation
  b1 = ((int(values['COUNTER_1']) & 0x0F) |
        ((int(values['PILOT_ASSIST_ENGAGED']) & 0x01) << 4) |
        ((int(values['BYTE_1_MSBS_3']) & 0x07) << 5))
  b2 = int(values['BYTE_2']) & 0xFF
  # Note: BRAKE_PEDAL_PRESSED_A has scale=-1, offset=1 in DBC, so we need to invert:
  # raw = (physical - offset) / scale = (physical - 1) / -1
  brake_pedal_a_raw = int((values['BRAKE_PEDAL_PRESSED_A'] - 1) / -1)
  b5 = ((int(values['COUNTER_2']) & 0x0F) |
        ((int(values['NEW_SIGNAL_3']) & 0x03) << 4) |
        ((int(values['BRAKE_PEDAL_PRESSED_B']) & 0x01) << 6) |
        ((brake_pedal_a_raw & 0x01) << 7))

  values['CHECKSUM'] = checksum_vcu1_message(b1, b2, b5)

  # Only validate when not active and message is valid (BYTE_0 should be 24, not 0)
  if not lat_active and msg_vcu1['BYTE_0'] != 0:
    if values != msg_vcu1:
      print(msg_vcu1)
      print(f"Values mismatch: {diff_dicts(values, msg_vcu1)}")
      raise ValueError("Values mismatch")
    if values['CHECKSUM'] != msg_vcu1['CHECKSUM']:
      print(f"Checksum mismatch: {values['CHECKSUM']} != {msg_vcu1['CHECKSUM']}: b1={b1}, b2={b2}, b5={b5}")
      raise ValueError("Checksum mismatch")

  return packer.make_can_msg('VCU1', 2, values)