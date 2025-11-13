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
    apply_torque: Steering torque to apply (-127 to 127)
                  Positive = steer LEFT, Negative = steer RIGHT
    msg_lca: Dictionary containing LCA message values
  """
  if not lat_active:
    return packer.make_can_msg('LCA', 2, msg_lca)

  # CURVE_RIGHT: Binary flag (0 or 63)
  # Data shows: 0 when torque >= 0 (left/straight), 63 when torque < 0 (right)
  if apply_torque < 0:
    curve_right = 63  # Right turn
  else:
    curve_right = 0   # Left/straight

  # LOOSELY signals: BOTH INCREASE with steering magnitude
  # Based on corrected analysis:
  # - Low torque (straight): LOOSELY_1=87, LOOSELY_2=136
  # - High torque (curves): LOOSELY_1=133, LOOSELY_2=181
  torque_abs = abs(apply_torque)

  if torque_abs < 10:
    # Straight or minimal steering
    loosely_1 = 87
    loosely_2 = 136
  elif torque_abs < 40:
    # Gentle curve - interpolate
    # Scale factor: torque range 10-40 maps to parameter range
    t = (torque_abs - 10) / 30.0
    loosely_1 = int(87 + t * (133 - 87))    # 87 -> 133
    loosely_2 = int(136 + t * (181 - 136))  # 136 -> 181
  else:
    # Strong steering
    loosely_1 = 133
    loosely_2 = 181

  # Clamp torque to valid range
  lca_steer = max(-127, min(127, apply_torque))

  values = {
    'NEW_SIGNAL_3': 0,
    'LCA_ENABLE_INV': 0 if lat_active else 1,
    'NEW_SIGNAL_1': 3,
    'LCA_STEER_LOOSELY_1': loosely_1 if lat_active else 0,
    'LCA_STEER_ACTIVE_INCOHERENT': 1 if lat_active else 0,
    'LCA_STEER_ACTIVE': 3 if lat_active else 0,
    'NEW_SIGNAL_7': 7,
    'LCA_STEER_LOOSELY_2': loosely_2 if lat_active else 0,
    'NEW_SIGNAL_4': 25 if lat_active else 251,
    'CURVE_RIGHT': curve_right,
    'NEW_SIGNAL_5': 3,
    'LCA_STEER': lca_steer if lat_active else 0,
    'NEW_SIGNAL_6': 15,
  }

  return packer.make_can_msg('LCA', 2, values)

def create_pscm_message(packer, lat_active: bool, msg_pscm: dict, frame: int):
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
  if lat_active:
    #values['DRIVER_INPUT_DEVIATION'] = -1 # Spoof hands on steering wheel
    values['DRIVER_INPUT_DEVIATION'] = 1 if frame % 2 == 0 else 0
    values['BYTE_3'] = 186 if frame % 2 == 0 else 154 # msg_pscm['BYTE_3']
    values['BYTE_2'] = 195 if frame % 2 == 0 else 249 # msg_pscm['BYTE_2']

  return packer.make_can_msg('PSCM', 0, values)