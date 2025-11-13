import random

def create_lca_steering(packer, lat_active: bool, apply_torque: int):
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
  """
  if apply_torque < 0: # If torque is negative
    curve_right = 63 # Turn right
    loosely_1 = 79
    loosely_2 = 115
  else:
    curve_right = 0 # Turn left
    loosely_1 = 115
    loosely_2 = 79
  if abs(apply_torque) < 5: # assume straight road
    loosely_1 = 100
    loosely_2 = 140

  values = {
    'NEW_SIGNAL_3': 0,
    'LCA_ENABLE_INV': 0 if lat_active else 1,
    'NEW_SIGNAL_1': 3,
    'LCA_STEER_LOOSELY_1': loosely_1 if lat_active else 0,
    'LCA_STEER_ACTIVE_INCOHERENT': 1 if lat_active else 0,
    'LCA_STEER_ACTIVE': 3 if lat_active else 0,
    'NEW_SIGNAL_7': 7,
    'LCA_STEER_LOOSELY_2': loosely_2 if lat_active else 0,
    'NEW_SIGNAL_4': 25 if lat_active else 251, # ?
    'CURVE_RIGHT': curve_right,
    'NEW_SIGNAL_5': 3,
    'LCA_STEER': apply_torque if lat_active else 0,
    'NEW_SIGNAL_6': 15, # ?
  }

  if not lat_active:
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
    }

  return packer.make_can_msg('LCA', 2, values)

def create_pscm_message(packer, lat_active: bool, msg_pscm: dict):
  # BO_ 22 PSCM: 8 XXX
  # SG_ PSCM_ANGLE_SENSOR : 6|15@0- (-0.05596,0) [-916|916] "º" XXX
  # SG_ BIT_0 : 7|1@0+ (1,0) [0|1] "" XXX
  # SG_ BYTE2 : 23|8@0+ (1,0) [0|255] "" XXX
  # SG_ BYTE_3 : 31|8@0+ (1,0) [0|255] "" XXX
  # SG_ BYTE_4 : 39|8@0+ (1,0) [0|255] "" XXX
  # SG_ DRIVER_INPUT_DEVIATION : 47|8@0- (1,0) [-128|127] "" XXX
  # SG_ BYTE_6 : 55|8@0+ (1,0) [0|255] "" XXX
  # SG_ BYTE_7 : 63|8@0+ (1,0) [0|255] "" XXX
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
    values['DRIVER_INPUT_DEVIATION'] = -1 # Spoof hands on steering wheel
  return packer.make_can_msg('PSCM', 0, values)

def create_driver_input_message(packer, lat_active: bool, msg_driver_input: dict):
  values = {
    'BYTE_0': msg_driver_input['BYTE_0'],
    'BYTE_1': msg_driver_input['BYTE_1'],
    'BYTE_2': msg_driver_input['BYTE_2'],
    'STEERING_DRIVER_RATE_OF_CHANGE': msg_driver_input['STEERING_DRIVER_RATE_OF_CHANGE'],
    'BYTE_4': msg_driver_input['BYTE_4'],
    'BYTE_5': msg_driver_input['BYTE_5'],
    'STEERING_DRIVER_INPUT': msg_driver_input['STEERING_DRIVER_INPUT'],
    'BYTE_7': msg_driver_input['BYTE_7'],
  }
  if lat_active:
    values['STEERING_DRIVER_INPUT'] = random.randint(-1, -2) # Spoof hands on steering wheel, random value -1 or -2
  return packer.make_can_msg('DRIVER_INPUT', 0, values)