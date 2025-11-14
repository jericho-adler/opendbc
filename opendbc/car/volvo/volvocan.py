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

  # Scale loosely values according to lca_steer
  # Maximum values for each direction
  max_loosely_1_right = 252  # RIGHT turn
  min_loosely_2_right = 47   # RIGHT turn
  min_loosely_1_left = 9     # LEFT turn
  min_loosely_2_left = 64    # LEFT turn
  #loosely_1 = 8             # LEFT turn
  #loosely_2 = 30            # LEFT turn

  # Maximum steering torque (typically 255 for openpilot)
  # Already scaled in CarController.update()
  #max_steer = 255.0
  max_steer = 127.0

  curve_right = 0 # LEFT turn or STRAIGHT
  if lca_steer < 0:
    # RIGHT turn: scale from baseline to max/min based on torque magnitude
    curve_right = 63 # RIGHT turn
    steer_ratio = abs(lca_steer) / max_steer
    loosely_1 = baseline_loosely_1 + (max_loosely_1_right - baseline_loosely_1) * steer_ratio
    loosely_2 = baseline_loosely_2 - (baseline_loosely_2 - min_loosely_2_right) * steer_ratio
  elif lca_steer > 0:
    # LEFT turn: scale from baseline to min values based on torque magnitude
    curve_right = 0 # LEFT turn or STRAIGHT
    steer_ratio = abs(lca_steer) / max_steer
    loosely_1 = baseline_loosely_1 - (baseline_loosely_1 - min_loosely_1_left) * steer_ratio
    loosely_2 = baseline_loosely_2 - (baseline_loosely_2 - min_loosely_2_left) * steer_ratio
  else:
    # Basically == 0
    loosely_1 = baseline_loosely_1
    loosely_2 = baseline_loosely_2

  # Ensure integer values for CAN messages
  loosely_1 = int(loosely_1)
  loosely_2 = int(loosely_2)

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

def create_vcu1_pscm_control(packer, lat_active: bool, msg_vcu1_pscm_control: dict,
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
  values = {
    'NEW_SIGNAL_3': msg_vcu1_pscm_control['NEW_SIGNAL_3'],
    'LCA_ACCEPT_COMMANDS_RELATED': 15 if lat_active else msg_vcu1_pscm_control['LCA_ACCEPT_COMMANDS_RELATED'],
    'NEW_SIGNAL_2': msg_vcu1_pscm_control['NEW_SIGNAL_2'],
    'NEW_SIGNAL_5': msg_vcu1_pscm_control['NEW_SIGNAL_5'],
    'LCA_ACCEPT_COMMANDS_INV': 0 if lat_active else msg_vcu1_pscm_control['LCA_ACCEPT_COMMANDS_INV'],
    'NEW_SIGNAL_4': msg_vcu1_pscm_control['NEW_SIGNAL_4'],
    'TIMER_1': timer_1,
    'TIMER_2': timer_2,
    'NEW_SIGNAL_8': msg_vcu1_pscm_control['NEW_SIGNAL_8'],
    'COUNTER_1': msg_vcu1_pscm_control['COUNTER_1'],
    'NEW_SIGNAL_7': msg_vcu1_pscm_control['NEW_SIGNAL_7'],
    'NEW_SIGNAL_9': msg_vcu1_pscm_control['NEW_SIGNAL_9'],
  }

  return packer.make_can_msg('VCU1_PSCM_CONTROL', 2, values)