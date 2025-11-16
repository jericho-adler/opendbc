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
    'COUNTER_1': msg_vcu1['COUNTER_1'],
    'PILOT_ASSIST_ENGAGED': 1 if lat_active else msg_vcu1['PILOT_ASSIST_ENGAGED'],
    'COUNTER_2': msg_vcu1['COUNTER_2'],
    'BRAKE_PEDAL_PRESSED_B': msg_vcu1['BRAKE_PEDAL_PRESSED_B'],
    'BRAKE_PEDAL_PRESSED_A': msg_vcu1['BRAKE_PEDAL_PRESSED_A'],
    'NEW_SIGNAL_1': msg_vcu1['NEW_SIGNAL_1'], # 192 always
    'NEW_SIGNAL_2': msg_vcu1['NEW_SIGNAL_2'],
    'NEW_SIGNAL_3': msg_vcu1['NEW_SIGNAL_3'],
    'NEW_SIGNAL_4': msg_vcu1['NEW_SIGNAL_4'],
  }

  return packer.make_can_msg('VCU1', 2, values)