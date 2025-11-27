class LiveTestingManager:
  TESTING_FILE = "/data/openpilot/live_testing.txt"

  def __init__(self):
    pass  # No state needed - reads file fresh every call

  def load_config(self) -> dict | None:
    """
    Load live testing configuration from file.

    Returns:
      - None if file doesn't exist, parse fails, or lat_active=False
      - dict with available override keys if lat_active=True or not specified
        (partial configs supported - missing keys will be absent from dict)

    Example return values:
      - File doesn't exist → None
      - File contains lat_active=False → None
      - File contains lat_active=True → {'lat_active': True}
      - File contains all params → {'lat_active': True, 'lca_turn_bits': 128, 'lca_5_steer': 255, 'lca_steer': 100}
      - File contains only bytes → {'lca_turn_bits': 128, 'lca_5_steer': 255, 'lca_steer': 50}
    """
    try:
      with open(self.TESTING_FILE, 'r') as f:
        lines = f.readlines()

      config = {}

      for line in lines:
        # Strip whitespace
        line = line.strip()

        # Skip empty lines and comments
        if not line or line.startswith('#'):
          continue

        # Skip lines without '='
        if '=' not in line:
          continue

        # Parse key=value
        key, value = line.split('=', 1)
        key = key.strip()
        value = value.strip()

        # Parse lat_active (boolean)
        if key == 'lat_active':
          if value == 'True':
            config['lat_active'] = True
          elif value == 'False':
            config['lat_active'] = False
          else:
            # Invalid boolean value, skip this line
            continue

        # Parse lca_turn_bits (0-255)
        elif key == 'lca_turn_bits':
          try:
            val = int(value)
            if 0 <= val <= 255:
              config['lca_turn_bits'] = val
            # else: out of bounds, skip this line
          except ValueError:
            # Invalid integer, skip this line
            continue

        # Parse lca_5_steer (0-255)
        elif key == 'lca_5_steer':
          try:
            val = int(value)
            if 0 <= val <= 255:
              config['lca_5_steer'] = val
            # else: out of bounds, skip this line
          except ValueError:
            # Invalid integer, skip this line
            continue

        # Parse lca_steer (-255 to 255) - LCA message (0x58) STEER signal
        elif key == 'lca_steer':
          try:
            val = int(value)
            if -255 <= val <= 255:
              config['lca_steer'] = val
            # else: out of bounds, skip this line
          except ValueError:
            # Invalid integer, skip this line
            continue

      # If lat_active is explicitly False in config, return None (ignore all overrides)
      if 'lat_active' in config and config['lat_active'] is False:
        return None

      # If config is empty, return None (no valid overrides)
      if not config:
        return None

      return config

    except FileNotFoundError:
      # File doesn't exist - use standard operation
      return None
    except Exception:
      # Any other error - fall back to standard operation
      return None
