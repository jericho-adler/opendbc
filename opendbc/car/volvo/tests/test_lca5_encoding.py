import unittest
from opendbc.car.volvo.volvocan import torque_to_lca5_bytes, lca5_bytes_to_torque
from opendbc.car.volvo.values import CarControllerParams as CCP


class TestLCA5Encoding(unittest.TestCase):
  """Test LCA_5 two-byte torque encoding/decoding."""

  def test_neutral(self):
    """Test neutral/zero torque encoding."""
    turn_bits, steer = torque_to_lca5_bytes(0)
    self.assertEqual(turn_bits, 186)  # 0xBA
    self.assertEqual(steer, 0)

    # Roundtrip
    torque = lca5_bytes_to_torque(186, 0)
    self.assertEqual(torque, 0)

  def test_left_turn_small(self):
    """Test small left turn (single byte range)."""
    # 255 should give LCA_TURN_BITS=128, STEER=255
    turn_bits, steer = torque_to_lca5_bytes(255)
    self.assertEqual(turn_bits, 128)
    self.assertEqual(steer, 255)

    # Roundtrip
    torque = lca5_bytes_to_torque(128, 255)
    self.assertEqual(torque, 255)

  def test_left_turn_transition(self):
    """Test left turn at 256-step boundary."""
    # 256 should increment LCA_TURN_BITS to 129, STEER wraps to 0
    turn_bits, steer = torque_to_lca5_bytes(256)
    self.assertEqual(turn_bits, 129)
    self.assertEqual(steer, 0)

    # Roundtrip
    torque = lca5_bytes_to_torque(129, 0)
    self.assertEqual(torque, 256)

  def test_left_turn_mid_range(self):
    """Test left turn in middle of second byte range."""
    # 512 + 128 = 640 should give LCA_TURN_BITS=130, STEER=128
    turn_bits, steer = torque_to_lca5_bytes(640)
    self.assertEqual(turn_bits, 130)
    self.assertEqual(steer, 128)

    # Roundtrip
    torque = lca5_bytes_to_torque(130, 128)
    self.assertEqual(torque, 640)

  def test_left_turn_max(self):
    """Test maximum left turn."""
    # Max torque: (134-128)*256 + 255 = 1791
    turn_bits, steer = torque_to_lca5_bytes(1791)
    self.assertEqual(turn_bits, 134)
    self.assertEqual(steer, 255)

    # Roundtrip
    torque = lca5_bytes_to_torque(134, 255)
    self.assertEqual(torque, 1791)

  def test_right_turn_small(self):
    """Test small right turn (single byte range)."""
    # -255 should give LCA_TURN_BITS=255, STEER=0
    turn_bits, steer = torque_to_lca5_bytes(-255)
    self.assertEqual(turn_bits, 255)
    self.assertEqual(steer, 0)

    # Roundtrip
    torque = lca5_bytes_to_torque(255, 0)
    self.assertEqual(torque, -255)

  def test_right_turn_transition(self):
    """Test right turn at 256-step boundary."""
    # -256 should decrement LCA_TURN_BITS to 254, STEER wraps to 255
    turn_bits, steer = torque_to_lca5_bytes(-256)
    self.assertEqual(turn_bits, 254)
    self.assertEqual(steer, 255)

    # Roundtrip
    torque = lca5_bytes_to_torque(254, 255)
    self.assertEqual(torque, -256)

  def test_right_turn_mid_range(self):
    """Test right turn in middle of second byte range."""
    # -512 - 128 = -640 should give LCA_TURN_BITS=253, STEER=127
    turn_bits, steer = torque_to_lca5_bytes(-640)
    self.assertEqual(turn_bits, 253)
    self.assertEqual(steer, 127)

    # Roundtrip
    torque = lca5_bytes_to_torque(253, 127)
    self.assertEqual(torque, -640)

  def test_right_turn_max(self):
    """Test maximum right turn."""
    # Max torque: -(255-249)*256 - (255-0) = -1791
    turn_bits, steer = torque_to_lca5_bytes(-1791)
    self.assertEqual(turn_bits, 249)
    self.assertEqual(steer, 0)

    # Roundtrip
    torque = lca5_bytes_to_torque(249, 0)
    self.assertEqual(torque, -1791)

  def test_clamping_positive(self):
    """Test that values beyond max are clamped."""
    turn_bits, steer = torque_to_lca5_bytes(2000)
    self.assertEqual(turn_bits, 134)
    self.assertEqual(steer, 255)

    # Should clamp to max value
    torque = lca5_bytes_to_torque(turn_bits, steer)
    self.assertEqual(torque, 1791)

  def test_clamping_negative(self):
    """Test that values beyond min are clamped."""
    turn_bits, steer = torque_to_lca5_bytes(-2000)
    self.assertEqual(turn_bits, 249)
    self.assertEqual(steer, 0)

    # Should clamp to min value
    torque = lca5_bytes_to_torque(turn_bits, steer)
    self.assertEqual(torque, -1791)

  def test_symmetry(self):
    """Test that left/right ranges are symmetric."""
    self.assertEqual(CCP.LCA_TORQUE_MAX, -CCP.LCA_TORQUE_MIN)
    self.assertEqual(CCP.LCA_TORQUE_MAX, 1791)

  def test_roundtrip_samples(self):
    """Test roundtrip encoding/decoding for various values."""
    test_values = [
      0, 1, 100, 255, 256, 512, 1000, 1500, 1791,
      -1, -100, -255, -256, -512, -1000, -1500, -1791
    ]

    for torque in test_values:
      turn_bits, steer = torque_to_lca5_bytes(torque)
      decoded = lca5_bytes_to_torque(turn_bits, steer)
      self.assertEqual(decoded, torque,
                      f"Roundtrip failed for {torque}: got {decoded}")

  def test_small_values(self):
    """Test small torque values near zero."""
    for torque in range(-10, 11):
      turn_bits, steer = torque_to_lca5_bytes(torque)
      decoded = lca5_bytes_to_torque(turn_bits, steer)
      if torque == 0:
        self.assertEqual(decoded, 0)
      else:
        self.assertEqual(decoded, torque,
                        f"Small value roundtrip failed for {torque}: got {decoded}")

  def test_boundary_values(self):
    """Test values around 256-step boundaries."""
    boundaries = [255, 256, 257, 511, 512, 513, 767, 768, 769,
                  -255, -256, -257, -511, -512, -513, -767, -768, -769]

    for torque in boundaries:
      turn_bits, steer = torque_to_lca5_bytes(torque)
      decoded = lca5_bytes_to_torque(turn_bits, steer)
      self.assertEqual(decoded, torque,
                      f"Boundary roundtrip failed for {torque}: got {decoded}")

  def test_constants_consistency(self):
    """Test that constants are internally consistent."""
    # Left range
    left_range = CCP.LCA_TURN_LEFT_MAX - CCP.LCA_TURN_LEFT_MIN
    self.assertEqual(left_range, 6)  # 134 - 128 = 6

    # Right range (should be symmetric)
    right_range = CCP.LCA_TURN_RIGHT_MAX - CCP.LCA_TURN_RIGHT_MIN
    self.assertEqual(right_range, 6)  # 255 - 249 = 6

    # Check torque calculations
    expected_max = left_range * 256 + 255
    self.assertEqual(CCP.LCA_TORQUE_MAX, expected_max)

    expected_min = -(right_range * 256 + 255)
    self.assertEqual(CCP.LCA_TORQUE_MIN, expected_min)

  def test_one_value(self):
    """Test encoding of 1 and -1."""
    # Positive 1
    turn_bits, steer = torque_to_lca5_bytes(1)
    self.assertEqual(turn_bits, 128)
    self.assertEqual(steer, 1)
    self.assertEqual(lca5_bytes_to_torque(turn_bits, steer), 1)

    # Negative 1
    turn_bits, steer = torque_to_lca5_bytes(-1)
    self.assertEqual(turn_bits, 255)
    self.assertEqual(steer, 254)
    self.assertEqual(lca5_bytes_to_torque(turn_bits, steer), -1)


if __name__ == '__main__':
  unittest.main()
