def checksum_lca_2_message(b1: int, b2: int, b5: int) -> int:
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

def checksum_2_0x69_message(b0: int, b1: int) -> int:
  """
  Compute checksum byte (b2) for CAN ID 0x69 based on bytes b0 and b1.

  Args:
      b0: First data byte of the 0x69 frame (0–255).
      b1: Second data byte of the 0x69 frame (0–255).

  Returns:
      The checksum byte (0–255) that should go in position b2.
  """
  b0 &= 0xFF
  b1 &= 0xFF

  c = 0

  # bit 0 of b2
  c |= ( ((b0 >> 0) & 1)
       ^ ((b1 >> 2) & 1)
       ^ ((b1 >> 3) & 1)
       ^ ((b1 >> 4) & 1) ) << 0

  # bit 1 of b2
  c |= ( ((b0 >> 1) & 1)
       ^ ((b0 >> 3) & 1)
       ^ ((b1 >> 0) & 1)
       ^ ((b1 >> 3) & 1)
       ^ ((b1 >> 6) & 1) ) << 1

  # bit 2 of b2
  c |= ( ((b0 >> 0) & 1)
       ^ ((b0 >> 3) & 1)
       ^ ((b1 >> 1) & 1)
       ^ ((b1 >> 2) & 1)
       ^ ((b1 >> 3) & 1)
       ^ ((b1 >> 4) & 1)
       ^ ((b1 >> 6) & 1) ) << 2

  # bit 3 of b2
  c |= ( ((b0 >> 0) & 1)
       ^ ((b0 >> 1) & 1)
       ^ ((b1 >> 0) & 1)
       ^ ((b1 >> 4) & 1)
       ^ ((b1 >> 6) & 1) ) << 3

  # bit 4 of b2
  c |= ( ((b0 >> 0) & 1)
       ^ ((b0 >> 1) & 1)
       ^ ((b0 >> 3) & 1)
       ^ ((b1 >> 1) & 1)
       ^ ((b1 >> 2) & 1)
       ^ ((b1 >> 3) & 1)
       ^ ((b1 >> 4) & 1) ) << 4

  # bit 5 of b2
  c |= ( ((b0 >> 1) & 1)
       ^ ((b1 >> 0) & 1)
       ^ ((b1 >> 2) & 1)
       ^ ((b1 >> 3) & 1) ) << 5

  # bit 6 of b2
  c |= ( ((b0 >> 3) & 1)
       ^ ((b1 >> 0) & 1)
       ^ ((b1 >> 1) & 1)
       ^ ((b1 >> 3) & 1)
       ^ ((b1 >> 6) & 1) ) << 6

  # bit 7 of b2
  c |= ( ((b0 >> 3) & 1)
       ^ ((b1 >> 1) & 1)
       ^ ((b1 >> 2) & 1)
       ^ ((b1 >> 4) & 1) ) << 7

  return c & 0xFF