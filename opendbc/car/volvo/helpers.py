def checksum_lca_2_message(b0: int, b5: int) -> int:
  """
  Compute checksum for VCU1 CAN ID 0x69 from bytes b0 and b5.

  b0: first data byte (MSB) of the frame (usually 0x18 in your logs)
  b5: sixth data byte of the frame (what you called Byte5)

  Returns: checksum byte (0..255) that goes into byte index 6.
  """
  if b0 == 0 and b5 == 128: # Hotfix openpilot test (don't know where this alleged test message comes from)
    return 0

  # Masks per checksum bit (bit 0..7) for b0 and b5
  M0 = [0x08, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00]
  M5 = [0x83, 0x86, 0xCF, 0xCD, 0x09, 0x02, 0x44, 0x89]

  def parity8(x: int) -> int:
    # 1 if x has an odd number of bits set, else 0
    x ^= x >> 4
    x ^= x >> 2
    x ^= x >> 1
    return x & 1

  b0 &= 0xFF
  b5 &= 0xFF

  c = 0
  for bit in range(8):
    p = 0
    if M0[bit]:
      p ^= parity8(b0 & M0[bit])
    if M5[bit]:
      p ^= parity8(b5 & M5[bit])
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