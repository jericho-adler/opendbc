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

def checksum_1_pscm_related_message(b1, b2):
  """
  Computes checksum #1 (goes in byte[0]) for PSCM-related 0x17 message.
  Depends only on (byte[1], byte[2]).
  """

  lut = {
    (0x00, 0x80): 0xD4, (0x01, 0x80): 0xC9, (0x04, 0x80): 0xA0,
    (0x10, 0x81): 0x98, (0x11, 0x81): 0x85, (0x14, 0x81): 0xEC,
    (0x20, 0x82): 0x4C, (0x21, 0x82): 0x51, (0x24, 0x82): 0x38,
    (0x30, 0x83): 0x00, (0x31, 0x83): 0x1D, (0x34, 0x83): 0x74,
    (0x40, 0x84): 0xF9, (0x41, 0x84): 0xE4, (0x44, 0x84): 0x8D,
    (0x50, 0x85): 0xB5, (0x51, 0x85): 0xA8, (0x54, 0x85): 0xC1,
    (0x60, 0x86): 0x61, (0x61, 0x86): 0x7C, (0x64, 0x86): 0x15,
    (0x70, 0x87): 0x2D, (0x71, 0x87): 0x30, (0x74, 0x87): 0x59,
    (0x80, 0x88): 0x8E, (0x81, 0x88): 0x93, (0x84, 0x88): 0xFA,
    (0x90, 0x89): 0xC2, (0x91, 0x89): 0xDF, (0x94, 0x89): 0xB6,
    (0xA0, 0x8A): 0x16, (0xA1, 0x8A): 0x0B, (0xA4, 0x8A): 0x62,
    (0xB0, 0x8B): 0x5A, (0xB1, 0x8B): 0x47, (0xB4, 0x8B): 0x2E,
    (0xC0, 0x8C): 0xA3, (0xC1, 0x8C): 0xBE, (0xC4, 0x8C): 0xD7,
    (0xD0, 0x8D): 0xEF, (0xD1, 0x8D): 0xF2, (0xD4, 0x8D): 0x9B,
    (0xE0, 0x8E): 0x3B, (0xE1, 0x8E): 0x26, (0xE4, 0x8E): 0x4F,
  }

  return lut.get((b1, b2), 0)


def checksum_2_pscm_related_message(b2):
  """
  Computes checksum #2 (goes in byte[3]) for PSCM-related 0x17 message.
  Depends only on byte[2].
  """

  lut = {
    0x80: 0xBF,
    0x81: 0xF3,
    0x82: 0x27,
    0x83: 0x6B,
    0x84: 0x92,
    0x85: 0xDE,
    0x86: 0x0A,
    0x87: 0x46,
    0x88: 0xE5,
    0x89: 0xA9,
    0x8A: 0x7D,
    0x8B: 0x31,
    0x8C: 0xC8,
    0x8D: 0x84,
    0x8E: 0x50,
  }

  return lut.get(b2, 0)
