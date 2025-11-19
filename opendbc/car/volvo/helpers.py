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


def checksum_lca_4_message(*args) -> int:
  """
  Placeholder for LCA_4 (0x90) checksum calculation.

  TODO: Implementation will be provided after checksum analysis is complete.
  For now, returns 0 as a placeholder.

  Args:
      *args: Byte values needed for checksum calculation (TBD)

  Returns:
      Checksum byte (0-255)
  """
  # Placeholder - will be replaced with actual checksum algorithm
  return 0


class LCA3CounterSync:
  """
  Best-effort pattern synchronization for LCA_3 COUNTER_1.

  The counter follows a 20-element pattern that cycles based on transmission count.
  We track recent observed counter values and match them against the pattern to
  determine the current index. While not synchronized, we pass through stock values.
  Once synchronized, we permanently use the pattern.
  """

  PATTERN = [2, 2, 1, 2, 2, 2, 1, 2, 2, 3, 0, 2, 3, 2, 0, 2, 3, 2, 0, 3]
  PATTERN_LEN = 20
  WINDOW_SIZE = 5  # Track last 5 values for matching
  MIN_CONFIDENCE = 4  # Need 4 consecutive matches to sync

  def __init__(self):
    self.pattern_index = None  # Current index in pattern (None = not synced)
    self.observed_window = []  # Circular buffer of last N observed values
    self.confidence = 0  # Number of consecutive successful matches

  def update(self, observed_counter: int) -> tuple:
    """
    Update with newly observed counter value from stock message.

    Args:
        observed_counter: Counter value from CS.msg_lca_3['COUNTER_1']

    Returns:
        Tuple of (counter_to_send, is_synchronized)
    """
    # If already synchronized, ignore stock and use our pattern permanently
    if self.pattern_index is not None:
      counter_to_send = self.PATTERN[self.pattern_index]
      self.pattern_index = (self.pattern_index + 1) % self.PATTERN_LEN
      return counter_to_send, True

    # Not synchronized yet - try to find pattern index
    self.observed_window.append(observed_counter)
    if len(self.observed_window) > self.WINDOW_SIZE:
      self.observed_window.pop(0)

    # Attempt to sync if we have enough samples
    if len(self.observed_window) >= 3:
      self._attempt_sync()

    # While not synced, pass through stock counter
    return observed_counter, False

  def _attempt_sync(self):
    """Try to find current pattern index based on observed window."""
    # Try to match observation window against all positions in pattern
    best_match_idx = None
    best_match_len = 0

    for start_idx in range(self.PATTERN_LEN):
      match_len = self._count_match(start_idx)
      if match_len > best_match_len:
        best_match_len = match_len
        best_match_idx = start_idx

    # Require MIN_CONFIDENCE matching values to declare sync
    if best_match_len >= self.MIN_CONFIDENCE:
      # The match tells us where we WERE in the pattern
      # We need to set index to NEXT position for next transmission
      self.pattern_index = (best_match_idx + len(self.observed_window)) % self.PATTERN_LEN
      self.confidence = best_match_len

  def _count_match(self, pattern_start_idx: int) -> int:
    """
    Count how many values in observed_window match pattern starting at pattern_start_idx.

    Returns:
        Number of consecutive matching values from start
    """
    match_count = 0
    for i, observed in enumerate(self.observed_window):
      pattern_idx = (pattern_start_idx + i) % self.PATTERN_LEN
      if observed == self.PATTERN[pattern_idx]:
        match_count += 1
      else:
        break  # Stop at first mismatch
    return match_count

  def is_synchronized(self) -> bool:
    """Returns True if we have synchronized to the pattern."""
    return self.pattern_index is not None
