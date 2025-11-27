"""
TRULY CORRECTED LCA Target Angle Encoder
==========================================
Byte6=186 is NOT the active 0° command - it's a "no command" state!

When LCA is active and commanding 0°, use:
- byte6=128, byte7=0 (approaching from LEFT or staying at 0)
- byte6=255, byte7~255 (approaching from RIGHT)

Encoding (when LCA ACTIVE):
- LEFT (angle > 0): byte6=128, PSCM = 1.731 * byte7 + 2.2
- RIGHT (angle < 0): byte6=255, PSCM = 1.731 * byte7 - 442.1
- NEUTRAL (angle = 0): byte6=128, byte7=0 (default to LEFT baseline)

Byte6=186, byte7=0 should ONLY be used when LCA is inactive/standby!
"""

SCALE = 1.731  # degrees per count (same for both directions)

class LCATargetAngleEncoder:
    """Encode/decode target steering angles to LCA_5 bytes 6+7"""

    @staticmethod
    def encode(target_angle_deg):
        """
        Encode target steering angle to (byte6, byte7) for ACTIVE LCA

        Args:
            target_angle_deg: Target angle in degrees
                            Positive = LEFT, Negative = RIGHT, 0 = straight

        Returns:
            (byte6, byte7): Tuple of bytes for LCA_5 message

        Note:
            For 0° or near-0° angles, defaults to byte6=128, byte7=0
            This is the "neutral" command when LCA is ACTIVE
        """

        # For angles at or near 0°, use LEFT baseline (byte6=128, byte7=0)
        if abs(target_angle_deg) < 0.5:
            return (128, 0)

        # LEFT direction
        elif target_angle_deg > 0:
            # PSCM = 1.731 * byte7 + 2.2
            # byte7 = (PSCM - 2.2) / 1.731
            byte7 = int(round((target_angle_deg - 2.2) / SCALE))
            byte7 = max(0, min(136, byte7))  # Clamp to observed range
            return (128, byte7)

        # RIGHT direction
        else:
            # PSCM = 1.731 * byte7 - 442.1
            # byte7 = (PSCM + 442.1) / 1.731
            byte7 = int(round((target_angle_deg + 442.1) / SCALE))
            byte7 = max(150, min(255, byte7))  # Clamp to observed range
            return (255, byte7)

    @staticmethod
    def encode_inactive():
        """
        Return the byte values for when LCA is inactive/standby

        Returns:
            (186, 0): The "no command" state
        """
        return (186, 0)

    @staticmethod
    def decode(byte6, byte7):
        """
        Decode (byte6, byte7) to target steering angle

        Args:
            byte6, byte7: LCA_5 message bytes

        Returns:
            angle_deg: Target steering angle in degrees
                      None if byte6=186 (inactive/no command state)
        """
        if byte6 == 186:
            # This is the "inactive" or "no active command" state
            # NOT a 0° command!
            return None

        elif byte6 == 128:  # LEFT or NEUTRAL
            # PSCM = 1.731 * byte7 + 2.2
            return SCALE * byte7 + 2.2

        elif byte6 == 255:  # RIGHT
            # PSCM = 1.731 * byte7 - 442.1
            return SCALE * byte7 - 442.1

        else:
            # Unknown byte6 value
            return None