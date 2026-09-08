import struct

from .kinematics import validate_angles


def encode_joint_command(angles_deg):
    """Build the firmware 0x12 frame from logical joint degrees, not motor degrees."""
    angles = validate_angles(angles_deg)
    payload = struct.pack('<3f', *angles)
    validate_angles(struct.unpack('<3f', payload))
    body = bytes((0x12, len(payload))) + payload
    return b'\xaa\x55' + body + bytes((sum(body) & 0xff,))
