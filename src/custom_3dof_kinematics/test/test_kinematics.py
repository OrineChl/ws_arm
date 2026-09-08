import ctypes
from itertools import product
from pathlib import Path
import shutil
import struct
import subprocess

import numpy as np
import pytest

from custom_3dof_kinematics.kinematics import (
    LIMITS_DEG, POSITION_TOLERANCE_M, forward_kinematics,
    inverse_kinematics, validate_angles,
)
from custom_3dof_kinematics.protocol import encode_joint_command


SAMPLES = list(product((-90, 90), (0, 90), (-90, 0)))
SAMPLES += [[0, 0, 0], [35, 40, -30]]
SAMPLES += [[q1, q2, q3] for q1 in (-90, 0, 90) for q2 in (0, 45, 90)
            for q3 in (-85.67, -85.667, -85.66)]
SAMPLES += [[0, 0, -90], [90, 0, -90], [45, 0, -89.9999]]
SAMPLES += [[q1, np.rad2deg(np.arctan2(0.020, 0.455)) + delta, -90]
            for q1 in (-90, 0, 90) for delta in (-0.0001, 0, 0.0001)]
SAMPLES += [[q1, q2, q3] for q1 in (-90, 0, 90) for q2 in (0, 45, 90)
            for q3 in (-90, -45, 0)]
SAMPLES += list(np.random.default_rng(42).uniform(LIMITS_DEG[:, 0], LIMITS_DEG[:, 1], (50, 3)))


@pytest.mark.parametrize('original', SAMPLES)
def test_fk_ik_roundtrip(original):
    target = forward_kinematics(original)
    solution = inverse_kinematics(target)
    validate_angles(solution)
    transmitted = struct.unpack('<3f', encode_joint_command(solution)[4:16])
    np.testing.assert_array_equal(solution, transmitted)
    assert np.linalg.norm(forward_kinematics(transmitted) - target) <= POSITION_TOLERANCE_M


def test_zero_pose():
    np.testing.assert_allclose(forward_kinematics([0, 0, 0]), [0.349, 0, 0.612], atol=1e-6)


@pytest.mark.parametrize('target', [
    [10, 0, 0], [1e300, 0, 0], [0.085, 0, 0.401],
    forward_kinematics([0, -60, 0]),
    [0, 0, float('nan')], [float('inf'), 0, 0], [], [0, 0],
    ['not a number', 0, 0],
])
def test_invalid_or_unreachable_targets(target):
    with pytest.raises(ValueError):
        inverse_kinematics(target)


def test_command_wire_format():
    frame = encode_joint_command([30, 45, -60])
    assert len(frame) == 17
    assert frame.hex(' ') == 'aa 55 12 0c 00 00 f0 41 00 00 34 42 00 00 70 c2 f7'
    assert struct.unpack('<3f', frame[4:16]) == (30, 45, -60)
    assert frame[-1] == sum(frame[2:-1]) & 0xff
    assert encode_joint_command([0, 0, 0]) == b'\xaa\x55\x12\x0c' + b'\0' * 12 + b'\x1e'


@pytest.mark.parametrize('angles', [
    [-90.001, 0, 0], [90.001, 0, 0], [0, -0.001, 0],
    [0, 90.001, 0], [0, 0, -90.001], [0, 0, 0.001],
    [0, float('nan'), 0], [float('inf'), 0, 0], [0, 0],
])
def test_protocol_rejects_invalid_angles(angles):
    with pytest.raises(ValueError):
        encode_joint_command(angles)


def test_fk_against_actual_firmware_c(tmp_path):
    firmware = Path(__file__).resolve().parents[3] / 'ws_arm_base/bsp/manipulator/kinematics.c'
    if not firmware.exists() or shutil.which('gcc') is None:
        pytest.skip('Firmware checkout and host gcc are needed for this independent comparison.')
    source = firmware.read_text().split('void Display_EndEffector_Data')[0]
    source = source.replace('#include "kinematics.h"', '').replace('#include "bsp_ili9341_lcd.h"', '')
    source = 'float g_Current_Joint_Angle[6];\nfloat g_Current_Joint_Velocity[6];\n' + source
    source += '''
void firmware_fk(const float angles[3], float output[3]) {
    float q[6] = {0, 0, 0, angles[0]*DEG_TO_RAD, angles[1]*DEG_TO_RAD, angles[2]*DEG_TO_RAD};
    forward_kinematics(q, output);
}
'''
    c_file = tmp_path / 'firmware_fk.c'
    library_file = tmp_path / 'firmware_fk.so'
    c_file.write_text(source)
    subprocess.run(['gcc', '-shared', '-fPIC', '-O0', str(c_file), '-lm', '-o', str(library_file)],
                   check=True, capture_output=True)
    library = ctypes.CDLL(str(library_file))
    vector = ctypes.c_float * 3
    library.firmware_fk.argtypes = [ctypes.POINTER(ctypes.c_float)] * 2
    library.firmware_fk.restype = None
    for angles in SAMPLES:
        output = vector()
        library.firmware_fk(vector(*angles), output)
        np.testing.assert_allclose(forward_kinematics(angles), list(output), atol=1e-6, rtol=0)
