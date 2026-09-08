from itertools import product

import numpy as np
from scipy.optimize import least_squares


# Preserve the decimal MDH rotations from ws_arm_base/bsp/manipulator/kinematics.c.
MDH_A = (0, 0, 0, 0.191, 0, 0, 0.020, 0, 0)
MDH_ALPHA = (0, 1.570796, -3.141593, -3.141593, -3.141593,
             -3.141593, 1.570796, 0, -1.570796)
MDH_D = (0.161, 0, 0, 0, 0, 0, 0.086, 0.178, 0)
BASE_OFFSET = (0.085, 0.0, 0.24)
LIMITS_DEG = np.array([[-90.0, 90.0], [0.0, 90.0], [-90.0, 0.0]])
POSITION_TOLERANCE_M = 1e-5


def _vector3(values):
    vector = np.asarray(values, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError('需要三个有限数值；不接受 NaN 或 Inf。')
    return vector


def validate_angles(angles_deg):
    angles = _vector3(angles_deg)
    if np.any(angles < LIMITS_DEG[:, 0]) or np.any(angles > LIMITS_DEG[:, 1]):
        raise ValueError('关节角超出底层软件限位：[-90,90]、[0,90]、[-90,0] 度。')
    return angles


def _forward_radians(angles_rad):
    theta = (angles_rad[0], 1.570796, angles_rad[1], 0.0,
             angles_rad[2], 0.0, 0.0, 0.0, 0.0)
    transform = np.eye(4)
    transform[:3, 3] = BASE_OFFSET
    for a, alpha, d, angle in zip(MDH_A, MDH_ALPHA, MDH_D, theta):
        ct, st = np.cos(angle), np.sin(angle)
        ca, sa = np.cos(alpha), np.sin(alpha)
        transform = transform @ np.array([
            [ct, -st, 0, a],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0, 0, 0, 1],
        ])
    return transform[:3, 3]


def forward_kinematics(angles_deg):
    """Joint degrees to firmware-frame XYZ in metres, with the chassis fixed."""
    return _forward_radians(np.deg2rad(_vector3(angles_deg)))


def inverse_kinematics(target_xyz_m):
    """Return one in-limit float32-degree solution validated against the target."""
    target = _vector3(target_xyz_m)
    # The sum of MDH translation lengths bounds every pose, independent of rotation.
    reach = sum(np.hypot(a, d) for a, d in zip(MDH_A, MDH_D))
    if np.any(np.abs(target - BASE_OFFSET) > reach + POSITION_TOLERANCE_M):
        raise ValueError('目标超出机械臂最大伸展范围。')

    lower, upper = np.deg2rad(LIMITS_DEG).T
    seeds = [(lower + upper) / 2]
    seeds.extend(lower + np.array(fractions) * (upper - lower)
                 for fractions in product((0.05, 0.5, 0.95), repeat=3)
                 if fractions != (0.5, 0.5, 0.5))
    for seed in seeds:
        result = least_squares(
            lambda angles: _forward_radians(angles) - target,
            seed, bounds=(lower, upper), method='trf',
            ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=500,
        )
        angles = np.rad2deg(result.x).astype(np.float32).astype(float)
        validate_angles(angles)
        residual = np.linalg.norm(forward_kinematics(angles) - target)
        if residual <= POSITION_TOLERANCE_M:
            return angles
    raise ValueError('未找到符合限位和精度要求的解；目标可能不可达或位于奇异附近。')
