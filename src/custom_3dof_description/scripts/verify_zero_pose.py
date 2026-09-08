#!/usr/bin/env python3
"""Check actual expanded URDF forward kinematics at q=0, in metres."""
import argparse
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def origin_transform(origin):
    xyz = [0.0] * 3 if origin is None else list(map(float, origin.get('xyz', '0 0 0').split()))
    rpy = [0.0] * 3 if origin is None else list(map(float, origin.get('rpy', '0 0 0').split()))
    if len(xyz) != 3 or len(rpy) != 3 or not all(map(math.isfinite, xyz + rpy)):
        raise ValueError('Origin must contain finite xyz and rpy triples')
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, xyz[0]],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, xyz[1]],
            [-sp, cp*sr, cp*cr, xyz[2]], [0, 0, 0, 1]]


# Chain mirroring the firmware model in ws_arm_base/bsp/manipulator/kinematics.c:
# T_car(x,y,phi) · T_base(0.085,0,0.24) · MDH(theta1,theta2,theta3); rigid wrist.
CHAIN = [
    # (joint, parent, child, type, axis, zero-pose position of child in world)
    ('car_x_joint',   'world',    'car_x',  'prismatic',  (1, 0, 0)),
    ('car_y_joint',   'car_x',    'car_y',  'prismatic',  (0, 1, 0)),
    ('car_yaw_joint', 'car_y',    'chassis', 'continuous', (0, 0, 1)),
    ('arm_mount_joint', 'chassis', 'base_link', 'fixed', None),
    ('joint1',        'base_link', 'link1', 'continuous', (0, 0, 1)),
    ('joint2',        'link1',    'link2',  'continuous', (0, 1, 0)),
    ('joint3',        'link2',    'link3',  'continuous', (0, 1, 0)),
    ('joint4',        'link3',    'link4',  'fixed', None),
    ('link4_to_tool0', 'link4',   'tool0',  'fixed', None),
    ('tool0_to_tcp',  'tool0',    'tcp',    'fixed', None),
]
EXPECTED_LINKS = ['world', 'car_x', 'car_y', 'chassis', 'base_link',
                  'link1', 'link2', 'link3', 'link4', 'tool0', 'tcp']
EXPECTED_ZERO = {
    'base_link': (0.085, 0, 0.24),
    'link1': (0.085, 0, 0.24),
    'link2': (0.085, 0, 0.401),
    'link3': (0.085, 0, 0.592),
    'link4': (0.257, 0, 0.592),
    'tool0': (0.257, 0, 0.592),
    'tcp': (0.349, 0, 0.612),
}


def verify(root):
    links = [link.attrib['name'] for link in root.findall('link')]
    if sorted(links) != sorted(EXPECTED_LINKS):
        raise ValueError(f'Actual links: {links}; expected: {EXPECTED_LINKS}')
    joints = root.findall('joint')
    if [j.attrib['name'] for j in joints] != [c[0] for c in CHAIN]:
        raise ValueError(f'Actual joints: {[j.attrib["name"] for j in joints]}; '
                         f'expected chain order: {[c[0] for c in CHAIN]}')
    by_name = {j.attrib['name']: j for j in joints}
    transforms = {'world': origin_transform(None)}
    for name, parent, child, jtype, axis in CHAIN:
        joint = by_name[name]
        if (joint.find('parent').attrib['link'], joint.find('child').attrib['link']) != (parent, child):
            raise ValueError(f'{name}: expected {parent} -> {child}')
        if joint.get('type') != jtype:
            raise ValueError(f'{name}: expected type {jtype}')
        if axis is not None:
            actual_axis = tuple(map(float, joint.find('axis').attrib['xyz'].split()))
            if actual_axis != axis:
                raise ValueError(f'{name}: actual axis {actual_axis}, expected {axis}')
        elif joint.find('axis') is not None:
            raise ValueError(f'{name}: fixed joint must not define an axis')
        # At q=0 the motion transform is identity, but origin rotations still matter.
        transforms[child] = multiply(transforms[parent], origin_transform(joint.find('origin')))
    success = True
    for link, target in EXPECTED_ZERO.items():
        actual = tuple(transforms[link][i][3] for i in range(3))
        passed = all(math.isfinite(a) and abs(a-b) <= 1e-9 for a, b in zip(actual, target))
        print(f'{"PASS" if passed else "FAIL"} {link}: actual={actual}, expected={target} m')
        success &= passed
    return 0 if success else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model', nargs='?', type=Path, help='Xacro source or expanded URDF')
    args = parser.parse_args()
    model = args.model
    if model is None:
        source = Path(__file__).resolve().parents[1] / 'urdf/custom_3dof.urdf.xacro'
        if source.exists():
            model = source
        else:
            from ament_index_python.packages import get_package_share_directory
            model = Path(get_package_share_directory('custom_3dof_description')) / 'urdf/custom_3dof.urdf.xacro'
    try:
        xml = subprocess.check_output(['xacro', str(model)], text=True) if model.suffix == '.xacro' else model.read_text()
        return verify(ET.fromstring(xml))
    except (OSError, ValueError, KeyError, AttributeError, ET.ParseError, subprocess.CalledProcessError) as exc:
        print(f'ERROR: cannot verify {model}: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
