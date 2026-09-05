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


def verify(root):
    links = [link.attrib['name'] for link in root.findall('link')]
    expected_links = ['base_link', 'link1', 'link2', 'link3', 'link4', 'tool0', 'tcp']
    if sorted(links) != sorted(expected_links):
        raise ValueError(f'Actual links: {links}; expected: {expected_links}')
    joints = root.findall('joint')
    expected_names = ['joint1', 'joint2', 'joint3', 'joint4', 'link4_to_tool0', 'tool0_to_tcp']
    if sorted(j.attrib['name'] for j in joints) != sorted(expected_names):
        raise ValueError('Expected exactly four named active joints and two named fixed joints')
    by_name = {j.attrib['name']: j for j in joints}
    transforms = {'base_link': origin_transform(None)}
    for i, name in enumerate(expected_names):
        joint = by_name[name]
        parent, child = joint.find('parent').attrib['link'], joint.find('child').attrib['link']
        if (parent, child) != (expected_links[i], expected_links[i+1]):
            raise ValueError(f'{name}: actual {(parent, child)}, expected {(expected_links[i], expected_links[i+1])}')
        if i < 4:
            if joint.get('type') not in ('continuous', 'revolute'):
                raise ValueError(f'{name}: expected rotary joint')
            axis = tuple(map(float, joint.find('axis').attrib['xyz'].split()))
            expected_axis = (0, 0, 1) if i == 0 else (0, -1, 0)
            if axis != expected_axis:
                raise ValueError(f'{name}: actual axis {axis}, expected {expected_axis}')
        elif joint.get('type') != 'fixed':
            raise ValueError(f'{name}: expected fixed joint')
        # At q=0 the motion transform is identity, but origin rotations still matter.
        transforms[child] = multiply(transforms[parent], origin_transform(joint.find('origin')))
    expected = {'link1': (0, 0, 0), 'link2': (0, 0, .161),
                'link3': (0, 0, .352), 'link4': (.172, 0, .352),
                'tool0': (.172, 0, .352), 'tcp': (.264, 0, .372)}
    success = True
    for link, target in expected.items():
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
        source = Path(__file__).resolve().parents[1] / 'urdf/custom_4dof.urdf.xacro'
        if source.exists():
            model = source
        else:
            from ament_index_python.packages import get_package_share_directory
            model = Path(get_package_share_directory('custom_4dof_description')) / 'urdf/custom_4dof.urdf.xacro'
    try:
        xml = subprocess.check_output(['xacro', str(model)], text=True) if model.suffix == '.xacro' else model.read_text()
        return verify(ET.fromstring(xml))
    except (OSError, ValueError, KeyError, AttributeError, ET.ParseError, subprocess.CalledProcessError) as exc:
        print(f'ERROR: cannot verify {model}: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
