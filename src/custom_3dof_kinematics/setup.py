from setuptools import setup


package_name = 'custom_3dof_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyQt5'],
    zip_safe=True,
    maintainer='Custom arm maintainer',
    maintainer_email='maintainer@example.com',
    description='Firmware-aligned three-joint inverse kinematics and offline command preview.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_test_gui = custom_3dof_kinematics.gui:main',
        ],
    },
)
