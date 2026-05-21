from setuptools import find_packages, setup
import os
package_name = 'farmbot_hri'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), [os.path.join(package_name, 'config', 'AutonomousCommand.yaml')])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aura_ws',
    maintainer_email='salome.deoliveira.2026@mumail.ie',
    description='Package containing the Keyboard and Autonomous controllers for the HRI',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "keyboard_controller = farmbot_hri.keyboard_teleop:main",
            "autonomous_controller = farmbot_hri.autonomous_controller:main"
        ],
    },
)
