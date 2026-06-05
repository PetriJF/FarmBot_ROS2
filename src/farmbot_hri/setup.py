from setuptools import find_packages, setup
import glob
import os
package_name = 'farmbot_hri'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Salome',
    maintainer_email='salome.deoliveira.2026@mumail.ie',
    description='Package used for human robot interaction',
    license='MIT',
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
