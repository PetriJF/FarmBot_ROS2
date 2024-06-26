from setuptools import find_packages, setup
import os

package_name = 'map_handler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), [os.path.join(package_name, 'config', 'plant_reference.yaml')]),
        (os.path.join('share', package_name, 'config'), [os.path.join(package_name, 'config', 'map_references.yaml')]),
        (os.path.join('share', package_name, 'config'), [os.path.join(package_name, 'config', 'tool_reference.yaml')]),
        (os.path.join('share', package_name, 'config'), [os.path.join(package_name, 'config', 'tray_reference.yaml')]),
        (os.path.join('share', package_name, 'config'), [os.path.join(package_name, 'config', '16_seed_tray.yaml')]),
        (os.path.join('share', package_name, 'config'), [os.path.join(package_name, 'config', 'watering_guide.yaml')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='James',
    maintainer_email='jamespetri28@gmail.com',
    description='Package handling map information and seed mapping. Also handles sequencing for some tasks',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "map_controller = map_handler.map_controller:main"
        ],
    },
)
