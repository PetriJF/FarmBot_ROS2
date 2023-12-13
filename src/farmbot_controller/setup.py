from setuptools import find_packages, setup

package_name = 'farmbot_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='farmbotdev',
    maintainer_email='jamespetri28@gmail.com',
    description='This Package deals with the interface interpreter that receives commands and forwards them as needed to the relevant nodes.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "interface_controller = farmbot_controller.control_interface_handler:main",
            "gantry_controller = farmbot_controller.gantry_movement_controller:main"
        ],
    },
)
