from setuptools import find_packages, setup
import os, glob

package_name = 'farmbot_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), [os.path.join(package_name, 'config', 'firmwareDefault.yaml')]),
        (os.path.join('share', package_name, 'config'), [os.path.join(package_name, 'config', 'labFB.yaml')]),
        (os.path.join('share', package_name, 'config'), [os.path.join(package_name, 'config', 'defaultParameterConfig.yaml')]),
        #(os.path.join('share', package_name, 'config'), glob.glob(package_name + 'config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='farmbotdev',
    maintainer_email='jamespetri28@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "farmbot_controller = farmbot_controllers.farmbot_controller:main",
            "keyboard_controller = farmbot_controllers.keyboard_teleop:main",
            "panel_controller = farmbot_controllers.panel_controller:main",
            "param_conf_server = farmbot_controllers.conf_mng_servers:main"
        ],
    },
)
