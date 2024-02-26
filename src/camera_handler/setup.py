from setuptools import find_packages, setup
import os
package_name = 'camera_handler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), [os.path.join(package_name, 'config', 'camera_config.yaml')]),

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
            "luxonis_node = camera_handler.luxonis_camera:main",
            "luxonis_publisher = camera_handler.luxonis_publisher:main",
            "luxonis_subscriber = camera_handler.luxonis_subscriber:main",
        ],
    },
)
