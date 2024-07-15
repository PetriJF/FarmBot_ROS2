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
        (os.path.join('share', package_name, 'config'), [os.path.join(package_name, 'config', 'luxonis_camera_config.yaml')]),
        (os.path.join('share', package_name, 'config'), [os.path.join(package_name, 'config', 'standard_camera_config.yaml')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Edward',
    maintainer_email='ejakunskas@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_controller = camera_handler.camera_controller:main',
            'luxonis_camera = camera_handler.luxonis_camera:main',
            'standard_camera = camera_handler.standard_camera:main',
            'realsense_405_camera = camera_handler.realsense_405_camera:main',
        ],
    },
)
