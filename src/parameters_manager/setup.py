from setuptools import find_packages, setup
import os, glob

package_name = 'parameters_manager'

setup(
    name=package_name,
    version='1.0.0',
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
    description='Package containing the parameter manager for the ROS2 Farmbot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "param_conf_server = parameters_manager.parameter_manager:main",
        ],
    },
)
