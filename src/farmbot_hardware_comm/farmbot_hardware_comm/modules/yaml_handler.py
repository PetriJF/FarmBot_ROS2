"""
YAML utility module for ROS2 packages.

Provides helper methods to retrieve package directories, safely load
YAML configuration files and save informations in YAML file.
"""
import os

from ament_index_python.packages import get_package_share_directory

from farmbot_hardware_comm.modules.exceptions import YAMLError

import yaml


class YAMLHandler:
    """
    Helper class for handling YAML configuration files.

    Provides static utility methods to locate configuration directories
    inside ROS2 packages, load YAML files safely and save informations in YAML file.
    """

    @staticmethod
    def join_path(directory_path: str, file: str) -> str:
        """
        Join the directory path and a file name into a single path.

        Args:
            directory_path {str}: Base directory path.
            file {str}: file name.
        """
        return os.path.join(directory_path, file)

    @staticmethod
    def make_dir(path: str):
        """
        Create a directory and its parent directories if they do not exist.

        Args:
            path (str): Directory path to create.
        """
        os.makedirs(path, exist_ok=True)

    @staticmethod
    def existing_path(path: str) -> bool:
        """
        Check whether a file or directory exists at the given path.

        Args:
            path (str): Path to the file or directory to check.
        """
        return os.path.exists(path)

    @staticmethod
    def get_directory_package(package_name: str, file_rel_path: str) -> str:
        """
        Get the path of a file directory inside a ROS2 package.

        Args:
            package_name {str}: Name of the ROS2 package.
            file_rel_path {str}: File directory path inside the package.
        """
        directory_path = get_package_share_directory(package_name)
        return YAMLHandler.join_path(directory_path, file_rel_path)

    @staticmethod
    def load_yaml(path='', file='') -> dict:
        """
        Load a YAML configuration file.

        Opens the YAML file and parses its content into a Python dictionary.

        Args:
            path {str}: Directory containing the YAML file.
            file {str}: Name of the YAML configuration file.
        """
        if path == '':
            raise YAMLError('Path not set for retrieving the parameter config file')
        if file == '':
            raise YAMLError('File name not set')
        if not YAMLHandler.existing_path(path):
            raise YAMLError(f'Invalid YAML directory path: {path}')

        file_path = YAMLHandler.join_path(path, file)
        if not YAMLHandler.existing_path(file_path):
            raise YAMLError(f'YAML file not found: {file_path}')

        with open(file_path, 'r') as config_file:
            loaded_data = yaml.safe_load(config_file)
            if isinstance(loaded_data, dict):
                return loaded_data
            else:
                raise YAMLError('Invalid YAML file format..')

    @staticmethod
    def save_to_yaml(data: dict, path='', file_name=''):
        """
        Save a Python dictionary into a YAML configuration file.

        Creates or overwrites a YAML file with the provided data.

        Args:
            data (dict): Data to save in the YAML file.
            path (str): Directory where the YAML file will be saved.
            file_name (str): Name of the YAML configuration file.
        """
        if path == '':
            raise YAMLError('Path not set for retrieving the parameter config file')
        if file_name == '':
            raise YAMLError('File name not set')
        if not isinstance(data, dict):
            raise YAMLError('Invalid YAML file format..')

        os.makedirs(path, exist_ok=True)

        file_path = YAMLHandler.join_path(path, file_name)
        with open(file_path, 'w') as yaml_file:
            yaml.dump(data, yaml_file, default_flow_style=False)
