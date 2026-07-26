"""
YAML utility module for ROS2 packages.

Provides helper methods to retrieve package directories and safely load
YAML configuration files.
"""
import os

from ament_index_python.packages import get_package_share_directory

import yaml


class YAMLLoader:
    """
    Helper class for handling YAML configuration files.

    Provides static utility methods to locate configuration directories
    inside ROS2 packages and load YAML files safely.
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
    def get_directory_package(package_name: str, file_rel_path: str) -> str:
        """
        Get the path of a file directory inside a ROS2 package.

        Args:
            package_name {str}: Name of the ROS2 package.
            file_rel_path {str}: File directory path inside the package.
        """
        directory_path = get_package_share_directory(package_name)
        return YAMLLoader.join_path(directory_path, file_rel_path)

    @staticmethod
    def load_yaml(path: str, file: str) -> dict:
        """
        Load a YAML configuration file.

        Opens the YAML file and parses its content into a Python dictionary.

        Args:
            path {str}: Directory containing the YAML file.
            file {str}: Name of the YAML configuration file.
        """
        file_path = YAMLLoader.join_path(path, file)
        with open(file_path, 'r') as config_file:
            return yaml.safe_load(config_file)
