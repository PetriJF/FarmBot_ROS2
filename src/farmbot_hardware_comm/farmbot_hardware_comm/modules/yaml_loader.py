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

    def join_path(self, parent_path: str, file: str) -> str:
        """
        Get the path of a child directory.

        Args:
            parent_path {str}: Path
            file {str}: Child file name.
        """
        return os.path.join(parent_path, file)

    def get_directory_package(self, package_folder: str, child_folder: str) -> str:
        """
        Get the path of a child directory inside a ROS2 package.

        Args:
            package_folder {str}: Name of the ROS2 package.
            child_folder {str}: Child directory path inside the package.
        """
        parent_path = get_package_share_directory(package_folder)
        return self.join_path(parent_path, child_folder)

    def load_yaml(self, path: str, file) -> dict:
        """
        Load a YAML configuration file.

        Opens the YAML file and parses its content into a Python dictionary.

        Args:
            file_name {str}: Path to the YAML configuration file.
        """
        file_path = self.join_path(path, file)
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
