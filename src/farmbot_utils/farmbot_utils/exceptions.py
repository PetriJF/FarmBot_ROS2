"""
Custom exception classes used throughout the FarmBot package.

This module defines project-specific exceptions to report common error
conditions such as unavailable servers, FCode encoding failures, and
YAML file parsing errors.
"""


class ServerError(Exception):
    """Raised when a server is not available."""

    pass


class EncodeError(Exception):
    """Raised when a request cannot be encoded into a valid FCode command."""

    pass


class InputError(Exception):
    """Raised when an input command is invalid."""

    pass


class YAMLError(Exception):
    """Raised when an error occurs while reading or parsing a YAML file."""

    pass
