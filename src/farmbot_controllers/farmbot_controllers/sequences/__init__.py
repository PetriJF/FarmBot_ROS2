"""Sequence definitions for auto discovery."""
import importlib
import pkgutil

for _module in pkgutil.iter_modules(__path__):
    if not _module.name.startswith('_'):
        importlib.import_module(f'{__name__}.{_module.name}')
