"""
The sequence registry: maps a name to a builder so sequences run by name.

A definition decorates its builder with @sequence('name'), so importing the
'sequence' package auto-loads the definitions, so the decorators run and the
node needs no hard-coded list.
"""
from typing import Callable, List

from farmbot_controllers.sequence_runner.steps import Sequence

# name -> zero-arg builder returning a Sequence
_REGISTRY: dict = {}

Builder = Callable[[], Sequence]


def sequence(name: str) -> Callable[[Builder], Builder]:
    """
    Register a builder under 'name' (decorator).

    The builder must be callable with no args - that is what 'build' invokes.
    Duplicate names raise at import phase.
    """
    def register(builder: Builder) -> Builder:
        if name in _REGISTRY:
            raise ValueError(f"duplicate sequence name '{name}'")
        _REGISTRY[name] = builder
        return builder
    return register


def build(name: str) -> Sequence:
    """Build the registered sequence `name` (raises KeyError if unknown)."""
    return _REGISTRY[name]()


def is_registered(name: str) -> bool:
    """Return True if `name` is a registered sequence."""
    return name in _REGISTRY


def names() -> List[str]:
    """Return the sorted list of registered sequence names."""
    return sorted(_REGISTRY)
