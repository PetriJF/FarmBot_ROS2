"""
Container class definitions for Step, Outcomes, and the Sequence.

A Step is a single action that the farmbot/code takes. A Sequence is a named,
ordered tuple of steps.
"""
from dataclasses import dataclass, field
from typing import Callable


class Outcome:
    """How one executed step finished."""

    OK = 'ok'              # succeeded - advance
    FAILED = 'failed'      # failed - fail the sequence
    ABORTED = 'aborted'    # aborted - pause (resumable)
    ESTOPPED = 'estopped'  # e-stopped - cancel the sequence


@dataclass
class StepResult:
    """Outcome of one executed step, passed to the engine's done-callback."""

    outcome: str
    message: str = ''

    @property
    def ok(self) -> bool:
        """Return True only when the step fully succeeded."""
        return self.outcome == Outcome.OK


class Step:
    """
    A single action in a sequence.

    Concrete steps subclass this and implement run(): start the action against
    the injected hardware and call done(result) exactly once when it finishes.
    """

    def run(self, hardware, done: Callable[[StepResult], None]) -> None:
        """Start the action; call done(result) exactly once when it finishes."""
        raise NotImplementedError


@dataclass(frozen=True)
class Sequence:
    """
    A named and ordered ordered collection of steps.

    Frozen so it can't be mutated mid-run. 'Sequence.of' composes one from steps
    and/or smaller sequences.
    A sequence can be extended by smaller sequences on formation.
    """

    name: str
    steps: tuple = field(default_factory=tuple)

    @classmethod
    def of(cls, name: str, *parts) -> 'Sequence':
        """Build from any mix of Steps, Sequences and step iterables (flattened in order)."""
        collected: list = []
        for part in parts:
            if isinstance(part, Sequence):
                collected.extend(part.steps)
            elif isinstance(part, Step):
                collected.append(part)
            else:
                collected.extend(part)
        return cls(name, tuple(collected))

    def __len__(self) -> int:
        """Return the number of steps in the sequence."""
        return len(self.steps)
