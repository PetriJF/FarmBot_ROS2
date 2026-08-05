"""Generalist adder: wrap any single client call as a one-step sequence."""
from dataclasses import dataclass
from typing import Callable

from farmbot_controllers.sequence_runner.steps import Outcome, Sequence, Step, StepResult


@dataclass(repr=False)
class Call(Step):
    """One client call; its result is mapped by hardware.to_outcome."""

    label: str
    invoke: Callable   # invoke(hardware, on_done)

    def run(self, hardware, done):
        """Run the call, reporting its outcome (or FAILED on a client error)."""
        try:
            self.invoke(hardware, lambda raw: done(hardware.to_outcome(raw)))
        except Exception as error:
            done(StepResult(Outcome.FAILED, str(error)))

    def __repr__(self):
        """Show the label in status output."""
        return self.label


def single_call(name: str, invoke: Callable) -> Sequence:
    """Build a one-step sequence around invoke(hardware, on_done)."""
    return Sequence.of(name, Call(name, invoke))
