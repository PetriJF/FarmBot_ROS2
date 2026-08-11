"""
Axis calibration sequence.

Defines the calibrate step and the sequence that uses it, and registers the sequence.
"""
from dataclasses import dataclass

from farmbot_controllers.sequence_runner.steps import Outcome, Sequence, Step, StepResult


@dataclass
class FindHome(Step):
    """
    Find the home position for one axis (only one of x/y/z should be true True).

    The firmware finds the home position for a single axis per command, so a full home finding is
    a sequence of these.
    """

    x: bool = False
    y: bool = False
    z: bool = False

    def run(self, hardware, done):
        """Start the axis homing and report the outcome once it finishes."""
        try:
            hardware.movement.find_axis_home(
                x=self.x, y=self.y, z=self.z,
                on_done=lambda result: done(hardware.result_to_outcome(result)))
        except Exception as error:  # any client error - report, never hang the engine
            done(StepResult(Outcome.FAILED, str(error)))

def find_home(x: bool = True, y: bool = True, z: bool = True) -> Sequence:
    """
    Build the axis homing sequence (registered as 'find_home').

    The homing order is Z, Y, and X if all the axes are selected.
    """
    steps = []
    if z:
        steps.append(FindHome(z=True))
    if y:
        steps.append(FindHome(y=True))
    if x:
        steps.append(FindHome(x=True))
    return Sequence.of('find_home', steps)