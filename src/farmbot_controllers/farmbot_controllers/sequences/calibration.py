"""
Axis calibration sequence.

Defines the calibrate step and the sequence that uses it, and registers the sequence.
"""
from dataclasses import dataclass

from farmbot_controllers.sequence_runner.registry import sequence
from farmbot_controllers.sequence_runner.steps import Outcome, Sequence, Step, StepResult


@dataclass
class CalibrateAxis(Step):
    """
    Calibrate one axis (only one of x/y/z should be true True).

    The firmware calibrates a single axis per command, so a full calibration is
    a sequence of these.
    """

    x: bool = False
    y: bool = False
    z: bool = False

    def run(self, hardware, done):
        """Start the axis calibration and report the outcome once it finishes."""
        try:
            hardware.movement.calibrate_axis(
                x=self.x, y=self.y, z=self.z,
                on_done=lambda result: done(hardware.result_to_outcome(result)))
        except Exception as error:  # any client error - report, never hang the engine
            done(StepResult(Outcome.FAILED, str(error)))


@sequence('calibrate_axes')
def calibrate_axes(x: bool = True, y: bool = True, z: bool = True) -> Sequence:
    """
    Build the axis calibration sequence (registered as 'calibrate_axes').

    The calibration order is Z, Y, and X if all the axes are selected.
    """
    steps = []
    if z:
        steps.append(CalibrateAxis(z=True))
    if y:
        steps.append(CalibrateAxis(y=True))
    if x:
        steps.append(CalibrateAxis(x=True))
    return Sequence.of('calibrate_axes', steps)
