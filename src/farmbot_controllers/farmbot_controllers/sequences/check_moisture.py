"""
Soil moisture checking sequence.

Define the steps required to move the gantry to a probing
location, read the soil moisture sensor, and return the gantry to its
initial height.
"""
import time
from dataclasses import dataclass

from farmbot_controllers.sequence_runner.steps import Outcome, Sequence, Step, StepResult


@dataclass
class MoveGantry(Step):
    """Move the gantry to an absolute position."""

    x_coord: float = 0.0
    y_coord: float = 0.0
    z_coord: float = 0.0

    def run(self, hardware, done):
        """Move the gantry to the specified position and report the result."""
        try:
            hardware.movement.move_gantry_abs(
                x=self.x_coord, y=self.y_coord, z=self.z_coord,
                on_done=lambda result: done(hardware.result_to_outcome(result)))
        except Exception as error:  # any client error - report, never hang the engine
            done(StepResult(Outcome.FAILED, str(error)))


@dataclass
class Pause(Step):
    """Pause the sequence for a specified duration."""

    delay: int = 0

    def run(self, hardware, done):
        """Set the vacuum device state and report the operation result."""
        try:
            hardware.states.abort_movement(
                on_done=lambda result: done(hardware.result_to_outcome(result)))
            time.sleep(self.delay)
            hardware.states.abort_movement(
                on_done=lambda result: done(hardware.result_to_outcome(result)))
        except Exception as error:  # any client error - report, never hang the engine
            done(StepResult(Outcome.FAILED, str(error)))


@dataclass
class ReadSoil(Step):
    """Read the soil moisture sensor through a hardware pin."""

    readsoil_pin: int = 59
    pin_mode: bool = False

    def run(self, hardware, done):
        """Set the vacuum device state and report the operation result."""
        try:
            hardware.devices.read_pin(
                pin=self.readsoil_pin, pin_mode=self.pin_mode,
                on_done=lambda result: done(hardware.result_to_outcome(result)))
        except Exception as error:  # any client error - report, never hang the engine
            done(StepResult(Outcome.FAILED, str(error)))


def check_moisture(max_z: float = 0.0, tick_delay: int = 0,
                   x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Sequence:
    """
    Build a sequence to check the soil moisture at a specified location.

    The sequence moves the gantry to the probing location, lowers it to
    the specified probing height, pauses briefly, reads the soil moisture
    sensor, and raises the gantry back to its initial height.

    Args:
        max_x (float): Maximum X coordinate for the probing location.
        max_y (float): Maximum Y coordinate for the probing location.
        max_z (float): Z coordinate used for the probing position.
        x (float): X coordinate of the probing location.
        y (float): Y coordinate of the probing location.
        z (float): Safe Z coordinate used before and after probing.
        tick_delay (int): Delay between sequence steps.
    """
    steps = []

    # Go over probing location
    steps.append(MoveGantry(x_coord=x, y_coord=y, z_coord=z))
    # Lower to probing location
    steps.append(MoveGantry(x_coord=x, y_coord=y, z_coord=max_z))

    # Wait for 2 ticks
    steps.append(Pause(delay=tick_delay))

    # Probe the moisture value
    steps.append(ReadSoil(pin_mode=True))

    # Raise from probing location
    steps.append(MoveGantry(x_coord=x, y_coord=y, z_coord=z))

    return Sequence.of('check_moisture', steps)
