"""
Watering sequence.

Define the steps required to move the gantry to a plant and activate the water pump
for a specified pulse duration.
"""
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
                x_coord=self.x_coord, y_coord=self.y_coord, z_coord=self.z_coord,
                on_done=lambda result: done(hardware.to_outcome(result)))
        except Exception as error:  # any client error - report, never hang the engine
            done(StepResult(Outcome.FAILED, str(error)))


@dataclass
class WaterPulse(Step):
    """Activate the water pump using a configurable pin pulse."""

    water_pin: int = 8
    value1: int = 1
    delay: int = 0
    pin_mode: bool = False
    pulse: bool = True
    value2: int = 0

    def run(self, hardware, done):
        """Set the vacuum device state and report the operation result."""
        try:
            hardware.devices.set_pin_value(
                pin=self.water_pin, value=self.value1, pin_mode=self.pin_mode, pulse=self.pulse,
                delay_ms=self.delay, value2=self.value2,
                on_done=lambda result: done(hardware.to_outcome(result)))
        except Exception as error:  # any client error - report, never hang the engine
            done(StepResult(Outcome.FAILED, str(error)))


def water_plant(plant_x: float = 0.0, plant_y: float = 0.0, plant_z: float = 0.0,
                delay_ms: int = 0.0) -> Sequence:
    """
    Build a sequence to water a plant at the specified position.

    The sequence moves the gantry to the plant position and activates
    the water pump for the specified pulse duration.

    Args:
        plant_x (float): X coordinate of the plant.
        plant_y (float): Y coordinate of the plant.
        plant_z (float): Z coordinate of the plant.
        delay_ms (int): Duration of the water pulse in milliseconds.
    """
    steps = []

    # Go to seed location
    steps.append(MoveGantry(x_coord=plant_x, y_coord=plant_y, z_coord=plant_z))

    # Turn on water pump
    steps.append(WaterPulse(delay=delay_ms))

    return Sequence.of('water_plant', steps)
