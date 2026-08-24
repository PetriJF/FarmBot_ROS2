"""
Soil moisture checking sequence.

Define the steps required to move the gantry to a probing
location, read the soil moisture sensor, and return the gantry to its
initial height.
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
                x=self.x_coord, y=self.y_coord, z=self.z_coord,
                on_done=lambda result: done(hardware.to_outcome(result)))
        except Exception as error:  # any client error - report, never hang the engine
            done(StepResult(Outcome.FAILED, str(error)))


@dataclass
class CheckMountedTool(Step):
    """Read the soil moisture sensor through a hardware pin."""

    tool_pin: int = 63
    pin_mode: bool = False

    def run(self, hardware, done):
        """Set the vacuum device state and report the operation result."""
        try:
            hardware.devices.read_pin(
                pin=self.tool_pin, pin_mode=self.pin_mode,
                on_done=lambda result: done(self._update_soil_moisture(hardware, result, done)))
        except Exception as error:  # any client error - report, never hang the engine
            done(StepResult(Outcome.FAILED, str(error)))

    def _print(self, hardware, result, done):
        """Print."""
        outcome = hardware.to_outcome(result)
        if outcome.ok:
            hardware.devices.node.get_logger().info(
                f"Tool {'mounted' if result.value else 'unmounted'} successfully")
        done(outcome)


def mount_tool(x: float = 0.0, y: float = 0, z: float = 0.0,
               x_inc: float = 0.0, y_inc: float = 0.0, z_inc: float = 0.0) -> Sequence:
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

    # Go to tool position at a safe z distance over it
    steps.append(MoveGantry(x_coord=x, y_coord=y, z_coord=z+z_inc))
    # Lower the z axis until the exact tool mounting position
    steps.append(MoveGantry(x_coord=x, y_coord=y, z_coord=z))
    # Move towards the release position
    steps.append(MoveGantry(x_coord=x+x_inc, y_coord=y+y_inc, z_coord=z))
    # Raise the tool head to a safe z-axis value
    steps.append(MoveGantry(x_coord=x+x_inc, y_coord=y+y_inc, z_coord=z+z_inc))

    # Check if tool was mounted properly
    steps.append(CheckMountedTool(pin_mode=False))

    return Sequence.of('mount_tool', steps)
