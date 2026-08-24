"""
Seed planting sequence.

Define the steps required to seed the plants.
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
class VacuumDevice(Step):
    """Control the vacuum device through a hardware pin."""

    vacuum_pin: int = 9
    state: int = 0
    pin_mode: bool = False
    pulse: bool = False

    def run(self, hardware, done):
        """Set the vacuum device state and report the operation result."""
        try:
            hardware.devices.set_pin_value(
                pin=self.vacuum_pin, value=self.state, pin_mode=self.pin_mode, pulse=self.pulse,
                on_done=lambda result: done(hardware.to_outcome(result)))
        except Exception as error:  # any client error - report, never hang the engine
            done(StepResult(Outcome.FAILED, str(error)))


def seed_plant(plant_x: float = 0.0, plant_y: float = 0.0, plant_z: float = 0.0,
               tray_x: float = 0.0, tray_y: float = 0.0, tray_z: float = 0.0,
               z_increment: float = 0.0) -> Sequence:
    """
    Build a sequence to pick up a seed from the seed tray and plant it.

    The sequence moves the gantry to the seed tray, activates the vacuum
    pump, picks up a seed, moves to the target plant position, releases
    the seed, and retracts the seeder.

    Args:
        plant_x (float): Target X coordinate where the seed will be planted.
        plant_y (float): Target Y coordinate where the seed will be planted.
        plant_z (float): Target Z coordinate where the seed will be planted.
        tray_x (float): X coordinate of the seed tray.
        tray_y (float): Y coordinate of the seed tray.
        tray_z (float): Z coordinate of the seed tray.
        z_increment (float): Additional Z height used for safe gantry movements.
    """
    steps = []

    # Go over seed tray
    steps.append(MoveGantry(x_coord=tray_x, y_coord=tray_y))
    # Go over seed tray at safe z
    steps.append(MoveGantry(x_coord=tray_x, y_coord=tray_y, z_coord=tray_z + z_increment))

    # Turn on vacuum pump
    steps.append(VacuumDevice(state=1))

    # Collect a seed
    steps.append(MoveGantry(x_coord=tray_x, y_coord=tray_y, z_coord=tray_z))
    # Retract with the seed
    steps.append(MoveGantry(x_coord=tray_x, y_coord=tray_y, z_coord=tray_z + z_increment))
    steps.append(MoveGantry(x_coord=tray_x, y_coord=tray_y))
    # Go to the plant at safe z
    steps.append(MoveGantry(x_coord=plant_x, y_coord=plant_y))
    steps.append(MoveGantry(x_coord=plant_x, y_coord=plant_y, z_coord=plant_z + z_increment))
    # Plant the seed
    steps.append(MoveGantry(x_coord=plant_x, y_coord=plant_y, z_coord=plant_z))

    # Turn off vacuum pump
    steps.append(VacuumDevice(state=0))

    # Retract the empty seeder
    steps.append(MoveGantry(x_coord=plant_x, y_coord=plant_y))

    return Sequence.of('seed_plant', steps)
