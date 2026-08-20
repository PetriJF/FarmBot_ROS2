"""
Plant radius sweep sequence.

Defines the plant radius sweep step and the sequence that uses it, and registers the sequence.
"""
from dataclasses import dataclass

from farmbot_controllers.sequence_runner.steps import Outcome, Sequence, Step, StepResult


def parse_plant_list(data: str) -> list:
    """Parse a map_info PlantList reply into a list of (index, x, y) tuples."""
    if not data:
        return []

    plants = []
    for row in data.split('\n'):
        row = row.strip()
        if not row:
            continue
        parts = row.split(' ')
        if len(parts) != 3:
            raise ValueError(f"expected 'index x y', got '{row}'")
        index, x, y = parts
        plants.append((int(index), float(x), float(y)))
    return plants


@dataclass
class MeasureAllPlantRadii(Step):
    """
    Visit every known plant, capture one image over it, and update its radius in the map.

    The plant list is only known at run time, so this single step fetches it and then chains
    move + measure calls for each plant in turn - the engine only ever sees this as one step.
    """

    def run(self, hardware, done):
        """Fetch the plant list, then measure each plant in turn."""
        try:
            hardware.map_info.list_plants(
                on_done=lambda response: self._on_list(hardware, response, done))
        except Exception as error:  # any client error - report, never hang the engine
            done(StepResult(Outcome.FAILED, str(error)))

    def _on_list(self, hardware, response, done):
        """Parse the plant list and start measuring, or fail the sweep."""
        if response is None:
            done(StepResult(Outcome.FAILED, 'map_info list_plants call failed or rejected'))
            return
        try:
            plants = parse_plant_list(response.data)
        except ValueError as error:
            done(StepResult(Outcome.FAILED, f'malformed plant list: {error}'))
            return
        if not plants:
            done(StepResult(Outcome.OK, 'no plants in map'))
            return
        self._measure_next(hardware, plants, 0, done)

    def _measure_next(self, hardware, plants, position, done):
        """Move over the next plant, or finish once every plant has been measured."""
        if position >= len(plants):
            done(StepResult(Outcome.OK, f'measured {len(plants)} plant(s)'))
            return
        _, x, y = plants[position]
        # The camera, not the gantry, needs to end up over the plant - move the gantry to
        # the plant's position minus the camera's known offset from it.
        gantry_x = x - hardware.vision.camera_offset_x
        gantry_y = y - hardware.vision.camera_offset_y
        try:
            hardware.movement.move_gantry_abs(
                gantry_x, gantry_y, hardware.vision.radius_capture_z,
                on_done=lambda result: self._on_move(hardware, plants, position, result, done))
        except Exception as error:
            done(StepResult(Outcome.FAILED, str(error)))

    def _on_move(self, hardware, plants, position, result, done):
        """Measure the plant just reached, or forward the move's outcome if it fell short."""
        outcome = hardware.to_outcome(result)
        if not outcome.ok:
            done(outcome)
            return
        index, _, _ = plants[position]
        try:
            hardware.vision.measure_plant_radius(
                index,
                on_done=lambda response: self._on_measure(
                    hardware, plants, position, response, done))
        except Exception as error:
            done(StepResult(Outcome.FAILED, str(error)))

    def _on_measure(self, hardware, plants, position, response, done):
        """Advance to the next plant, skipping a bare plant but failing on any other error."""
        if response is None:
            done(StepResult(Outcome.FAILED, 'plant_radius call failed or rejected'))
            return
        if not response.success and not response.no_plant_detected:
            done(StepResult(Outcome.FAILED, response.message))
            return
        if not response.success:
            hardware.vision.node.get_logger().warn(
                f'plant {plants[position][0]}: {response.message}')
        self._measure_next(hardware, plants, position + 1, done)


def measure_all_plant_radii() -> Sequence:
    """Build the plant radius sweep sequence (registered as 'plant_radius_sweep')."""
    return Sequence.of('plant_radius_sweep', MeasureAllPlantRadii())
