"""
Full-bed image stitch sequence.

Defines the bed stitch sweep step and the sequence that uses it, and registers the sequence.
"""
from dataclasses import dataclass

from farmbot_controllers.sequence_runner.steps import Outcome, Sequence, Step, StepResult


def parse_map_size(data: str) -> tuple:
    """Parse a map_info MapSize reply ('x_len y_len' in mm) into an (x, y) tuple."""
    parts = data.strip().split(' ') if data else []
    if len(parts) != 2:
        raise ValueError(f"expected 'x_len y_len', got '{data}'")
    x_len, y_len = (float(value) for value in parts)
    if x_len <= 0.0 or y_len <= 0.0:
        raise ValueError(f'bed dimensions are not set on the map ({x_len} x {y_len})')
    return (x_len, y_len)


def _axis_positions(length: float, step: float) -> list:
    """Return 0, step, 2*step, ... up to length, with the final stop snapped to length."""
    positions = []
    position = 0.0
    while position < length:
        positions.append(position)
        position += step
    if not positions or positions[-1] != length:
        positions.append(length)
    return positions


def grid_positions(x_len: float, y_len: float, step: float) -> list:
    """
    Build a column-major serpentine grid covering (0, 0) to (x_len, y_len).

    Each column runs the y axis end to end, alternating direction so consecutive
    positions are always adjacent - the gantry never crosses the bed empty.
    """
    if step <= 0.0:
        raise ValueError(f'stitch_step_mm must be positive, got {step}')

    xs = _axis_positions(x_len, step)
    ys = _axis_positions(y_len, step)

    positions = []
    for column, x in enumerate(xs):
        column_ys = ys if column % 2 == 0 else list(reversed(ys))
        positions.extend((x, y) for y in column_ys)
    return positions


@dataclass
class StitchWholeBed(Step):
    """
    Sweep a grid covering the whole bed, stitching one frame per position into the bed map.

    The bed dimensions are only known at run time, so this single step fetches them and
    then chains move + stitch calls across the grid in turn - the engine only ever sees
    this as one step. Unlike the plant radius sweep, every grid position has bed underneath
    the camera, so there is no "expected, nothing there" case: any stitch failure is
    systemic and fails the whole sequence outright, with no skip-and-continue branch.
    """

    def run(self, hardware, done):
        """Fetch the bed size, then stitch each grid position in turn."""
        try:
            hardware.map_info.map_size(
                on_done=lambda response: self._on_size(hardware, response, done))
        except Exception as error:  # any client error - report, never hang the engine
            done(StepResult(Outcome.FAILED, str(error)))

    def _on_size(self, hardware, response, done):
        """Parse the bed size, build the grid, and start stitching, or fail the sweep."""
        if response is None:
            done(StepResult(Outcome.FAILED, 'map_info MapSize call failed or rejected'))
            return
        try:
            x_len, y_len = parse_map_size(response.data)
            positions = grid_positions(x_len, y_len, hardware.vision.stitch_step_mm)
        except ValueError as error:
            done(StepResult(Outcome.FAILED, str(error)))
            return
        self._stitch_next(hardware, positions, 0, done)

    def _stitch_next(self, hardware, positions, position, done):
        """Move to the next grid position, or finish once every position has been stitched."""
        if position >= len(positions):
            done(StepResult(Outcome.OK, f'stitched {len(positions)} frame(s)'))
            return
        x, y = positions[position]
        try:
            hardware.movement.move_gantry_abs(
                x, y, hardware.vision.stitch_capture_z,
                on_done=lambda result: self._on_move(hardware, positions, position, result, done))
        except Exception as error:
            done(StepResult(Outcome.FAILED, str(error)))

    def _on_move(self, hardware, positions, position, result, done):
        """Stitch the frame at the position just reached, or forward the move's outcome."""
        outcome = hardware.to_outcome(result)
        if not outcome.ok:
            done(outcome)
            return
        x, y = positions[position]
        try:
            hardware.vision.stitch(
                x, y,
                on_done=lambda response: self._on_stitch(
                    hardware, positions, position, response, done))
        except Exception as error:
            done(StepResult(Outcome.FAILED, str(error)))

    def _on_stitch(self, hardware, positions, position, response, done):
        """Advance to the next grid position, or fail the sweep on any stitch failure."""
        if response is None:
            done(StepResult(Outcome.FAILED, 'map/stitch call failed or rejected'))
            return
        if not response.success:
            done(StepResult(Outcome.FAILED, response.message))
            return
        self._stitch_next(hardware, positions, position + 1, done)


def stitch_whole_bed() -> Sequence:
    """Build the full-bed stitch sweep sequence (registered as 'bed_stitch_sweep')."""
    return Sequence.of('bed_stitch_sweep', StitchWholeBed())
