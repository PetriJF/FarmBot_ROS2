"""Unit tests for the plant radius sweep sequence, run without rclpy."""
from types import SimpleNamespace

from farmbot_controllers.sequence_runner.steps import Outcome, StepResult
from farmbot_controllers.sequences.plant_radius import MeasureAllPlantRadii, parse_plant_list

import pytest

_ACTION_OUTCOMES = {0: Outcome.OK, 2: Outcome.ESTOPPED, 3: Outcome.ABORTED}


def _to_outcome(raw) -> StepResult:
    """Mirror task_sequencer.Hardware.to_outcome without importing the rclpy-backed module."""
    if raw is None:
        return StepResult(Outcome.FAILED, 'call failed or rejected')
    if hasattr(raw, 'code'):
        return StepResult(_ACTION_OUTCOMES.get(raw.code, Outcome.FAILED), raw.message)
    return StepResult(Outcome.OK if raw.success else Outcome.FAILED,
                      getattr(raw, 'message', ''))


class _NullLogger:
    """Swallow log calls so the fake vision module doesn't need a real node."""

    def warn(self, *_args, **_kwargs):
        """Discard the warning."""
        pass

    def info(self, *_args, **_kwargs):
        """Discard the info message."""
        pass


class _FakeMovement:
    """Return one preset action result per move_gantry_abs call, in order."""

    def __init__(self, results):
        self.results = list(results)
        self.calls = []

    def move_gantry_abs(self, x, y, z, on_done=None):
        """Record the call and hand back the next queued result."""
        self.calls.append((x, y, z))
        on_done(self.results.pop(0))


class _FakeVision:
    """Return one preset MeasurePlantRadius response per call, in order."""

    def __init__(self, capture_z, responses):
        self.capture_z = capture_z
        self.responses = list(responses)
        self.calls = []
        self.node = SimpleNamespace(get_logger=lambda: _NullLogger())

    def measure_plant_radius(self, index, on_done=None):
        """Record the call and hand back the next queued response."""
        self.calls.append(index)
        on_done(self.responses.pop(0))


class _FakeMapInfo:
    """Return a fixed PlantList reply (or None to simulate a communication failure)."""

    def __init__(self, data):
        self.data = data

    def list_plants(self, on_done=None):
        """Reply with the fixed PlantList data."""
        on_done(None if self.data is None else SimpleNamespace(data=self.data))


class _FakeHardware:
    """Duck-typed hardware bundle exercising only what the sweep step calls."""

    def __init__(self, plant_list_data, move_results=(), vision_responses=(), capture_z=-100.0):
        self.movement = _FakeMovement(move_results)
        self.vision = _FakeVision(capture_z, vision_responses)
        self.map_info = _FakeMapInfo(plant_list_data)
        self.to_outcome = _to_outcome


def _run(hardware) -> StepResult:
    """Drive the step to completion and return the single result passed to done."""
    results = []
    MeasureAllPlantRadii().run(hardware, results.append)
    assert len(results) == 1
    return results[0]


def test_parse_plant_list_valid_rows():
    """Each 'index x y' row becomes an (int, float, float) tuple."""
    assert parse_plant_list('1 10.0 20.0\n2 30.5 -5.0') == [(1, 10.0, 20.0), (2, 30.5, -5.0)]


def test_parse_plant_list_empty_string():
    """An empty PlantList reply (no plants in the map) parses to an empty list."""
    assert parse_plant_list('') == []


def test_parse_plant_list_malformed_row():
    """A row that isn't exactly 'index x y' is a parse error, not a silently dropped plant."""
    with pytest.raises(ValueError):
        parse_plant_list('1 10.0')


def test_sweep_measures_every_plant():
    """Every plant in the list is visited and measured, in order."""
    move_ok = SimpleNamespace(code=0, message='')
    measure_ok = SimpleNamespace(success=True, message='', no_plant_detected=False)
    hardware = _FakeHardware(
        plant_list_data='1 10.0 20.0\n2 30.0 40.0',
        move_results=[move_ok, move_ok],
        vision_responses=[measure_ok, measure_ok])

    result = _run(hardware)

    assert result.ok
    assert hardware.movement.calls == [(10.0, 20.0, -100.0), (30.0, 40.0, -100.0)]
    assert hardware.vision.calls == [1, 2]


def test_sweep_skips_plant_with_no_plant_detected():
    """A bare plant is logged and skipped, and the sweep still reaches the next one."""
    move_ok = SimpleNamespace(code=0, message='')
    bare = SimpleNamespace(success=False, message='No plant detected in the image',
                           no_plant_detected=True)
    measure_ok = SimpleNamespace(success=True, message='', no_plant_detected=False)
    hardware = _FakeHardware(
        plant_list_data='1 10.0 20.0\n2 30.0 40.0',
        move_results=[move_ok, move_ok],
        vision_responses=[bare, measure_ok])

    result = _run(hardware)

    assert result.ok
    assert hardware.vision.calls == [1, 2]


def test_sweep_fails_on_systemic_measurement_failure():
    """A failure that isn't 'no plant detected' (e.g. camera down) fails the whole sweep."""
    move_ok = SimpleNamespace(code=0, message='')
    capture_down = SimpleNamespace(success=False, message='Camera capture service not available',
                                   no_plant_detected=False)
    hardware = _FakeHardware(
        plant_list_data='1 10.0 20.0\n2 30.0 40.0',
        move_results=[move_ok],
        vision_responses=[capture_down])

    result = _run(hardware)

    assert result.outcome == Outcome.FAILED
    assert result.message == 'Camera capture service not available'
    assert hardware.vision.calls == [1]  # the second plant was never reached


def test_sweep_ends_early_on_aborted_movement():
    """An aborted move ends the sweep at that plant instead of continuing."""
    move_aborted = SimpleNamespace(code=3, message='aborted')
    hardware = _FakeHardware(
        plant_list_data='1 10.0 20.0\n2 30.0 40.0',
        move_results=[move_aborted])

    result = _run(hardware)

    assert result.outcome == Outcome.ABORTED
    assert hardware.vision.calls == []  # never reached the measure call for plant 1


def test_sweep_with_no_plants_succeeds_without_moving():
    """An empty map is not an error - the sweep has nothing to do."""
    hardware = _FakeHardware(plant_list_data='')

    result = _run(hardware)

    assert result.ok
    assert hardware.movement.calls == []


def test_sweep_fails_when_map_info_unavailable():
    """A failed plant list fetch fails the sweep before any movement is attempted."""
    hardware = _FakeHardware(plant_list_data=None)

    result = _run(hardware)

    assert result.outcome == Outcome.FAILED
    assert hardware.movement.calls == []
