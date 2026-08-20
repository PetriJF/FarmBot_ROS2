"""Unit tests for the bed stitch sweep sequence, run without rclpy."""
from types import SimpleNamespace

from farmbot_controllers.sequence_runner.steps import Outcome, StepResult
from farmbot_controllers.sequences.stitch_bed import (
    grid_positions, parse_map_size, StitchWholeBed)

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
    """Return one preset StitchImage response per call, in order."""

    def __init__(self, stitch_capture_z, stitch_step_mm, responses):
        self.stitch_capture_z = stitch_capture_z
        self.stitch_step_mm = stitch_step_mm
        self.responses = list(responses)
        self.calls = []
        self.node = SimpleNamespace(get_logger=lambda: _NullLogger())

    def stitch(self, x, y, on_done=None):
        """Record the call and hand back the next queued response."""
        self.calls.append((x, y))
        on_done(self.responses.pop(0))


class _FakeMapInfo:
    """Return a fixed MapSize reply (or None to simulate a communication failure)."""

    def __init__(self, data):
        self.data = data

    def map_size(self, on_done=None):
        """Reply with the fixed MapSize data."""
        on_done(None if self.data is None else SimpleNamespace(data=self.data))


class _FakeHardware:
    """Duck-typed hardware bundle exercising only what the sweep step calls."""

    def __init__(self, map_size_data, move_results=(), stitch_responses=(),
                 stitch_capture_z=-100.0, stitch_step_mm=150.0):
        self.movement = _FakeMovement(move_results)
        self.vision = _FakeVision(stitch_capture_z, stitch_step_mm, stitch_responses)
        self.map_info = _FakeMapInfo(map_size_data)
        self.to_outcome = _to_outcome


def _run(hardware) -> StepResult:
    """Drive the step to completion and return the single result passed to done."""
    results = []
    StitchWholeBed().run(hardware, results.append)
    assert len(results) == 1
    return results[0]


def test_parse_map_size_valid():
    """A well-formed 'x_len y_len' reply parses to a (float, float) tuple."""
    assert parse_map_size('300.0 100.0') == (300.0, 100.0)


def test_parse_map_size_malformed():
    """A reply that isn't exactly 'x_len y_len' is a parse error."""
    with pytest.raises(ValueError):
        parse_map_size('300.0')


def test_parse_map_size_non_positive():
    """Bed dimensions that are zero or unset are a parse error, not a valid empty bed."""
    with pytest.raises(ValueError):
        parse_map_size('0.0 100.0')


def test_grid_positions_covers_edges_and_serpentines():
    """The grid snaps to both far edges and alternates y direction column to column."""
    positions = grid_positions(300.0, 100.0, 150.0)

    assert positions == [
        (0.0, 0.0), (0.0, 100.0),
        (150.0, 100.0), (150.0, 0.0),
        (300.0, 0.0), (300.0, 100.0),
    ]


def test_grid_positions_rejects_non_positive_step():
    """A zero or negative step is a configuration error, not an infinite loop."""
    with pytest.raises(ValueError):
        grid_positions(300.0, 100.0, 0.0)


def test_sweep_stitches_every_grid_position():
    """Every grid position is visited and stitched, in order, at stitch_capture_z."""
    move_ok = SimpleNamespace(code=0, message='')
    stitch_ok = SimpleNamespace(success=True, message='')
    hardware = _FakeHardware(
        map_size_data='150.0 100.0',
        move_results=[move_ok, move_ok, move_ok, move_ok],
        stitch_responses=[stitch_ok, stitch_ok, stitch_ok, stitch_ok],
        stitch_capture_z=-100.0)

    result = _run(hardware)

    assert result.ok
    assert result.message == 'stitched 4 frame(s)'
    expected_positions = [(0.0, 0.0), (0.0, 100.0), (150.0, 100.0), (150.0, 0.0)]
    assert hardware.movement.calls == [(x, y, -100.0) for x, y in expected_positions]
    assert hardware.vision.calls == expected_positions


def test_sweep_fails_on_any_stitch_failure():
    """Unlike the radius sweep, every stitch failure is systemic and fails the whole sweep."""
    move_ok = SimpleNamespace(code=0, message='')
    camera_down = SimpleNamespace(success=False, message='Camera capture service not available')
    hardware = _FakeHardware(
        map_size_data='150.0 100.0',
        move_results=[move_ok],
        stitch_responses=[camera_down])

    result = _run(hardware)

    assert result.outcome == Outcome.FAILED
    assert result.message == 'Camera capture service not available'
    assert hardware.vision.calls == [(0.0, 0.0)]  # the second position was never reached


def test_sweep_fails_on_none_stitch_reply():
    """A communication failure on the stitch call fails the sweep, not just that position."""
    move_ok = SimpleNamespace(code=0, message='')
    hardware = _FakeHardware(
        map_size_data='150.0 100.0',
        move_results=[move_ok],
        stitch_responses=[None])

    result = _run(hardware)

    assert result.outcome == Outcome.FAILED


def test_sweep_ends_early_on_aborted_movement():
    """An aborted move ends the sweep at that position instead of continuing."""
    move_aborted = SimpleNamespace(code=3, message='aborted')
    hardware = _FakeHardware(
        map_size_data='150.0 100.0',
        move_results=[move_aborted])

    result = _run(hardware)

    assert result.outcome == Outcome.ABORTED
    assert hardware.vision.calls == []  # never reached the stitch call for the first position


def test_sweep_fails_when_map_info_unavailable():
    """A failed MapSize fetch fails the sweep before any movement is attempted."""
    hardware = _FakeHardware(map_size_data=None)

    result = _run(hardware)

    assert result.outcome == Outcome.FAILED
    assert hardware.movement.calls == []


def test_sweep_fails_on_malformed_map_size():
    """A malformed MapSize reply fails the sweep before any movement is attempted."""
    hardware = _FakeHardware(map_size_data='not a size')

    result = _run(hardware)

    assert result.outcome == Outcome.FAILED
    assert hardware.movement.calls == []
