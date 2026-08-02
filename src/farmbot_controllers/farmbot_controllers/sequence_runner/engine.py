"""
The sequence engine runs a Sequence one step at a time.

The step tuple plus an index are the program counter, with the engine running one
step against the injected hardware calls and returns. The step's result callback
advances the index and runs the next. Between callbacks the node is idle (non-blocking).

Outcome handling:
* OK advances
* FAILED fails the sequence
* ESTOPPED cancels it
* ABORTED pauses at the current step
"""
from typing import Callable, Optional

from farmbot_controllers.sequence_runner.steps import Outcome, Sequence, Step, StepResult

# Engine states
IDLE = 'IDLE'
RUNNING = 'RUNNING'
PAUSED = 'PAUSED'
DONE = 'DONE'
FAILED = 'FAILED'
CANCELLED = 'CANCELLED'

# (sequence name, state, step_index, step_total, detail)
StatusCallback = Callable[[str, str, int, int, str], None]


def describe(step: Step) -> str:
    """Return a one-line description of a step for status."""
    return repr(step)


class SequenceEngine:
    """Runs a Sequence against hardware calls and tracks its progress."""

    def __init__(self, hardware, on_status: StatusCallback,
                 log: Optional[Callable[[str], None]] = None):
        """Link the engine to the hardware handle, a status sink and a logger."""
        self._hardware = hardware
        self._on_status = on_status
        self._log = log or (lambda message: None)
        self._name = ''
        self._steps: tuple = ()
        self._index = 0
        self._state = IDLE
        # A late result from a superseded step (after resume or cancel)
        # must not advance whatever is running now.
        self._generation = 0

    @property
    def state(self) -> str:
        """Return the current engine state."""
        return self._state

    @property
    def active(self) -> bool:
        """Return True while a sequence is running or paused."""
        return self._state in (RUNNING, PAUSED)

    def start(self, sequence: Sequence) -> bool:
        """Begin a sequence; refused (returns False) if one is already active."""
        if self.active:
            self._log(f"refused '{sequence.name}': '{self._name}' is still active")
            return False
        self._name = sequence.name
        self._steps = tuple(sequence.steps)
        self._index = 0
        self._generation += 1
        self._state = RUNNING
        self._dispatch_next()
        return True

    def pause(self) -> None:
        """Pause a running sequence."""
        if self._state != RUNNING:
            return
        self._state = PAUSED
        self._publish(PAUSED, 'paused')

    def resume(self) -> None:
        """Resume a paused sequence by re-running the interrupted step."""
        if self._state != PAUSED:
            return
        self._state = RUNNING
        self._generation += 1  # supersede any late result from the held step
        self._publish(RUNNING, 're-dispatching interrupted step')
        self._dispatch_next()

    def cancel(self, reason: str) -> None:
        """Abandon the active sequence and clear it."""
        if not self.active:
            return
        self._log(f"sequence '{self._name}' cancelled: {reason}")
        self._finish(CANCELLED, reason)

    # ------------------------ Engine -----------------------------

    def _dispatch_next(self) -> None:
        """Run the step at the current index, or finish when the list is exhausted."""
        if self._index >= len(self._steps):
            self._finish(DONE, 'sequence complete')
            return
        step = self._steps[self._index]
        self._publish(RUNNING, describe(step))
        generation = self._generation
        step.run(self._hardware,
                 lambda result: self._on_step_done(generation, step, result))

    def _on_step_done(self, generation: int, step: Step, result: StepResult) -> None:
        """Route one step's result: drop if stale, else cancel/pause/fail/advance."""
        if generation != self._generation:
            self._log(f'ignoring stale result for {describe(step)}')
            return
        if result.outcome == Outcome.ESTOPPED:
            self._finish(CANCELLED, result.message or 'e-stopped')
            return
        if result.outcome == Outcome.ABORTED:
            self._enter_paused()
            return
        if self._state == PAUSED:
            # Paused before this result landed - hold the index for resume.
            return
        if result.outcome != Outcome.OK:
            self._fail(step, result.message or 'step failed')
            return
        self._index += 1
        self._dispatch_next()

    def _enter_paused(self) -> None:
        """Enter PAUSED once, keeping the current index for resume to re-dispatch."""
        if self._state == PAUSED:
            return
        self._state = PAUSED
        self._publish(PAUSED, 'paused')

    def _fail(self, step: Step, reason: str) -> None:
        """Log which step failed and why, then end the sequence as FAILED."""
        self._log(f"sequence '{self._name}' failed at step "
                  f'{self._index + 1}/{len(self._steps)} ({describe(step)}): {reason}')
        self._finish(FAILED, reason)

    def _finish(self, state: str, detail: str) -> None:
        """Settle on a terminal state and bump the generation so late results are dropped."""
        self._state = state
        self._generation += 1   # invalidate any still-in-flight done-callback
        self._publish(state, detail)

    def _publish(self, state: str, detail: str) -> None:
        """Report current progress (name, state, index/total, detail) to the status sink."""
        self._on_status(self._name, state, self._index, len(self._steps), detail)
