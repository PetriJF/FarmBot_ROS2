"""
task_sequencer: the node hosting the programmatic sequence engine.

It wires together 3 separable pieces:
- the existing client modules (movement.py, ...)
- the 'sequence_runner' engine
- the 'sequences' definitions
"""
from collections import deque

from farmbot_controllers import sequences  # noqa: F401  (registers every sequence)
from farmbot_controllers.devices import DeviceControl
from farmbot_controllers.movement import Movement
from farmbot_controllers.parameters import Parameters
from farmbot_controllers.sequence_runner import engine, registry
from farmbot_controllers.sequence_runner.steps import Outcome, StepResult
from farmbot_controllers.states import State

from farmbot_interfaces.action import RunSequence
from farmbot_interfaces.msg import SequenceStatus

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.task import Future

from std_msgs.msg import Bool

_STATE_CODES = {
    engine.IDLE: SequenceStatus.IDLE,
    engine.RUNNING: SequenceStatus.RUNNING,
    engine.PAUSED: SequenceStatus.PAUSED,
    engine.DONE: SequenceStatus.DONE,
    engine.FAILED: SequenceStatus.FAILED,
    engine.CANCELLED: SequenceStatus.CANCELLED,
}
_TERMINAL_CODES = (SequenceStatus.DONE, SequenceStatus.FAILED, SequenceStatus.CANCELLED)
_MAX_QUEUED = 16   # bounded pending queue

# 0 OK, 1 FIRMWARE_ERROR, 2 ESTOPPED, 3 ABORTED, # 4 REJECTED.
# Map the codes needing special handling; the rest -> FAILED.
_RESULT_OK, _RESULT_ESTOPPED, _RESULT_ABORTED = 0, 2, 3
_ACTION_OUTCOMES = {
    _RESULT_OK: Outcome.OK,
    _RESULT_ESTOPPED: Outcome.ESTOPPED,
    _RESULT_ABORTED: Outcome.ABORTED,
}


class Hardware:
    """
    The handle a step runs against, i.e. the node's client modules + result mapping.

    Holds references to the existing modules and the helpers steps use to turn an 
    action result or a service response into a StepResult.
    """

    def __init__(self, movement: Movement, devices: DeviceControl,
                 states: State, parameters: Parameters):
        """Bundle the client modules the steps use."""
        self.movement = movement
        self.devices = devices
        self.states = states
        self.parameters = parameters

    @staticmethod
    def result_to_outcome(result) -> StepResult:
        """Map an action result (or None = rejected) to a StepResult."""
        if result is None:
            return StepResult(Outcome.FAILED, 'goal rejected by bridge')
        return StepResult(_ACTION_OUTCOMES.get(result.code, Outcome.FAILED), result.message)

    @staticmethod
    def service_to_outcome(response) -> StepResult:
        """Map a service response (success/message; None = failed) to a StepResult."""
        if response is None:
            return StepResult(Outcome.FAILED, 'service call failed')
        return StepResult(Outcome.OK if response.success else Outcome.FAILED,
                          getattr(response, 'message', ''))

    @staticmethod
    def to_outcome(raw) -> StepResult:
        """Map any client result to a StepResult (action if it has a code, else service)."""
        if raw is None:
            return StepResult(Outcome.FAILED, 'call failed or rejected')
        if hasattr(raw, 'code'):
            return Hardware.result_to_outcome(raw)
        return Hardware.service_to_outcome(raw)


class _Pending:
    """A queued sequence: its goal handle and a future for the terminal status."""

    def __init__(self, goal_handle, sequence):
        self.goal_handle = goal_handle
        self.sequence = sequence
        self.done = Future()


class TaskSequencer(Node):
    """Serves the RunSequence action over the sequence engine and client modules."""

    def __init__(self):
        """Wire the engine to the client modules, the status topic and estop/abort."""
        super().__init__('task_sequencer')

        # Register the client modules with the engine.
        self.movement = Movement(self)
        self.devices = DeviceControl(self)
        self.states = State(self)
        self.parameters = Parameters(self)
        self._engine = engine.SequenceEngine(
            hardware=Hardware(self.movement, self.devices, self.states, self.parameters),
            on_status=self._publish_status,
            log=lambda message: self.get_logger().warn(message),
        )

        # One status contract for observers and for the running goal's feedback.
        self._status_pub = self.create_publisher(SequenceStatus, 'sequence_status', 10)

        # Hold the queue after a failed sequence until abort is released.
        self.declare_parameter('autopause_on_failure', True)
        self._autopause = bool(self.get_parameter('autopause_on_failure').value)

        # Bounded FIFO of pending sequences; one runs at a time.
        self._queue = deque()
        self._active = None              # the running _Pending (feedback sink)
        self._paused = False             # queue held after a sequence failed

        # Main sequence running action. Reentrant group so the async goal can await.
        self._run_server = ActionServer(
            self, RunSequence, 'run_sequence',
            execute_callback=self._execute_sequence,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup())

        # E-Stop/Abort subscriptions
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self._estop_active = False
        self.create_subscription(Bool, 'estop_active', self._on_estop, latched)
        self.create_subscription(Bool, 'abort_active', self._on_abort, latched)

        self.get_logger().info(
            f'task sequencer initialized. sequences: {", ".join(registry.names())}')

    # --- starting sequences -------------------------------------------------------

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Accept a start request, or reject it (e-stopped / unknown / queue full)."""
        name = goal_request.name
        if self._estop_active:
            self.get_logger().warn(f"rejected '{name}': e-stop is active")
            return GoalResponse.REJECT
        if not registry.is_registered(name):
            known = ', '.join(registry.names()) or '(none)'
            self.get_logger().warn(f"rejected unknown sequence '{name}'. known: {known}")
            return GoalResponse.REJECT
        if len(self._queue) >= _MAX_QUEUED:
            self.get_logger().warn(f"rejected '{name}': queue full ({_MAX_QUEUED})")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        """Accept a cancel: cancel it if running, else it is dropped when its turn comes."""
        if self._active is not None and self._active.goal_handle.goal_id == goal_handle.goal_id:
            self._engine.cancel('caller cancelled the sequence')
        return CancelResponse.ACCEPT

    async def _execute_sequence(self, goal_handle) -> RunSequence.Result:
        """Queue the requested sequence, then finalize the goal when it finishes."""
        pending = _Pending(goal_handle, registry.build(goal_handle.request.name))
        self._queue.append(pending)
        self._pump()

        status = await pending.done

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif status.state == SequenceStatus.DONE:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        result = RunSequence.Result()
        result.status = status
        return result

    def _pump(self) -> None:
        """Start the next queued sequence when idle (dropping any cancelled while waiting)."""
        if self._estop_active or self._paused or self._active is not None:
            return
        while self._queue:
            pending = self._queue.popleft()
            if pending.goal_handle.is_cancel_requested:
                self._resolve(pending, engine.CANCELLED, 'cancelled before running')
                continue
            self._active = pending
            self._engine.start(pending.sequence)
            return

    def _flush_queue(self, reason: str) -> None:
        """Drop every pending sequence (the e-stop path)."""
        while self._queue:
            self._resolve(self._queue.popleft(), engine.CANCELLED, f'dropped: {reason}')

    def _resolve(self, pending: _Pending, state: str, detail: str) -> None:
        if not pending.done.done():
            pending.done.set_result(self._make_status(
                pending.sequence.name, state, 0, len(pending.sequence), detail))

    # --- e-stop / abort -----------------------------------------------------

    def _on_estop(self, msg: Bool) -> None:
        """E-stop: block new starts, drop the queue, cancel the running sequence."""
        self._estop_active = msg.data
        if msg.data:
            self._paused = False
            self._flush_queue('e-stop')
            self._engine.cancel('e-stop')

    def _on_abort(self, msg: Bool) -> None:
        """Abort: pause the running sequence; release also resumes a failure-paused queue."""
        if msg.data:
            self._engine.pause()
        else:
            self._engine.resume()
            if self._paused:
                self._paused = False
                self._pump()

    # --- tracking ----------------------------------------------------------

    def _publish_status(self, name: str, state: str, index: int,
                        total: int, detail: str) -> None:
        """Publish the status (progress) of the sequence runner."""
        status = self._make_status(name, state, index, total, detail)
        self._status_pub.publish(status)           # broadcast to any observer
        active = self._active
        if active is not None:
            if status.state in _TERMINAL_CODES:
                if not active.done.done():
                    active.done.set_result(status)  # hand the terminal status to the goal
                self._active = None
                if self._autopause and status.state == SequenceStatus.FAILED and self._queue:
                    self._paused = True             # hold the queue after a failure
                    self.get_logger().warn(
                        f"'{name}' failed - queue paused ({len(self._queue)} waiting); "
                        'release abort to resume')
                else:
                    self._pump()                    # start the next queued sequence
            else:
                feedback = RunSequence.Feedback()
                feedback.status = status
                active.goal_handle.publish_feedback(feedback)
        self.get_logger().info(f'[{name}] {state} step {index}/{total}: {detail}')

    @staticmethod
    def _make_status(name: str, state: str, index: int,
                     total: int, detail: str) -> SequenceStatus:
        status = SequenceStatus()
        status.sequence = name
        status.state = _STATE_CODES[state]
        status.step_index = index
        status.step_total = total
        status.detail = detail
        return status


def main(args=None):
    """Initialize and run the task sequencer node."""
    rclpy.init(args=args)
    node = TaskSequencer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
