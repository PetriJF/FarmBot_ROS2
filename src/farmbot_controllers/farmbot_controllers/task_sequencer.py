"""
task_sequencer: the node hosting the programmatic sequence engine.

It wires together 3 separable pieces:
- the existing client modules (movement.py, ...)
- the 'sequence_runner' engine
- the 'sequences' definitions
"""
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

        # Per-goal routing state. Only one sequence runs at a time.
        self._goal_in_progress = False   # reserved synchronously in goal_callback
        self._active_goal = None         # the executing goal handle (feedback sink)
        self._done = None                # resolved with the terminal status

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
        """Accept a start request, or reject it (busy / e-stopped / unknown)."""
        name = goal_request.name
        if self._estop_active:
            self.get_logger().warn(f"rejected '{name}': e-stop is active")
            return GoalResponse.REJECT
        if self._goal_in_progress or self._engine.active:
            self.get_logger().warn(f"rejected '{name}': a sequence is already active")
            return GoalResponse.REJECT
        if not registry.is_registered(name):    # verify that the task is registered
            known = ', '.join(registry.names()) or '(none)'
            self.get_logger().warn(f"rejected unknown sequence '{name}'. known: {known}")
            return GoalResponse.REJECT
        self._goal_in_progress = True           # ensure we lock it
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        """Accept a cancel: abandon the sequence without e-stopping the hardware."""
        self.get_logger().info('cancel requested - abandoning the sequence')
        self._engine.cancel('caller cancelled the sequence')
        return CancelResponse.ACCEPT

    async def _execute_sequence(self, goal_handle) -> RunSequence.Result:
        """Run the requested sequence."""
        name = goal_handle.request.name
        self._active_goal = goal_handle
        self._done = Future()

        sequence = registry.build(name)
        started = self._engine.start(sequence)
        if not started:
            # Safety net: goal_callback already guards this.
            self._clear_goal()
            goal_handle.abort()
            result = RunSequence.Result()
            result.status = self._make_status(
                name, engine.FAILED, 0, len(sequence), 'refused: a sequence is already active')
            return result

        # The engine drives itself through the callbacks
        # _publish_status resolves this future when it reaches a terminal state.
        final_status = await self._done
        self._clear_goal()

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif final_status.state == SequenceStatus.DONE:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        result = RunSequence.Result()
        result.status = final_status
        return result

    def _clear_goal(self) -> None:
        self._active_goal = None
        self._done = None
        self._goal_in_progress = False

    # --- e-stop / abort -----------------------------------------------------

    def _on_estop(self, msg: Bool) -> None:
        """E-stoped: cancel the active sequence; block starts while set."""
        self._estop_active = msg.data
        if msg.data:
            self._engine.cancel('e-stop')

    def _on_abort(self, msg: Bool) -> None:
        """Aborted: pause the sequence; on release, resume the held step."""
        if msg.data:
            self._engine.pause()
        else:
            self._engine.resume()

    # --- tracking ----------------------------------------------------------

    def _publish_status(self, name: str, state: str, index: int,
                        total: int, detail: str) -> None:
        """Publish the status (progress) of the sequence runner."""
        status = self._make_status(name, state, index, total, detail)
        self._status_pub.publish(status)           # broadcast to any observer
        goal = self._active_goal
        if goal is not None:
            if status.state in _TERMINAL_CODES:
                if self._done is not None and not self._done.done():
                    self._done.set_result(status)  # hand the terminal status to the goal
            else:
                feedback = RunSequence.Feedback()
                feedback.status = status
                goal.publish_feedback(feedback)
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
