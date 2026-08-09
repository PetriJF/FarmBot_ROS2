"""
task_sequencer: turns operator commands into queued tasks.

Every command becomes a task and every task runs through the same engine and queue,
so ordering, status, e-stop, abort and autopause apply identically to a single move
and to a multi-step sequence.
"""
from collections import deque

from farmbot_controllers import command_map
from farmbot_controllers.devices import DeviceControl
from farmbot_controllers.movement import Movement
from farmbot_controllers.parameters import Parameters
from farmbot_controllers.sequence_runner import engine
from farmbot_controllers.sequence_runner.steps import Outcome, StepResult
from farmbot_controllers.sequences.single_call import single_call
from farmbot_controllers.states import State

from farmbot_interfaces.msg import SequenceStatus

from geometry_msgs.msg import PointStamped

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from std_msgs.msg import Bool, String

_MAX_QUEUED = 16   # bounded pending queue

# Manual movement: the increment presets (mm) and the (x, y) direction each key nudges.
_INCREMENTS = {'1': 10.0, '2': 100.0, '3': 500.0}
_TELEOP_DIR = {'w': (1.0, 0.0), 's': (-1.0, 0.0), 'd': (0.0, 1.0), 'a': (0.0, -1.0)}

_STATE_CODES = {
    engine.IDLE: SequenceStatus.IDLE,
    engine.RUNNING: SequenceStatus.RUNNING,
    engine.PAUSED: SequenceStatus.PAUSED,
    engine.DONE: SequenceStatus.DONE,
    engine.FAILED: SequenceStatus.FAILED,
    engine.CANCELLED: SequenceStatus.CANCELLED,
}
_TERMINAL = (engine.DONE, engine.FAILED, engine.CANCELLED)

# Motion/homing action result codes
_ACTION_OUTCOMES = {0: Outcome.OK, 2: Outcome.ESTOPPED, 3: Outcome.ABORTED}


class Hardware:
    """Client module helper."""

    def __init__(self, movement: Movement, devices: DeviceControl,
                 states: State, parameters: Parameters):
        """Bundle the client modules the steps call."""
        self.movement = movement
        self.devices = devices
        self.states = states
        self.parameters = parameters

    @staticmethod
    def to_outcome(raw) -> StepResult:
        """Map any client result to a StepResult."""
        if raw is None:
            return StepResult(Outcome.FAILED, 'call failed or rejected')
        if hasattr(raw, 'code'):
            return StepResult(_ACTION_OUTCOMES.get(raw.code, Outcome.FAILED), raw.message)
        return StepResult(Outcome.OK if raw.success else Outcome.FAILED,
                          getattr(raw, 'message', ''))


class TaskSequencer(Node):
    """Owns the client modules and runs every operator command as a queued task."""

    def __init__(self):
        """Wire the engine to the client modules, the command input, status and e-stop/abort."""
        super().__init__('task_sequencer')

        self.movement = Movement(self)
        self.devices = DeviceControl(self)
        self.states = State(self)
        self.parameters = Parameters(self)
        hardware = Hardware(self.movement, self.devices, self.states, self.parameters)
        self._engine = engine.SequenceEngine(
            hardware=hardware,
            on_status=self._publish_status,
            log=lambda message: self.get_logger().warn(message))

        self.declare_parameter('autopause_on_failure', True)
        self._autopause = bool(self.get_parameter('autopause_on_failure').value)

        self._queue = deque()        # pending Sequences, one runs at a time
        self._active = None          # name of the running task, or None
        self._paused = False         # queue held after a failure

        self._increment = 10.0       # step multiplier in mm (set by 1/2/3)
        self._position = None        # latest gantry position, for manual movement

        self._status_pub = self.create_publisher(SequenceStatus, 'sequence_status', 10)
        self.create_subscription(String, 'request_command', self._on_command, 10)
        self.create_subscription(PointStamped, 'farmbot_position', self._on_position, 10)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self._estop_active = False
        self.create_subscription(Bool, 'estop_active', self._on_estop, latched)
        self.create_subscription(Bool, 'abort_active', self._on_abort, latched)

        broken = command_map.unresolved(hardware)
        if broken:
            self.get_logger().error(f'command map has unresolved calls: {", ".join(broken)}')
        self.get_logger().info('task sequencer initialized')

    # --- dispatch: grammar -> the task the command map defines ---------------

    def _on_command(self, msg: String) -> None:
        """Handle a manual movement key locally."""
        if self._inc_mvm(msg.data):
            return
        try:
            task = command_map.to_task(msg.data)
        except (ValueError, IndexError) as error:
            self.get_logger().warn(f"ignored '{msg.data}': {error}")
            return
        self._submit(task)

    def _on_position(self, msg: PointStamped) -> None:
        """Tracks the latest gantry position."""
        self._position = msg.point

    def _inc_mvm(self, command: str) -> bool:
        """Set the manual movement increment or queue a relative move."""
        if command in _INCREMENTS:
            self._increment = _INCREMENTS[command]
            return True
        if command not in _TELEOP_DIR:
            return False
        if self._position is None:
            self.get_logger().warn(f"'{command}' ignored: position not known yet")
            return True
        dx, dy = _TELEOP_DIR[command]
        x = self._position.x + dx * self._increment
        y = self._position.y + dy * self._increment
        z = self._position.z
        self._submit(single_call(
            'manual_movement', lambda hw, done: hw.movement.move_gantry_abs(x, y, z, on_done=done)))
        return True

    # --- queue --------------------------------------------------------------

    def _submit(self, sequence) -> None:
        """Add a task to the queue (dropped while e-stopped or when the queue is full)."""
        if self._estop_active:
            self.get_logger().warn(f"'{sequence.name}' ignored: e-stop active")
            return
        if len(self._queue) >= _MAX_QUEUED:
            self.get_logger().warn(f"'{sequence.name}' ignored: queue full ({_MAX_QUEUED})")
            return
        self._queue.append(sequence)
        self._pump()

    def _pump(self) -> None:
        """Start the next queued task when nothing is running and nothing holds the queue."""
        if self._estop_active or self._paused or self._active is not None or not self._queue:
            return
        sequence = self._queue.popleft()
        self._active = sequence.name
        self._engine.start(sequence)

    # --- status / e-stop / abort --------------------------------------------

    def _publish_status(self, name: str, state: str, index: int,
                        total: int, detail: str) -> None:
        """Broadcast task progress."""
        status = SequenceStatus()
        status.sequence = name
        status.state = _STATE_CODES[state]
        status.step_index = index
        status.step_total = total
        status.detail = detail
        self._status_pub.publish(status)
        self.get_logger().info(f'[{name}] {state} step {index}/{total}: {detail}')

        if self._active is None or state not in _TERMINAL:
            return
        self._active = None
        if self._autopause and state == engine.FAILED and self._queue:
            self._paused = True
            self.get_logger().warn(
                f"'{name}' failed - queue paused ({len(self._queue)} waiting); "
                'release abort to resume')
        else:
            self._pump()

    def _on_estop(self, msg: Bool) -> None:
        """E-stop functionality on the sequence and queue."""
        self._estop_active = msg.data
        if msg.data:
            self._paused = False
            self._queue.clear()
            self._engine.cancel('e-stop')

    def _on_abort(self, msg: Bool) -> None:
        """Abort functionality on the sequence and queue."""
        if msg.data:
            self._engine.pause()
        else:
            self._engine.resume()
            if self._paused:
                self._paused = False
                self._pump()


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
