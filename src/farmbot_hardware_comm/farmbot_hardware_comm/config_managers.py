"""
Configuration parameter management module for the FarmBot SerialController.

Contains the ParameterTable and ConfigServer classes.
ParameterTable is used for organizing the parameter indexing and values, while the
ConfigServer wraps it in the services the rest of the system calls.
"""
from farmbot_hardware_comm.modules.param_info import ParameterList

from farmbot_interfaces.action import LoadingParameters
from farmbot_interfaces.srv import UpdateMap, WriteParameter

from farmbot_utils.exceptions import EncodeError, ServerError, YAMLError
from farmbot_utils.yaml_handler import YAMLHandler

from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

# from std_srvs.srv import Trigger

LOAD_FAILURES = {
    'firmware error': LoadingParameters.Result.FIRMWARE_ERROR,
    'aborted': LoadingParameters.Result.ABORTED,
    'estopped': LoadingParameters.Result.ESTOPPED,
}

BASE_CONFIG = 'firmwareDefault.yaml'    # what the firmware boots with
ACTIVE_CONFIG = 'activeConfig.yaml'     # configuration loaded from previous run

PROFILES = {
    'genesis': 'Genesis.yaml',
    'express': 'Express.yaml',
    'custom': 'Custom1.yaml',           # modify in source and build
}
PROFILE_ALIASES = {'gen': 'genesis', 'exp': 'express'}
ACTIVE_PROFILE = 'active'               # reload what the previous run saved


class ParameterTable:
    """The farmduino parameter values, their source files and their pending writes."""

    def __init__(self, config_path: str, default_path: str, log=None):
        """Load the table, preferring the previous run's config over the firmware defaults."""
        self.config_path = config_path
        self.default_path = default_path
        self.params = ParameterList()
        self._log = log or (lambda message: None)

        YAMLHandler.make_dir(config_path)

        try:
            self.firmware_defaults = YAMLHandler.load_yaml(default_path, BASE_CONFIG)
        except YAMLError as e:
            self._log(f'Could not load the firmware defaults: {e}')
            self.firmware_defaults = {}

        try:
            self.values = YAMLHandler.load_yaml(config_path, ACTIVE_CONFIG)
        except YAMLError as e:
            self._log(f'{e}. Starting from the firmware defaults - load a profile with the '
                      'loading_params action to configure the farmbot.')
            self.values = dict(self.firmware_defaults)

    def get_value(self, param: int) -> int:
        """Return the stored value of a parameter."""
        return self.values[param]

    def set_value(self, param: int, value: int, loading: bool):
        """Store the firmware reported value and save it to .yaml if no loading is in progress."""
        self.values[param] = value
        if not loading:
            try:
                self.save()
            except YAMLError as e:
                return (f'Could not save the parameter config: {e}')

    def pending_writes(self) -> list[tuple[int, int]]:
        """Return the (param, value) pairs that still differ from the firmware defaults."""
        return [(param, value) for param, value in self.values.items()
                if self.firmware_defaults.get(param) != value]

    def resolve_profile(self, name: str) -> str:
        """Return the canonical name of a profile, raising ValueError if it is unknown."""
        requested = name.strip().lower()
        canonical = PROFILE_ALIASES.get(requested, requested)

        if canonical != ACTIVE_PROFILE and canonical not in PROFILES:
            known = ', '.join(sorted(PROFILES) + [ACTIVE_PROFILE])
            raise ValueError(f"unknown profile '{name}'. known profiles: {known}")
        return canonical

    def load_profile(self, name: str) -> str:
        """Replace the table with a stored profile and return its canonical name."""
        canonical = self.resolve_profile(name)

        if canonical == ACTIVE_PROFILE:
            self.values = YAMLHandler.load_yaml(self.config_path, ACTIVE_CONFIG)
        else:
            self.values = YAMLHandler.load_yaml(self.default_path, PROFILES[canonical])
        return canonical

    def save(self) -> str:
        """Save the table as the active config and return where it was written."""
        YAMLHandler.save_to_yaml(self.values, self.config_path, ACTIVE_CONFIG)
        return YAMLHandler.join_path(self.config_path, ACTIVE_CONFIG)

    def axis_lengths(self) -> dict:
        """Return the {x, y, z} axis lengths in mm implied by the table."""
        axes = (
            ('x_len', self.params.MOVEMENT_AXIS_NR_STEPS_X, self.params.MOVEMENT_STEP_PER_MM_X),
            ('y_len', self.params.MOVEMENT_AXIS_NR_STEPS_Y, self.params.MOVEMENT_STEP_PER_MM_Y),
            ('z_len', self.params.MOVEMENT_AXIS_NR_STEPS_Z, self.params.MOVEMENT_STEP_PER_MM_Z),
        )

        lengths = {}
        for axis, steps, steps_per_mm in axes:
            nr_steps = self.values.get(steps)
            per_mm = self.values.get(steps_per_mm)
            if not nr_steps or not per_mm:
                self._log(f'Axis {axis} length unknown: parameters {steps}/{steps_per_mm} unset.')
                continue
            lengths[axis] = nr_steps / per_mm
        return lengths


class ConfigServer:
    """Handle the parameter recording and loading onto the farmduino."""

    # Node contructor
    def __init__(self, node: Node, config_path: str, run_command=None, encoder=None):
        """
        Build the parameter table and call the configuration services.

        Args:
            node {Node}: Node the servers are created on.
            config_path {str}: Workspace directory holding the active configuration.
            run_command {callable}: Awaitable the node uses to send one FCode.
            encoder {Encoder}: Encoder turning a write request into FCode.
        """
        self.node = node
        self.run_command = run_command
        self.encoder = encoder
        self.loading = False

        self.default_path = YAMLHandler.get_directory_package('farmbot_hardware_comm', 'config')
        self.table = ParameterTable(config_path, self.default_path,
                                    log=lambda message: self.node.get_logger().warn(message))
        self.params = self.table.params

        # FIXME TODO these should be handled automatically and removed in the future
        # Config Service Servers
        # self.map_dimensions_server = self.node.create_service(Trigger, 'publish_map_dimensions',
        #                                                       self.map_dimensions_server_cb)

        # Update map client
        self.update_map_client = self.node.create_client(UpdateMap, 'update_map')

        # Log the initialization
        self.node.get_logger().info('Config Server Initialized..')

    @property
    def param_vals(self) -> dict:
        """Return the parameter table the encoder validates and scales against."""
        return self.table.values

    def set_value(self, param: int, value: int):
        """Set the selected parameter to the parsed value."""
        self.node.get_logger().info(f'Set parameter {param} to {value}')
        try:
            self.table.set_value(param, value, self.loading)
        except YAMLError:
            self.node.get_logger().warn(f'{self.table.set_value(param, value, self.loading)}')
        if param in (self.table.params.MOVEMENT_AXIS_NR_STEPS_X,
                     self.table.params.MOVEMENT_STEP_PER_MM_X,
                     self.table.params.MOVEMENT_AXIS_NR_STEPS_Y,
                     self.table.params.MOVEMENT_STEP_PER_MM_Y,
                     self.table.params.MOVEMENT_AXIS_NR_STEPS_Z,
                     self.table.params.MOVEMENT_STEP_PER_MM_Z):
            self.update_map_dimensions()

    def get_value(self, param: int) -> int:
        """Return a value of a selected parameter."""
        return self.table.get_value(param)

    def pending_writes(self) -> list[tuple[int, int]]:
        """Return the parameters that still differ from the firmware defaults."""
        return self.table.pending_writes()

    def writes_for_goal(self, goal) -> list[tuple[int, int]]:
        """
        Return the (param, value) pairs a LoadingParameters goal asks for.

        A named profile is loaded and diffed against the firmware defaults; otherwise
        the goal's own params/values are used as given.
        """
        if not goal.profile:
            return list(zip(goal.params, goal.values))

        profile = self.table.load_profile(goal.profile)
        self.node.get_logger().info(f'Loading the {profile} configuration')
        return self.table.pending_writes()

    async def load_params_execute_callback(
            self, goal_handle: ServerGoalHandle) -> LoadingParameters.Result:
        """
        Execute a loading parameters action.

        Resolves the goal into the parameters to write, then writes them one by one,
        reporting the progress as feedback.

        Args:
            goal_handle {ServerGoalHandle}: Accepted action goal.
        """
        result = LoadingParameters.Result()

        try:
            writes = self.writes_for_goal(goal_handle.request)
        except (ValueError, YAMLError) as e:
            result.code = LoadingParameters.Result.REJECTED
            result.message = str(e)
            goal_handle.abort()
            return result

        result.code, result.message = await self.write_parameters(writes, goal_handle)

        if result.code == LoadingParameters.Result.OK:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    async def write_parameters(self, writes: list, goal_handle=None) -> tuple[int, str]:
        """
        Write parameters to the Farmduino one at a time, publishing the progress.

        Args:
            writes {list}: (parameter, value) pairs to write.
            goal_handle {ServerGoalHandle}: Goal to publish feedback on, if any.
        """
        total = len(writes)
        if not total:
            return LoadingParameters.Result.OK, 'the firmware already matches the config'

        self.loading = True
        self.node.get_logger().info(f'Parameters to load: {total}')

        try:
            for done, (param, value) in enumerate(writes, start=1):
                request = WriteParameter.Request()
                request.param = param
                request.value = value
                request.during_calibration = False

                try:
                    fcode = self.encoder.encode_write_parameter(request, self.param_vals)
                except EncodeError as e:
                    return LoadingParameters.Result.REJECTED, str(e)

                success, message, _ = await self.run_command(fcode)
                if not success:
                    return (LOAD_FAILURES.get(message, LoadingParameters.Result.REJECTED),
                            f'{message} triggered by the parameter {param} '
                            f'and its value {value}')

                if goal_handle is not None:
                    feedback = LoadingParameters.Feedback()
                    feedback.progress = done / total
                    goal_handle.publish_feedback(feedback)
                self.table.firmware_defaults[param] = value
                self.node.get_logger().info(f'Loading parameter progression: '
                                            f'{done / total * 100:.2f} %')
        finally:
            self.loading = False
            self.save_config()

        return LoadingParameters.Result.OK, f'{total} parameters loaded'

    def schedule_startup_load(self):
        """Reload the configuration onto the Farmduino after a firmware restart."""
        if self.node.executor is None:
            self.node.get_logger().warn('No executor available, skipping the parameter load.')
            return

        self.node.executor.create_task(self.startup_load())

    async def startup_load(self):
        """Write the stored config back to a freshly booted firmware and log the outcome."""
        code, message = await self.write_parameters(self.pending_writes())
        if code == LoadingParameters.Result.OK:
            self.node.get_logger().info(f'Parameter loading complete! {message}')
        else:
            self.node.get_logger().error(f'Parameter loading failed: {message}')

    def save_config(self):
        """Save the current parameter table."""
        try:
            path = self.table.save()
        except YAMLError as e:
            self.node.get_logger().warn(f'Could not save the parameter config: {e}')
            return

        self.node.get_logger().info(f'Saved the current parameter configuration at {path}')

    def update_map_dimensions(self, on_done=None):
        """Update the map with the bed dimensions reported by the table configuration."""
        lengths = self.table.axis_lengths()
        if len(lengths) != 3:
            raise Exception('axis lengths are not set in the parameter configuration')

        update_info = [f'map_reference {axis} {length}' for axis, length in lengths.items()]

        self._server_availability('UpdateMap', self.update_map_client)
        request = UpdateMap.Request()
        request.update_info = update_info
        self.update_map_client.call_async(request=request).add_done_callback(
                    lambda future: self._complete(future, on_done))

    def _server_availability(self, cmd_name: str, client):
        if not client.wait_for_service(1.0):
            self.node.get_logger().fatal(f'{cmd_name} Server not available!')
            raise ServerError('ConfigServer failed: server unavailable')

    def _complete(self, future, on_done=None):
        """Log the outcome and forward the response (or None on failure) to on_done."""
        try:
            response = future.result()
        except Exception as error:  # call failed - report, never leave on_done hanging
            self.node.get_logger().error('Service call failed %r' % (error, ))
            response = None
        if response is None:
            self.node.get_logger().warn('Command failure!')
        elif not response.success:
            self.node.get_logger().warn(f'Command failed: {response.message}')
        elif response.message:
            self.node.get_logger().info(response.message)
        else:
            self.node.get_logger().info('Command successful')
        if on_done is not None:
            on_done(response)
