| Interface | Name | Type | Description | Example |
|---|---|---|---|---|
| Topic | `hri/request_command` | `std_msgs/msg/String` | Publishes high-level commands requested through the HRI, such as panel button commands. | `` |
| Topic | `controller/sequence_status` | `std_msgs/msg/String` | Publishes the current status of a sequence being executed by the controller. | `` |
| Service | `priority_cmd/estop` | `std_srvs/srv/Trigger` | Immediately triggers the FarmBot emergency stop. | `` |
| Service | `priority_cmd/abort` | `std_srvs/srv/Trigger` | Aborts the currently running FarmBot command. | `` |
| Service | `priority_cmd/resume` | `std_srvs/srv/Trigger` | Resumes the FarmBot after an emergency stop or abort. | `` |
| Topic | `farmbot_status/farmbot_position` | `geometry_msgs/msg/PointStamped` | Publishes the current FarmBot gantry position. | `` |
| Topic | `farmbot_status/estop_active` | `std_msgs/msg/Bool` | Publishes whether the emergency stop is currently active. | `` |
| Topic | `farmbot_status/abort_active` | `std_msgs/msg/Bool` | Publishes whether an abort state is currently active. | `` |
| Topic | `farmbot_status/serial_feedback` | `std_msgs/msg/String` | Publishes raw feedback messages received from the Farmduino. | `` |
| Service | `hardware_comm/set_led` | `farmbot_interfaces/srv/LedPanelHandler` | Controls the state of a FarmBot panel LED. | `` |
| Action | `hardware_comm/loading_params` | `farmbot_interfaces/action/LoadingParameters` | Loads FarmBot configuration parameters onto the controller. | `` |
| Service | `hardware_comm/end_stop` | `std_srvs/srv/Trigger` | Reads the state of the FarmBot end stops. | `` |
| Service | `hardware_comm/sw_version` | `std_srvs/srv/Trigger` | Retrieves the Farmduino firmware/software version. | `` |
| Service | `hardware_comm/curr_pos` | `std_srvs/srv/Trigger` | Retrieves the current FarmBot position. | `` |
| Service | `hardware_comm/read_i2c` | `farmbot_interfaces/srv/ReadI2C` | Reads data from an I2C device connected to the FarmBot. | `` |
| Service | `hardware_comm/set_i2c` | `farmbot_interfaces/srv/SetI2C` | Writes data to an I2C device connected to the FarmBot. | `` |
| Service | `hardware_comm/watering` | `farmbot_interfaces/srv/Watering` | Controls the FarmBot watering system. | `` |
| Service | `hardware_comm/read_pin` | `farmbot_interfaces/srv/ReadPin` | Reads the state of a Farmduino GPIO pin. | `` |
| Service | `hardware_comm/write_pin` | `farmbot_interfaces/srv/WritePin` | Sets the state of a Farmduino GPIO pin. | `` |
| Service | `hardware_comm/configure_pin` | `farmbot_interfaces/srv/ConfigurePin` | Configures a Farmduino GPIO pin. | `` |
| Service | `hardware_comm/move_servo` | `farmbot_interfaces/srv/MoveServo` | Moves a FarmBot servo to a requested position. | `` |
| Service | `hardware_comm/read_parameter` | `farmbot_interfaces/srv/ReadParameter` | Reads a Farmduino configuration parameter. | `` |
| Service | `hardware_comm/write_parameter` | `farmbot_interfaces/srv/WriteParameter` | Writes a Farmduino configuration parameter. | `` |
| Service | `hardware_comm/list_all_parameters` | `std_srvs/srv/Trigger` | Lists the available Farmduino configuration parameters. | `` |
| Action | `hardware_comm/move_gantry` | `farmbot_interfaces/action/MoveGantry` | Moves the FarmBot gantry to a specified position. | `` |
| Action | `hardware_comm/home_axes` | `farmbot_interfaces/action/HomeAxes` | Homes one or more FarmBot axes. | `` |
| Service | `map_cmd/add_plant` | `farmbot_interfaces/srv/AddPlant` | Adds a plant and its properties to the active map. | `` |
| Service | `map_cmd/add_tool` | `farmbot_interfaces/srv/AddTool` | Adds a tool and its position to the active map. | `` |
| Service | `map_cmd/add_seed_tray` | `farmbot_interfaces/srv/AddSeedTray` | Adds a seed tray and its properties to the active map. | `` |
| Service | `map_cmd/remove_map_object` | `farmbot_interfaces/srv/RemoveMapObject` | Removes a plant, tool, or seed tray from the active map. | `` |
| Service | `map_cmd/get_map` | `std_srvs/srv/Trigger` | Returns the current active map. | `` |
| Service | `map_cmd/update_map` | `farmbot_interfaces/srv/UpdateMap` | Updates values in the active map using specified key paths. | `` |
| Topic | `camera/rgb_img` | `sensor_msgs/msg/Image` | Publishes RGB images from the Luxonis camera. | `` |
| Topic | `camera/depth_img` | `sensor_msgs/msg/Image` | Publishes processed depth images from the Luxonis camera. | `` |
| Topic | `camera/image_raw` | `sensor_msgs/msg/Image` | Publishes RGB images from the standard FarmBot USB camera. | `` |
| Service | `camera/enable` | `farmbot_interfaces/srv/EnableCamera` | Enables or disables continuous image streaming. | `` |
| Service | `camera/capture` | `farmbot_interfaces/srv/CaptureImage` | Captures a single image and returns it in the response. | `` |