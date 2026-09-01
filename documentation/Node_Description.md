This section provides a package-by-package description of the nodes and their respective roles within the architecture.

# farmbot_hri

Package containing the nodes responsible for providing the human–robot interface (HRI).

### user_cli

The user_cli node provides the Human-Robot Interface (HRI) for the FarmBot. It allows the user to enter commands through the terminal and forward them to the different ROS 2 components.

It handles:

* sending commands to the task sequencer via hri/request_command;
* priority commands (ESTOP, ABORT, RESUME);
* asking for the loading hardware parameters ;
* adding and removing objects from the map;
* command validation and execution status feedback.

### autonomous_controller

The autonomous_controller node provides scheduled autonomous command execution for the FarmBot. It loads commands and their configured execution times from AutonomousCommand.yaml and publishes them to hri/request_command.

The node checks the current time every minute and publishes the corresponding command when its scheduled time is reached.


# farmbot_controllers

Package containing the nodes and modules that enable the creation of sequences for the queue, as well as the sending of command requests.

### task_sequencer

The task_sequencer node converts incoming commands into queued tasks and manages their execution. It receives commands from hri/request_command, translates them into sequences, and executes them one at a time through the sequence engine.

Sequences are used to execute complex operations that require some actions or movements to be performed in a specific order.

It handles:

* task queuing and sequential execution;
* task status and progress publication;
* e-stop and abort/pause management;
* automatic queue pausing after task failures;
* communication with the robot’s movement, hardware, parameter, map, and tool controllers.

# farmbot_hardware_comm

This package contains the nodes that handle communication with the hardware.

### serial_controller

The serial_controller node provides the communication bridge between ROS 2 and the FarmBot hardware. It converts ROS 2 commands into FCode, sends them to the Farmduino via the serial connection, and processes the responses received from the robot.

It handles:

* gantry movement and homing actions;
* hardware parameter loading;
* emergency stop, abort and resume commands;
* serial communication with the Farmduino;
* robot position and command feedback;
* firmware responses and hardware state updates.

### gpio_controller

The gpio_controller node manages the FarmBot physical control panel, including its buttons and LEDs through the Raspberry Pi GPIO interface.

It handles:

* emergency stop, reset and abort buttons;
* configurable user buttons and their associated commands;
* LED states, including steady and flashing modes;
* communication with the ROS 2 HRI and hardware control nodes.

# map_handler

This package is responsible for retrieving or creating the active map and then modifying it if necessary.

### map_controller

The map_controller node manages the FarmBot map and its persistent data. It loads the active map at startup and provides ROS 2 services to add, update, remove and retrieve map objects.

It handles:

* plant, tool and seed tray information;
* map persistence using YAML files;
* adding, updating and removing map objects;
* retrieving the current map for other nodes.

# farmbot_vision

This package contains the nodes that manage the cameras.

### luxonis_camera

The luxonis_camera node interfaces with the Luxonis depth camera to acquire and publish RGB and depth images to ROS 2 topics.

It handles:

* camera configuration and calibration loading;
* RGB and stereo depth image acquisition;
* depth processing from disparity data;
* publishing images via camera/rgb_img and camera/depth_img.

Note: This node is currently not in use and requires updating to the current DepthAI API.

### standard_camera

The Standard Camera node manages the standard USB camera integrated into the FarmBot. It handles image acquisition and publishes RGB images in ROS 2 on the camera/image_raw topic.

The camera can be used in two modes:

* Continuous streaming: images are continuously captured and published when the camera is enabled.
* Single capture: a single image can be captured on request through the camera/capture service.

The node also provides configuration options for camera parameters such as resolution, frame rate, pixel format, and exposure. Continuous streaming can be enabled or disabled through the camera/enable service.

This camera provides an image source for FarmBot perception and vision functionalities.



