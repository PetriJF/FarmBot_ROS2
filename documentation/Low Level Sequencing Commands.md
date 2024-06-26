These commands are used for communicating with the Farmbot Controller's Sequencing Manager. They cannot be used as user input and represent the low-level commands that are used for converting high level tasks into F-Code (Farmbot G-Code).

In order for the Sequencing Manager to execute a command properly, the command type must be specified, followed by the command type specific information.
In the current version of the codebase there are 4 command types: **Coordinate Command** (CC), **Device Command** (DC), **Vision Command** (VC) and **Tick Delay** (TD).
# Coordinate Command

A command that tells the farmbot to move at a certain position based on the calibration information.

### **Command Type Format:**

``CC_T_x_1\n``
-  **CC** -> Represents the coordinate command type.
-  **T_x_1** -> Where the sequencing command was requested from (e.g. T_x_1 represents the code for Mounting (1) Tool X).
- ``\n`` -> **NOTE:** make sure that you add the end line character at the end.

### **Command Information**:

``0.0 0.0 0.0\n``
- ``0.0 0.0 0.0`` -> The coordinates the Farmbot EE moves to relative to home in X Y Z format
- ``\n`` -> **NOTE:** ensure that you add the end line character at the end.

# Servo Command

A command that tells the farmbot to move the a servo attached to pin X to a specified angle A.

### **Command Type Format:**

``SC_T_x_1\n``
-  **SC** -> Represents the servo command type.
-  **S_x_1** -> Where the sequencing command was requested from.
- ``\n`` -> **NOTE:** make sure that you add the end line character at the end.

### **Command Information**:

``4 90.0\n``
- ``4`` -> The pin the servo is attached to.
- ``90.0`` -> The angle the servo should move to.
- ``\n`` -> **NOTE:** ensure that you add the end line character at the end.


# Device Command

These commands are used to communicate with devices connected to the Farmduino (e.g. LED Strip, Water Pump, etc.)

### Command Type Format

``DC_T_x_1\n``
- DC -> Represents the device command type.
- **T_x_1** -> Where the sequencing command was requested from (e.g. T_x_1 represents the code for Mounting (1) Tool X).
- ``\n`` -> **NOTE:** ensure that you add the end line character at the end.

### Command Information

The following options exist:

| Info Code     | Meaning                                                                                                                                                             |
| ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| CHECK a       | Checks if a tool is currently mounted. if a = 0, we are expecting no tool to be mounted; if a = 1, we are expecting a tool to be mounted. E.g. CHECK 0 and CHECK 1. |
| Vacuum a      | Turns on (a = 1) or off (a = 0) the Vacuum Pump.                                                                                                                    |
| WaterPulses a | Turns on the Water Pump for the specified pulse length (a).                                                                                                         |
**NOTE:** ensure that you add the end line character at the end of the command information.
# Vision Command

These commands include everything vision and imaging related.

### Command Type Format

``VC_P\n``
- VC -> Represents the vision command type.
- P -> Where the sequencing command was requested from (e.g. P represents the code for the panorama sequencer).
- ``\n`` -> **NOTE:** ensure that you add the end line character at the end.

### Command Information

The following options exist:

| Info Code     | Meaning                                                                 |
| ------------- | ----------------------------------------------------------------------- |
| PAN           | Represents the vision command that forms a panorama of the whole field. |
| CALIB a       | Calibrating the camera with ``a`` being the run count out of 3          |
**NOTE:** ensure that you add the end line character at the end of the command information.
# Tick Delay

A command that adds a tick delay to the sequencing. The length of the tick is determined by the frequency set to the sequencing timer in the farmbot controller.
### Command Type Format

``TD_CALIB\n``
- TD -> Represents the Tick Delay command type.
- CALIB -> Where the sequencing command was requested from (e.g. CALIB represents the code for the Camera Calibration sequencer).
- ``\n`` -> **NOTE:** ensure that you add the end line character at the end.

### Command Information

``Tn\n``
- T -> Tick
- n -> n ticks
- ``\n`` -> **NOTE:** ensure that you add the end line character at the end.
- e.g. T5 will make the sequencer wait for 5 ticks.