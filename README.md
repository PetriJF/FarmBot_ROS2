
# Description

This repository contains a ROS2 alternative to the Farmbot OS, which was originally written in Elixir, Lua and Python.

The Raspberry Pi need to have Ubuntu 22.04 installed on them (server version is fine) with ROS2 Humble installed over it (Ensure it is NOT the barebones version).
# Disclaimer

Please note that this is not a 1:1 refactoring of Farmbot OS. It contains all the tools necessary to run and utilize the main features of the Farmbots. The software was tested on the Genesis XL platform (v1.6 and v1.7), but it should work on the Express platform (needs testing).

This repository was developed in the AURA Project at Maynooth University and is a public fork of the research repository. Some features are specific to the research taking place in the project, and missing features will be added based on the priority dictated by the tasks and projects going on at the time.

# Preparing the Environment

### 1. Clone the respository

Execute the following to clone the repo as a new subdirectory `/home/<yourname>/` containing the Farmbot ROS2 code base.
   ```bash
   git clone git@github.com:PetriJF/FarmBot_ROS2.git
   ```

### 2. Set your bashrc (Optional)

Open the bashrc file in your user's directory

```bash
cd ~
ls -a
nano .bashrc
```

Once open your terminal should look like a command editor. Go to the very end and add the following lines there:

```bash
# ROS2 sourcing
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source ~/FarmBot_ROS2/install/setup.bash
```

Note, if you installed something in a different directory than the default ones, you will have to adapt the 3 sourcing commands

### 3. Test the workspace

Attempt to run the farmbot launch file, starting up all the nodes needed for it to run

```bash
ros2 launch farmbot_bringup standardLaunch.launch.py
```

You can also run the keyboard controller in another terminal to test a command

```bash
ros2 run farmbot_controllers keyboard_controller
```

You can now check the High Level Controller Commands list to test the various functions of the Farmbot.

# How to run everything together (WIP - Subject to change)

Before anything else, ensure that you have the most recent commit, and you properly built the workspace.

You will need a minimum of 2 terminals for each robot (3 if you want manual user control). All three terminals are explained below:

### 1. The Farmbot Packages

These packages include everything from the main controller to the imagine controller. This will run all the nodes needed to interpret and form the commands that communicate with the Farmduino. Also, imaging and plant management happen here as well.

For running the launch file use:
``` bash
ros2 launch farmbot_bringup standard.launch.py
```

### 2. Launch the Autonomous Command Controller

This is a node that sends commands to the farmbot controller at specific times set in the script. If you want to create a specific plan for watering or managing your plants, make sure to modify the python script to suit your needs.

``` bash
ros2 run farmbot_controllers keyboard_controller
```

### 3. User Command Handler (Optional)

If you want to send commands to the farmbot that are outside the plan you have created for your plants, you can turn on the keyboard controller. For a complete list of all the High Level commands, check the documentation.

``` bash
ros2 run farmbot_controllers autonomous_controller
```
