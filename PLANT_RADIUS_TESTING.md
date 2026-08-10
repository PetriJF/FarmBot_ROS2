# FarmBot plant-radius test (temporary — delete before PR)

Use this checklist to test the plant-radius calculator with the Standard USB camera
on the FarmBot. It tests one known plant at a time; the all-plants sequence is a later
task.

> Delete this file before opening the pull request. It is a temporary operator
> checklist, not project documentation.

## 1. Prepare safely

- Check out the plant-radius branch on the FarmBot PC.
- Keep the emergency stop accessible and make sure the gantry path is clear.
- Connect the Standard USB camera and the FarmBot serial cable.
- Choose an existing plant from the active map and note its `index`, `x`, and `y`.
- Choose a safe camera Z position that puts the camera directly above the plant.
- Account for any physical offset between the camera and the gantry reference point.

Back up the active map before changing a radius:

```bash
cd ~/FarmBot_ROS2
cp farmbot_data/local_config/active_map.yaml \
  farmbot_data/local_config/active_map.before-radius-test.yaml
```

Inspect the known plants:

```bash
rg -n -C 5 "index:|plant_name:|position:|plant_radius:" \
  farmbot_data/local_config/active_map.yaml
```

## 2. Build and source

```bash
cd ~/FarmBot_ROS2
source /opt/ros/jazzy/setup.bash

colcon build --packages-up-to \
  farmbot_bringup farmbot_vision map_handler \
  --symlink-install

source install/setup.bash
```

Every new terminal used below needs:

```bash
cd ~/FarmBot_ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Optional local checks before using the hardware:

```bash
colcon test --packages-select farmbot_vision --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/farmbot_vision
```

## 3. Launch the FarmBot stack

```bash
ros2 launch farmbot_bringup standard.launch.py \
  camera:=Standard \
  ws_path:=$HOME/FarmBot_ROS2/farmbot_data \
  serial_port:=/dev/ttyACM0
```

Wait for the serial controller's startup delay and for the camera, map, and radius
nodes to report that they are initialized.

In another sourced terminal, confirm the required interfaces:

```bash
ros2 service list -t | rg "camera/capture|camera/enable|map_info|plant_radius"
ros2 action list -t | rg "move_gantry"
```

Expected services include:

- `/camera/capture`
- `/camera/enable`
- `/map_info`
- `/plant_radius`

## 4. Check the camera framing

Enable the preview stream:

```bash
ros2 service call /camera/enable \
  farmbot_interfaces/srv/EnableCamera \
  "{enable: true}"

ros2 run rqt_image_view rqt_image_view
```

Select `/camera/image_raw`. Check that the image is clear and correctly exposed.
Do not call `/camera/capture` directly just to inspect it: the CLI prints the entire
image byte array.

Disable continuous streaming after checking the image:

```bash
ros2 service call /camera/enable \
  farmbot_interfaces/srv/EnableCamera \
  "{enable: false}"
```

## 5. Move directly above one known plant

The safest option is to use the FarmBot movement control you normally use. Move the
camera to the selected plant's mapped X/Y position and the chosen safe camera Z.

If you normally use the ROS action directly, replace `PLANT_X`, `PLANT_Y`, and
`SAFE_CAMERA_Z` with verified numeric coordinates:

```bash
ros2 action send_goal /move_gantry \
  farmbot_interfaces/action/MoveGantry \
  "{target: {x: PLANT_X, y: PLANT_Y, z: SAFE_CAMERA_Z}, speed_percent_x: 30.0, speed_percent_y: 30.0, speed_percent_z: 30.0, interpolated: true}" \
  --feedback
```

Do not copy the placeholders literally. Confirm through the camera preview that the
plant is centred before measuring. The calculator assumes the camera is perfectly
above the plant.

## 6. Measure the plant radius

Replace `PLANT_INDEX` with the selected plant's numeric index:

```bash
ros2 service call /plant_radius \
  farmbot_interfaces/srv/MeasurePlantRadius \
  "{index: PLANT_INDEX}"
```

Expected response:

```text
success=True
message='Plant radius measured'
plant_radius=<non-zero value>
```

The value should be greater than the configured 20 mm padding. Its physical accuracy
is not yet final because `mm_per_pixel=0.5` is awaiting calibration.

Confirm that the returned value was persisted:

```bash
rg -n -C 5 "index: PLANT_INDEX|plant_radius:" \
  ~/FarmBot_ROS2/farmbot_data/local_config/active_map.yaml
```

Run the measurement three times without moving the gantry. Record the three returned
values; they should be reasonably repeatable under unchanged lighting and framing.

## 7. Check the failure paths

### Unknown plant index

Keep a green plant visible, but use an index that is definitely absent:

```bash
ros2 service call /plant_radius \
  farmbot_interfaces/srv/MeasurePlantRadius \
  "{index: 9999}"
```

Expected result:

```text
success=False
message='Plant radius measured but map update failed'
```

### No plant visible

Move the camera to a clear area with no green plant, then call the service using the
valid plant index. Expect:

```text
success=False
message='No plant detected in the image'
```

The valid plant's stored radius should remain unchanged after this failed call.

### Camera unavailable

Stop the `standard_camera` node only if it is safe and convenient in your test setup,
then call `/plant_radius` again. Expect:

```text
success=False
message='Camera capture service not available'
```

This failure test is optional because stopping the full launch may stop every node.

## 8. Record the result

Record:

- Plant index and approximate physical canopy radius.
- Camera height and image resolution.
- Three returned radius values.
- Value written to `active_map.yaml`.
- Lighting conditions and any obvious false detections.
- Any ROS errors or service timeouts.

## 9. Restore the map if required

If the measured placeholder value should not remain in the real active map, stop the
nodes and restore the backup:

```bash
cp ~/FarmBot_ROS2/farmbot_data/local_config/active_map.before-radius-test.yaml \
  ~/FarmBot_ROS2/farmbot_data/local_config/active_map.yaml
```

Do not restore the file while `map_controller` is still running because it holds its
own in-memory copy of the map.

## 10. Before opening the pull request

After testing and saving your results somewhere appropriate:

```bash
cd ~/FarmBot_ROS2
rm PLANT_RADIUS_TESTING.md
git status --short
```

Confirm that `PLANT_RADIUS_TESTING.md`, `.cache/`, `.vscode/`, build artifacts, and
temporary camera images are not included in the pull request.
