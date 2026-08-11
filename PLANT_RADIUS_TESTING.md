# FarmBot plant-radius test (temporary — delete before PR)

Simple checklist for testing the plant-radius calculator on the FarmBot PC with the
Standard USB camera. Only brings up the three nodes this feature actually needs
(camera, map, plant-radius) — no serial cable, no gantry movement, no full stack.
Since the calculator assumes the camera is already above the plant, you just hold or
place a plant/green object under the camera by hand.

> Delete this file before opening the pull request. It is a temporary operator
> checklist, not project documentation.

## 1. Build and source

```bash
cd ~/FarmBot_ROS2
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to farmbot_vision map_handler --symlink-install
source install/setup.bash
```

Every new terminal used below needs:
```bash
cd ~/FarmBot_ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

**If the build fails**: usually a missing dependency. Try `rosdep install --from-paths src --ignore-src -r -y` from `~/FarmBot_ROS2`, then build again.

Optional local sanity check before touching hardware at all (these don't need a camera):
```bash
colcon test --packages-select farmbot_vision --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/farmbot_vision
```

## 2. Launch the three nodes

Three terminals (each sourced as above):

**Terminal A — map_controller:**
```bash
ros2 run map_handler map_controller --ros-args \
  -p ws_path:=$HOME/FarmBot_ROS2/farmbot_data \
  -p folder_config_name:=local_config
```

**Terminal B — standard_camera:**
```bash
ros2 run farmbot_vision standard_camera --ros-args \
  -p camera_index:=0 -p image_width:=640 -p image_height:=480 \
  -p frame_rate:=30.0 -p pixel_format:=MJPG -p frame_id:=camera \
  -p auto_exposure:=false -p exposure:=156.0
```
**If this fails with "Could not open video device"**: check the camera is actually
plugged in and shows up as `/dev/video0` (`ls /dev/video*`); if it's a different
index, change `camera_index`. Also make sure no other program (e.g. `cheese`,
another ROS node) already has the camera open.

**Terminal C — plant_radius:**
```bash
ros2 run farmbot_vision plant_radius --ros-args \
  -p hsv_min:="[40, 50, 50]" -p hsv_max:="[90, 255, 255]" \
  -p min_contour_area_px:=100 -p mm_per_pixel:=0.5 -p plant_radius_padding_mm:=20.0
```

## 3. Sanity check everything is up

In a fourth sourced terminal:
```bash
ros2 node list
ros2 service list -t | grep -E "camera/capture|camera/enable|map_info|plant_radius"
```
Expect `/camera/capture`, `/camera/enable`, `/map_info`, `/plant_radius`. If a
service is missing, check that terminal's node for an error/crash before continuing.

## 4. Make sure there's a plant to test against

```bash
cat ~/FarmBot_ROS2/farmbot_data/local_config/active_map.yaml
```

**If you see an existing plant** under `plant_details.plants`, note its `index` and
skip to section 5.

**If `plant_count: 0` or the file has no plants** (a fresh map), add a throwaway test
plant:
```bash
ros2 topic pub -1 /plant_mng farmbot_interfaces/msg/PlantManage \
"{add: true, autopos: false, x: 100.0, y: 100.0, z: 0.0, exclusion_radius: 10.0,
canopy_radius: 10.0, water_quantity: 0.0, max_z: 0.0, plant_name: 'test_plant',
growth_stage: 'Seedling', remove: false, index: 0}"
```
Then re-check the file — the index field above is ignored on add, `map_controller`
assigns the next free one, so confirm which index it actually got:
```bash
grep -A2 identifiers ~/FarmBot_ROS2/farmbot_data/local_config/active_map.yaml
```
Remember this index for cleanup in section 8.

## 5. (Optional) Confirm the camera is framing correctly

```bash
ros2 service call /camera/enable farmbot_interfaces/srv/EnableCamera "{enable: true}"
ros2 run rqt_image_view rqt_image_view
```
Pick `/camera/image_raw`, hold the plant/green object under the camera, confirm it's
roughly centred in frame, then:
```bash
ros2 service call /camera/enable farmbot_interfaces/srv/EnableCamera "{enable: false}"
```

## 6. Run the measurement

With the plant/object held centred under the camera, replace `PLANT_INDEX` with the
index from section 4:
```bash
ros2 service call /plant_radius farmbot_interfaces/srv/MeasurePlantRadius "{index: PLANT_INDEX}"
```
Expect:
```text
success=True
message='Plant radius measured'
plant_radius=<non-zero value, > 20.0>
```
**If `success=False, message='No plant detected in the image'`**: the green mask
isn't picking anything up — check lighting, make sure the object is actually green
and large enough (over `min_contour_area_px`), or try widening `hsv_min`/`hsv_max`
and restarting terminal C.

**If the call just hangs**: `/camera/capture` or `/map_info` probably isn't
responding — check terminals A/B for errors.

Then confirm it landed in the map:
```bash
grep -A5 "index: PLANT_INDEX" ~/FarmBot_ROS2/farmbot_data/local_config/active_map.yaml
```
The `plant_radius` value should match what the service call reported.

Run it a couple more times without moving anything — values should be reasonably
repeatable.

## 7. Check the failure paths (optional but useful)

- **Bad index**: `ros2 service call /plant_radius farmbot_interfaces/srv/MeasurePlantRadius "{index: 9999}"` → expect `success=False, message='Plant radius measured but map update failed'` (this is what the map-update-failure fix on this branch specifically covers).
- **No plant visible**: pull the object out from under the camera (or cover the lens), call again with the real index → expect `success=False, message='No plant detected in the image'`.
- **Camera down**: `Ctrl+C` terminal B, call again → expect `success=False, message='Camera capture service not available'`.

## 8. Clean up

If you added a test plant in section 4:
```bash
ros2 topic pub -1 /plant_mng farmbot_interfaces/msg/PlantManage \
"{add: false, remove: true, index: PLANT_INDEX, autopos: false, x: 0.0, y: 0.0, z: 0.0,
exclusion_radius: 0.0, canopy_radius: 0.0, water_quantity: 0.0, max_z: 0.0,
plant_name: '', growth_stage: ''}"
```
If you tested against a real existing plant and want its `plant_radius` reset,
restore from a backup taken before testing (take one first if you haven't):
```bash
cp ~/FarmBot_ROS2/farmbot_data/local_config/active_map.yaml \
  ~/FarmBot_ROS2/farmbot_data/local_config/active_map.before-radius-test.yaml   # before testing
cp ~/FarmBot_ROS2/farmbot_data/local_config/active_map.before-radius-test.yaml \
  ~/FarmBot_ROS2/farmbot_data/local_config/active_map.yaml                      # to restore
```
Don't restore the file while `map_controller` is still running — it holds its own
in-memory copy of the map and would just overwrite your restore on its next write.
Stop terminal A first.

## 9. Before opening the pull request

```bash
cd ~/FarmBot_ROS2
rm PLANT_RADIUS_TESTING.md
git status --short
```
Confirm `PLANT_RADIUS_TESTING.md`, `.cache/`, `.vscode/`, and any backup/test map
files aren't included in the pull request.
