These are the high level commands that the user inputs in order to achieve a specific functionality. The low level commands, including the sequencing commands are covered in another wiki page.

# Movement commands

Movement commands are used to move the gantry relative to the home position.

| Code | Subcodes    | Description                                                                                                                                                                                                                                                                      |
| ---- | ----------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| w    |             | Adds a set increment to the current gantry x-axis position and moves to it. NOTE: The final position is relative to the current position at the time of the movement request (i.e. if you want the robot to go '2 w-s' forward, you have to wait for the first one to complete). |
| a    |             | Same as above, but substracts increment from the y-axis                                                                                                                                                                                                                          |
| s    |             | Same as above, but substracts increment from x-axis                                                                                                                                                                                                                              |
| d    |             | Same as above, but adds increment to y-axis                                                                                                                                                                                                                                      |
| 1    |             | Sets movement increment to 10mm                                                                                                                                                                                                                                                  |
| 2    |             | Sets movement increment to 100mm                                                                                                                                                                                                                                                 |
| 3    |             | Sets movement increment to 500mm                                                                                                                                                                                                                                                 |
| H_0  |             | Go to Home Position                                                                                                                                                                                                                                                              |
| H_1  |             | Find all Home Positions                                                                                                                                                                                                                                                          |
| H_2  | X, Y or Z   | Find the home position for the specified axis. **Note that you can only select one at a time!**                                                                                                                                                                                  |
| M    | {x} {y} {z} | Move the gantry to the parsed position (e.g. M 100 11.2 -150.0)                                                                                                                                                                                                                  |
# Farmbot Configuration and Status Commands

These represent high priority commands and configuration commands for the farmbot.

| Code | Subcodes | Description                                                                                                                                                                                                                                         |
| ---- | -------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| e    |          | ESTOP command (Electronic Stop). Halts any ongoing operation on the farmbot, and clears all sequences and queues. The robot axis are un actuated in this case and the robot cannot perform any physical action until the ESTOP RESET command is set |
| E    |          | ESTOP RESET. Resets the farmbot from the EStop state back to the working state                                                                                                                                                                      |
| C_0  |          | Calibrate all the axis length and home position                                                                                                                                                                                                     |
|      | X        | Calibrate X axis length and home position                                                                                                                                                                                                           |
|      | Y        | Calibrate Y axis length and home position                                                                                                                                                                                                           |
|      | Z        | Calibrate Z axis length and home position                                                                                                                                                                                                           |
| C_1  | {conf}   | Load parameter configuration for the farmbot version you are using. For Farmbot Genesis {conf} = Gen, gen, Genesis, genesis (any of the 4 is accepted)                                                                                              |
| CONF | {var}    | Saves the config and/or map information to memory. If *var* is left **empty**, both the map and parameter configs are saved. If you want to save a specific config use **S** for parameter configuration and **M** for map information.             |
# Tool and Tray commands

These commands handle the position and information of the toolheads and trays that are mounted on the side of the farmbot's bed.

| Code  | Subcodes         | Description                                                                                                                                                                                                  |
| ----- | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| T_n_0 | tn x y z dir     | Set the location of the Toolhead **n** by specifying its name (**tn**), position (**x, y, z**) and release direction (**dir**). E.g. *T_1_0 Seeder 1198.0 332.4 -240.0 1*                                    |
| T_n_1 |                  | Mount the tool with the index **n**. E.g. *T_1_1*                                                                                                                                                            |
| T_n_2 |                  | Unmount the tool with the index **n**. E.g. *T_1_2*                                                                                                                                                          |
| S_n_0 | type plant x y z | Setting the location of the Seed Tray of index **n** by specifying the tray type (**type**), seed loaded in the tray (**plant**) and position (**x, y, z**). E.g. *S_1_0 0 Tray1 Radish 1198.0 332.4 -240.0* |

# Plant Commands

Commands used to manage the plants that are loaded onto the farmbot.

| Code | Subcodes                                 | Description                                                                                                                                                                                                                                                                                                                                                                                                 |
| ---- | ---------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| P_1  | x y z exl_r can_r water max_z name stage | Adds a plant to a position with personalized information. **x, y, z** represent the position of the plant, **exl_r** represents the exclusion radius, **can_r** represents the canopy radius, **water** represents the water quantity, **name** represents the plant's name and **stage** represents the growth stage the plant is currently at. E.g. *P_1 100.0 200.0 -290.0, 50.0 30.0 6 Tomato Planning* |
| P_2  | index                                    | Removes the plant with the parsed **index**. E.g. *P_2 3*                                                                                                                                                                                                                                                                                                                                                   |
| P_3  |                                          | Seed all the plants that are in the "Planning" growth stage.                                                                                                                                                                                                                                                                                                                                                |
| P_4  |                                          | Water all the plants                                                                                                                                                                                                                                                                                                                                                                                        |

# Device Commands

Commands that control the different devices connected to the farmbot.

| Code  | Subcodes | Description                                                           |
| ----- | -------- | --------------------------------------------------------------------- |
| D_L_a |          | Turns on (**a** = 1) or off (**a** = 0) the LED strip. E.g. *D_L_1*   |
| D_W_a |          | Turns on (**a** = 1) or off (**a** = 0) the water pump. E.g. *D_W_1*  |
| D_V_a |          | Turns on (**a** = 1) or off (**a** = 0) the vacuum pump. E.g. *D_V_1* |
| D_C   |          | Checks if there is a tool mounted on the UTP mount                    |

# Vision Commands

The commands that are related to the vision system.

| Code | Subcodes | Description                                                                                             |
| ---- | -------- | ------------------------------------------------------------------------------------------------------- |
| I_0  |          | Calibrates the camera                                                                                   |
| I_1  |          | Takes a picture at the current location and stitches it to the panorama                                 |
| I_2  |          | Creates a sequence that has the farmbot create a panorama of the whole bed by taking pictures in a grid |
