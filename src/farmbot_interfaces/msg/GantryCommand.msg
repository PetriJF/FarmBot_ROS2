# Message to represent the movement of the farmbot's gantry
bool mode   # The movement mode. 0 - standard, 1 - straight line (needs constant feed rate)
float64 x     # Desired x coordinate
float64 y     # Desired y coordinate
float64 z     # Desired z coordinate
float64 a     # Travel Speed for the x-axis
float64 b     # Travel Speed for the y-axis
float64 c     # Travel Speed for the z-axis