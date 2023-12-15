# Message Interface to quickly transmit state commands
bool estop          # Electronic Stop
bool abort_movement # Abort current movement command
bool reset_estop    # Reset the Electronic Stop
bool rep_end_stop   # Report End Stop
bool rep_curr_pos   # Report the current position
bool rep_sw_ver     # Report Software Version