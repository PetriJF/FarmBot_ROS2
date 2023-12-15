# Message interface used to encode the parameter handling commands
bool list       # List all parameters
bool write      # Write mode
bool read       # Read Mode
bool update     # Update mode (during calibration)
int64 param     # The parameter we are working on
int64 value     # Value to write/update