# Message Interface used for representing set and read I2C Commands
bool mode   # mode = 1 for SET, = 0  for READ
int64 e     # Element in tool holder
int64 p     # Pin number
int64 v     # Value number