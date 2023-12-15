# Message interface used for setting or reading from the pins
# on the farmduino

bool mode       # 1 for SET, 0 for READ
bool set_io     # If mode = True, and set_IO, the IO state of the pin will be set
bool set_value  # if mode = True, and set)value, the value on the pin will be set
bool set_value2 # if mode and set_value2 are true, the double value set command will be used
int64 pin       # PIN worked on
int64 value     # value to write on pin
int64 value2    # a second value to set on the pin after a delay
int64 delay     # time delay in millis
bool pin_mode   # (0-digital / 1-analog) OR (0-input / 1-output)