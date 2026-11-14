# Command-type constants
uint8 CMD_SAVE = 1
uint8 CMD_GET = 2
uint8 CMD_SET_ALL = 3
uint8 CMD_SET_SINGLE = 4

uint8 command_type
uint8 param_id        # Only effective when CMD_SET_SINGLE is used

# GUIThe GUI passes actual floating-point values; the chassis converts, packages, and sends them to the serial port
float32 param_value   # Only effective when CMD_SET_SINGLE is used
float32 balance_kp
float32 balance_kd
float32 velocity_kp
float32 velocity_ki
float32 turn_kp
float32 turn_ki
float32 turn_kd
float32 middle_angle