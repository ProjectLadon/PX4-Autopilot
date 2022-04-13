 # This is the uORB equivalent of the PID_TUNE MAVLink message

uint8 PID_TUNE_AXIS_ROLL        = 1
uint8 PID_TUNE_AXIS_PITCH       = 2
uint8 PID_TUNE_AXIS_YAW         = 3
uint8 PID_TUNE_AXIS_ACCZ        = 4
uint8 PID_TUNE_AXIS_STEER       = 5
uint8 PID_TUNE_AXIS_LANDING     = 6

uint64 timestamp

uint8 axis          # The axis this tuning message refers to
float32 desired     # Desired rate
float32 achieved    # Achieved rate
float32 ff          # Feed-forward component
float32 p           # Proportional component
float32 i           # Integral component
float32 d           # Derivative component
float32 srate       # Slew rate
float32 pdmod       # P/D oscillation modifier

# TOPIC pid_tune
