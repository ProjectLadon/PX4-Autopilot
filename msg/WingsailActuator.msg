uint64 timestamp            # time since system start (microseconds)

# should match the SAIL_ANGLE_TYPE MAVLink enum 1:1
uint8 SAIL_ANGLE_TYPE_AOA = 1
uint8 SAIL_ANGLE_TYPE_ROT = 2

# should match the SAIL_POS_ID MAVLink enum 1:1
uint8 SAIL_POS_ID_SINGLE    = 0
uint8 SAIL_POS_ID_MIXED     = 201
uint8 SAIL_POS_ID_1         = 1
uint8 SAIL_POS_ID_2         = 2
uint8 SAIL_POS_ID_3         = 3
uint8 SAIL_POS_ID_4         = 4
uint8 SAIL_POS_ID_5         = 5
uint8 SAIL_POS_ID_6         = 6
uint8 SAIL_POS_ID_7         = 7
uint8 SAIL_POS_ID_8         = 8
uint8 SAIL_POS_ID_FORE      = 20
uint8 SAIL_POS_ID_MAIN      = 21
uint8 SAIL_POS_ID_MIZZEN    = 22

uint8 target_sail           # The target MAVLink wingsail ID (values from the relevant
uint8 sail_angle_type       # The type of the sail angle
float32 sail_angle          # Angle of the sail (deg)
uint8 flap_active_map       # Bitmap of active flaps
float32[8] flap_angle       # Angle of each flap (deg)

# TOPICS wingsail_actuator forewing_actuator mizzenwing_actuator
