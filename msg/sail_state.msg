# SAIL_SYS_STATE
uint8 SAIL_SYS_STATE_BEGIN              = 0
uint8 SAIL_SYS_STATE_PORT_TACK   	= 1
uint8 SAIL_SYS_STATE_STBD_TACK   	= 2
uint8 SAIL_SYS_STATE_TACK_PORT_STBD_1   = 3
uint8 SAIL_SYS_STATE_TACK_PORT_STBD_2   = 4
uint8 SAIL_SYS_STATE_TACK_STBD_PORT_1   = 5
uint8 SAIL_SYS_STATE_TACK_STBD_PORT_2   = 6
uint8 SAIL_SYS_STATE_GYBE_PORT_STBD_1   = 7
uint8 SAIL_SYS_STATE_GYBE_PORT_STBD_2   = 8
uint8 SAIL_SYS_STATE_GYBE_STBD_PORT_1   = 9
uint8 SAIL_SYS_STATE_GYBE_STBD_PORT_2   = 10
uint8 SAIL_SYS_STATE_STBD_TACK_RECOVERY = 11
uint8 SAIL_SYS_STATE_PORT_TACK_RECOVERY = 12

uint64 timestamp	# time since system start, microseconds
uint8 state 		# Current sailing system state
bool power_assist	# True if auxiliary mechanical propulsion should be turned on

# TOPICS sail_state
