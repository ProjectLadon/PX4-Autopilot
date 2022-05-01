# should match the SAIL_POS_ID MAVLink enum 1:1
uint8 SAIL_POS_ID_SINGLE    = 0
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

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample # the timestamp of the raw data (microseconds)

uint8 source_pos		# source of the incoming wind data

float32 true_dir		# True wind direction, degrees counter-clockwise from north.
float32 true_speed		# True wind speed (m/s)
bool true_valid			# True if the true wind data is valid

float32 app_dir			# Apparent wind direction, degrees counter-clockwise from north.
float32 app_speed		# Apparent wind speed (m/s)
bool app_valid			# True if the apparent wind data is valid

float32 raw_dir			# Raw wind direction, degrees counter-clockwise from north.
float32 raw_speed		# Raw wind speed (m/s)

# TOPICS wind_data nmea_wind wing_wind forewing_wind mizzenwing_wind
