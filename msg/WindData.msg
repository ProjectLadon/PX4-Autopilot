# SAIL_POS_ID -- should match 1:1 with the mavlink enum defined in ladon_robotics.xml
uint8 SAIL_POS_ID_SINGLE 	= 0
uint8 SAIL_POS_ID_MIXED 	= 201
uint8 SAIL_POS_ID_FORE 	= 20
uint8 SAIL_POS_ID_MAIN 	= 21
uint8 SAIL_POS_ID_MIZZEN	= 22
uint8 SAIL_POS_ID_1 		= 1
uint8 SAIL_POS_ID_2 		= 2
uint8 SAIL_POS_ID_3 		= 3
uint8 SAIL_POS_ID_4 		= 4
uint8 SAIL_POS_ID_5 		= 5
uint8 SAIL_POS_ID_6 		= 6
uint8 SAIL_POS_ID_7 		= 7
uint8 SAIL_POS_ID_8 		= 8
uint8 SAIL_POS_ID_NONE	= 255

uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample 	# the timestamp of the raw data (microseconds)

uint8 sail_position		# position of the sail the data is from, if any -- see SAIL_POS_ID

float32 true_dir		# True wind direction, degrees counter-clockwise from north.
float32 true_speed		# True wind speed (m/s)
bool true_valid			# True if the true wind data is valid

float32 app_dir			# Apparent wind direction, degrees counter-clockwise from north
float32 app_speed		# Apparent wind speed (m/s)
bool app_valid			# True if the apparent wind data is valid

float32 raw_dir			# Raw wind direction, degrees counter-clockwise from north.
float32 raw_speed		# Raw wind speed (m/s)

# TOPICS wind_data forewing_wind mizzenwing_wind mixed_wind
