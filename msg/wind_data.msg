uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32 true_dir		# True wind direction, degrees counter-clockwise from north.
float32 true_speed		# True wind speed (m/s)
bool true_valid			# True if the true wind data is valid

float32 app_dir			# Apparent wind direction, degrees counter-clockwise from north.
float32 app_speed		# Apparent wind speed (m/s)
bool app_valid			# True if the apparent wind data is valid

float32 raw_dir			# Raw wind direction, degrees counter-clockwise from north.
float32 raw_speed		# Raw wind speed (m/s)

# TOPICS wind_data nmea_wind wing_wind
