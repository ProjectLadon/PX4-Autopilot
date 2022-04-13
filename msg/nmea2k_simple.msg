# Incoming NMEA2000 simple (i.e. single frame) message
uint64 timestamp 	# time since system start (microseconds)

uint8 priority		# message priority
uint8 source		# source address
uint32 pgn 		# PGN of incoming message
uint8[8] data 		# Data in incoming message

# TOPICS nmea2k_simple nmea2k_simple0 nmea2k_simple1
