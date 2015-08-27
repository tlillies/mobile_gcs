#mobile-gcs.py
#Research and Engineering Center for Unmanned Vehicles

class GCS:
	""" Class for GCS data """
	def __init__(self, port):
		# lat lon alt of actual car
		self.lat = None
		self.lon = None
		self.alt = None
		# position where last waypoint was set
		self.lat_wp = None
		self.lon_wp = None
		self.alt_wp = None

		self.lastset_lat = None
		self.lastset_lon = None
		self.lastset_alt = None

		self.port= port
		self.gps = None

		self.knots = None
		self.speed = None
		self.heading = None

		self.update_distance = 25
		self.wp_distance = 200

		self.error = True
		self.active = False
		self.active_timeout = 0
		self.active_time = time.time()

	def lat_diff(self):
		# distance between current lat and lat where last wp was set
		return self.lat - self.lat_wp

	def lon_diff(self):
		# distance between current lon and lon where last wp was set
		return self.lon - self.lon_wp

	def connect(self):
		try:
			print("Connecting to GPS...")
			self.gps = serial.Serial(port=self.port, baudrate=4800, timeout=.1)
			print("Got GPS!")
		except:
		    print("GPS CONECT FAILED")

	def set_point(self):
		self.lastset_lat = self.lat
		self.lastset_lon = self.lon
		self.lastset_alt = self.alt

	def update(self):
		global readable
		#Check for timeout
		self.active_timeout = time.time() - self.active_time
		if self.active_timeout > 3:
			print("LOST GCS GPS!!!")
			self.active = False
			self.active_time = time.time()

		# Get and handle message
		if self.gps in readable:
			raw = self.gps.readline()
			self.active_time = time.time()
			self.active = True
			data = raw.split(',')
			if data[0] == "$GPRMC":
				if data[2] == 'A': # check for valid GPS fix

					lat = float(data[3])/100
					lat = divmod(lat,1)[0]+divmod(lat,1)[1]*100/60
					if data[4] == 'S':
						lat *= (-1)

					lon = float(data[5])/100
					lon = divmod(lon,1)[0]+divmod(lon,1)[1]*100/60
					if data[6] == 'W':
						lon *= (-1)

					speed = float(data[7]) * (0.514444)

					self.lat = lat
					self.lon = lon
					self.heading = float(data[8])
					self.speed = speed
					self.error = False

				if data[2] == 'V':
					self.error = True