#Aircraft.py
#Research and Engineering Center for Unmanned Vehicles
import time
import sys

from pymavlink import mavutil
from pymavlink import mavwp

class Aircraft:
	""" Class for Aircraft data """
	def __init__(self, host):

		self.heartbeat = False
		self.heartbeat_timeout = 0
		self.heartbeat_time = time.time()

		self.mav = None
		self.host = host

		self.speed = None
		self.heading = None

		# actual location of ac in lat/lon/alt
		self.lat = None
		self.lon = None
		self.alt = None
		self.relative_alt = None

		# set point of the wp in lat/lon/alt
		self.wp_lat = None
		self.wp_lon = None
		self.wp_alt = None

		#set point of the intended position in lat/lon/alt
		self.set_lat = None
		self.set_lon = None
		self.set_alt = 100


		# Location of ac relative to car in meters
		self.rel_x = None
		self.rel_y = None
		self.rel_z = None

		# Set point of ac position relative to car in meters
		self.x_offset = 0
		self.y_offset = 0
		self.set_z = 100

		# max and min alt
		self.min_alt = 75
		self.max_alt = 250

		# max x and y distance in meters from car
		self.max_x = 200
		self.max_y = 200

		# Min and max airspeed
		self.as_min = 14
		self.as_max = 25

		self.rally = None
		self.rallypoint = None

		self.wp = None

		self.wind_speed = 0
		self.wind_direction = 0

		# Custom set wind
		self.set_wind = False
		self.set_wind_speed = 0
		self.set_wind_direction = 0

		# Speed AC has been set to
		self.got_speed = 0

	def connect(self):
		try:
			print("Connecting to autopilot...")
			self.mav = mavutil.mavlink_connection(self.host, use_native=False)
			print("Connected to autopilot!")

			print("Waiting for heartbeat...")
			self.mav.wait_heartbeat(blocking=True)
			print("Got Heartbeat!")
			print("Waiting for plane GPS")
			msg = self.mav.recv_match(type=['GLOBAL_POSITION_INT'],blocking=True)
			print("Got plane GPS!")

			self.lat = (msg.lat) / (10000000.0)
			self.lon = (msg.lon) / (10000000.0)
			self.alt = msg.alt
			self.relative_alt = (msg.relative_alt) / (1000.0)
			self.heading = (msg.hdg) / (100.0)

			self.rally = mavwp.MAVRallyLoader(self.mav.target_system,self.mav.target_component)
			self.rally.create_and_append_rally_point(self.lat * 1e7,self.lon * 1e7 ,100,50,0,0)
			self.rallypoint = self.rally.rally_point(0)
			self.rallypoint.target_system = self.mav.target_system
			self.rallypoint.target_component = self.mav.target_component
			self.mav.mav.send(self.rallypoint)
			return True
		except:
			return False

	def set_point(self, lat, lon, alt):
		# Set new guided wp for ac
		self.wp_lat = lat
		self.wp_lon = lon
		self.wp_alt = alt

		seq = 0
		frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
		radius = 0
		current = 2  # flag for guided mode

		msg = mavutil.mavlink.MAVLink_mission_item_message(self.mav.target_system,
            self.mav.target_component,
            seq,
            frame,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            current, 0, 0, radius, 1.0, 0, lat, lon, alt)

		self.mav.mav.send(msg)

	def set_x(self,x):
		if x > 500:
			x = 500
		if x < -500:
			x = -500
		self.x_offset = x

	def set_y(self,y):
		if y > 500:
			y = 500
		if y < -500:
			y = -500
		self.y_offset = y

	# def set_alt(self,alt):
	# 	if alt > 500:
	# 		alt = 500
	# 	if alt < 50:
	# 		alt = 50
	# 	self.set_alt = alt

	def set_wind_speed(self,wind_speed):
		self.set_wind_speed = wind_speed

	def set_wind_dir(self,wind_dir):
		self.set_wind_direction = wind_dir

	def set_wind(self, set_wind):
		self.set_wind = set_wind

	def set_as_minmax(as_min,as_max):
		if as_min < 5:
			as_min = 5
		if as_max > 40:
			as_max = 40
		self.as_min = as_min
		self.as_max = as_max

	def set_rally(self,gcs):
		self.rally.move(1,gcs.lat,gcs.lon)
		self.rallypoint = self.rally.rally_point(0)
		self.rallypoint.target_system = self.mav.target_system
		self.rallypoint.target_component = self.mav.target_component
		self.mav.mav.send(self.rallypoint)

	def set_speed(self, speed):
		if speed < self.as_min:
			speed = self.as_min
		if speed > self.as_max:
			speed = self.as_max

		msg = mavutil.mavlink.MAVLink_param_set_message(self.mav.target_system,
            self.mav.target_component,
            'TRIM_ARSPD_CM',
            speed*100,
            1)

		self.mav.mav.send(msg)
		

	def update(self,readable,debug):
		self.heartbeat_timeout = time.time() - self.heartbeat_time
		if self.heartbeat_timeout > 3:
			print("LOST HEARTBEAT!!!")
			self.heartbeat = False
			self.heartbeat_time = time.time()

		# Handle message
		if self.mav.fd in readable:
			msg = self.mav.recv_match(blocking=False)
			if msg:
				if msg.get_type() == "HEARTBEAT":
					self.heartbeat_time = time.time()
					self.heartbeat = True
				if msg.get_type() == "GLOBAL_POSITION_INT":
					self.lat = (msg.lat) / (10000000.0)
					self.lon = (msg.lon) / (10000000.0)
					self.alt = msg.alt
					self.relative_alt = (msg.relative_alt) / (1000.0)
					self.heading = (msg.hdg) / (100.0)
				if msg.get_type() == "WIND":
					self.wind_speed = msg.speed
					self.wind_direction = msg.direction
				if msg.get_type() == "MISSION_REQUEST":
					if self.wp:
						self.mav.mav.send(self.wp.wp(msg.seq))
						if debug['debug_mission']:
							print('Sending waypoint {0}'.format(msg.seq))
				if msg.get_type() == "MISSION_ACK":
					if debug['debug_mission']:
						print('MISSION_ACK %i' % msg.type)
				if msg.get_type() == "COMMAND_ACK":
					if debug['debug_mission']:
						print('COMMAND_ACK command: {0} result: {1}'.format(msg.command,msg.result))
				if msg.get_type() == "PARAM_VALUE":
						self.got_speed = msg.param_value/100.0