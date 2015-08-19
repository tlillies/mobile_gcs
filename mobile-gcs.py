#mobile-gcs.py
#Research and Engineering Center for Unmanned Vehicles

import json
import math
import serial
import socket
import re
import select
import time
from pymavlink import mavutil
from pymavlink import mavwp
import sys
import select

## Connection settings

# KML
#UDP_IP = "127.0.0.1"
UDP_IP = "192.168.8.223"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

# Autopilot connection
ac_host = "udpin:0.0.0.0:14550"
#ac_host = '/dev/ttyUSB0'

# GPS dongle port
#gps_port = '/tmp/ttyV0'
gps_port = '/dev/ttyUSB0' 

## Gains

gain_front = .001
gain_behind = .0005


## Flight settings

alt_base = 200
alt_amp = 0
alt_per = 240
alt_fre = (2*math.pi) / alt_per


def LatLon2Dist(lat_diff, lon_diff, lat_ref):
    #LatLon2Dist converts differences in lat/lon position to cartesian
    #Inputs:
    #       lat_diff = latitude difference [deg]
    #       lon_diff = longitude difference [deg]
    #       lat_ref  = latitude reference [deg]
    #Outputs:
    #       x = east/west difference
    #       y = north/south difference
    #Assumptions:
    #       differences are small enough such that east/west distances
    #       are constant for variable lattitude
    #radius of the Earth [m]
    re = 6378100
    #north/south difference (just arc length)
    y = lat_diff*(math.pi*re)/180
    #correct for longitude lines getting closer together near poles
    re_c = re*math.cos(abs(lat_ref*math.pi/180))
    #east/west difference
    x = lon_diff*(math.pi*re_c)/180
    return x, y

def Dist2LatLon(lat,lon,dn,de):
	# William's aviation
	# dn -- distance north
	# de -- distance east
	re = 6378137

	dLat = dn/re
	dLon = de/(re*math.cos(math.pi*lat/180))

	lat0 = lat + dLat * 180/math.pi
	lon0 = lon + dLon * 180/math.pi

	return lat0, lon0

def AzimuthNearest(az, az_prev, cal):
    az_nominal = cal['az_min']+(az/math.pi/2)*(cal['az_max']-cal['az_min'])
    az_candidates = [ az_nominal+x*(cal['az_max']-cal['az_min']) for x in range(-3,4)]
    az_candidates = [x for x in az_candidates if (x>2600 and x<9400)]
    az_cmd = az_candidates[0]
    for i in az_candidates:
        if abs(i-az_prev) < abs(az_cmd-az_prev): az_cmd = i
    return az_cmd

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
		self.set_x = 0
		self.set_y = 0
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
		print("Connecting to autopilot...")
		#self.mav = mavutil.mavlink_connection('/dev/ttyACM0', use_native=False,baud=921600, dialect="ardupilotmega")
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

	def set_rally(self,gcs):
		self.rally.move(1,gcs.lat,gcs.lon)
		self.rallypoint = self.rally.rally_point(0)
		self.rallypoint.target_system = self.mav.target_system
		self.rallypoint.target_component = self.mav.target_component
		self.mav.mav.send(self.rallypoint)

	def set_speed(self, speed):
		if speed < 14:
			speed = 14
		if speed > 30:
			speed = 30

		msg = mavutil.mavlink.MAVLink_param_set_message(self.mav.target_system,
            self.mav.target_component,
            'TRIM_ARSPD_CM',
            speed*100,
            1)

		self.mav.mav.send(msg)
		

	def update(self):
		global readable
		global debug_mission


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
						if debug_mission:
							print('Sending waypoint {0}'.format(msg.seq))
				if msg.get_type() == "MISSION_ACK":
					if debug_mission:
						print('MISSION_ACK %i' % msg.type)
				if msg.get_type() == "COMMAND_ACK":
					if debug_mission:
						print('COMMAND_ACK command: {0} result: {1}'.format(msg.command,msg.result))
				if msg.get_type() == "PARAM_VALUE":
						ac.got_speed = msg.param_value/100.0


def handle_input(self):
	global ac
	global gcs
	global readable
	global debug
	global debug_mission
	global debug_speed
	global debug_dist
	global debug_position
	global debug_alt
	global debug_wind

	try:
		if sys.stdin in readable:
			line = sys.stdin.readline()
			match = re.search(r'(\w+) (\w+) (\S+)', line)
			if match:
				if match.group(1) == 'debug':
					if match.group(2) == 'speed':
						if match.group(3) == 'on':
							print("DEBUG SPEED SET ON")
							debug_speed = True
						else:
							print("DEBUG SPEED SET OFF")
							debug_speed = False
					if match.group(2) == 'mission':
						if match.group(3) == 'on':
							print("DEBUG MISSION SET ON")
							debug_mission = True
						else:
							print("DEBUG MISSION SET OFF")
							debug_mission = False
					if match.group(2) == 'general':
						if match.group(3) == 'on':
							print("DEBUG GENERAL SET ON")
							debug = True
						else:
							print("DEBUG GENERAL SET OFF")
							debug = False
					if match.group(2) == 'position':
						if match.group(3) == 'on':
							print("DEBUG POSITION SET ON")
							debug_position = True
						else:
							print("DEBUG POSITION SET OFF")
							debug_position = False
					if match.group(2) == 'distance':
						if match.group(3) == 'on':
							print("DEBUG DISTANCE SET ON")
							debug_dist = True
						else:
							print("DEBUG DISTANCE SET OFF")
							debug_dist = False
					if match.group(2) == 'alt':
						if match.group(3) == 'on':
							print("DEBUG ALT SET ON")
							debug_alt = True
						else:
							print("DEBUG ALT SET OFF")
							debug_alt = False
					if match.group(2) == 'wind':
						if match.group(3) == 'on':
							print("DEBUG WIND SET ON")
							debug_wind = True
						else:
							print("DEBUG WIND SET OFF")
							debug_wind = False

				elif match.group(1) == 'set':
					if match.group(2) == 'alt':
						print('SET ALT TO {0}'.format(match.group(3)))
						z = int(match.group(3))
						if z > 500:
							z = 500
						if z < 50:
							z = 50
						ac.set_alt = z

					elif match.group(2) == 'x':
						x = int(match.group(3))
						if x > 500:
							x = 500
						if x < -500:
							x = -500
						ac.set_x = x
						print('SET X TO {0}'.format(match.group(3)))

					elif match.group(2) == 'y':
						y = int(match.group(3))
						if y > 500:
							y = 500
						if y < -500:
							y = -500
						ac.set_y = y
						print('SET Y TO {0}'.format(match.group(3)))
					elif match.group(2) == 'wind':
						if match.group(3) == "off":
							ac.set_wind = False
							print('SET WIND TO OFF')
						else:
							wind_speed = int(match.group(3))
							ac.set_wind_speed = wind_speed
							ac.set_wind = True
							print('SET WIND SPEED TO {0}'.format(ac.set_wind_speed))
							print('SET WIND DIRECTION TO {0}'.format(ac.set_wind_direction))

					elif match.group(2) == 'wind_dir':
						if match.group(3) == "off":
							ac.set_wind = False
							print('SET WIND TO OFF')
						else:
							wind_dir = int(match.group(3))
							ac.set_wind_direction = wind_dir
							ac.set_wind = True
							print('SET WIND SPEED TO {0}'.format(ac.set_wind_speed))
							print('SET WIND DIRECTION TO {0}'.format(ac.set_wind_direction))

					else:
						print("NOT A KNOWN COMMAND: {0}".format(match.group(2)))
				else:
					print("NOT A KNOWN COMMAND: {0}".format(match.group(1)))
			else:
				print('COMMAND NOT FORMATED PROPERLY')

	except:
	 	print("Couldn't read from stdin")


## Setup

ac = Aircraft(ac_host)
ac.connect()

gcs = GCS(gps_port)
gcs.connect()

previous_speed = 0

error_prev = 0

output_timer = time.time()
output_timer_last = time.time()

debug = False
debug_speed = False
debug_dist = False
debug_mission = False
debug_position = False
debug_alt = False
debug_wind = False

# Wait for GPS
while (not gcs.lat):
	readable, writable, exceptional = select.select([x for x in [ac.mav.fd, gcs.gps, sys.stdin] if x is not None], [], [])
	gcs.update()

ac.set_point(gcs.lat,gcs.lon,100)
gcs.set_point()
alt = 100

# Wait for AC GPS
while (not ac.lat):
	readable, writable, exceptional = select.select([x for x in [ac.mav.fd, gcs.gps, sys.stdin] if x is not None], [], [])
	ac.update()



speed_time = time.time()

time_prev = time.time()

print_timer_last = time.time()

speed = 15

lat = gcs.lat
lon = gcs.lon

print("Entering main loop...")
while sys.stdin:
	readable, writable, exceptional = select.select([x for x in [ac.mav.fd, gcs.gps, sys.stdin] if x is not None], [], [])


	## Update devices
	ac.update()
	gcs.update()
	handle_input()


	## Calculate lat lon of set point
	alpha = math.radians(gcs.heading * -1)
	dist_x = ac.set_x*math.cos(alpha) - ac.set_y*math.sin(alpha)
	dist_y = ac.set_x*math.sin(alpha) + ac.set_y*math.cos(alpha)
	ac.set_lat, ac.set_lon = Dist2LatLon(gcs.lat,gcs.lon,dist_y,dist_x)


	## Calculate alt
	ac.set_alt = alt_base + alt_amp * math.sin(alt_fre *time.time())


	## Speed controller
	# Get distance between car and ac. x east/west, y north/south
	ac_x,ac_y = LatLon2Dist(ac.set_lat-ac.lat,ac.set_lon-ac.lon, gcs.lat)
	ac_y *= -1
	ac_x *= -1
	# Calculate the position of ac relative to car
	alpha = math.radians(gcs.heading * (-1))
	ac.rel_x = ac_x*math.cos(alpha) + ac_y*math.sin(alpha)
	ac.rel_y = (-1)*ac_x*math.sin(alpha) + ac_y*math.cos(alpha)
	# Calculate error
	error = ac.rel_y - ac.set_y
	# Calculate corrected speed
	p_offset = error * error
	if error < 0:
		p_offset *= gain_front
	else:
		p_offset *= gain_behind *-1

	speed = gcs.speed + p_offset
	speed_temp = speed


	## Wind adjustment
	if ac.set_wind: # Custom set wind vector vs autopilot wind
		wind_speed = ac.set_wind_speed
		wind_direction = ac.set_wind_direction
	else:
		wind_speed = ac.wind_speed
		wind_direction = ac.set_wind_direction
	speed_x = -1 * speed * math.sin(math.radians(gcs.heading))
	speed_y = -1 * speed * math.cos(math.radians(gcs.heading))
	wind_x = wind_speed * math.sin(math.radians(wind_direction))
	wind_y = wind_speed * math.cos(math.radians(wind_direction))
	ac_speed_x = speed_x - wind_x
	ac_speed_y = speed_y - wind_y
	speed = math.sqrt((ac_speed_x*ac_speed_x)+(ac_speed_y*ac_speed_y))


	## Send out data and update wp
	output_timer = time.time()
	if (output_timer - output_timer_last) > .2:
		output_timer_last = time.time()
		
		# Sound out JSON data for kml updater
		ac_msg = '{"type":"ac","lat":' + str(ac.lat) + ',"lon":' + str(ac.lon) + ',"alt":' + str(ac.relative_alt) + ',"heading":' + str(ac.heading) + '}'
		wp_msg = '{"type":"wp","lat":' + str(ac.wp_lat) + ',"lon":' + str(ac.wp_lon) + ',"alt":' + str(ac.wp_alt) + ',"heading":' + str(0) + '}'
		set_msg = '{"type":"set","lat":' + str(ac.set_lat) + ',"lon":' + str(ac.set_lon) + ',"alt":' + str(ac.set_alt) + ',"heading":' + str(0) + '}'
		gcs_msg = '{"type":"gcs","lat":' + str(gcs.lat) + ',"lon":' + str(gcs.lon) + ',"alt":' + str(0.0) + ',"heading":' + str(gcs.heading) +'}'

		sock.sendto(ac_msg, (UDP_IP, UDP_PORT))
		sock.sendto(wp_msg, (UDP_IP, UDP_PORT))
		sock.sendto(set_msg, (UDP_IP, UDP_PORT))
		sock.sendto(gcs_msg, (UDP_IP, UDP_PORT))


		# Calculate next waypoint
		y = gcs.wp_distance * math.cos(math.radians(gcs.heading))
		x = gcs.wp_distance * math.sin(math.radians(gcs.heading))

		# Calculate lat/lon and set guided wp
		lat, lon = Dist2LatLon(ac.set_lat,ac.set_lon,y,x)
		gcs.set_point() #Set the current point to gcs update position

		# Send new waypoint
		ac.set_point(lat,lon,ac.set_alt)
		ac.set_rally(gcs)

		# Send Speed
		ac.set_speed(speed)

	## Print out debug info
	print_timer = time.time()
	if (print_timer - print_timer_last) > 1:
		print_timer_last = time.time()
		# Print out data
		if debug_position:
			print("Aircraft-- Lat: {0:14}, Lon: {1:14}, Heading: {2:6}".format(ac.lat,ac.lon,ac.heading))
			print("Ground  -- Lat: {0:14}, Lon: {1:14}, Heading: {2:6}".format(gcs.lat,gcs.lon,gcs.heading))
		if debug_alt:
			print("Set Alt: {0}, Base Alt: {1}, Alt Amp: {2}, Alt Freq: {3}".format(ac.set_alt,alt_base,alt_amp,alt_fre))
		if debug_dist:
			print("dela_x:{0} dela_y:{1}".format(delta_x,delta_y))
			print("x:{0} y:{1}".format(x,y))
		if debug:
			print("CMDSpeed:{0} GCSSpeed:{1} Pf:{2} Error:{4}".format(speed,gcs.speed,p_offset,error))
		if debug_speed:
			print("Current Set Speed: {0}".format(ac.got_speed))
			print("Controller: {0}, Wind Adjust: {1}".format(speed_temp,speed))
			print("Current GCS Speed: {0}".format(gcs.speed))
			print("GCS Speed_x: {0}, GCS Speed_y: {1}".format(speed_x,speed_y))
			print("AC_speed_x: {0}, AC_speed_y: {1}".format(ac_speed_x,ac_speed_y))
		if debug_wind:
			print("Wind Speed: {0}, Wind Direction: {1}".format(wind_speed,wind_direction))
			print("Wind_x: {0}, Wind_y: {1}".format(wind_x,wind_y))
			print("Set Wind: {0}".format(ac.set_wind))
			print("Autopilot Wind: {0}, Autopilot Direction: {1}".format(ac.wind_speed, ac.wind_direction))
			print("Set Wind Speed: {0}, Set Wind Direction: {1}".format(ac.set_wind_speed, ac.set_wind_direction))
			print("Send Speed: {0}".format(speed))
			