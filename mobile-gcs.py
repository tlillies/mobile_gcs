#mobile-gcs.py
#Research and Engineering Center for Unmanned Vehicles
#Based off some code in antenna-tracker.py

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
		self.wp_distance = 400

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


		            # Assign GPS to present location, convert decimal minutes to decimal cegrees
		            #loc['lat'] = float(b[2])/100
		            #loc['lat'] = divmod(loc['lat'],1)[0]+divmod(loc['lat'],1)[1]*100/60
		            #if b[3] == 'S' : loc['lat'] = -loc['lat']
		            #loc['lon'] = float(b[4])/100
		            #loc['lon'] = divmod(loc['lon'],1)[0]+divmod(loc['lon'],1)[1]*100/60
		            #if b[5] == 'W' : loc['lon'] = -loc['lon']
		            #loc['alt'] = float(b[9])
		            #print loc


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

		## Set point of wp relative to car in meters
		#self.set_x = 0
		#self.set_y = 0
		#self.set_z = 0

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
		#self.wp = mavwp.MAVWPLoader()

		# self.wp.add(mavutil.mavlink.MAVLink_mission_item_message(self.mav.target_system,
  #           self.mav.target_component,
  #           seq,
  #           frame,
  #           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
  #           0, 0, 0, radius, 0, 0,
  #           self.wp_lat,self.wp_lon,self.wp_alt))
		#self.mav.waypoint_clear_all_send()                                     
		#self.mav.waypoint_count_send(self.wp.count())  
		msg = mavutil.mavlink.MAVLink_mission_item_message(self.mav.target_system,
            self.mav.target_component,
            seq,
            frame,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2, 0, 0, radius, 1.0, 0, lat, lon, alt)

		self.mav.mav.send(msg)

		self.rally.move(1,lat,lon)
		self.rallypoint = self.rally.rally_point(0)
		self.rallypoint.target_system = self.mav.target_system
		self.rallypoint.target_component = self.mav.target_component
		self.mav.mav.send(self.rallypoint)
		#self.mav.mav.send(rally)
		#print("Sent position!")

	def set_speed(self, speed):
		if speed < 10:
			speed = 10
		if speed > 40:
			speed = 40

		# msg = mavutil.mavlink.MAVLink_command_long_message(self.mav.target_system,
  #           self.mav.target_component,
  #           mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
  #           0, 0, speed, 0, 0, 0, 0, 0)
		# self.mav.mav.command_long_send(self.mav.target_system,
		#     self.mav.target_component,
		#     mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
		#     0, 0, speed, 0, 0, 0, 0, 0)
		#self.mav.set_mode_auto()

		# seq = 2
		# frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
		# msg = mavutil.mavlink.MAVLink_mission_item_message(self.mav.target_system,
  #           self.mav.target_component,
  #           seq,
  #           frame,
  #           mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
  #           2, 0, 0, speed, 0, 0, 0, 0, 0)
		msg = mavutil.mavlink.MAVLink_param_set_message(self.mav.target_system,
            self.mav.target_component,
            'TRIM_ARSPD_CM',
            speed*100,
            1)

		self.mav.mav.send(msg)



	def set(self,speed,lat,lon,alt):
		# Set new guided wp for ac
		self.wp_lat = lat
		self.wp_lon = lon
		self.wp_alt = alt

		self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)

		self.mav.mav.command_long_send(self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)

		seq = 0
		frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
		radius = 0
		current = 2  # flag for guided mode
		self.wp = mavwp.MAVWPLoader()

		self.wp.add(mavutil.mavlink.MAVLink_mission_item_message(self.mav.target_system,
            self.mav.target_component,
            seq,
            frame,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0, radius, 1.0, 0, self.lat, self.lon, alt))

		seq += 1

		self.wp.add(mavutil.mavlink.MAVLink_mission_item_message(self.mav.target_system,
            self.mav.target_component,
            seq,
            frame,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0, 0, 0, speed, 0, 0, 0, 0, 0))

		seq += 1

		self.wp.add(mavutil.mavlink.MAVLink_mission_item_message(self.mav.target_system,
            self.mav.target_component,
            seq,
            frame,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, radius, 1.0, 0, lat, lon, alt))

		#self.mav.waypoint_clear_all_send()                                     
		self.mav.waypoint_count_send(self.wp.count()) 

	def set_mode(self, mode):
		# msg = mavutil.mavlink.MAVLink_set_mode_message(self.mav.target_system,
  #           mode,
  #           0)
		msg = mavutil.mavlink.MAVLink_command_long_message(self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0, 0, 0, 0, 0, 0, 0, 0)
		self.mav.mav.send(msg)

	def rtl(self):
		msg = mavutil.mavlink.MAVLink_command_long_message(self.mav.target_system,
            self.mav.target_component,
            seq,
            frame,
            mavutil.mavlink.MAV_CMD_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0, 0, 0)

	def set_home(self):
		msg = mavutil.mavlink.MAVLink_mission_item_message(self.mav.target_system,
            self.mav.target_component,
            seq,
            frame,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0, 0, 1, 0, 0, 0, 0, 0, 0)
		

	def update(self):
		global readable

		self.heartbeat_timeout = time.time() - self.heartbeat_time
		if self.heartbeat_timeout > 3:
			print("LOST HEARTBEAT!!!")
			self.heartbeat = False
			self.heartbeat_time = time.time()

		# Handle message
		if self.mav.fd in readable:
			msg = self.mav.recv_match(blocking=False)
			if msg:
				#print(msg)
				if msg.get_type() == "HEARTBEAT":
					self.heartbeat_time = time.time()
					self.heartbeat = True
					#print("heartbeat")
				if msg.get_type() == "GLOBAL_POSITION_INT":
					#print(msg.lat,msg.lon,msg.relative_alt)
					self.lat = (msg.lat) / (10000000.0)
					self.lon = (msg.lon) / (10000000.0)
					self.alt = msg.alt
					self.relative_alt = (msg.relative_alt) / (1000.0)
					self.heading = (msg.hdg) / (100.0)
				if msg.get_type() == "MISSION_REQUEST":
					if self.wp:
						self.mav.mav.send(self.wp.wp(msg.seq))
						print('Sending waypoint {0}'.format(msg.seq))
				if msg.get_type() == "MISSION_ACK":
					print('MISSION_ACK %i' % msg.type)
				if msg.get_type() == "COMMAND_ACK":
					print('COMMAND_ACK command: {0} result: {1}'.format(msg.command,msg.result))
				if msg.get_type() == "PARAM_VALUE":
					print(str(msg))


def handle_input():
	global ac
	global gcs
	global readable
	global debug

	timeout = 0.01 #timeout for read from sdtin
	
	#try:
	if sys.stdin in readable:
		line = sys.stdin.readline()
		match = re.search(r'(\w+) (\w+) (\S+)', line)
		if match:
			if match.group(1) == 'debug':
				if debug == True:
					debug = False
					print("Debugging turned off")
				if debug == False:
					debug = True
					print("Debugging turned on")

			elif match.group(1) == 'set':
				# if match.group(2) == 'minspeed':
				# 	print("SET MINIMUM ALT TO {0}".format(match.group(3)))
				# elif match.group(2) == 'maxspeed':	
				# 	print("SET MAXIMUM ALT TO {0}".format(match.group(3)))
				# elif match.group(2) == 'minalt':
				# 	print("SET MINIMUM ALT TO {0}".format(match.group(3)))
				# elif match.group(2) == 'maxalt':
				# 	print("SET MAXIUM ALT TO {0}".format(match.group(3)))
				# elif match.group(2) == 'point':
				# 	print("SET POINT TO {0}".format(match.group(3)))
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

				else:
					print("NOT A KNOWN COMMAND: {0}".format(match.group(2)))
			else:
				print("NOT A KNOWN COMMAND: {0}".format(match.group(1)))
		else:
			print('COMMAND NOT FORMATED PROPERLY')

	# except:
	# 	print("Couldn't read from stdin")
	# 	exit(0)


UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

# Hardcoded location (GPS device used if available)
#loc = {'lat':33.6074232, 'lon':-102.0425716, 'alt':1004}
#heading = 245

# Autopilot connection
host = "udpin:0.0.0.0:14550"
#host = '/dev/ttyUSB0'
ac = Aircraft(host)
ac.connect()

# GPS dongle port
gps_port = '/tmp/ttyV0'
gcs = GCS(gps_port)
gcs.connect()

previous_speed = 0

output_timer = time.time()
output_timer_last = time.time()

debug = False

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

gain = .1

speed_time = time.time()

speed = 15

print("Entering main loop...")
while sys.stdin:
	readable, writable, exceptional = select.select([x for x in [ac.mav.fd, gcs.gps, sys.stdin] if x is not None], [], [])
	ac.update()
	gcs.update()
	handle_input()

	## Calculate lat lon of set point
	#
	alpha = math.radians(gcs.heading * -1)
	dist_x = ac.set_x*math.cos(alpha) - ac.set_y*math.sin(alpha)
	dist_y = ac.set_x*math.sin(alpha) + ac.set_y*math.cos(alpha)
	ac.set_lat, ac.set_lon = Dist2LatLon(gcs.lat,gcs.lon,dist_y,dist_x)

	#Print every 2 seconds
	output_timer = time.time()
	if (output_timer - output_timer_last) > 1:
		output_timer_last = time.time()
		#print("Aircraft-- Lat: {0:14}, Lon: {1:14}, Heading: {2:6}".format(ac.lat,ac.lon,ac.heading))
		#print("Ground  -- Lat: {0:14}, Lon: {1:14}, Heading: {2:6}".format(gcs.lat,gcs.lon,gcs.heading))
		ac_msg = '{"type":"ac","lat":' + str(ac.lat) + ',"lon":' + str(ac.lon) + ',"alt":' + str(ac.relative_alt) + ',"heading":' + str(ac.heading) + '}'
		wp_msg = '{"type":"wp","lat":' + str(ac.wp_lat) + ',"lon":' + str(ac.wp_lon) + ',"alt":' + str(ac.wp_alt) + ',"heading":' + str(0) + '}'
		set_msg = '{"type":"set","lat":' + str(ac.set_lat) + ',"lon":' + str(ac.set_lon) + ',"alt":' + str(ac.set_alt) + ',"heading":' + str(0) + '}'
		gcs_msg = '{"type":"gcs","lat":' + str(gcs.lat) + ',"lon":' + str(gcs.lon) + ',"alt":' + str(0.0) + ',"heading":' + str(gcs.heading) +'}'
		sock.sendto(ac_msg, (UDP_IP, UDP_PORT))
		sock.sendto(wp_msg, (UDP_IP, UDP_PORT))
		sock.sendto(set_msg, (UDP_IP, UDP_PORT))
		sock.sendto(gcs_msg, (UDP_IP, UDP_PORT))


	## Guided Way Point Update
	#Add a new guided waypoint if the ground vehicle has moved gcs.update_distance
	delta_x,delta_y = LatLon2Dist(gcs.lastset_lat-gcs.lat, gcs.lastset_lon-gcs.lon, gcs.lat)
	dist_moved = math.sqrt((delta_x*delta_x) + (delta_y*delta_y))
	if dist_moved > gcs.update_distance:
		#print("dela_x:{0} dela_y:{1} distance:{2}".format(delta_x,delta_y, dist_moved))
		#Calculate x and y distance from car based off of heading
		y = gcs.wp_distance * math.cos(math.radians(gcs.heading))
		x = gcs.wp_distance * math.sin(math.radians(gcs.heading))
		#print("x:{0} y:{1}".format(x,y))
		# Calculate lat/lon and set guided wp
		lat, lon = Dist2LatLon(ac.set_lat,ac.set_lon,y,x)
		ac.set_point(lat,lon,ac.set_alt)
		#ac.set(speed,lat,lon,ac.set_alt)
		gcs.set_point() #Set the current point to gcs update position

		# tag gcs location for next distance calculation
		gcs.lat_wp = gcs.lat
		gcs.lon_wp = gcs.lon


	
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
	p_offset = error * gain *-1
	speed = gcs.speed + p_offset
	# Update speed if changed
	if speed != previous_speed:
		if (time.time() - speed_time) > 1:
			if speed < 1:
				speed = 1
			ac.set_speed(speed)
			previous_speed = speed
			if debug:
				print("CMDSpeed:{0} GCSSpeed:{1} Error:{2} Rel_y:{3} AC_y:{4} AC_x:{5}".format(speed,gcs.speed,error,ac.rel_y,ac_y,ac_x))
			speed_time = time.time()