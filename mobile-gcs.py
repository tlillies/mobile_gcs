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
	lon0 = lon + dLat * 180/math.pi

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

		self.port= port
		self.gps = None

		self.knots = None
		self.speed = None
		self.heading = None

		self.update_distance = 100
		self.wp_distance

	def lat_diff(self):
		# distance between current lat and lat where last wp was set
		return self.lat - self.lat_old

	def lon_diff(self):
		# distance between current lon and lon where last wp was set
		return self.lon - self.lon_old

	def connect(self):
		try:
			print("Connecting to GPS...")
			self.gps = serial.Serial(port=self.port, baudrate=4800)
			print("Got GPS!")
		except:
		    pass

	def update(self):
		pass
		# if gps in readable: # GPS dongle data available
		#     a = gps.readline()
		#     b = a.split(',')
		#     if b[0] == "$GPGGA":
		#         if b[6] is not '0': # check for valid GPS fix
		#             # Assign GPS to present location, convert decimal minutes to decimal cegrees
		#             loc['lat'] = float(b[2])/100
		#             loc['lat'] = divmod(loc['lat'],1)[0]+divmod(loc['lat'],1)[1]*100/60
		#             if b[3] == 'S' : loc['lat'] = -loc['lat']
		#             loc['lon'] = float(b[4])/100
		#             loc['lon'] = divmod(loc['lon'],1)[0]+divmod(loc['lon'],1)[1]*100/60
		#             if b[5] == 'W' : loc['lon'] = -loc['lon']
		#             loc['alt'] = float(b[9])
		#             #print loc


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

		# set point of the wp in lat/lon/alt
		self.set_lat = None
		self.set_lon = None
		self.set_alt = None

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

	def connect(self):
		print("Connecting to autopilot...")
		self.mav = mavutil.mavlink_connection(self.host, use_native=False)
		print("Connected to autopilot!")

		print("Waiting for heartbeat...")
		self.mav.wait_heartbeat(blocking=True)
		print("Got Heartbeat!")

	def set_point(self, lat, lon, alt):
		# Set new guided wp for ac
		self.set_lat = lat
		self.set_lon = lon
		self.set_alt = alt

		seq = 1
		frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
		radius = 10
		current = 2  # flag for guided mode
		wp = mavwp.MAVWPLoader()

		wp.add(mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
            master.target_component,
            seq,
            frame,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            current, 0, 0, radius, 0, 0,
            lat,lon,alt))
		self.mav.waypoint_clear_all_send()                                     
		self.mav.waypoint_count_send(wp.count())  

	def set_speed(self, speed):
		# set new speed of ac
		pass

	def update(self):
		msg = self.mav.recv_match(blocking=False)

		self.heartbeat_timeout = time.time() - self.heartbeat_time
		if self.heartbeat_timeout > 3:
			print("LOST HEARTBEAT!!!")
			self.heartbeat = False
			self.heartbeat_time = time.time()

		# Handle message
		if msg:
			if msg.get_type() == "HEARTBEAT":
				self.heartbeat_time = time.time()
				self.heartbeat = True
			if msg.get_type() == "GLOBAL_POSITION_INT":
				#print(msg.lat,msg.lon,msg.relative_alt)
				self.lat = msg.lat
				self.lon = msg.lon
				self.alt = msg.alt
			if msg.get_type() == "MISSION_REQUEST":
				print("GOT MISSION REQUEST")
				self.mav.mav.send(wp.wp(msg.seq))
			if msg.get_type() == "MISSION_ACK":
				print('MISSION_ACK %i' % msg.type)

def handle_input():
	global ac
	global gcs

	timeout = 0.01 #timeout for read from sdtin
	
	try:
		ready = select.select([sys.stdin], [], [], timeout)[0]

		if ready:
			for file in ready:
				line = file.readline()
				match = re.search(r'(\w+) (\w+) (\S+)', line)
				if match:
					if match.group(1) == 'set':
						if match.group(2) == 'minspeed':
							print("SET MINIMUM ALT TO {0}".format(match.group(3)))
						elif match.group(2) == 'maxspeed':	
							print("SET MAXIMUM ALT TO {0}".format(match.group(3)))
						elif match.group(2) == 'minalt':
							print("SET MINIMUM ALT TO {0}".format(match.group(3)))
						elif match.group(2) == 'maxalt':
							print("SET MAXIUM ALT TO {0}".format(match.group(3)))
						elif match.group(2) == 'point':
							print("SET POINT TO {0}".format(match.group(3)))
						elif match.group(2) == 'alt':
							print('SET ALT TO {0}'.format(match.group(3)))
						else:
							print("NOT A KNOWN COMMAND: {0}".format(match.group(2)))
					else:
						print("NOT A KNOWN COMMAND: {0}".format(match.group(1)))
				else:
					print('COMMAND NOT FORMATED PROPERLY')

	except:
		print("Couldn't read from stdin")
		exit(0)


# Hardcoded location (GPS device used if available)
loc = {'lat':33.6074232, 'lon':-102.0425716, 'alt':1004}
heading = 245

# Autopilot connection
host = "udpin:0.0.0.0:14550"
ac = Aircraft(host)
ac.connect()

# GPS dongle port
gps_port = '/dev/ttyUSB1'
gcs = GCS(gps_port)
#gcs.connect()

previous_speed = 0

print("Entering main loop...")
while sys.stdin:
	ac.update()
	gcs.update()
	handle_input()
	
	## Guided Way Point Update
	#Add a new guided waypoint if the ground vehicle has moved gcs.update_distance
	delta_x,delta_y = LatLon2Dist(gcs.lat_diff(),gcs.lon_diff(), gcs.lat)
	dist_moved = sqrt((delta_x*delta_x) + (delta_y*delta_y))
	if dist_moved > gcs.update_distance:
		#Calculate x and y distance from car based off of heading
		y = gcs.wp_distance * math.cos(math.radians(gcs.heading))
		x = gcs.wp_distance * math.sin(math.radians(gcs.heading))

		# Calculate lat/lon and set guided wp
		lat, lon = Dist2LatLon(gcs.lat,gcs.lon,x,y)
		ac.set_point(lat,lon,alt)

		# tag gcs location for next distance calculation
		gcs.lat_wp = gcs.lat
		gcs.lon_wp = gcs.lon


	## Speed controller
	# Get distance between car and ac. x east/west, y north/south
	ac_x,ac_y = LatLon2Dist(gcs.lat-ac.lat,gcs.lon-ac.lon, gcs.lat)
	# Calculate the position of ac relative to car
	alpha = gcs.heading #math.atan2(ay_y/ac_x)
	ac.rel_y = (-1)*x*math.sin(alpha) + y*math.cos(alpha)
	ac.rel_x = x*math.cos(alpha) + y*math.cos(alpha)
	# Calculate error
	error = ac.set_y - ac.rel_y
	# Calculate corrected speed
	p_offset = error * gain
	speed = gcs.speed + p_offset
	# Update speed if changed
	if speed != previous_speed:
		ac.set_speed(speed)
		previous_speed = speed