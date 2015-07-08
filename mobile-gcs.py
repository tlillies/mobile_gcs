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
		self.lat = None
		self.lon = None
		self.alt = None
		self.port= port
		self.gps = None

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
		self.lat = None
		self.lon = None
		self.alt = None
		self.set_lat = None
		self.set_lon = None
		self.set_alt = None
		self.min_alt = None
		self.max_alt = None
		self.max_x = None
		self.max_y = None

	def connect(self):
		print("Connecting to autopilot...")
		self.mav = mavutil.mavlink_connection(self.host, use_native=False)
		print("Connected to autopilot!")

		print("Waiting for heartbeat...")
		self.mav.wait_heartbeat(blocking=True)
		print("Got Heartbeat!")

	def set(self, lat, lon, alt):
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

	def update(self):
		msg = self.mav.recv_match(blocking=False)
		self.heartbeat_timeout = time.time() - self.heartbeat_time
		if self.heartbeat_timeout > 3:
			print("LOST HEARTBEAT!!!")
			self.heartbeat_time = time.time()
		if msg:
			if msg.get_type() == "HEARTBEAT":
				self.heartbeat_timeout = 0
				self.heartbeat_time = time.time()
			if msg.get_type() == "GLOBAL_POSITION_INT":
				print(msg.lat,msg.lon,msg.relative_alt)
			if msg.get_type() == "MISSION_REQUEST":
				print("GOT MISSION REQUEST")
				self.mav.mav.send(wp.wp(msg.seq))
			if msg.get_type() == "MISSION_ACK":
				print('MISSION_ACK %i' % msg.type)


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

print("Entering main loop...")
while True:
	ac.update()
	gcs.update()