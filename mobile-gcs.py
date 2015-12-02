#mobile-gcs.py
#Research and Engineering Center for Unmanned Vehicles

import select
import time
import math
import sys

import Aircraft
import GCS
import Server
import helpers


## Connection settings ##

# KML
UDP_IP = "127.0.0.1"
#UDP_IP = "192.168.8.223"
UDP_PORT = 5005

# Autopilot connection
ac_host = "udpin:0.0.0.0:14550"
#ac_host = '/dev/ttyUSB0'

# GPS dongle port
gps_port = '/tmp/ttyV0'
#gps_port = '/dev/ttyUSB0' 

## Gains ##

P_GAIN_FRONT = .001  # When the ac is in front of the car
P_GAIN_BACK = .0005  # When the ac is behind car

## Flight settings ##

ALT_BASE = 200
ALT_AMP = 0
ALT_PER = 240
ALT_FRE = (2*math.pi) / ALT_PER

WP_DISTANCE = 200 # Guided waypoint set distance in front of car

AIRSPEED_MAX = 30
AIRSPEED_MIN = 14

UPDATE_RATE = .2 # rate to send waypoints/speed in seconds

PRINT_RATE = 1 # rate to print debug options in seconds




## Setup
debug = {'debug_gen': False,
		 'debug_speed': False,
		 'debug_dist': False,
		 'debug_mission': False,
		 'debug_position':False,
		 'debug_alt': False,
		 'debug_wind': False }

# Telemetry dictionaries 
telemetry = {}
ac_tel = {}
gcs_tel = {}

settings = {}

# Aircraft
ac = Aircraft.Aircraft(ac_host)
if not ac.connect():
	print("Unable to connect to aircraft! Exiting!")
	exit(0)

gcs = GCS.GroundControlStation(gps_port)
if not gcs.connect():
	print("Unable to connect to GPS! Exiting!")
	exit(0)

# Ground Vehicle
gcs.set_wp_dist(WP_DISTANCE)

# Web Server
servaddr = ('0.0.0.0', 9000)
server = Server.ImageServer(servaddr, ac,settings)
server.set_status("Setting up.")


settings['p_gain_front'] = P_GAIN_FRONT
settings['p_gain_back'] = P_GAIN_BACK
settings['alt_base'] = ALT_BASE
settings['alt_amp'] = ALT_AMP
settings['alt_per'] = ALT_PER
settings['alt_fre'] = ALT_FRE
settings['wp_distance'] = WP_DISTANCE
settings['rate'] = UPDATE_RATE
settings['alt_base'] = ALT_BASE

print_timer = time.time()
print_timer_last = time.time()
rate_timer = time.time()
rate_timer_last = time.time()

# just as safety set safe initial values
speed = AIRSPEED_MIN + 2 
ac.set_point(gcs.lat,gcs.lon,ALT_BASE)
lat = gcs.lat
lon = gcs.lon
server.set_status("Entering main loop...")
print("Entering main loop...")
try:
	while sys.stdin:
		readable, writable, exceptional = select.select([x for x in [ac.mav.fd, gcs.gps, sys.stdin] if x is not None], [], [])


		## Update devices
		ac.update(readable, debug)
		gcs.update(readable)
		helpers.handle_input(debug,ac,gcs,readable)


		## Calculate lat lon of set point
		alpha = math.radians(gcs.heading * -1)
		dist_x = ac.x_offset*math.cos(alpha) - ac.y_offset*math.sin(alpha)
		dist_y = ac.x_offset*math.sin(alpha) + ac.y_offset*math.cos(alpha)
		ac.set_lat, ac.set_lon = helpers.dist_to_latlon(gcs.lat,gcs.lon,dist_y,dist_x)


		## Calculate alt to set
		ac.set_alt = settings['alt_base'] + settings['alt_amp'] * math.sin(settings['alt_fre'] *time.time())


		## Speed controller
		# Get distance between car and ac. x east/west, y north/south
		ac_x,ac_y = helpers.latlon_to_dist(ac.set_lat-ac.lat,ac.set_lon-ac.lon, gcs.lat)
		ac_y *= -1
		ac_x *= -1
		# Calculate the position of ac relative to car
		alpha = math.radians(gcs.heading * (-1))
		ac.rel_x = ac_x*math.cos(alpha) + ac_y*math.sin(alpha)
		ac.rel_y = (-1)*ac_x*math.sin(alpha) + ac_y*math.cos(alpha)
		# Calculate error
		error = ac.rel_y - ac.y_offset
		# Calculate corrected speed
		p_offset = error * error
		if error < 0:
			p_offset *= settings['p_gain_front']
		else:
			p_offset *= settings['p_gain_back'] *-1

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
		rate_timer = time.time()
		if (rate_timer - rate_timer_last) > settings['rate']:
			rate_timer_last = time.time()
			
			# Calculate next waypoint
			y = gcs.wp_distance * math.cos(math.radians(gcs.heading))
			x = gcs.wp_distance * math.sin(math.radians(gcs.heading))

			# Send new waypoint
			lat, lon = helpers.dist_to_latlon(ac.set_lat,ac.set_lon,y,x)
			ac.set_point(lat,lon,ac.set_alt)
			ac.set_rally(gcs)

			# Send Speed
			ac.set_speed(speed)


		## Print out debug info and send json to kml updater
		print_timer = time.time()
		if (print_timer - print_timer_last) > PRINT_RATE:
			print_timer_last = time.time()
			# Print out data
			if debug['debug_position']:
				print("Aircraft-- Lat: {0:14}, Lon: {1:14}, Heading: {2:6}".format(ac.lat,ac.lon,ac.heading))
				print("Ground  -- Lat: {0:14}, Lon: {1:14}, Heading: {2:6}".format(gcs.lat,gcs.lon,gcs.heading))
			if debug['debug_alt']:
				print("Set Alt: {0}, Base Alt: {1}, Alt Amp: {2}, Alt Freq: {3}".format(ac.set_alt,settings['alt_base'],settings['alt_amp'],settings['alt_fre']))
			if debug['debug_dist']:
				print("dela_x:{0} dela_y:{1}".format(delta_x,delta_y))
				print("x:{0} y:{1}".format(x,y))
			if debug['debug_gen']:
				print("CMDSpeed:{0} GCSSpeed:{1} Pf:{2} Error:{4}".format(speed,gcs.speed,p_offset,error))
			if debug['debug_speed']:
				print("Current Set Speed: {0}".format(ac.got_speed))
				print("Controller: {0}, Wind Adjust: {1}".format(speed_temp,speed))
				print("Current GCS Speed: {0}".format(gcs.speed))
				print("GCS Speed_x: {0}, GCS Speed_y: {1}".format(speed_x,speed_y))
				print("AC_speed_x: {0}, AC_speed_y: {1}".format(ac_speed_x,ac_speed_y))
			if debug['debug_wind']:
				print("Wind Speed: {0}, Wind Direction: {1}".format(wind_speed,wind_direction))
				print("Wind_x: {0}, Wind_y: {1}".format(wind_x,wind_y))
				print("Set Wind: {0}".format(ac.set_wind))
				print("Autopilot Wind: {0}, Autopilot Direction: {1}".format(ac.wind_speed, ac.wind_direction))
				print("Set Wind Speed: {0}, Set Wind Direction: {1}".format(ac.set_wind_speed, ac.set_wind_direction))
				print("Send Speed: {0}".format(speed))

			helpers.udp_json_output(gcs,ac,UDP_IP,UDP_PORT)

			ac_tel['lat'] = '{:13.7f}'.format(ac.lat)
			ac_tel['lon'] = '{:13.7f}'.format(ac.lon)
			ac_tel['heading'] = '{:7.3f}'.format(ac.heading)
			ac_tel['airspeed'] = '{:7.3f}'.format(ac.airspeed)
			ac_tel['groundspeed'] = '{:7.3f}'.format(ac.groundspeed)
			ac_tel['setspeed'] = '{:7.3f}'.format(speed)
			ac_tel['nowinspeed'] = '{:7.3f}'.format(speed_temp)
			telemetry['wind'] = '{:7.3f}'.format(ac.wind_speed)
			telemetry['wind_dir'] = '{:7.3f}'.format(ac.wind_direction)
			if ac.set_wind:
				telemetry['setwind'] = '{:7.3f}'.format(ac.set_wind_speed)
				telemetry['setwind_dir'] = '{:7.3f}'.format(ac.set_wind_direction)
				telemetry['wind'] = '{:7.3f}'.format(ac.set_wind_speed)
				telemetry['wind_dir'] = '{:7.3f}'.format(ac.set_wind_direction)
			else:
				telemetry['setwind'] = 'None'
			ac_tel['alt'] = '{:7.3f}'.format(ac.relative_alt)
			ac_tel['setalt'] = '{:7.3f}'.format(ac.set_alt)
			ac_tel['setx'] = '{0}'.format(ac.x_offset)
			ac_tel['sety'] = '{0}'.format(ac.y_offset)
			gcs_tel['lat'] = '{:13.7f}'.format(gcs.lat)
			gcs_tel['lon'] = '{:13.7f}'.format(gcs.lon)
			gcs_tel['heading'] = '{:7.3f}'.format(gcs.heading)
			gcs_tel['speed'] = '{:7.3f}'.format(gcs.speed)

			telemetry['rate'] = '{:7.3f}'.format(settings['rate'])
			telemetry['gain_f'] = '{:7.7f}'.format(settings['p_gain_front'])
			telemetry['gain_b'] = '{:7.7f}'.format(settings['p_gain_back'])
			telemetry['altbase'] = '{:7.3f}'.format(settings['alt_base'])
			telemetry['altamp'] = '{:7.3f}'.format(settings['alt_amp'])
			telemetry['altper'] = '{:7.3f}'.format(settings['alt_per'])
			telemetry['wp_dist'] = '{:7.3f}'.format(settings['wp_distance'])

			telemetry['ac'] = ac_tel
			telemetry['gcs'] = gcs_tel

			server.set_status("Running")
			server.set_telemetry(telemetry)

		# End if web server is finished
		if server.is_finished():
			break
except KeyboardInterrupt:
	server.shutdown()
except Exception as ex:
	print(ex)
	server.shutdown()
	#server.set_status(str(ex))
			
