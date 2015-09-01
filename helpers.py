#helpers.py
#Research and Engineering Center for Unmanned Vehicles
import math
import socket
import sys
import re

def latlon_to_dist(lat_diff, lon_diff, lat_ref):
    #latlon_to_dist converts differences in lat/lon position to cartesian
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

def dist_to_latlon(lat,lon,dn,de):
	# William's aviation
	# dn -- distance north
	# de -- distance east
	re = 6378137

	dLat = dn/re
	dLon = de/(re*math.cos(math.pi*lat/180))

	lat0 = lat + dLat * 180/math.pi
	lon0 = lon + dLon * 180/math.pi

	return lat0, lon0


def handle_input(debug,ac,gcs,readable):
	try:
		if sys.stdin in readable:
			line = sys.stdin.readline()
			match = re.search(r'(\w+) (\w+) (\S+)', line)
			if match:
				if match.group(1) == 'debug':
					if match.group(2) == 'speed':
						if match.group(3) == 'on':
							print("DEBUG SPEED SET ON")
							debug['debug_speed'] = True
						else:
							print("DEBUG SPEED SET OFF")
							debug['debug_speed'] = False
					if match.group(2) == 'mission':
						if match.group(3) == 'on':
							print("DEBUG MISSION SET ON")
							debug['debug_mission'] = True
						else:
							print("DEBUG MISSION SET OFF")
							debug['debug_mission'] = False
					if match.group(2) == 'general':
						if match.group(3) == 'on':
							print("DEBUG GENERAL SET ON")
							debug['debug'] = True
						else:
							print("DEBUG GENERAL SET OFF")
							debug['debug'] = False
					if match.group(2) == 'position':
						if match.group(3) == 'on':
							print("DEBUG POSITION SET ON")
							debug['debug_position'] = True
						else:
							print("DEBUG POSITION SET OFF")
							debug['debug_position'] = False
					if match.group(2) == 'distance':
						if match.group(3) == 'on':
							print("DEBUG DISTANCE SET ON")
							debug['debug_dist'] = True
						else:
							print("DEBUG DISTANCE SET OFF")
							debug['debug_dist'] = False
					if match.group(2) == 'alt':
						if match.group(3) == 'on':
							print("DEBUG ALT SET ON")
							debug['debug_alt'] = True
						else:
							print("DEBUG ALT SET OFF")
							debug['debug_alt'] = False
					if match.group(2) == 'wind':
						if match.group(3) == 'on':
							print("DEBUG WIND SET ON")
							debug['debug_wind'] = True
						else:
							print("DEBUG WIND SET OFF")
							debug['debug_wind'] = False

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

def udp_json_output(gcs, ac, ip, port):
	# Send out JSON data for kml updater
	ac_msg = '{"type":"ac","lat":' + str(ac.lat) + ',"lon":' + str(ac.lon) + ',"alt":' + str(ac.relative_alt) + ',"heading":' + str(ac.heading) + '}'
	wp_msg = '{"type":"wp","lat":' + str(ac.wp_lat) + ',"lon":' + str(ac.wp_lon) + ',"alt":' + str(ac.wp_alt) + ',"heading":' + str(0) + '}'
	set_msg = '{"type":"set","lat":' + str(ac.set_lat) + ',"lon":' + str(ac.set_lon) + ',"alt":' + str(ac.set_alt) + ',"heading":' + str(0) + '}'
	gcs_msg = '{"type":"gcs","lat":' + str(gcs.lat) + ',"lon":' + str(gcs.lon) + ',"alt":' + str(0.0) + ',"heading":' + str(gcs.heading) +'}'

	sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

	sock.sendto(ac_msg, (ip, port))
	sock.sendto(wp_msg, (ip, port))
	sock.sendto(set_msg, (ip, port))
	sock.sendto(gcs_msg, (ip, port))