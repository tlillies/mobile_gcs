#!/usr/bin/env python

#antenna-tracker.py
#Original written by Kevin Rauhauser
#Updates by Emily Ranquist and James Mack
#Research and Engineering Center for Unmanned Vehicles
#The purpose of this script is to point a 2 DOF servo actuated gimbal at an
#aircraft. The aircraft will feed the script altitude and lat/lon.

import pdb

import json
import math
import serial
import socket
import re
import select
import time
import pickle
from pymavlink import mavutil
from pymavlink import mavwp

# Hardcoded location (GPS device used if available)
loc = {'lat':33.6074232, 'lon':-102.0425716, 'alt':1004}
heading = 245

# Autopilot connection
host = "udpin:0.0.0.0:14551"

# Waypoint server info
wp_ip = "0.0.0.0"
wp_port = 14552

# Antenna servo controller port
ant_port = '/dev/ttyACM0'

# GPS dongle port
gps_port = '/dev/ttyUSB1'

# Minimum pointing distance
min_dist = 25

# Time between updates (seconds)
min_time = 0.25

# Set waypoint timeout
wp_timeout = 10

def main():
    # Initialize waypoint object and waypoint connection state
    wp = None
    wp_last = time.time()
    conn = None

    messages_queue = []
    sending_waypoints = False

    #open up the port to the maestro board
    ser = serial.Serial(port=ant_port, baudrate=9600)

    #open the port to the GPS receiver
    try:
        gps = serial.Serial(port=gps_port, baudrate=4800)
        #gps.close()
        #gps = None
    except:
        gps = None

    #open waypoint TCP server
    wpserv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    wpserv.setblocking(0)
    wpserv.bind((wp_ip, wp_port))
    wpserv.listen(1)

    #load gimbal calibration
    fd = open("cal.json",'r')
    cal = json.load(fd)
    fd.close()

    #Open connection to autopilot
    mav = mavutil.mavlink_connection(host, use_native=False)
    az_cmd = (cal['az_min']+cal['az_max'])/2
    el_cmd = cal['el_max']
    ser.write(b"\x84\x00")
    ser.write(("%c%c" % (int(el_cmd) & 0x7F, int(el_cmd) >> 7)).encode('utf-8'))
    ser.write(b"\x84\x01")
    ser.write(("%c%c" % (int(az_cmd) & 0x7F, int(az_cmd) >> 7)).encode('utf-8'))
    ac_close = False
    last_time = time.time()

    try:
        while True:
            #pdb.set_trace()
            if sending_waypoints and time.time() > (wp_last + wp_timeout):
		sending_waypoints = False
		print("Mission send timeout.")
            # Set up select to read the next available file
            readable, writable, exceptional = select.select([x for x in [mav.fd, gps, wpserv] if x is not None], [], [])
            if mav.fd in readable: # autopilot data available
                msg = mav.recv_match(blocking=True)
                if msg:
                    #print msg.get_type()print("Got MAV msg.")
                    # Operate on GPS lines (not GPS_RAW_INT)
                    if msg.get_type() == "GLOBAL_POSITION_INT" and time.time()>last_time+min_time:
                        last_time = time.time()
                        # Calculate bearing to plane
                        #print(msg)
                        x, y = LatLon2Dist(msg.lat/1e7 - loc['lat'], msg.lon/1e7 - loc['lon'], loc['lat'])
                        z = msg.alt/1e3 - loc['alt']
                        az = math.atan2(x, y)
                        el = math.atan2(z, (math.sqrt(x*x+y*y)))
                        if el < -10*math.pi/180:
                            el = -10*math.pi/180
                        # Point up if aircraft is very close
                        if ( x**2+y**2 < (0.9*min_dist)**2 and ac_close == False) :
                            ac_close = True
                        if ( x**2+y**2 > (1.1*min_dist)**2 and ac_close == True) :
                            ac_close = False
                        if ( ac_close ) :
                            #az = 180*math.pi/180
                            el = 90*math.pi/180
                        # Calculate servo commands
                        az_cmd = AzimuthNearest(az, az_cmd, cal)
                        el_cmd = cal['el_min']+(el/math.pi*2)*(cal['el_max']-cal['el_min'])
                        # Send position to gimbal
                        ser.write(b"\x84\x00")
                        ser.write(("%c%c" % (int(el_cmd) & 0x7F, int(el_cmd) >> 7)).encode('utf-8'))
                        ser.write(b"\x84\x01")
                        ser.write(("%c%c" % (int(az_cmd) & 0x7F, int(az_cmd) >> 7)).encode('utf-8'))
                        # Print debugging
                        #print (x,y)
                        #print az*180/math.pi
                        #print(time.time()-last_time)

                    if messages_queue and sending_waypoints == False:
                        sending_waypoints = True
			st = messages_queue[0][0].seq
			en = messages_queue[0][-1].seq + 1
			print("Sending waypoints %s thru %s" % (st, en))
			mav.mav.mission_write_partial_list_send(mav.target_system, mav.target_component, st, en)
                    if msg.get_type() == "MISSION_REQUEST" and sending_waypoints == True:
                            wp_last = time.time()
                            #print('Handling waypoints...')
                            mav.mav.send(messages_queue[0][msg.seq-messages_queue[0][0].seq])
                            print 'Sending waypoint {0}'.format(msg.seq)
                    if msg.get_type() == 'MISSION_ACK':
                        print 'MISSION_ACK %i' % msg.type
                        if msg.type == 0:
				sending_waypoints = False
                        	messages_queue.pop(0)


            if gps in readable: # GPS dongle data available
                a = gps.readline()
                b = a.split(',')
                if b[0] == "$GPGGA":
                    if b[6] is not '0': # check for valid GPS fix
                        # Assign GPS to present location, convert decimal minutes to decimal cegrees
                        loc['lat'] = float(b[2])/100
                        loc['lat'] = divmod(loc['lat'],1)[0]+divmod(loc['lat'],1)[1]*100/60
                        if b[3] == 'S' : loc['lat'] = -loc['lat']
                        loc['lon'] = float(b[4])/100
                        loc['lon'] = divmod(loc['lon'],1)[0]+divmod(loc['lon'],1)[1]*100/60
                        if b[5] == 'W' : loc['lon'] = -loc['lon']
                        loc['alt'] = float(b[9])
                        #print loc

            if wpserv in readable: # New waypoint client connection
                print "Got waypoint upload connection"
                conn, addr = wpserv.accept()
                data = ""
                while 1:
                    this_data = conn.recv(1024)
                    #print this_data
                    if not this_data: break
                    data += this_data
                conn.close()
                print 'Got Pickle...'
                wp_list = pickle.loads(data)
                messages_queue.append(wp_list)
                wp_last = time.time()
                print('Received {0} waypoints').format(len(messages_queue[0]))
                print('Start index: {0}').format(messages_queue[0][0].seq)
                print('End index: {0}').format(messages_queue[0][-1].seq + 1)

    except (KeyboardInterrupt):
        mav.close()
        ser.close()
        wpserv.close()
        if ( gps is not None ) : gps.close()

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

if __name__ == "__main__":
    main()
