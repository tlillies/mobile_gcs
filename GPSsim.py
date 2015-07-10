#!/usr/bin/python

#Input minimum and maximum values for the time spent on a leg of travel and the speed of the ground station

min_leg_time_seconds = 5
max_leg_time_seconds = 10
min_speed_kt = 10
max_speed_kt = 20

import random
import time
import math
import pdb

#Input starting coordinates of the ground station

x = float(input("Enter Starting Longitude (Use positive values for E and negative values for W.):"))
y = float(input("Enter Starting Latitude (Use positive values for N and negative values for S.):"))

#Hardcode starting coordinates for testing (optional)

#x = -12345.1234
#y = 8634.1234

while 1:
    
    #Create a heading for the "reciever"
    
    h = int(random.randrange(0,3))
    t = random.randrange(min_leg_time_seconds, max_leg_time_seconds)

    #Convert kts into min/sec
    
    if h == 0 or h == 2:
        min_speed_min_per_sec = min_speed_kt / 3600
        max_speed_min_per_sec = max_speed_kt / 3600

    if h == 1 or h == 3:
        min_speed_min_per_sec = (min_speed_kt / 3600) / math.cos(math.radians(float(str(abs(y))[:2])))
        max_speed_min_per_sec = (max_speed_kt / 3600) / math.cos(math.radians(float(str(abs(y))[:2])))

    s = float(random.uniform(min_speed_min_per_sec, max_speed_min_per_sec))

    for counter in range(t):
        
        #Create change coordinates according to a random speed within the set parameters along the generated heading
        
        
        if h == 0:
            y = y + s
            c = 0.00
        elif h == 1:
            x = x + s
            c = 90.00
        elif h == 2:
            y = y - s
            c = 180.00
        else:
            x = x - s
            c = 270.00

        #Round coordinates to four decimal places to emulate decimal minute coordinates

        x = round(x, 4)
        y = round(y, 4)

        #Create the correct direction for the GPS output

        if x >= 0:
            str1 = 'E'
        else:
            str1 = 'W'

        if y >= 0:
            str2 = 'N'
        else:
            str2 = 'S'
                         
        #Convert coordinate change to knots for use in the GPS output
        #1 minute of latitude = 1 nautical mile
        #1 degree of longitude = (cos(latitude angle) * 60 nm) / 60 = nm
        
        if h == 0 or h == 2:
            kt = s * 3600
            kt = round(kt, 2)
                
        if h == 1 or h == 3:
            kt = s * 3600 * math.cos(math.radians(float(str(abs(y))[:2])))
            kt = round(kt, 2)
        
        #Print a sample line of GPS data

        print("$GPRMC", time.strftime("%I%M%S.000"), "A", abs(y), str2, abs(x), str1, kt, c, time.strftime("%d%m%y"), "", "*00", sep=',')

        #Send a new line every second

        time.sleep(1)


#while True:
#   x = x + random.uniform(0, .5)
#   y = y + random.uniform(0, .5)

#    x = round(x, 4)
#   y = round(y, 4)

#   print('New Longitude Is:', x )
#   print('New Latitude Is:', y )

#   time.sleep(1)

