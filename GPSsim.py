
#Input minimum and maximum values for the time spent on a leg of travel and the speed of the ground station
#Set delay betwen recieving GPRMC sentences

import random
import time
import math
import pdb
import serial
from fastkml import kml, styles
from fastkml.geometry import Geometry, Point, LineString, Polygon

min_leg_time_seconds = 10
max_leg_time_seconds = 20

#Type .0 after the speeds in kts so that it is correctly defined as a float

min_speed_kt = 25.0
max_speed_kt = 40.0
delay = 1

def kml_write(x, y, c):

    #Put coordinates into an xml file

    ns = '{http://www.opengis.net/kml/2.2}'
    d = kml.Document(ns=ns, name='MOBILE GCS')
    k = kml.KML(ns=ns)

    p= kml.Placemark(ns, name='MOBILE GCS', styleUrl='sn_van')
    s = styles.Style(id='sn_van')
    IS = styles.IconStyle(scale=1.2, icon_href='http://maps.google.com/mapfiles/kml/shapes/truck.png', heading=(c - 90))
    s.append_style(IS)

    #AC Geometry

    geom = Geometry()
    geom.geometry = Point(x, y, 0)
    geom.altitude_mode = 'relativeToGround'
    p.geometry = geom
    d.append_style(s)
    d.append(p)

    #Write

    k.append(d)
    kmlfile = open('TOL_GCS.kml',"w")
    kmlfile.write(k.to_string(prettyprint=True))
    kmlfile.close()


#--------------------Main--------------------#


port = '/tmp/ttyV1'
print("Opening port {0}...".format(port))
output = serial.Serial(port=port, baudrate=4800)
print("{0} opened".format(port))

#Input starting coordinates of the ground station

x = float(input("Enter Starting Longitude (Use positive values for E and negative values for W.):"))
y = float(input("Enter Starting Latitude (Use positive values for N and negative values for S.):"))

#Hardcode starting coordinates for testing (optional)

#x = -105.00000000
#y = 40.00000000

while True:
    
    #Create a heading and time for the "reciever" during the leg
    
    h = int(random.randrange(0,1))
    t = random.randrange(min_leg_time_seconds, max_leg_time_seconds)

    #Convert kts into min/sec
    
    if h == 0 or h == 2:
        min_speed_deg_per_sec = (min_speed_kt / 3600 / 60)
        max_speed_deg_per_sec = (max_speed_kt / 3600 / 60)

    if h == 1 or h == 3:
        min_speed_deg_per_sec = (min_speed_kt / 3600 / 60) / math.cos(math.radians(abs(y)))
        max_speed_deg_per_sec = (max_speed_kt / 3600 / 60) / math.cos(math.radians(abs(y)))
    
       #Create a speed in min/sec for the leg

    s = random.uniform(min_speed_deg_per_sec, max_speed_deg_per_sec)

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

        #Create the correct direction for the GPS output

        if x >= 0:
            str1 = 'E,'
        else:
            str1 = 'W,'

        if y >= 0:
            str2 = 'N,'
        else:
            str2 = 'S,'
                         
        #Convert coordinate change to knots for use in the GPS output
        #1 minute of latitude = 1 nautical mile
        #1 degree of longitude = (cos(latitude angle) * 60 nm) / 60 = nm
        
        if h == 0 or h == 2:
            kt = s * 3600 * 60
            kt = round(kt, 2)
                
        if h == 1 or h == 3:
            kt = s * 3600 * 60 * math.cos(math.radians(abs(y)))
            kt = round(kt, 2)
        
        #Convert decimal degrees to NMEA format

        x_NM = math.modf(x)
        y_NM = math.modf(y)

        x_NM = (x_NM[0] * .6 + x_NM[1]) * 100
        y_NM = (y_NM[0] * .6 + y_NM[1]) * 100
        
        #Round coordinates to four decimal places to emulate decimal minute coordinates
        
        x_NM = round(x_NM, 4)
        y_NM = round(y_NM, 4)

        #Print a sample line of GPS data
        
        string = "$GPRMC," \
                + time.strftime("%I%M%S.000,") \
                + "A," \
                + "%.4f" %(abs(y_NM)) \
                + "," \
                + str2\
                + "%.4f" %(abs(x_NM,)) \
                + "," \
                + str1 \
                + "%.2f" %(kt) \
                + "," \
                + "%.2f" %(c)\
                +"," \
                + time.strftime("%d%m%y,") \
                + "," \
                + "*00"
        print(string)
        output.write(string)

        #Create a kml file for Google Earth

        kml_write(x, y, c)

        #Send a new line every second

        time.sleep(delay)


#while True:
#   x = x + random.uniform(0, .5)
#   y = y + random.uniform(0, .5)

#    x = round(x, 4)
#   y = round(y, 4)

#   print('New Longitude Is:', x )
#   print('New Latitude Is:', y )

#   time.sleep(1)

