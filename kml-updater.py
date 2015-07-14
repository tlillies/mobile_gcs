from fastkml import kml, styles
from fastkml.geometry import Geometry, Point, LineString, Polygon
import json
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

def kml_write(x, y, alt, c, type):


	ns = '{http://www.opengis.net/kml/2.2}'
	if type == "wp":
		p= kml.Placemark(ns, name='WP', styleUrl='wp')
		s = styles.Style(id='wp')
		IS = styles.IconStyle(scale=1.2, icon_href='https://maps.google.com/mapfiles/kml/shapes/capital_big_highlight.png', heading=(int(c) - 90))
		s.append_style(IS)
		d = kml.Document(ns=ns, name='WP')
	elif type == "ac":
		p= kml.Placemark(ns, name='AC', styleUrl='ac')
		s = styles.Style(id='ac')
		IS = styles.IconStyle(scale=1.2, icon_href='https://maps.google.com/mapfiles/kml/shapes/airports.png', heading=(int(c) - 90))
		s.append_style(IS)
		d = kml.Document(ns=ns, name='AC')
	elif type == "gcs":
		p= kml.Placemark(ns, name='MOBILE GCS', styleUrl='gcs')
		s = styles.Style(id='gcs')
		IS = styles.IconStyle(scale=1.2, icon_href='http://maps.google.com/mapfiles/kml/shapes/truck.png', heading=(int(c) - 90))
		s.append_style(IS)
		d = kml.Document(ns=ns, name='MOBILE GCS')
	else:
		return

	

	geom = Geometry()
	geom.geometry = Point(float(x), float(y), float(alt))
	geom.altitude_mode = 'relativeToGround'
	p.geometry = geom
	d.append_style(s)
	d.append(p)

	return d
	
ac = False
wp = False
gcs = False

while True:
	type = 0
	lat = 0
	lon = 0
	alt = 0
	heading = 0

	data, addr = sock.recvfrom(1024)
	print(data)

	#try:
	parsed_json = json.loads(data)
	if parsed_json['type']:
		type = parsed_json['type']
	if parsed_json['lat']:
		lat = parsed_json['lat']
	if parsed_json['lon']:
		lon = parsed_json['lon']
	if parsed_json['alt']:
		alt = parsed_json['alt']
	if parsed_json['heading']:
		heading = parsed_json['heading']
	d = kml_write(lat,lon,alt,heading,type)

	ns = '{http://www.opengis.net/kml/2.2}'
	k = kml.KML(ns=ns)
	k.append(d)

	if type == 'ac':
		ac = True
	if type == 'wp':
		wp = True
	if type == 'gcs':
		gcs = True

	if ac and wp and gcs:
		kmlfile = open('TEST.kml',"w")
		kmlfile.write(k.to_string(prettyprint=True))
		kmlfile.close()
		ac = False
		wp = False
		gcs = False
	#except: 
	#	pass