from fastkml import kml, styles
from fastkml.geometry import Geometry, Point, LineString, Polygon
import json
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

def make_kml(x, y, alt, c, type):


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
		IS = styles.IconStyle(scale=1.2, icon_href='https://maps.google.com/mapfiles/kml/shapes/airports.png', heading=(int(c)))
		s.append_style(IS)
		d = kml.Document(ns=ns, name='AC')
	elif type == "gcs":
		p= kml.Placemark(ns, name='MOBILE GCS', styleUrl='gcs')
		s = styles.Style(id='gcs')
		IS = styles.IconStyle(scale=1.2, icon_href='http://maps.google.com/mapfiles/kml/shapes/truck.png', heading=(int(c) - 90))
		s.append_style(IS)
		d = kml.Document(ns=ns, name='MOBILE GCS')
	elif type == "set":
		p= kml.Placemark(ns, name='SET POINT', styleUrl='set_point')
		s = styles.Style(id='set_point')
		IS = styles.IconStyle(scale=1.2, icon_href='https://maps.google.com/mapfiles/kml/pal4/icon58.png', heading=(int(c) - 90))
		s.append_style(IS)
		d = kml.Document(ns=ns, name='SET POINT')
	else:
		return

	

	geom = Geometry()
	geom.geometry = Point(float(y), float(x), float(alt))
	geom.altitude_mode = 'relativeToGround'
	p.geometry = geom
	d.append_style(s)
	d.append(p)

	return d

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

	d = make_kml(lat,lon,alt,heading,type)

	ns = '{http://www.opengis.net/kml/2.2}'

	if type == 'ac':
		ac_k = kml.KML(ns=ns)
		ac_k.append(d)
		kmlfile = open('AC.kml',"w")
		kmlfile.write(ac_k.to_string(prettyprint=True))
		kmlfile.close()
	if type == 'wp':
		wp_k = kml.KML(ns=ns)
		wp_k.append(d)
		kmlfile = open('WP.kml',"w")
		kmlfile.write(wp_k.to_string(prettyprint=True))
		kmlfile.close()
	if type == 'gcs':
		gcs_k = kml.KML(ns=ns)
		gcs_k.append(d)
		kmlfile = open('GCS.kml',"w")
		kmlfile.write(gcs_k.to_string(prettyprint=True))
		kmlfile.close()
	if type == 'set':
		set_k = kml.KML(ns=ns)
		set_k.append(d)
		kmlfile = open('SET.kml',"w")
		kmlfile.write(set_k.to_string(prettyprint=True))
		kmlfile.close()