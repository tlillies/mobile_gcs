from BaseHTTPServer import BaseHTTPRequestHandler
from BaseHTTPServer import HTTPServer
import threading
import json
import time
	
class ImageServerRequestHandler(BaseHTTPRequestHandler):
	
	def do_GET(self):
		print("GOT")
		request_path = self.path.split("/")[1:]
		# Aircraft and image system status
		if request_path[0] == "status":
			print("STATUS")
			message = self.server.get_status()
			
			self.send_response(200)
			self.send_header("Content-Type", "application/json")
			self.send_header("Access-Control-Allow-Origin", self.headers['Origin'])
			self.end_headers()
			self.wfile.write(json.dumps(message))
			return
		
		elif request_path[0] == "finish":
			self.server.finish()
			
			self.send_response(200)
			self.send_header("Content-Type", "text/plain")
			self.send_header("Access-Control-Allow-Origin", self.headers['Origin'])
			self.end_headers()
			self.wfile.write("OK")
			return

		elif request_path[0] == "set":
			print("got set!")
			print(request_path[1])
			print(request_path[2])
			print(float(request_path[2]))
			
			if request_path[1] == "x":
				self.server.set_x(float(request_path[2]))
			elif request_path[1] == "y":
				self.server.set_y(float(request_path[2]))
			elif request_path[1] == "wind":
				if request_path[2] == 'off':
					self.server.set_wind(False)
				else:
					self.server.set_wind_speed(float(request_path[2]))
					self.server.set_wind(True)
			elif request_path[1] == "wind_dir":
				if request_path[2] == 'off':
					self.server.set_wind(False)
				else:
					self.server.set_wind_dir(int(request_path[2]))
					self.server.set_wind(True)
			elif request_path[1] == "alt":
				self.server.set_alt_base(float(request_path[2]))
			elif request_path[1] == "alt_amplitude":
				self.server.set_alt_amp(float(request_path[2]))
			elif request_path[1] == "alt_period":
				self.server.set_alt_per(float(request_path[2]))
				self.server.set_alt_fre((2*math.pi) / float(request_path[2]))
			elif request_path[1] == "wp_dist":
				self.server.set_wp_distance(float(request_path[2]))
			elif request_path[1] == "rate":
				self.server.set_rate(float(request_path[2]))
			elif request_path[1] == "gain_f":
				self.server.set_gain_front(float(request_path[2]))
			elif request_path[1] == "gain_b":
				self.server.set_gain_back(float(request_path[2]))
			self.send_response(200)
			self.send_header("Content-Type", "text/plain")
			self.send_header("Access-Control-Allow-Origin", self.headers['Origin'])
			self.end_headers()
			self.wfile.write("OK")
			return
		self.send_error(404)

class ImageServer(HTTPServer):
	
	statusmsg = "Status unavailable"
	finish_mission = False
	status_lock = threading.Lock()

	def __init__(self, servaddr,ac,settings):
		HTTPServer.__init__(self, servaddr, ImageServerRequestHandler)
		self.ac = ac
		self.settings = settings
		self.telemetry = {}
		print 'Starting server, use <Ctrl-C> to stop'
		self.server_thread = threading.Thread(target=self.serve_forever)
		self.server_thread.start()
		
	def get_status(self):
		self.status_lock.acquire()
		retval = self.telemetry
		retval['status'] = self.statusmsg
		self.status_lock.release()
		return retval
	
	def set_status(self, msg):
		print(msg)
		self.status_lock.acquire()
		self.statusmsg = msg
		self.status_lock.release()
		return

	def set_telemetry(self,telemetry):
		self.status_lock.acquire()
		self.telemetry = telemetry
		self.status_lock.release()
		return

	def set_x(self,x):
		self.status_lock.acquire()
		self.ac.set_x(x)
		self.status_lock.release()
		return

	def set_y(self,y):
		self.status_lock.acquire()
		self.ac.set_y(y)
		self.status_lock.release()
		return

	# def set_alt(self,alt):
	# 	self.status_lock.acquire()
	# 	self.ac.set_alt(alt)
	# 	self.status_lock.release()
	# 	return

	def set_wind(self,set_wind):
		self.status_lock.acquire()
		self.ac.sw(set_wind)
		self.status_lock.release()
		return

	def set_wind_speed(self,set_wind):
		self.status_lock.acquire()
		self.ac.sw_speed(set_wind)
		self.status_lock.release()
		return

	def set_gain_front(self,gain):
		self.status_lock.acquire()
		self.settings['p_gain_front'] = gain
		self.status_lock.release()
		return

	def set_gain_back(self,gain):
		self.status_lock.acquire()
		self.settings['p_gain_back'] = gain
		self.status_lock.release()
		return

	def set_alt_base(self,alt):
		self.status_lock.acquire()
		self.settings['alt_base'] = alt
		self.status_lock.release()
		return

	def set_alt_amp(self,amplitutde):
		self.status_lock.acquire()
		self.settings['alt_amp'] = amplitutde
		self.status_lock.release()
		return

	def set_alt_per(self,period):
		self.status_lock.acquire()
		self.settings['alt_per'] = period
		self.status_lock.release()
		return

	def set_alt_fre(self,frequency):
		self.status_lock.acquire()
		self.settings['alt_fre'] = frequency
		self.status_lock.release()
		return

	def set_wp_distance(self,distance):
		self.status_lock.acquire()
		self.settings['wp_distance'] = distance
		self.status_lock.release()
		return

	def set_rate(self,rate):
		self.status_lock.acquire()
		self.settings['rate'] = rate
		self.status_lock.release()
		return
		
	def finish(self):
		self.finish_mission = True
		return
		
	def is_finished(self):
		return self.finish_mission

if __name__ == "__main__":
	server = ImageServer(servaddr)
	try:
		while True:
			time.sleep(1)
	except KeyboardInterrupt:
		server.shutdown()
