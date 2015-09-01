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
			self.end_headers()
			self.wfile.write(json.dumps(message))
			return
		
		if request_path[0] == "finish":
			self.server.finish()
			
			self.send_response(200)
			self.send_header("Content-Type", "text/plain")
			self.end_headers()
			self.wfile.write("OK")
			return

		if request_path[0] == "set":
			print("got set!")
			if request_path[1] == "x":
				self.server.set_x(int(request_path[2]))
			if request_path[1] == "y":
				self.server.set_y(int(request_path[2]))
			# if request_path[1] == "alt":
			# 	self.set_alt(int(request_path[2]))
			if request_path[1] == "wind":
				if request_path[2] == 'off':
					self.server.set_wind(False)
				else:
					self.server.set_wind_speed(int(request_path[2]))
					self.server.set_wind(True)
			if request_path[1] == "wind_dir":
				if request_path[2] == 'off':
					self.server.set_wind(False)
				else:
					self.server.set_wind_dir(int(request_path[2]))
					self.server.set_wind(True)
			self.send_response(200)
			self.send_header("Content-Type", "text/plain")
			self.end_headers()
			self.wfile.write("OK")
			return
		self.send_error(404)

class ImageServer(HTTPServer):
	
	statusmsg = "Status unavailable"
	finish_mission = False
	status_lock = threading.Lock()

	def __init__(self, servaddr,ac):
		HTTPServer.__init__(self, servaddr, ImageServerRequestHandler)
		self.ac = ac
		print 'Starting server, use <Ctrl-C> to stop'
		self.server_thread = threading.Thread(target=self.serve_forever)
		self.server_thread.start()
		
	def get_status(self):
		self.status_lock.acquire()
		retval = {}
		retval['status'] = self.statusmsg
		self.status_lock.release()
		return retval
	
	def set_status(self, msg):
		print(msg)
		self.status_lock.acquire()
		self.statusmsg = msg
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
		self.ac.set_wind(set_wind)
		self.status_lock.release()
		return

	def set_wind_speed(self,set_wind):
		self.status_lock.acquire()
		self.ac.set_wind(set_wind)
		self.status_lock.release()
		return

	def set_wind_dir(self,wind_dir):
		self.status_lock.acquire()
		self.ac.set_wind_dir(wind_dir)
		self.status_lock.release()
		return
		
	def finish(self):
		self.finish_mission = True
		
	def is_finished(self):
		return self.finish_mission

if __name__ == "__main__":
	server = ImageServer(servaddr)
	try:
		while True:
			time.sleep(1)
	except KeyboardInterrupt:
		server.shutdown()