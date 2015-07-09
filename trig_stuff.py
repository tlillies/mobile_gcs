import math

degrees = 0

while degrees <= 360:
	y = 10 * math.cos(math.radians(degrees))
	x = 10 * math.sin(math.radians(degrees))

	print("Degrees: {0}".format(degrees))
	print("x: {0}".format(x))
	print("y: {0}".format(y))

	degrees += 30