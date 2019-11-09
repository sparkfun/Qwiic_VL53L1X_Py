"""
	Reading distance from the laser based VL53L1X
	By: Nathan Seidle
	Ported: Wes Furuya
	SparkFun Electronics
	Date: October 31, 2019
	License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

	SparkFun labored with love to create this code. Feel like supporting open source hardware?
	Buy a board from SparkFun! https://www.sparkfun.com/products/14667

	This example prints the distance to an object.

	Are you getting weird readings? Be sure the vacuum tape has been removed from the sensor.
"""

from qwiic_VL53L1X_v0012 import VL53L1X

print("VL53L1X Qwiic Test\n")
ToF = VL53L1X()

if (ToF.SensorInit() == 0):					 # Begin returns 0 on a good init
	print("Sensor online!\n")

ToF.SetDistanceMode(1)	# Sets Distance Mode Short (Long- Change value to 2)

while True:
	try:
		ToF.StartRanging()						 # Write configuration bytes to initiate measurement
		time.sleep(.005)
		distance = ToF.GetDistance()	 # Get the result of the measurement from the sensor
		time.sleep(.005)
		ToF.StopRanging()

		distanceInches = distance / 25.4
		distanceFeet = distanceInches / 12.0

		print("Distance(mm): %s Distance(ft): %s" % (distance, distanceFeet))

	except Exception as e:
		print(e)
