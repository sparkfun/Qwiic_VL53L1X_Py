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

from qwiic_VL53L1X import QwiicVL53L1X
import time statistics

print("VL53L1X Qwiic Test\n")
ToF = QwiicVL53L1X()

if (ToF.SensorInit() == None):					 # Begin returns 0 on a good init
	print("Sensor online!\n")

ToF.SetDistanceMode(1)	# Sets Distance Mode Short (Long- Change value to 2)

distance = [] # Initialize list

while True:
	start = time.time()

	try:
		ToF.StartRanging()									# Write configuration bytes to initiate measurement
		time.sleep(.005)
		distance.append(ToF.GetDistance())	# Get the result of the measurement from the sensor
		time.sleep(.005)
		ToF.StopRanging()

	except Exception as e:
		print(e)

	end = time.time()

	distanceInches = distance[len(distance)-1] / 25.4
	distanceFeet = distanceInches / 12.0
	
	if len(distance) < 10:
		avgdistance = statistics.mean(distance)
	else:
		distance.remove(distance[0])
		avgdistance = statistics.mean(distance[len(distance)-10:len(distance)+1]) # Running average of last 10 measurements
	
	signalrate = ToF.GetSignalRate()
	rangestatus = ToF.GetRangeStatus()
	
	#print("Distance(mm): %s avgDistance(mm): %s Distance(ft): %.3f Signal Rate: %s Range Status: %s Hz: %.5f" % (distance[len(distance)-1], avgdistance, distanceFeet, signalrate, rangestatus,(end-start)))
	print("Distance(mm): %s avgDistance(mm): %.2f Signal Rate: %s Range Status: %s Hz: %.5f" % (distance[len(distance)-1], avgdistance, signalrate, rangestatus,(end-start)))


	#if rangestatus == 0:
		#print("Good ")
	#elif rangestatus == 1:
		#print("Signal Fail ")
	#elif rangestatus == 2:
		#print("Sigma Fail ")
	#elif rangestatus == 7:
		#print("Wrapped Target Fail ")
	#else:
		#print("Unknown (code: %s) ", rangestatus)
