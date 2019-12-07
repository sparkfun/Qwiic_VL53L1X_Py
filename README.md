Qwiic_VL53L1X_Py
==============

<p align="center">
   <img src="https://cdn.sparkfun.com/assets/custom_pages/2/7/2/qwiic-logo-registered.jpg"  width=200>  
   <img src="https://www.python.org/static/community_logos/python-logo-master-v3-TM.png"  width=240>   
</p>
<p align="center">
	<a href="https://pypi.org/project/sparkfun-qwiic-vl53l1x/" alt="Package">
		<img src="https://img.shields.io/pypi/pyversions/sparkfun_qwiic_vl53l1x.svg" /></a>
	<a href="https://github.com/sparkfun/Qwiic_VL53L1X_Py/issues" alt="Issues">
		<img src="https://img.shields.io/github/issues/sparkfun/Qwiic_VL53L1X_Py.svg" /></a>
	<a href="https://qwiic-VL53L1X-py.readthedocs.io/en/latest/?" alt="Documentation">
		<img src="https://readthedocs.org/projects/qwiic-vl53l1x-py/badge/?version=latest&style=flat" /></a>
	<a href="https://github.com/sparkfun/Qwiic_VL53L1X_Py/blob/master/LICENSE" alt="License">
		<img src="https://img.shields.io/badge/license-MIT-blue.svg" /></a>
	<a href="https://twitter.com/intent/follow?screen_name=sparkfun">
        	<img src="https://img.shields.io/twitter/follow/sparkfun.svg?style=social&logo=twitter"
           	 alt="follow on Twitter"></a>
	
</p>

<img src="https://cdn.sparkfun.com/assets/parts/1/2/9/4/8/14722-SparkFun_Distance_Sensor_Breakout-_4_Meter__VL53L1X__Qwiic_-01.jpg"  align="right" width=300 alt="SparkFun Servo pHAT for the Raspberry Pi">

Python module for the [SparkFun Distance Sensor Breakout - 4 Meter, VL53L1X (Qwiic)](https://www.sparkfun.com/products/14722).

This package should be used in conjunction with the overall [SparkFun qwiic Python Package](https://github.com/sparkfun/Qwiic_Py). New to qwiic? Take a look at the entire [SparkFun qwiic ecosystem](https://www.sparkfun.com/qwiic).

## Contents
* [Supported Platforms](#supported-platforms)
* [Dependencies](#dependencies)
* [Installation](#installation)
* [Documentation](#documentation)
* [Example Use](#example-use)

Supported Platforms
--------------------
The qwiic VL53L1X Python package current supports the following platforms:
* [Raspberry Pi](https://www.sparkfun.com/search/results?term=raspberry+pi)
<!-- Platforms to be tested
* [NVidia Jetson Nano](https://www.sparkfun.com/products/15297)
* [Google Coral Development Board](https://www.sparkfun.com/products/15318)
-->

Dependencies 
---------------
This package depends on the qwiic I2C driver: [Qwiic_I2C_Py](https://github.com/sparkfun/Qwiic_I2C_Py)

Documentation
-------------
The SparkFun qwiic VL53L1X module documentation is hosted at [ReadTheDocs](https://qwiic-VL53L1X-py.readthedocs.io/en/latest/?)

Installation
-------------

### PyPi Installation
This repository is hosted on PyPi as the [sparkfun-qwiic-vl53l1x](https://pypi.org/project/sparkfun-qwiic-vl53l1x/) package. On systems that support PyPi installation via pip, this library is installed using the following commands

For all users (note: the user must have sudo privileges):
```sh
sudo pip install sparkfun-qwiic-vl53l1x
```
For the current user:

```sh
pip install sparkfun-qwiic-vl53l1x
```

### Local Installation
To install, make sure the setuptools package is installed on the system.

Direct installation at the command line:
```sh
python setup.py install
```

To build a package for use with pip:
```sh
python setup.py sdist
 ```
A package file is built and placed in a subdirectory called dist. This package file can be installed using pip.
```sh
cd dist
pip install sparkfun_qwiic_vl53l1x-<version>.tar.gz
  
```
Example Use
 ---------------
See the examples directory for more detailed use examples.

```python
import qwiic_vl53l1x
import time
import math
import sys

def runExample():

	print("\nSparkFun VL53L1X Example 1\n")
	mySensor = qwiic_vl53l1x.QwiicVL53L1X()

	if mySensor.isConnected() == False:
		print("The Qwiic VL53L1X device isn't connected to the system. Please check your connection", \
			file=sys.stderr)
		return

	mySensor.SensorInit()
  
	while True:
        try:
            mySensor.StartRanging()						 # Write configuration bytes to initiate measurement
            time.sleep(.005)
            distance = mySensor.GetDistance()	 # Get the result of the measurement from the sensor
            time.sleep(.005)
            mySensor.StopRanging()

            print("Distance(mm): %s" % distance)

        except Exception as e:
            print(e)
```
<p align="center">
<img src="https://cdn.sparkfun.com/assets/custom_pages/3/3/4/dark-logo-red-flame.png" alt="SparkFun - Start Something">
</p>
