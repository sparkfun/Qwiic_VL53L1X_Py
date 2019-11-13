"""
********************************************************************************
* @file		vl53l0x_class.cpp
* @author	IMG
* @version	V0.0.1
* @date		14-December-2018
* @brief	Implementation file for the VL53L1X driver class
********************************************************************************
* @attention
* <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics International N.V. All rights reserved.</center></h2>
********************************************************************************
*
* Copyright (c) 2017, STMicroelectronics - All Rights Reserved
*
* This file is part of VL53L1 Core and is dual licensed,
* either 'STMicroelectronics
* Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
********************************************************************************
*
* Alternatively, VL53L1 Core may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************
"""

# Load Necessary Modules:
#----------------------------------------------
import time							# Time access and conversion package
import math							# Basic math package
import qwiic_i2c					# I2C bus driver package
# from smbus2 import SMBus, i2c_msg	# I2C bus driver package

# From vL53l1x_class.h Header File
###############################################################################
###############################################################################

# if _MSC_VER != None:
# 	if VL53L1X_API_EXPORTS != None:
# 		VL53L1X_API = __declspec(dllexport)
# 	else:
# 		VL53L1X_API
# else:
# 	VL53L1X_API

SOFT_RESET =															0x0000
VL53L1_I2C_SLAVE__DEVICE_ADDRESS =										0x0001
VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND =							0x0008
ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS =						0x0016
ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS =					0x0018
ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS =					0x001A
ALGO__PART_TO_PART_RANGE_OFFSET_MM =									0x001E
MM_CONFIG__INNER_OFFSET_MM =											0x0020
MM_CONFIG__OUTER_OFFSET_MM =											0x0022
GPIO_HV_MUX__CTRL =														0x0030
GPIO__TIO_HV_STATUS =													0x0031
SYSTEM__INTERRUPT_CONFIG_GPIO =											0x0046
PHASECAL_CONFIG__TIMEOUT_MACROP =										0x004B
RANGE_CONFIG__TIMEOUT_MACROP_A_HI =										0x005E
RANGE_CONFIG__VCSEL_PERIOD_A =											0x0060
RANGE_CONFIG__VCSEL_PERIOD_B =											0x0063
RANGE_CONFIG__TIMEOUT_MACROP_B_HI =										0x0061
RANGE_CONFIG__TIMEOUT_MACROP_B_LO =										0x0062
RANGE_CONFIG__SIGMA_THRESH =											0x0064
RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS =							0x0066
RANGE_CONFIG__VALID_PHASE_HIGH =										0x0069
VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD =								0x006C
SYSTEM__THRESH_HIGH =													0x0072
SYSTEM__THRESH_LOW =													0x0074
SD_CONFIG__WOI_SD0 =													0x0078
SD_CONFIG__INITIAL_PHASE_SD0 =											0x007A
ROI_CONFIG__USER_ROI_CENTRE_SPAD =										0x007F
ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE =							0x0080
SYSTEM__SEQUENCE_CONFIG =												0x0081
VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD =									0x0082
SYSTEM__INTERRUPT_CLEAR =												0x0086
SYSTEM__MODE_START =													0x0087
VL53L1_RESULT__RANGE_STATUS =											0x0089
VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0 =							0x008C
RESULT__AMBIENT_COUNT_RATE_MCPS_SD =									0x0090
VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 =					0x0096
VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 =	0x0098
VL53L1_RESULT__OSC_CALIBRATE_VAL =										0x00DE
VL53L1_FIRMWARE__SYSTEM_STATUS =										0x00E5
VL53L1_IDENTIFICATION__MODEL_ID =										0x010F
VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD =								0x013E

_VL53L1X_DEFAULT_DEVICE_ADDRESS =										0x52
###############################################################################
###############################################################################

_DEFAULT_NAME = "Qwiic 4m Distance Sensor (ToF)"

###############################################################################
###############################################################################
_FULL_ADDRESS_LIST = list(range(0x08,0x77+1))					# Full I2C Address List (excluding resrved addresses)
_FULL_ADDRESS_LIST.remove(_VL53L1X_DEFAULT_DEVICE_ADDRESS >> 1) # Remove Default Address of VL53L1X from list
_AVAILABLE_I2C_ADDRESS = [_VL53L1X_DEFAULT_DEVICE_ADDRESS >> 1]	# Initialize with Default Address of VL53L1X
_AVAILABLE_I2C_ADDRESS.extend(_FULL_ADDRESS_LIST)				# Add Full Range of I2C Addresses


# From vL53l1x_class.cpp C++ File
###############################################################################
###############################################################################
ALGO__PART_TO_PART_RANGE_OFFSET_MM =									0x001E
MM_CONFIG__INNER_OFFSET_MM =											0x0020
MM_CONFIG__OUTER_OFFSET_MM = 											0x0022

# DEBUG_MODE

VL51L1X_DEFAULT_CONFIGURATION = [
0x00,	# 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch
0x00,	# 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
0x00,	# 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
0x01,	# 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity()
0x02,	# 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady()
0x00,	# 0x32 : not user-modifiable
0x02,	# 0x33 : not user-modifiable
0x08,	# 0x34 : not user-modifiable
0x00,	# 0x35 : not user-modifiable
0x08,	# 0x36 : not user-modifiable
0x10,	# 0x37 : not user-modifiable
0x01,	# 0x38 : not user-modifiable
0x01,	# 0x39 : not user-modifiable
0x00,	# 0x3a : not user-modifiable
0x00,	# 0x3b : not user-modifiable
0x00,	# 0x3c : not user-modifiable
0x00,	# 0x3d : not user-modifiable
0xff,	# 0x3e : not user-modifiable
0x00,	# 0x3f : not user-modifiable
0x0F,	# 0x40 : not user-modifiable
0x00,	# 0x41 : not user-modifiable
0x00,	# 0x42 : not user-modifiable
0x00,	# 0x43 : not user-modifiable
0x00,	# 0x44 : not user-modifiable
0x00,	# 0x45 : not user-modifiable
0x20,	# 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC
0x0b,	# 0x47 : not user-modifiable
0x00,	# 0x48 : not user-modifiable
0x00,	# 0x49 : not user-modifiable
0x02,	# 0x4a : not user-modifiable
0x0a,	# 0x4b : not user-modifiable
0x21,	# 0x4c : not user-modifiable
0x00,	# 0x4d : not user-modifiable
0x00,	# 0x4e : not user-modifiable
0x05,	# 0x4f : not user-modifiable
0x00,	# 0x50 : not user-modifiable
0x00,	# 0x51 : not user-modifiable
0x00,	# 0x52 : not user-modifiable
0x00,	# 0x53 : not user-modifiable
0xc8,	# 0x54 : not user-modifiable
0x00,	# 0x55 : not user-modifiable
0x00,	# 0x56 : not user-modifiable
0x38,	# 0x57 : not user-modifiable
0xff,	# 0x58 : not user-modifiable
0x01,	# 0x59 : not user-modifiable
0x00,	# 0x5a : not user-modifiable
0x08,	# 0x5b : not user-modifiable
0x00,	# 0x5c : not user-modifiable
0x00,	# 0x5d : not user-modifiable
0x01,	# 0x5e : not user-modifiable
0xdb,	# 0x5f : not user-modifiable
0x0f,	# 0x60 : not user-modifiable
0x01,	# 0x61 : not user-modifiable
0xf1,	# 0x62 : not user-modifiable
0x0d,	# 0x63 : not user-modifiable
0x01,	# 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm 
0x68,	# 0x65 : Sigma threshold LSB
0x00,	# 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold()
0x80,	# 0x67 : Min count Rate LSB
0x08,	# 0x68 : not user-modifiable
0xb8,	# 0x69 : not user-modifiable
0x00,	# 0x6a : not user-modifiable
0x00,	# 0x6b : not user-modifiable
0x00,	# 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs()
0x00,	# 0x6d : Intermeasurement period
0x0f,	# 0x6e : Intermeasurement period
0x89,	# 0x6f : Intermeasurement period LSB
0x00,	# 0x70 : not user-modifiable
0x00,	# 0x71 : not user-modifiable
0x00,	# 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold()
0x00,	# 0x73 : distance threshold high LSB
0x00,	# 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold()
0x00,	# 0x75 : distance threshold low LSB
0x00,	# 0x76 : not user-modifiable
0x01,	# 0x77 : not user-modifiable
0x0f,	# 0x78 : not user-modifiable
0x0d,	# 0x79 : not user-modifiable
0x0e,	# 0x7a : not user-modifiable
0x0e,	# 0x7b : not user-modifiable
0x00,	# 0x7c : not user-modifiable
0x00,	# 0x7d : not user-modifiable
0x02,	# 0x7e : not user-modifiable
0xc7,	# 0x7f : ROI center, use SetROI()
0xff,	# 0x80 : XY ROI (X=Width, Y=Height), use SetROI()
0x9B,	# 0x81 : not user-modifiable
0x00,	# 0x82 : not user-modifiable
0x00,	# 0x83 : not user-modifiable
0x00,	# 0x84 : not user-modifiable
0x01,	# 0x85 : not user-modifiable
0x00,	# 0x86 : clear interrupt, use ClearInterrupt()
0x00	# 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after self.init() call, put 0x40 in location 0x87
]
###############################################################################
###############################################################################


# From vL53l1_error_codes.h Header File
###############################################################################
###############################################################################
"""
* @file vl53l1_error_codes.h
* @brief Error Code definitions for VL53L1 API.
*
***************************************
* PRIVATE define do not edit
***************************************
"""

"""
* @defgroup VL53L1_define_Error_group Error and Warning code returned by API
* The following DEFINE are used to identify the PAL ERROR
* @{
"""
VL53L1_ERROR_NONE =															  0
VL53L1_ERROR_CALIBRATION_WARNING =											 -1
"""!< Warning invalid calibration data may be in used
	\a VL53L1_InitData()
	\a VL53L1_GetOffsetCalibrationData
	\a VL53L1_SetOffsetCalibrationData"""
VL53L1_ERROR_MIN_CLIPPED =													 -2
"""!< Warning parameter passed was clipped to min before to be applied"""

VL53L1_ERROR_UNDEFINED =													 -3
"""!< Unqualified error"""
VL53L1_ERROR_INVALID_PARAMS =												 -4
"""!< Parameter passed is invalid or out of range"""
VL53L1_ERROR_NOT_SUPPORTED =												 -5
"""!< Function is not supported in current mode or configuration"""
VL53L1_ERROR_RANGE_ERROR =													 -6
"""!< Device report a ranging error interrupt status"""
VL53L1_ERROR_TIME_OUT =														 -7
"""!< Aborted due to time out"""
VL53L1_ERROR_MODE_NOT_SUPPORTED =											 -8
"""!< Asked mode is not supported by the device"""
VL53L1_ERROR_BUFFER_TOO_SMALL =												 -9
"""!< ..."""
VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL =										-10
"""!< Supplied buffer is larger than I2C supports"""
VL53L1_ERROR_GPIO_NOT_EXISTING =											-11
"""!< User tried to setup a non-existing GPIO pin"""
VL53L1_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED =								-12
"""!< unsupported GPIO functionality"""
VL53L1_ERROR_CONTROL_INTERFACE =											-13
"""!< error reported from IO functions"""
VL53L1_ERROR_INVALID_COMMAND =												-14
"""!< The command is not allowed in the current device state
* (power down"""
VL53L1_ERROR_DIVISION_BY_ZERO =												-15
"""!< In the function a division by zero occurs"""
VL53L1_ERROR_REF_SPAD_INIT =												-16
"""!< Error during reference SPAD initialization"""
VL53L1_ERROR_GPH_SYNC_CHECK_FAIL =											-17
"""!< GPH sync interrupt check fail - API out of sync with device"""
VL53L1_ERROR_STREAM_COUNT_CHECK_FAIL =										-18
"""!< Stream count check fail - API out of sync with device"""
VL53L1_ERROR_GPH_ID_CHECK_FAIL =											-19
"""!< GPH ID check fail - API out of sync with device"""
VL53L1_ERROR_ZONE_STREAM_COUNT_CHECK_FAIL =									-20
"""!< Zone dynamic config stream count check failed - API out of sync"""
VL53L1_ERROR_ZONE_GPH_ID_CHECK_FAIL =										-21
"""!< Zone dynamic config GPH ID check failed - API out of sync"""

VL53L1_ERROR_XTALK_EXTRACTION_NO_SAMPLE_FAI =								-22
"""!< Thrown when run_xtalk_extraction fn has 0 succesful samples
* when using the full array to sample the xtalk. In this case there is
* not enough information to generate new Xtalk parm info. The function
* will exit and leave the current xtalk parameters unaltered"""
VL53L1_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL =							-23
"""!< Thrown when run_xtalk_extraction fn has found that the
* avg sigma estimate of the full array xtalk sample is > than the
* maximal limit allowed. In this case the xtalk sample is too noisy for
* measurement. The function will exit and leave the current xtalk parameters
* unaltered."""


VL53L1_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL =									-24
"""!< Thrown if there one of stages has no valid offset calibration
*	samples. A fatal error calibration not valid"""
VL53L1_ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL =								-25
"""!< Thrown if there one of stages has zero effective SPADS
*	Traps the case when MM1 SPADs is zero.
*	A fatal error calibration not valid"""
VL53L1_ERROR_ZONE_CAL_NO_SAMPLE_FAIL =										-26
"""!< Thrown if then some of the zones have no valid samples
*	A fatal error calibration not valid"""
VL53L1_ERROR_TUNING_PARM_KEY_MISMATCH =										-27
"""!< Thrown if the tuning file key table version does not match with
* expected value. The driver expects the key table version to match
* the compiled default version number in the define
* #VL53L1_TUNINGPARM_KEY_TABLE_VERSION_DEFAULT*"""
VL53L1_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS =								-28
"""!< Thrown if there are less than 5 good SPADs are available."""
VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH =								-29
"""!< Thrown if the final reference rate is greater than
		the upper reference rate limit - default is 40 Mcps.
		Implies a minimum Q3 (x10) SPAD (5) selected"""
VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW =									-30
"""!< Thrown if the final reference rate is less than
		the lower reference rate limit - default is 10 Mcps.
		Implies maximum Q1 (x1) SPADs selected"""


VL53L1_WARNING_OFFSET_CAL_MISSING_SAMPLES =									-31
"""!< Thrown if there is less than the requested number of
*	valid samples."""
VL53L1_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH =									-32
"""!< Thrown if the offset calibration range sigma estimate is greater
*	than 8.0 mm. This is the recommended min value to yield a stable
*	offset measurement"""
VL53L1_WARNING_OFFSET_CAL_RATE_TOO_HIGH =									-33
"""!< Thrown when VL53L1_run_offset_calibration() peak rate is greater
		than that 50.0Mcps. This is the recommended max rate to avoid
		pile-up influencing the offset measurement"""
VL53L1_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW =								-34
"""!< Thrown when VL53L1_run_offset_calibration() when one of stages
		range has less that 5.0 effective SPADS. This is the recommended
		min value to yield a stable offset"""


VL53L1_WARNING_ZONE_CAL_MISSING_SAMPLES =									-35
"""!< Thrown if one of more of the zones have less than
		the requested number of valid samples"""
VL53L1_WARNING_ZONE_CAL_SIGMA_TOO_HIGH =									-36
"""!< Thrown if one or more zones have sigma estimate value greater
*	than 8.0 mm. This is the recommended min value to yield a stable
*	offset measurement"""
VL53L1_WARNING_ZONE_CAL_RATE_TOO_HIGH =										-37
"""!< Thrown if one of more zones have peak rate higher than
		that 50.0Mcps. This is the recommended max rate to avoid
		pile-up influencing the offset measurement"""


VL53L1_WARNING_XTALK_MISSING_SAMPLES =										-38
"""!< Thrown to notify that some of the xtalk samples did not yield
* valid ranging pulse data while attempting to measure
* the xtalk signal in vl53l1_run_xtalk_extract(). This can signify any of
* the zones are missing samples, for further debug information the
* xtalk_results struct should be referred to. This warning is for
* notification only, the xtalk pulse and shape have still been generated"""
VL53L1_WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT =								-39
"""!< Thrown to notify that some of teh xtalk samples used for gradient
* generation did not yield valid ranging pulse data while attempting to
* measure the xtalk signal in vl53l1_run_xtalk_extract(). This can signify
* that any one of the zones 0-3 yielded no successful samples. The
* xtalk_results struct should be referred to for further debug info.
* This warning is for notification only, the xtalk pulse and shape
* have still been generated."""
VL53L1_WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT =								-40
"""!< Thrown to notify that some of the xtalk samples used for gradient
* generation did not pass the sigma limit check while attempting to
* measure the xtalk signal in vl53l1_run_xtalk_extract(). This can signify
* that any one of the zones 0-3 yielded an avg sigma_mm value > the limit.
* The xtalk_results struct should be referred to for further debug info.
* This warning is for notification only, the xtalk pulse and shape
* have still been generated."""
VL53L1_ERROR_NOT_IMPLEMENTED =												-41
"""!< Tells requested functionality has not been implemented yet or
* not compatible with the device"""
VL53L1_ERROR_PLATFORM_SPECIFIC_START =										-60
"""!< Tells the starting code for platform
	 @} VL53L1_define_Error_group"""

# _VL53L1_ERROR_CODES_H_
###############################################################################
###############################################################################




###############################################################################
# Classes ------------------------------------------------
# 	Class representing a VL53L1 sensor component
###############################################################################
class VL53L1X(object):

	# Software Version Information
	VL53L1X_IMPLEMENTATION_VER_MAJOR=		1
	VL53L1X_IMPLEMENTATION_VER_MINOR=		0
	VL53L1X_IMPLEMENTATION_VER_SUB=			1
	VL53L1X_IMPLEMENTATION_VER_REVISION=	0000

	"""
	SparkFunVL53L1X
	Initialise the VL53L1X chip at ``address`` with ``i2c_driver``.
		:param address:		The I2C address to use for the device.
							If not provided, the default address is
							used.
		:param i2c_driver:	An existing i2c driver object. If not
							provided a driver object is created.
		
		:return:			Constructor Initialization
							True-	Successful
							False-	Issue loading I2C driver
		:rtype:				Bool
	"""
	#----------------------------------------------
	# Device Name:
	device_name = _DEFAULT_NAME

	#----------------------------------------------
	# Available Addresses:
	available_addresses = _AVAILABLE_I2C_ADDRESS

	#----------------------------------------------
	# Constructor
	def __init__(self, address = None, debug = None, i2c_driver = None):
		"""
		This method initializes the class object. If no 'address' or
		'i2c_driver' are inputed or 'None' is specified, the method will
		use the defaults.
		:param address: 	The I2C address to use for the device.
							If not provided, the method will default to
							the first address in the
							'available_addresses' list.
								Default = 0x40
		:param debug:		Designated whether or not to print debug
							statements.
							0-	Don't print debug statements
							1-	Print debug statements
		:param i2c_driver:	An existing i2c driver object. If not
							provided a driver object is created from the
							'qwiic_i2c' I2C driver of the SparkFun Qwiic
							library.
		"""

		# Did the user specify an I2C address?
		# Defaults to 0x52 if unspecified.
		self.address = address if address != None else self.available_addresses[0]

		# Load the I2C driver if one isn't provided
		if i2c_driver == None:
			self._i2c = qwiic_i2c.getI2CDriver()
	
			if self._i2c == None:
				print("Unable to load I2C driver for this platform.")
				return
		else:
			self._i2c = i2c_driver


		# Do you want debug statements?
		if debug == None:
			self.debug = 0	# Debug Statements Disabled
		else:
			self.debug = debug	# Debug Statements Enabled (1)


	def _Init(self):
		"""
		* @brief	One time device initialization
		* @param	void
		* @return	0 on success, @a #CALIBRATION_WARNING if failed
		"""
		return self.SensorInit()


	def _ReadID(self):
		"""
		Read function of the ID device 
		"""
		if (self.GetSensorId() == 0xEEAC):
			return 0
		return -1
	

	def GetDistance(self):
		"""
		* @brief			Get ranging result and only that
		* @param pRange_mm	Pointer to range distance
		* @return			0 on success
		"""
		distance = self._GetDistance_()
		return distance

	def InitSensor(self, address):
		"""
		* @brief	Initialize the sensor with default values
		* @return	0 on Success
		"""
		self.status = 0
		sensorState = 0
		# VL53L1_Off()
		# VL53L1_On()
		self.status = self.SetI2CAddress(address)
	
		if self.debug == 1:
			byteData = self.__i2cRead(self.address, 0x010F, 1)
			print("VL53L1X Model_ID: %s \n", byteData)

			byteData = self.__i2cRead(self.address, 0x0110, 1)
			print("VL53L1X Module_Type: %s \n", byteData)

			wordData = self.__i2cRead(self.address, 0x010F, 2)
			print("VL53L1X: %s \n", wordData)
	
	
		while (not sensorState and not self.status):
			sensorState = self.BootState()
			time.sleep(2/1000)
		
		if (not self.status):
			self.status = self.SensorInit()
	
		return self.status



	# VL53L1X_api.h functions
	###############################################################################
	###############################################################################

	def GetSWVersion(self):
		"""
		* @brief	This function returns the SW driver version
		"""
		self.status = 0
		major = self.VL53L1X_IMPLEMENTATION_VER_MAJOR
		minor = self.VL53L1X_IMPLEMENTATION_VER_MINOR
		build = self.VL53L1X_IMPLEMENTATION_VER_SUB
		revision = self.VL53L1X_IMPLEMENTATION_VER_REVISION

		return [major, minor, build, revision]



	def SetI2CAddress(self, new_address):
		"""
		* @brief	This function sets the sensor I2C address used in case multiple devices application, default address 0x52 
		"""
		self.status = 0

		self.status = self.__i2cWrite(self.address, VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_address >> 1, 1)
		self.address = new_address
		
		return self.status

	def SensorInit(self):
		"""
		* @brief		This function loads the 135 bytes default values to initialize the sensor.
		* @param dev	Device address
		* @return		0:success, != 0:failed
		"""
		self.status = 0
		Addr = 0x00
		tmp=0

		for Addr in range(0x2D, 0x87 + 1):
			self.status = self.__i2cWrite(self.address, Addr, VL51L1X_DEFAULT_CONFIGURATION[Addr - 0x2D], 1)
		
		self.status = self.StartRanging()
		while(tmp==0):
				tmp = self.CheckForDataReady()
		
		tmp = 0
		self.status = self.ClearInterrupt()
		self.status = self.StopRanging()
		self.status = self.__i2cWrite(self.address, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09, 1) #  two bounds VHV
		self.status = self.__i2cWrite(self.address, 0x0B, 0, 1) #  start VHV from the previous temperature
		
		return self.status


	def ClearInterrupt(self):
		"""
		* @brief	This function clears the interrupt, to be called after a ranging data reading
		* 			to arm the interrupt for the next data ready event.
		"""
		self.status = 0
		self.status = self.__i2cWrite(self.address, SYSTEM__INTERRUPT_CLEAR, 0x01, 1)

		return self.status


	def SetInterruptPolarity(self, NewPolarity):
		"""
		* @brief	This function programs the interrupt polarity\n
		* 			1=active high (default), 0=active low
		"""
		self.status = 0
		Temp = self.__i2cRead(self.address, GPIO_HV_MUX__CTRL, 1)

		Temp = Temp & 0xEF
		self.status = self.__i2cWrite(self.address, GPIO_HV_MUX__CTRL, Temp | (not (NewPolarity & 1)) << 4, 1)

		return self.status


	def GetInterruptPolarity(self):
		"""
		* @brief	This function returns the current interrupt polarity\n
		* 			1=active high (default), 0=active low
		"""
		self.status = 0
		Temp = self.__i2cRead(self.address, GPIO_HV_MUX__CTRL, 1)
		Temp = Temp & 0x10
		pInterruptPolarity = not (Temp >> 4)

		return pInterruptPolarity


	def StartRanging(self):
		"""
		* @brief This function starts the ranging distance operation\n
		* The ranging operation is continuous. The clear interrupt has to be done after each get data to allow the interrupt to raise when the next data is ready\n
		* 1=active high (default), 0=active low, use SetInterruptPolarity() to change the interrupt polarity if required.
		"""
		self.status = 0
		self.status = self.__i2cWrite(self.address, SYSTEM__MODE_START, 0x40, 1)	#  Enable VL53L1X

		return self.status


	def StopRanging(self):
		"""
		* @brief This function stops the ranging.
		"""
		self.status = 0
		self.status = self.__i2cWrite(self.address, SYSTEM__MODE_START, 0x00, 1)	#  Disable VL53L1X

		return self.status


	def CheckForDataReady(self):
		"""
		* @brief This function checks if the new ranging data is available by polling the dedicated register.
		* @param : isDataReady==0 -> not ready isDataReady==1 -> ready
		"""
		self.status = 0

		IntPol = self.GetInterruptPolarity()
		Temp = self.__i2cRead(self.address, GPIO__TIO_HV_STATUS, 1)
		
		# Read in the register to check if a new value is available
		if (self.status == 0):
			if ((Temp & 1) == IntPol):
				isDataReady = 1
			else:
				isDataReady = 0
		
		return isDataReady


	def SetTimingBudgetInMs(self, TimingBudgetInMs):
		"""
		* @brief This function programs the timing budget in ms.
		* Predefined values = 15, 20, 33, 50, 100(default), 200, 500.
		"""
		self.status = 0

		DM = self.GetDistanceMode()
		if (DM == 0):
			return 1
		elif (DM == 1):	# Short DistanceMode
			if TimingBudgetInMs == 15: # only available in short distance mode
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01D, 2)
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0027, 2)
			elif TimingBudgetInMs == 20:
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0051, 2)
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E, 2)
			elif TimingBudgetInMs == 33:
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00D6, 2)
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E, 2)
			elif TimingBudgetInMs == 50:
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x1AE, 2)
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01E8, 2)
			elif TimingBudgetInMs == 100:
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02E1, 2)
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0388, 2)
			elif TimingBudgetInMs == 200:
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x03E1, 2)
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0496, 2)
			elif TimingBudgetInMs == 500:
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0591, 2)
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x05C1, 2)
			else:
				self.status = 1
			
		else:
			if TimingBudgetInMs == 20:
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001E, 2)
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0022, 2)
			elif TimingBudgetInMs == 33:
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0060, 2)
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E, 2)
			elif TimingBudgetInMs == 50:
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00AD, 2)
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x00C6, 2)
			elif TimingBudgetInMs == 100:
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01CC, 2)
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01EA, 2)
			elif TimingBudgetInMs == 200:
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02D9, 2)
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x02F8, 2)
			elif TimingBudgetInMs == 500:
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x048F, 2)
				self.__i2cWrite(self.address, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x04A4, 2)
			else:
				self.status = 1
		
		return self.status


	def GetTimingBudgetInMs(self):
		"""
		* @brief This function returns the current timing budget in ms.
		"""
		self.status = 0

		Temp = self.__i2cRead(self.address, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 2)

		def GetTimingBudgetInMs_switch(var):
			switcher = {
				0x001D:15,
				0x0051:20,
				0x001E:20,
				0x00D6:33,
				0x0060:33,
				0x1AE :50,
				0x00AD:50,
				0x02E1:100,
				0x01CC:100,
				0x03E1:200,
				0x02D9:200,
				0x0591:500,
				0x048F:500}
			return switcher.get(var,0)

		pTimingBudget = GetTimingBudgetInMs_switch(Temp)

		return pTimingBudget


	def SetDistanceMode(self, DM):
		"""
		* @brief This function programs the distance mode (1=short, 2=long(default)).
		* Short mode max distance is limited to 1.3 m but better ambient immunity.\n
		* Long mode can range up to 4 m in the dark with 200 ms timing budget.
		"""
		self.status = 0

		TB = self.GetTimingBudgetInMs()
		if DM == 1:
			self.status = self.__i2cWrite(self.address, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14, 1)
			self.status = self.__i2cWrite(self.address, RANGE_CONFIG__VCSEL_PERIOD_A, 0x07, 1)
			self.status = self.__i2cWrite(self.address, RANGE_CONFIG__VCSEL_PERIOD_B, 0x05, 1)
			self.status = self.__i2cWrite(self.address, RANGE_CONFIG__VALID_PHASE_HIGH, 0x38, 1)
			self.status = self.__i2cWrite(self.address, SD_CONFIG__WOI_SD0, 0x0705, 2)
			self.status = self.__i2cWrite(self.address, SD_CONFIG__INITIAL_PHASE_SD0, 0x0606, 2)
		elif DM == 2:
			self.status = self.__i2cWrite(self.address, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A, 1)
			self.status = self.__i2cWrite(self.address, RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F, 1)
			self.status = self.__i2cWrite(self.address, RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D, 1)
			self.status = self.__i2cWrite(self.address, RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8, 1)
			self.status = self.__i2cWrite(self.address, SD_CONFIG__WOI_SD0, 0x0F0D, 2)
			self.status = self.__i2cWrite(self.address, SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E, 2)
		else:
			if self.debug == 1:
				print("Invalid DIstance Mode")

		self.status = self.SetTimingBudgetInMs(TB)
		return self.status


	def GetDistanceMode(self):
		"""
		* @brief This function returns the current distance mode (1=short, 2=long).
		"""
		self.status = 0

		TempDM = self.__i2cRead(self.address,PHASECAL_CONFIG__TIMEOUT_MACROP, 1)
		if (TempDM == 0x14):
			DM=1
		if(TempDM == 0x0A):
			DM=2
		return DM


	def SetInterMeasurementInMs(self, InterMeasMs):
		"""
		* @brief This function programs the Intermeasurement period in ms\n
		* Intermeasurement period must be >/= timing budget. This condition is not checked by the API,
		* the customer has the duty to check the condition. Default = 100 ms
		"""
		self.status = 0

		ClockPLL = self.__i2cRead(self.address, VL53L1_RESULT__OSC_CALIBRATE_VAL, 2)
		ClockPLL = ClockPLL&0x3FF

		self.status = self.__i2cWrite(self.address, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD,
				(ClockPLL * InterMeasMs * 1.075), 4)

		return self.status


	def GetInterMeasurementInMs(self):
		"""
		* @brief This function returns the Intermeasurement period in ms.
		"""
		tmp = 0
		ClockPLL = 0
		pIM = 0

		tmp = self.__i2cRead(self.address, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, 4)
		ClockPLL = self.__i2cRead(self.address, VL53L1_RESULT__OSC_CALIBRATE_VAL, 2)
		ClockPLL = ClockPLL&0x3FF
		pIM= (tmp/(ClockPLL*1.065))

		return pIM


	def BootState(self):
		"""
		* @brief This function returns the boot state of the device (1:booted, 0:not booted)
		"""
		self.status = 0
		state = 0

		state = self.__i2cRead(self.address,VL53L1_FIRMWARE__SYSTEM_STATUS, 1)

		return state


	def GetSensorId(self):
		"""
		* @brief This function returns the sensor id, sensor Id must be 0xEEAC
		"""
		self.status = 0
		sensorId = 0

		sensorId = self.__i2cRead(self.address, VL53L1_IDENTIFICATION__MODEL_ID, 2)

		return sensorId


	def _GetDistance_(self):
		"""
		* @brief This function returns the distance measured by the sensor in mm
		"""
		self.status = 0
		distance = self.__i2cRead(self.address,
				VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, 2)

		return distance


	def GetSignalPerSpad(self):
		"""
		* @brief This function returns the returned signal per SPAD in kcps/SPAD.
		* With kcps stands for Kilo Count Per Second
		"""
		self.status = 0
		SpNb=1

		signal = self.__i2cRead(self.address,
			VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, 2)
		SpNb = self.__i2cRead(self.address,
			VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, 2)
		signalRate = (2000.0*signal/SpNb)

		return signalRate


	def GetAmbientPerSpad(self):
		"""
		* @brief This function returns the ambient per SPAD in kcps/SPAD
		"""
		self.status = 0
		SpNb=1

		AmbientRate = self.__i2cRead(self.address, RESULT__AMBIENT_COUNT_RATE_MCPS_SD, 2)
		SpNb = self.__i2cRead(self.address, VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, 2)
		ambPerSp=(2000.0 * AmbientRate / SpNb)
		
		return ambPerSp


	def GetSignalRate(self):
		"""
		* @brief This function returns the returned signal in kcps.
		"""
		self.status = 0
		tmp = self.__i2cRead(self.address,
			VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, 2)
		signal = tmp*8
		
		return signal


	def GetSpadNb(self):
		"""
		* @brief This function returns the current number of enabled SPADs
		"""
		self.status = 0
		tmp = self.__i2cRead(self.address,
					VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, 2)
		spNb = tmp >> 8
		
		return spNb


	def GetAmbientRate(self):
		"""
		* @brief This function returns the ambient rate in kcps
		"""
		self.status = 0
		tmp = self.__i2cRead(self.address, RESULT__AMBIENT_COUNT_RATE_MCPS_SD, 2)
		ambRate = tmp*8
		
		return ambRate


	def GetRangeStatus(self):
		"""
		* @brief This function returns the ranging status error \n
		* (0:no error, 1:sigma failed, 2:signal failed, ..., 7:wrap-around)
		"""
		self.status = 0
		RgSt = self.__i2cRead(self.address, VL53L1_RESULT__RANGE_STATUS, 1)
		RgSt = RgSt&0x1F

		def GetRangeStatus_switch(var):
			switcher = {
				9:0,
				6:1,
				4:2,
				8:3,
				5:4,
				3:5,
				19:6,
				7:7,
				12:9,
				18:10,
				22:11,
				23:12,
				13:13}
			return switcher.get(var,255)
		
		rangeStatus = GetRangeStatus_switch(RgSt)

		return rangeStatus


	def SetOffset(self, OffsetValue):
		"""
		* @brief This function programs the offset correction in mm
		* @param OffsetValue:the offset correction value to program in mm
		"""
		self.status = 0
		Temp = OffsetValue*4

		self.__i2cWrite(self.address, ALGO__PART_TO_PART_RANGE_OFFSET_MM,
				Temp, 2)
		self.__i2cWrite(self.address, MM_CONFIG__INNER_OFFSET_MM, 0x0, 2)
		self.__i2cWrite(self.address, MM_CONFIG__OUTER_OFFSET_MM, 0x0, 2)

		return self.status


	def GetOffset(self):
		"""
		* @brief This function returns the programmed offset correction value in mm
		"""
		self.status = 0
		Temp = self.__i2cRead(self.address,ALGO__PART_TO_PART_RANGE_OFFSET_MM, 2)
		Temp = Temp << 3
		Temp = Temp >> 5
		offset = Temp

		return offset


	def SetXtalk(self, XtalkValue):
		"""
		* @brief This function programs the xtalk correction value in cps (Count Per Second).\n
		* This is the number of photons reflected back from the cover glass in cps.

		XTalkValue in count per second to avoid float type
		"""
		self.status = 0
		
		self.status = self.__i2cWrite(self.address,
				ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS,
				0x0000, 2)
		self.status = self.__i2cWrite(self.address, ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS,
				0x0000, 2)
		self.status = self.__i2cWrite(self.address, ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
				(XtalkValue << 9)/1000, 2) # * << 9 (7.9 format) and /1000 to convert cps to kpcs
		
		return self.status


	def GetXtalk(self):
		"""
		* @brief This function returns the current programmed xtalk correction value in cps
		"""
		self.status = 0
		tmp = self.__i2cRead(self.address,ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, 2)
		xtalk = (tmp*1000) >> 9 # * 1000 to convert kcps to cps and >> 9 (7.9 format)

		return xtalk


	def SetDistanceThreshold(self,
					ThreshLow,
					ThreshHigh, Window,
					IntOnNoTarget):
		"""
		* @brief This function programs the threshold detection mode\n
		* Example:\n
		* self.SetDistanceThreshold(dev,100,300,0,1): Below 100 \n
		* self.SetDistanceThreshold(dev,100,300,1,1): Above 300 \n
		* self.SetDistanceThreshold(dev,100,300,2,1): Out of window \n
		* self.SetDistanceThreshold(dev,100,300,3,1): In window \n
		* @param	dev : device address
		* @param	ThreshLow(in mm) : the threshold under which one the device raises an interrupt if Window = 0
		* @param 	ThreshHigh(in mm) : the threshold above which one the device raises an interrupt if Window = 1
		* @param	Window detection mode : 0=below, 1=above, 2=out, 3=in
		* @param	IntOnNoTarget = 1 (No longer used - just use 1)
		"""
		self.status = 0
		Temp = 0

		Temp = self.__i2cRead(self.address, SYSTEM__INTERRUPT_CONFIG_GPIO, 1)
		Temp = Temp & 0x47
		if (IntOnNoTarget == 0):
			self.status = self.__i2cWrite(self.address, SYSTEM__INTERRUPT_CONFIG_GPIO,
					(Temp | (Window & 0x07)), 1)
		else:
			self.status = self.__i2cWrite(self.address, SYSTEM__INTERRUPT_CONFIG_GPIO,
					((Temp | (Window & 0x07)) | 0x40), 1)
		
		self.status = self.__i2cWrite(self.address, SYSTEM__THRESH_HIGH, ThreshHigh, 2)
		self.status = self.__i2cWrite(self.address, SYSTEM__THRESH_LOW, ThreshLow, 2)
		
		return self.status


	def GetDistanceThresholdWindow(self):
		"""
		* @brief This function returns the window detection mode (0=below 1=above 2=out 3=in)
		"""
		self.status = 0
		tmp = self.__i2cRead(self.address,SYSTEM__INTERRUPT_CONFIG_GPIO, 1)
		window = (tmp & 0x7)
		
		return window


	def GetDistanceThresholdLow(self):
		"""
		* @brief This function returns the low threshold in mm
		"""
		self.status = 0
		low = self.__i2cRead(self.address,SYSTEM__THRESH_LOW, 2)
		
		return low


	def GetDistanceThresholdHigh(self):
		"""
		* @brief This function returns the high threshold in mm
		"""
		self.status = 0
		high = self.__i2cRead(self.address,SYSTEM__THRESH_HIGH, 2)
		
		return high


	def SetROI(self, X, Y):
		"""
		* @brief This function programs the ROI (Region of Interest)\n
		* The ROI position is centered, only the ROI size can be reprogrammed.\n
		* The smallest acceptable ROI size = 4\n
		* @param X:ROI Width Y=ROI Height
		"""
		self.status = 0
		OpticalCenter =self.__i2cRead(self.address, VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD, 1)
		if (X > 16):
			X = 16
		if (Y > 16):
			Y = 16
		if (X > 10 or Y > 10):
			OpticalCenter = 199

		self.status = self.__i2cWrite(self.address, ROI_CONFIG__USER_ROI_CENTRE_SPAD, OpticalCenter, 1)
		self.status = self.__i2cWrite(self.address, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
				(Y - 1) << 4 | (X - 1), 1)
		
		return self.status


	def GetROI_XY(self):
		"""
		*@brief This function returns width X and height Y
		"""
		self.status = 0
		tmp = self.__i2cRead(self.address,ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, 1)
		ROI_X = (tmp & 0x0F) + 1
		ROI_Y = ((tmp & 0xF0) >> 4) + 1

		return ROI_X, ROI_Y


	def SetSignalThreshold(self, Signal):
		"""
		* @brief This function programs a new signal threshold in kcps (default=1024 kcps\n
		"""
		self.status = 0

		self.__i2cWrite(self.address,RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,Signal >> 3, 2)
		
		return self.status


	def GetSignalThreshold(self):
		"""
		* @brief This function returns the current signal threshold in kcps
		"""
		self.status = 0
		tmp = self.__i2cRead(self.address,
					RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 2)
		signal = tmp << 3
		return signal


	def SetSigmaThreshold(self, Sigma):
		"""
		* @brief This function programs a new sigma threshold in mm (default=15 mm)
		"""
		self.status = 0

		if(Sigma>(0xFFFF >> 2)):
			return 1
		
		# 16 bits register 14.2 format
		self.status = self.__i2cWrite(self.address,RANGE_CONFIG__SIGMA_THRESH,Sigma << 2, 2)

		return self.status


	def GetSigmaThreshold(self):
		"""
		* @brief This function returns the current sigma threshold in mm
		"""
		self.status = 0
		
		tmp = self.__i2cRead(self.address,RANGE_CONFIG__SIGMA_THRESH, 2)
		sigma = tmp >> 2

		return sigma


	def StartTemperatureUpdate(self):
		"""
		* @brief This function performs the temperature calibration.
		* It is recommended to call this function any time the temperature might have changed by more than 8 deg C
		* without sensor ranging activity for an extended period.
		"""
		self.status = 0
		tmp=0

		self.status = self.__i2cWrite(self.address,VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,0x81, 1) #  full VHV
		self.status = self.__i2cWrite(self.address,0x0B,0x92, 1)
		self.status = self.StartRanging()
		while(tmp==0):
			tmp = self.CheckForDataReady()
		
		tmp = 0
		self.status = self.ClearInterrupt()
		self.status = self.StopRanging()
		self.status = self.__i2cWrite(self.address, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09, 1) #  two bounds VHV
		self.status = self.__i2cWrite(self.address, 0x0B, 0, 1) #  start VHV from the previous temperature

		return self.status



	# VL53L1X_calibration.h functions
	###############################################################################
	###############################################################################

	def CalibrateOffset(self, TargetDistInMm):
		"""
		* @brief This function performs the offset calibration.\n
		* The function returns the offset value found and programs the offset compensation into the device.
		* @param TargetDistInMm target distance in mm, ST recommended 100 mm
		* Target reflectance = grey17%
		* @return 0:success, !=0: failed
		* @return offset pointer contains the offset found in mm
		"""
		tmp = 0
		AverageDistance = 0
		self.status = 0

		self.status = self.__i2cWrite(self.address, ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0x0, 2)
		self.status = self.__i2cWrite(self.address, MM_CONFIG__INNER_OFFSET_MM, 0x0, 2)
		self.status = self.__i2cWrite(self.address, MM_CONFIG__OUTER_OFFSET_MM, 0x0, 2)
		self.status = self.StartRanging()	# Enable VL53L1X sensor
		for i in range(0, 50):
			while (tmp == 0):
				tmp = self.CheckForDataReady()
			
			tmp = 0
			distance = self._GetDistance_()
			self.status = self.ClearInterrupt()
			AverageDistance = AverageDistance + distance
		
		self.status = self.StopRanging()
		AverageDistance = AverageDistance / 50
		offset = TargetDistInMm - AverageDistance
		self.status = self.__i2cWrite(self.address, ALGO__PART_TO_PART_RANGE_OFFSET_MM, offset*4, 2)

		return self.status #,offset???


	def CalibrateXtalk(self, TargetDistInMm):
		"""
		* @brief This function performs the xtalk calibration.\n
		* The function returns the xtalk value found and programs the xtalk compensation to the device
		* @param TargetDistInMm target distance in mm\n
		* The target distance : the distance where the sensor start to "under range"\n
		* due to the influence of the photons reflected back from the cover glass becoming strong\n
		* It's also called inflection point\n
		* Target reflectance = grey 17%
		* @return 0: success, !=0: failed
		* @return xtalk pointer contains the xtalk value found in cps (number of photons in count per second)
		"""
		tmp= 0
		AverageSignalRate = 0
		AverageDistance = 0
		AverageSpadNb = 0
		distance = 0
		self.status = 0

		self.status = self.__i2cWrite(self.address, 0x0016,0, 2)
		self.status = self.StartRanging()
		for i in range(0, 50):
			while (tmp == 0):
				tmp = self.CheckForDataReady()
			
			tmp=0
			sr= self.GetSignalRate()
			distance= self._GetDistance_()
			self.status = self.ClearInterrupt()
			AverageDistance = AverageDistance + distance
			spadNum = self.GetSpadNb()
			AverageSpadNb = AverageSpadNb + spadNum
			AverageSignalRate =	AverageSignalRate + sr
		
		self.status = self.StopRanging()
		AverageDistance = AverageDistance / 50
		AverageSpadNb = AverageSpadNb / 50
		AverageSignalRate = AverageSignalRate / 50
		# Calculate Xtalk value
		xtalk = (512*(AverageSignalRate*(1-(AverageDistance/TargetDistInMm)))/AverageSpadNb)
		self.status = self.__i2cWrite(self.address, 0x0016, xtalk, 2)
		
		return self.status #,xtalk???


###############################################################################
###############################################################################

# protected:

	def __GetTickCount(self):
		"""
		Returns current tick count in [ms]
		"""
		self.status = VL53L1_ERROR_NONE

		# *ptick_count_ms = timeGetTime()
		ptick_count_ms = 0

		return ptick_count_ms


	def __WaitUs(self, wait_us):
		time.sleep(wait_us/1000/1000)
		return VL53L1_ERROR_NONE


	def __WaitMs(self, wait_ms):
		time.sleep(wait_ms/1000)
		return VL53L1_ERROR_NONE
	

	def __WaitValueMaskEx(self, timeout_ms, index, value, mask, poll_delay_ms):
		"""
		* Platform implementation of WaitValueMaskEx V2WReg script command
		*
		* WaitValueMaskEx(
		* 		duration_ms,
		* 		index,
		* 		value,
		* 		mask,
		* 		poll_delay_ms)
		"""

		self.status	 = VL53L1_ERROR_NONE
		start_time_ms	= 0
		current_time_ms	= 0
		polling_time_ms	= 0
		byte_value		= 0
		found			= 0



		# calculate time limit in absolute time

		start_time_ms = self.__GetTickCount()

		# remember current trace functions and temporarily disable function logging

		# wait until value is found, timeout reached on error occurred

		while ((self.status == VL53L1_ERROR_NONE) and
			(polling_time_ms < timeout_ms) and
			(found == 0)):

			if (self.status == VL53L1_ERROR_NONE):
				byte_value = self.__i2cRead(self.address, index, 1)

			if ((byte_value & mask) == value):
				found = 1

			if (self.status == VL53L1_ERROR_NONE and
				found == 0 and
				poll_delay_ms > 0):
				self.status = self.__WaitMs(poll_delay_ms)

			# Update polling time (Compare difference rather than absolute to negate 32bit wrap around issue)
			current_time_ms = self.__GetTickCount()
			polling_time_ms = current_time_ms - start_time_ms

		if (found == 0 and self.status == VL53L1_ERROR_NONE):
			self.status = VL53L1_ERROR_TIME_OUT

		return self.status


# Write and read functions for I2C
###############################################################################
###############################################################################

	def __i2cWrite(self, address, register, data, nbytes):
		"""
		A wrapper for the I2C driver since device needs 16-bit register addresses. Formats register and data values so that they can be written to device as a block for proper I2C transactions.

		:param	register:	16-bit register address
							(can be 8-bit, just writes 0x00 byte prior to value)
		:param	data:		Data to be set in register
							(should be 4, 2, or 1 bytes in length)
		:param	nbytes:		number of bytes in data (*to be set*)
							(needs to be specified as python passes in integer value, but device expects a specific nuber of bytes for that value)
		
		:return:	status- (*self*) Indicator for I2C transaction success???
		:rtype:		Boolean
		"""
		
		registerMSB = register >> 8
		registerLSB = register & 0xFF

		buffer = [registerLSB]

		if nbytes == 4:
			buffer.append( (data >> 24) & 0xFF )
			buffer.append( (data >> 16) & 0xFF )
			buffer.append( (data >>  8) & 0xFF )
			buffer.append( (data >>  0) & 0xFF )
		elif nbytes == 2:
			buffer.append( (data >>  8) & 0xFF )
			buffer.append( (data >>  0) & 0xFF )
		elif nbytes == 1:
			buffer.append( data )
		else:
			if self.debug == 1:
				print("in __i2cWriteBlock, nbytes entered invalid")
			
			return
		
		self.status = self._i2c.writeBlock(address, registerMSB, buffer)

		return self.status
	

	def __i2cRead(self, address, register, nbytes):
		"""
		A wrapper for the I2C driver since device needs 16-bit register addresses. Formats register and data values so that they can be written to device as a block for proper I2C transactions.

		:param	register:	16-bit register address
							(can be 8-bit, just writes 0x00 byte prior to value)
		:param	nbytes:		number of bytes in data (*to be read*)
							(needs to be specified for transaction)
		
		:return:	data
		:rtype:		integer
		"""
		
		data = 0
		registerMSB = register >> 8
		registerLSB = register & 0xFF

		if nbytes not in [1, 2, 4]:
			if self.debug == 1:
				print("in __i2cWriteBlock, nbytes entered invalid")
			return

		# Setup for read/write transactions on smbus 2
		# write = self.i2c_custom.write(address, [registerMSB, registerLSB])	# Write part of transaction
		# read = self.i2c_custom.read(address, nbytes)						# Read part of transaction

		# self._i2c.i2c_rdwr(write, read)
		read_data = self._i2c.__i2c_rdwr__(address, [registerMSB, registerLSB], nbytes)
		buffer = list(read_data)

		for i in range(0, nbytes):
			data = ( buffer[ (nbytes - 1) - i ] << (i*8) ) + data

		#for i, val in enumerate(read):
		#	data = ( val << (i*8) ) + data

		return data
