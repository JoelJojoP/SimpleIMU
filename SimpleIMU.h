/*
 *  Header for SimpleIMU library
 *  Original code by Joel Jojo
 *
 *  This is free software. You can redistribute it and/or modify it under
 *  the terms of MIT Licence.
 *  To view a copy of this license, visit http://opensource.org/licenses/mit-license.php
 */

#ifndef SIMPLEIMU_H
#define SIMPLEIMU_H

#include <Wire.h>

typedef struct
{
	float x;
	float y;
	float z;
} AccelData;

typedef struct
{
	float x;
	float y;
	float z;
} GyroData;

class SimpleIMU
{
private:
	/* The address of the IMU */
	uint8_t IMU_Addr;

	/* The gyroscope offset about the x axis*/
	int16_t IMU_GyroOffsetX;

	/* The gyroscope offset about the y axis*/
	int16_t IMU_GyroOffsetY;

	/* The gyroscope offset about the z axis*/
	int16_t IMU_GyroOffsetZ;

	/* The accelerometer offset along the x axis*/
	int16_t IMU_AccelOffsetX;

	/* The accelerometer offset along the y axis*/
	int16_t IMU_AccelOffsetY;

	/* The accelerometer offset along the z axis*/
	int16_t IMU_AccelOffsetZ;

	/* The gyroscope sensitivity */
	uint8_t IMU_GyroFullScale;

	/* The accelerometer sensitivity */
	uint8_t IMU_AccelFullScale;

public:
	/*
	 * Constructor for SimpleIMU object.
	 *
	 * params: address I2C address of the IMU
	 * returns: SimpleIMU object
	 */
	SimpleIMU(uint8_t address);

	/*
	 * Function to initialize the IMU.
	 *
	 * params: None
	 * returns: bool, true if initialization is successful, false otherwise
	 */
	bool init();

	/*
	 * Function to set the gyroscope sensitivity.
	 *
	 * params: range, the sensitivity range of the gyroscope.
	 *     			  Can be 0, 1, 2, 3 for 250, 500, 1000, 2000 degrees
	 * 				  per second respectively.
	 * returns: None
	 */
	void setGyroRange(uint8_t range);

	/*
	 * Function to get the sensititvity range of the gyroscope.
	 *
	 * params: None
	 * returns: uint8_t, the sensistivity range of the gyroscope.
	 * 				     Can be 0, 1, 2, 3 for 250, 500, 1000, 2000 deg/s
	 * 					 respectively.
	 */
	uint8_t getGyroRange();

	/*
	 * Function to calibrate the gyroscope.
	 *
	 * params: samples, number of samples to take for calibration
	 * returns: None
	 */
	void calibGyro(int samples = 100);

	/*
	 * Function to read the gyroscope data.
	 *
	 * params: gyro, pointer to GyroData struct to store the gyroscope data
	 * returns: None
	 */
	void readGyro(GyroData *gyro);

	/*
	 * Function to set the sensitivity range of the accelerometer.
	 *
	 * params: range, the sensitivity range of the accelerometer.
	 *   			  Can be 0, 1, 2, 3 for 2, 4, 8, 16 g respectively.
	 * returns: None
	 */
	void setAccelRange(uint8_t range);

	/*
	 * Function to get the sensitivity range of the accelerometer.
	 *
	 * params: None
	 * returns: uint8_t, the sensitivity range of the accelerometer.
	 * 					 Can be 0, 1, 2, 3 for 2, 4, 8, 16 g respectively.
	 */
	uint8_t getAccelRange();

	/*
	 * Function to calibrate the accelerometer.
	 *
	 * params: samples, number of samples to take for calibration
	 * returns: None
	 */
	void calibAccel(int samples = 100);

	/*
	 * Function to read the accelerometer data.
	 *
	 * params: accel, pointer to AccelData struct to store the accelerometer data
	 * returns: None
	 */
	void readAccel(AccelData *accel);
};

#endif /* SIMPLEIMU_H */