/*
 *  Porting layer for MPU6050 IMU
 *  Original code by Joel Jojo
 *
 *  This is free software. You can redistribute it and/or modify it under
 *  the terms of MIT Licence.
 *  To view a copy of this license, visit http://opensource.org/licenses/mit-license.php
 */

// Importing required libraries
#include <Arduino.h>
#include "../SimpleIMU.h"
#include "SimpleIMU_MPU6050.h"

bool SimpleIMU::init()
{
	/* Disable sleep mode */
	Wire.beginTransmission(SimpleIMU::IMU_Addr);
	Wire.write(MPU6050_IMU::MPU6050_RA_PWR_MGMT_1);
	Wire.write(0x00);
	Wire.endTransmission(false);

	/* Check whether device is connected */
	Wire.write(MPU6050_IMU::MPU6050_RA_WHO_AM_I);
	Wire.endTransmission(false);
	Wire.requestFrom(SimpleIMU::IMU_Addr, 1, true);
	uint8_t response = Wire.read();
	if (response == 255)
		return false;

	/* Enable DMP */
	Wire.beginTransmission(SimpleIMU::IMU_Addr);
	Wire.write(MPU6050_IMU::MPU6050_RA_USER_CTRL);
	Wire.endTransmission(false);
	Wire.requestFrom(SimpleIMU::IMU_Addr, 1, false);
	uint8_t user_ctrl = Wire.read();
	user_ctrl |= (1 << MPU6050_IMU::MPU6050_USERCTRL_DMP_EN_BIT);
	Wire.write(MPU6050_IMU::MPU6050_RA_USER_CTRL);
	Wire.write(user_ctrl);
	Wire.endTransmission(true);
	return true;
}

// Read gyroscope values
void SimpleIMU::readGyro(GyroData *gyro)
{
	Wire.beginTransmission(SimpleIMU::IMU_Addr);
	Wire.write(MPU6050_IMU::MPU6050_RA_GYRO_XOUT_H);
	Wire.endTransmission(false);
	Wire.requestFrom(SimpleIMU::IMU_Addr, 6, true);
	int16_t x = (Wire.read() << 8 | Wire.read()) - SimpleIMU::IMU_GyroOffsetX;
	int16_t y = (Wire.read() << 8 | Wire.read()) - SimpleIMU::IMU_GyroOffsetY;
	int16_t z = (Wire.read() << 8 | Wire.read()) - SimpleIMU::IMU_GyroOffsetZ;
	if (SimpleIMU::IMU_GyroFullScale == MPU6050_IMU::MPU6050_GYRO_FS_250)
	{
		gyro->x = x / 131.0;
		gyro->y = y / 131.0;
		gyro->z = z / 131.0;
	}
	else if (SimpleIMU::IMU_GyroFullScale == MPU6050_IMU::MPU6050_GYRO_FS_500)
	{
		gyro->x = x / 65.5;
		gyro->y = y / 65.5;
		gyro->z = z / 65.5;
	}
	else if (SimpleIMU::IMU_GyroFullScale == MPU6050_IMU::MPU6050_GYRO_FS_1000)
	{
		gyro->x = x / 32.8;
		gyro->y = y / 32.8;
		gyro->z = z / 32.8;
	}
	else if (SimpleIMU::IMU_GyroFullScale == MPU6050_IMU::MPU6050_GYRO_FS_2000)
	{
		gyro->x = x / 16.4;
		gyro->y = y / 16.4;
		gyro->z = z / 16.4;
	}
}

// Calibrate gyroscope
void SimpleIMU::calibGyro(int samples)
{
	GyroData gyro;
	int16_t x, y, z;
	long int sumx = 0, sumy = 0, sumz = 0;
	for (int i = 0; i < samples; i++)
	{
		Wire.beginTransmission(SimpleIMU::IMU_Addr);
		Wire.write(MPU6050_IMU::MPU6050_RA_GYRO_XOUT_H);
		Wire.endTransmission(false);
		Wire.requestFrom(SimpleIMU::IMU_Addr, 6, true);
		x = (Wire.read() << 8 | Wire.read());
		y = (Wire.read() << 8 | Wire.read());
		z = (Wire.read() << 8 | Wire.read());
		sumx += x;
		sumy += y;
		sumz += z;
	}
	SimpleIMU::IMU_GyroOffsetX = sumx / samples;
	SimpleIMU::IMU_GyroOffsetY = sumy / samples;
	SimpleIMU::IMU_GyroOffsetZ = sumz / samples;
}

// Set range of gyroscope
void SimpleIMU::setGyroRange(uint8_t scale)
{
	Wire.beginTransmission(SimpleIMU::IMU_Addr);
	Wire.write(MPU6050_IMU::MPU6050_RA_GYRO_CONFIG);
	Wire.endTransmission(false);
	Wire.requestFrom(SimpleIMU::IMU_Addr, 1, true);
	uint8_t gyro_config = Wire.read();
	if (scale == MPU6050_IMU::MPU6050_GYRO_FS_250)
		gyro_config &= ~((1 << 3) | (1 << 4));
	else if (scale == MPU6050_IMU::MPU6050_GYRO_FS_500)
	{
		gyro_config |= (1 << 3);
		gyro_config &= ~(1 << 4);
	}
	else if (scale == MPU6050_IMU::MPU6050_GYRO_FS_1000)
	{
		gyro_config &= ~(1 << 3);
		gyro_config |= (1 << 4);
	}
	else if (scale == MPU6050_IMU::MPU6050_GYRO_FS_2000)
		gyro_config |= (1 << 3) | (1 << 4);
	else
		return;
	Wire.beginTransmission(SimpleIMU::IMU_Addr);
	Wire.write(MPU6050_IMU::MPU6050_RA_GYRO_CONFIG);
	Wire.write(gyro_config);
	Wire.endTransmission(true);
	SimpleIMU::IMU_GyroFullScale = scale;
}

// Get range of gyroscope
uint8_t SimpleIMU::getGyroRange()
{
	Wire.beginTransmission(SimpleIMU::IMU_Addr);
	Wire.write(MPU6050_IMU::MPU6050_RA_GYRO_CONFIG);
	Wire.endTransmission(false);
	Wire.requestFrom(SimpleIMU::IMU_Addr, 1, true);
	uint8_t gyro_config = Wire.read() & 0x18;
	gyro_config >>= 3;
	return gyro_config;
}

// Set range of accelerometer
void SimpleIMU::setAccelRange(uint8_t scale)
{
	Wire.beginTransmission(SimpleIMU::IMU_Addr);
	Wire.write(MPU6050_IMU::MPU6050_RA_ACCEL_CONFIG);
	Wire.endTransmission(false);
	Wire.requestFrom(SimpleIMU::IMU_Addr, 1, true);
	uint8_t accel_config = Wire.read();
	if (scale == MPU6050_IMU::MPU6050_ACCEL_FS_2)
		accel_config &= ~((1 << 3) | (1 << 4));
	else if (scale == MPU6050_IMU::MPU6050_ACCEL_FS_4)
	{
		accel_config |= (1 << 3);
		accel_config &= ~(1 << 4);
	}
	else if (scale == MPU6050_IMU::MPU6050_ACCEL_FS_8)
	{
		accel_config &= ~(1 << 3);
		accel_config |= (1 << 4);
	}
	else if (scale == MPU6050_IMU::MPU6050_ACCEL_FS_16)
		accel_config |= (1 << 3) | (1 << 4);
	else
		return;
	Wire.beginTransmission(SimpleIMU::IMU_Addr);
	Wire.write(MPU6050_IMU::MPU6050_RA_ACCEL_CONFIG);
	Wire.write(accel_config);
	Wire.endTransmission(true);
	SimpleIMU::IMU_AccelFullScale = scale;
}

// Get range of accelerometer
uint8_t SimpleIMU::getAccelRange()
{
	Wire.beginTransmission(SimpleIMU::IMU_Addr);
	Wire.write(MPU6050_IMU::MPU6050_RA_ACCEL_CONFIG);
	Wire.endTransmission(false);
	Wire.requestFrom(SimpleIMU::IMU_Addr, 1, true);
	uint8_t accel_config = Wire.read() & 0x18;
	accel_config >>= 3;
	return accel_config;
}

// Calibrate gyroscope
void SimpleIMU::calibAccel(int samples)
{
	AccelData accel;
	int16_t x, y, z;
	long int sumx = 0, sumy = 0, sumz = 0;
	for (int i = 0; i < samples; i++)
	{
		Wire.beginTransmission(SimpleIMU::IMU_Addr);
		Wire.write(MPU6050_IMU::MPU6050_RA_ACCEL_XOUT_H);
		Wire.endTransmission(false);
		Wire.requestFrom(SimpleIMU::IMU_Addr, 6, true);
		x = (Wire.read() << 8 | Wire.read());
		y = (Wire.read() << 8 | Wire.read());
		z = (Wire.read() << 8 | Wire.read());
		sumx += x;
		sumy += y;
		sumz += z;
	}
	SimpleIMU::IMU_AccelOffsetX = sumx / samples;
	SimpleIMU::IMU_AccelOffsetY = sumy / samples;
	SimpleIMU::IMU_AccelOffsetZ = sumz / samples;
}

// Read accelerometer values
void SimpleIMU::readAccel(AccelData *accel)
{
	Wire.beginTransmission(SimpleIMU::IMU_Addr);
	Wire.write(MPU6050_IMU::MPU6050_RA_ACCEL_XOUT_H);
	Wire.endTransmission(false);
	Wire.requestFrom(SimpleIMU::IMU_Addr, 6, true);
	int16_t x = (Wire.read() << 8 | Wire.read()) - SimpleIMU::IMU_AccelOffsetX;
	int16_t y = (Wire.read() << 8 | Wire.read()) - SimpleIMU::IMU_AccelOffsetY;
	int16_t z = (Wire.read() << 8 | Wire.read()) - SimpleIMU::IMU_AccelOffsetZ;
	if (SimpleIMU::IMU_AccelFullScale == MPU6050_IMU::MPU6050_ACCEL_FS_2)
	{
		accel->x = x / 16384.0;
		accel->y = y / 16384.0;
		accel->z = z / 16384.0;
	}
	else if (SimpleIMU::IMU_AccelFullScale ==MPU6050_IMU::MPU6050_ACCEL_FS_4)
	{
		accel->x = x / 8192.0;
		accel->y = y / 8192.0;
		accel->z = z / 8192.0;
	}
	else if (SimpleIMU::IMU_AccelFullScale == MPU6050_IMU::MPU6050_ACCEL_FS_8)
	{
		accel->x = x / 4096.0;
		accel->y = y / 4096.0;
		accel->z = z / 4096.0;
	}
	else if (SimpleIMU::IMU_AccelFullScale == MPU6050_IMU::MPU6050_ACCEL_FS_16)
	{
		accel->x = x / 2048.0;
		accel->y = y / 2048.0;
		accel->z = z / 2048.0;
	}
	accel->x = accel->x * 9.81;
	accel->y = accel->y * 9.81;
	accel->z = accel->z * 9.81;
}