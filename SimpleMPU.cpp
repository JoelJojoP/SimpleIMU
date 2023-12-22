/*
 *  Program to read the temperature sensor on the ATMega328P
 *  Original code by Joel Jojo
 *
 *  This is free software. You can redistribute it and/or modify it under
 *  the terms of MIT Licence.
 *  To view a copy of this license, visit http://opensource.org/licenses/mit-license.php
 */

// Importing required libraries
#include <Arduino.h>
#include "SimpleMPU.h"

// Constructor
SimpleMPU::SimpleMPU(uint8_t address)
{
	Wire.begin(address);
	SimpleMPU::MPU6050_ADR = address;
}

// Initialize the MPU6050
bool SimpleMPU::init(bool bypass_check)
{
	Wire.beginTransmission(SimpleMPU::MPU6050_ADR);
	Wire.write(MPU6050_IMU::MPU6050_RA_PWR_MGMT_1);
	Wire.write(0x00);
	Wire.endTransmission(true);
	Wire.beginTransmission(SimpleMPU::MPU6050_ADR);
	Wire.write(MPU6050_IMU::MPU6050_RA_WHO_AM_I);
	Wire.endTransmission(false);
	Wire.requestFrom(SimpleMPU::MPU6050_ADR, 1, true);
	uint8_t response = Wire.read();
	if (response != MPU6050_IMU::MPU6050_ADDRESS_AD0_LOW && response != MPU6050_IMU::MPU6050_ADDRESS_AD0_HIGH && bypass_check == false)
		return false;
	return true;
}

// Read gyroscope values
void SimpleMPU::readGyro(GyroData *gyro)
{
	Wire.beginTransmission(SimpleMPU::MPU6050_ADR);
	Wire.write(MPU6050_IMU::MPU6050_RA_GYRO_XOUT_H);
	Wire.endTransmission(false);
	Wire.requestFrom(SimpleMPU::MPU6050_ADR, 6, true);
	int16_t x = (Wire.read() << 8 | Wire.read()) - SimpleMPU::MPU6050_GYRO_OFF_X;
	int16_t y = (Wire.read() << 8 | Wire.read()) - SimpleMPU::MPU6050_GYRO_OFF_Y;
	int16_t z = (Wire.read() << 8 | Wire.read()) - SimpleMPU::MPU6050_GYRO_OFF_Z;
	if (SimpleMPU::MPU6050_GYRO_FS_SEL == 0)
	{
		gyro->x = x / 131.0;
		gyro->y = y / 131.0;
		gyro->z = z / 131.0;
	}
	else if (SimpleMPU::MPU6050_GYRO_FS_SEL == 1)
	{
		gyro->x = x / 65.5;
		gyro->y = y / 65.5;
		gyro->z = z / 65.5;
	}
	else if (SimpleMPU::MPU6050_GYRO_FS_SEL == 2)
	{
		gyro->x = x / 32.8;
		gyro->y = y / 32.8;
		gyro->z = z / 32.8;
	}
	else if (SimpleMPU::MPU6050_GYRO_FS_SEL == 3)
	{
		gyro->x = x / 16.4;
		gyro->y = y / 16.4;
		gyro->z = z / 16.4;
	}
}

// Calibrate gyroscope
void SimpleMPU::calibGyro(int samples)
{
	GyroData gyro;
	int16_t x, y, z;
	long int sumx = 0, sumy = 0, sumz = 0;
	for (int i = 0; i < samples; i++)
	{
		Wire.beginTransmission(SimpleMPU::MPU6050_ADR);
		Wire.write(MPU6050_IMU::MPU6050_RA_GYRO_XOUT_H);
		Wire.endTransmission(false);
		Wire.requestFrom(SimpleMPU::MPU6050_ADR, 6, true);
		x = (Wire.read() << 8 | Wire.read());
		y = (Wire.read() << 8 | Wire.read());
		z = (Wire.read() << 8 | Wire.read());
		sumx += x;
		sumy += y;
		sumz += z;
	}
	SimpleMPU::MPU6050_GYRO_OFF_X = sumx / samples;
	SimpleMPU::MPU6050_GYRO_OFF_Y = sumy / samples;
	SimpleMPU::MPU6050_GYRO_OFF_Z = sumz / samples;
}

// Set range of gyroscope
void SimpleMPU::setGyroRange(uint8_t scale)
{
	Wire.beginTransmission(SimpleMPU::MPU6050_ADR);
	Wire.write(MPU6050_IMU::MPU6050_RA_GYRO_CONFIG);
	Wire.endTransmission(false);
	Wire.requestFrom(SimpleMPU::MPU6050_ADR, 1, true);
	uint8_t gyro_config = Wire.read();
	if (scale == 0)
		gyro_config &= ~((1 << 3) | (1 << 4));
	else if (scale == 1)
	{
		gyro_config |= (1 << 3);
		gyro_config &= ~(1 << 4);
	}
	else if (scale == 2)
	{
		gyro_config &= ~(1 << 3);
		gyro_config |= (1 << 4);
	}
	else if (scale == 3)
		gyro_config |= (1 << 3) | (1 << 4);
	Wire.beginTransmission(SimpleMPU::MPU6050_ADR);
	Wire.write(MPU6050_IMU::MPU6050_RA_GYRO_CONFIG);
	Wire.write(gyro_config);
	Wire.endTransmission(true);
	SimpleMPU::MPU6050_GYRO_FS_SEL = scale;
}

// Get range of gyroscope
uint8_t SimpleMPU::getGyroRange()
{
	Wire.beginTransmission(SimpleMPU::MPU6050_ADR);
	Wire.write(MPU6050_IMU::MPU6050_RA_GYRO_CONFIG);
	Wire.endTransmission(false);
	Wire.requestFrom(SimpleMPU::MPU6050_ADR, 1, true);
	uint8_t gyro_config = Wire.read() & 0x18;
	gyro_config >>= 3;
	return gyro_config;
}

// Set range of accelerometer
void SimpleMPU::setAccelRange(uint8_t scale)
{
	Wire.beginTransmission(SimpleMPU::MPU6050_ADR);
	Wire.write(MPU6050_IMU::MPU6050_RA_ACCEL_CONFIG);
	Wire.endTransmission(false);
	Wire.requestFrom(SimpleMPU::MPU6050_ADR, 1, true);
	uint8_t accel_config = Wire.read();
	if (scale == 0)
		accel_config &= ~((1 << 3) | (1 << 4));
	else if (scale == 1)
	{
		accel_config |= (1 << 3);
		accel_config &= ~(1 << 4);
	}
	else if (scale == 2)
	{
		accel_config &= ~(1 << 3);
		accel_config |= (1 << 4);
	}
	else if (scale == 3)
		accel_config |= (1 << 3) | (1 << 4);
	Wire.beginTransmission(SimpleMPU::MPU6050_ADR);
	Wire.write(MPU6050_IMU::MPU6050_RA_ACCEL_CONFIG);
	Wire.write(accel_config);
	Wire.endTransmission(true);
	SimpleMPU::MPU6050_ACCEL_FS_SEL = scale;
}

// Get range of accelerometer
uint8_t SimpleMPU::getAccelRange()
{
	Wire.beginTransmission(SimpleMPU::MPU6050_ADR);
	Wire.write(MPU6050_IMU::MPU6050_RA_ACCEL_CONFIG);
	Wire.endTransmission(false);
	Wire.requestFrom(SimpleMPU::MPU6050_ADR, 1, true);
	uint8_t accel_config = Wire.read() & 0x18;
	accel_config >>= 3;
	return accel_config;
}

// Calibrate gyroscope
void SimpleMPU::calibAccel(int samples)
{
	AccelData accel;
	int16_t x, y, z;
	long int sumx = 0, sumy = 0, sumz = 0;
	for (int i = 0; i < samples; i++)
	{
		Wire.beginTransmission(SimpleMPU::MPU6050_ADR);
		Wire.write(MPU6050_IMU::MPU6050_RA_ACCEL_XOUT_H);
		Wire.endTransmission(false);
		Wire.requestFrom(SimpleMPU::MPU6050_ADR, 6, true);
		x = (Wire.read() << 8 | Wire.read());
		y = (Wire.read() << 8 | Wire.read());
		z = (Wire.read() << 8 | Wire.read());
		sumx += x;
		sumy += y;
		sumz += z;
	}
	SimpleMPU::MPU6050_ACCEL_OFF_X = sumx / samples;
	SimpleMPU::MPU6050_ACCEL_OFF_Y = sumy / samples;
	SimpleMPU::MPU6050_ACCEL_OFF_Z = sumz / samples;
}

// Read accelerometer values
void SimpleMPU::readAccel(AccelData *accel)
{
	Wire.beginTransmission(SimpleMPU::MPU6050_ADR);
	Wire.write(MPU6050_IMU::MPU6050_RA_ACCEL_XOUT_H);
	Wire.endTransmission(false);
	Wire.requestFrom(SimpleMPU::MPU6050_ADR, 6, true);
	int16_t x = (Wire.read() << 8 | Wire.read()) - SimpleMPU::MPU6050_ACCEL_OFF_X;
	int16_t y = (Wire.read() << 8 | Wire.read()) - SimpleMPU::MPU6050_ACCEL_OFF_Y;
	int16_t z = (Wire.read() << 8 | Wire.read()) - SimpleMPU::MPU6050_ACCEL_OFF_Z;
	if(SimpleMPU::MPU6050_ACCEL_FS_SEL == 0)
	{
		accel->x = x / 16384.0;
		accel->y = y / 16384.0;
		accel->z = z / 16384.0;
	}
	else if(SimpleMPU::MPU6050_ACCEL_FS_SEL == 1)
	{
		accel->x = x / 8192.0;
		accel->y = y / 8192.0;
		accel->z = z / 8192.0;
	}
	else if(SimpleMPU::MPU6050_ACCEL_FS_SEL == 2)
	{
		accel->x = x / 4096.0;
		accel->y = y / 4096.0;
		accel->z = z / 4096.0;
	}
	else if(SimpleMPU::MPU6050_ACCEL_FS_SEL == 3)
	{
		accel->x = x / 2048.0;
		accel->y = y / 2048.0;
		accel->z = z / 2048.0;
	}
	accel->x = accel->x * 9.81;
	accel->y = accel->y * 9.81;
	accel->z = accel->z * 9.81;
}