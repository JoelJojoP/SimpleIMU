/*
 *  Program to read the temperature sensor on the ATMega328P
 *  Original code by Joel Jojo
 *
 *  This is free software. You can redistribute it and/or modify it under
 *  the terms of MIT Licence.
 *  To view a copy of this license, visit http://opensource.org/licenses/mit-license.php
 */

// Importing required libraries
#include <Wire.h>
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
	gyro->x = (Wire.read() << 8 | Wire.read()) - SimpleMPU::MPU6050_GYRO_OFF_X;
	gyro->y = (Wire.read() << 8 | Wire.read()) - SimpleMPU::MPU6050_GYRO_OFF_Y;
	gyro->z = (Wire.read() << 8 | Wire.read()) - SimpleMPU::MPU6050_GYRO_OFF_Z;
	if(SimpleMPU::MPU6050_GYRO_FS_SEL == 0)
	{
		gyro->x /= 131;
		gyro->y /= 131;
		gyro->z /= 131;
	}
	else if(SimpleMPU::MPU6050_GYRO_FS_SEL == 1)
	{
		gyro->x /= 65.5;
		gyro->y /= 65.5;
		gyro->z /= 65.5;
	}
	else if(SimpleMPU::MPU6050_GYRO_FS_SEL == 2)
	{
		gyro->x /= 32.8;
		gyro->y /= 32.8;
		gyro->z /= 32.8;
	}
	else if(SimpleMPU::MPU6050_GYRO_FS_SEL == 3)
	{
		gyro->x /= 16.4;
		gyro->y /= 16.4;
		gyro->z /= 16.4;
	}
}

// Calibrate gyroscope
void SimpleMPU::calibGyro(int samples)
{
	GyroData gyro;
	int x = 0, y = 0, z = 0;
	for (int i = 0; i < samples; i++)
	{
		Wire.beginTransmission(SimpleMPU::MPU6050_ADR);
		Wire.write(MPU6050_IMU::MPU6050_RA_GYRO_XOUT_H);
		Wire.endTransmission(false);
		Wire.requestFrom(SimpleMPU::MPU6050_ADR, 6, true);
		x += (Wire.read() << 8 | Wire.read());
		y += (Wire.read() << 8 | Wire.read());
		z += (Wire.read() << 8 | Wire.read());
	}
	SimpleMPU::MPU6050_GYRO_OFF_X = x / samples;
	SimpleMPU::MPU6050_GYRO_OFF_Y = y / samples;
	SimpleMPU::MPU6050_GYRO_OFF_Z = z / samples;
}

// Set scale of gyroscope
void SimpleMPU::setGyroRange(uint8_t scale)
{
	Wire.beginTransmission(SimpleMPU::MPU6050_ADR);
	Wire.write(MPU6050_IMU::MPU6050_RA_GYRO_CONFIG);
	Wire.endTransmission(false);
	Wire.requestFrom(SimpleMPU::MPU6050_ADR, 1, true);
	uint8_t gyro_config = Wire.read();
	if(scale == 0)
		gyro_config &= ~((1 << 3)|(1<<4));
	else if(scale == 1) {
		gyro_config |= (1 << 3);
		gyro_config &= ~(1 << 4);
	}
	else if(scale == 2) {
		gyro_config &= ~(1 << 3);
		gyro_config |= (1 << 4);
	}
	else if(scale == 3)
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