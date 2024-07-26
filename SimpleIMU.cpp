/*
 *  Main file for SimpleIMU library
 *  Original code by Joel Jojo
 *
 *  This is free software. You can redistribute it and/or modify it under
 *  the terms of MIT Licence.
 *  To view a copy of this license, visit http://opensource.org/licenses/mit-license.php
 */

#include "SimpleIMU.h"

// Constructor
SimpleIMU::SimpleIMU(uint8_t address)
{
	Wire.begin(address);
	SimpleIMU::IMU_Addr = address;
}