#include <SimpleMPU.h>

SimpleMPU mpu;

void setup()
{
	Serial.begin(115200);

	// Initialize the MPU6050 
	// Use mpu.init(1) if MPU is connected but code says initialization failed
	while (!mpu.init())
	{
		Serial.println("MPU initialization failed. Please check your wiring.");
		delay(1000);
	}
	Serial.println("MPU initialized successfully!");

	// Set the MPU Gyroscope range
	// 0 = +/- 250 degrees/sec (default)
	// 1 = +/- 500 degrees/sec
	// 2 = +/- 1000 degrees/sec
	// 3 = +/- 2000 degrees/sec
	mpu.setGyroRange(0);

	// Set the MPU Accelerometer range
	// 0 = +/- 2g (default)
	// 1 = +/- 4g
	// 2 = +/- 8g
	// 3 = +/- 16g
	mpu.setAccelRange(0);

	// Calibrate the MPU. This will calculate the offset values for the MPU
	mpu.calibGyro();
	mpu.calibAccel();
}

void loop()
{
	// Create structs to hold the data
	GyroData gyro;
	AccelData accel;

	// Read the gyro and accel data
	mpu.readGyro(&gyro);
	mpu.readAccel(&accel);

	// Print the data
	Serial.print("Gyro: ");
	Serial.print(gyro.x);
	Serial.print(" ");
	Serial.print(gyro.y);
	Serial.print(" ");
	Serial.print(gyro.z);
	Serial.print(" | Accel: ");
	Serial.print(accel.x);
	Serial.print(" ");
	Serial.print(accel.y);
	Serial.print(" ");
	Serial.print(accel.z);
	Serial.println();

	delay(1000);
}