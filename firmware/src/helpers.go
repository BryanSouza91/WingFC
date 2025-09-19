package main

import "golang.org/x/exp/constraints"

// Read raw IMU data from the LSM6DS3TR sensor and apply a low-pass filter.
func readLSMData() {
	// Read raw sensor data from the IMU
	rawAccelX, rawAccelY, rawAccelZ, err := lsm.ReadAcceleration()
	if err != nil {
		println("Error reading acceleration:", err)
	}
	rawGyroX, rawGyroY, rawGyroZ, err := lsm.ReadRotation()
	if err != nil {
		println("Error reading rotation:", err)
	}

	// Low-pass filter
	imuData.AccelX = imuData.AccelX*LPF_ALPHA + float64(rawAccelX)*microGToMS2*LPF_ALPHA
	imuData.AccelY = imuData.AccelY*LPF_ALPHA + float64(rawAccelY)*microGToMS2*LPF_ALPHA
	imuData.AccelZ = imuData.AccelZ*LPF_ALPHA + float64(rawAccelZ)*microGToMS2*LPF_ALPHA
	imuData.GyroX = imuData.GyroX*LPF_ALPHA + float64(rawGyroX)*microDPSToRadS*LPF_ALPHA
	imuData.GyroY = imuData.GyroY*LPF_ALPHA + float64(rawGyroY)*microDPSToRadS*LPF_ALPHA
	imuData.GyroZ = imuData.GyroZ*LPF_ALPHA + float64(rawGyroZ)*microDPSToRadS*LPF_ALPHA

	// Convert raw sensor readings to standard units (rad/s and m/s^2)
	imuData.AccelX = float64(rawAccelX) * microGToMS2
	imuData.AccelY = float64(rawAccelY) * microGToMS2
	imuData.AccelZ = float64(rawAccelZ) * microGToMS2
	imuData.GyroX = float64(rawGyroX) * microDPSToRadS
	imuData.GyroY = float64(rawGyroY) * microDPSToRadS
	imuData.GyroZ = float64(rawGyroZ) * microDPSToRadS
}

// Process the raw IMU data by applying calibration offsets and computing roll/pitch angles.
func processLSMData() {
	imuData.AccelX -= accelBiasX
	imuData.AccelY -= accelBiasY
	imuData.AccelZ -= accelBiasZ
	imuData.GyroX -= gyroBiasX
	imuData.GyroY -= gyroBiasY
	imuData.GyroZ -= gyroBiasZ
	imuData.Roll = imuData.rollAccel()
	imuData.Pitch = imuData.pitchAccel()
}

// Calibrate the IMU by averaging a number of samples to determine bias offsets.
// This function should be called when the aircraft is stationary and level.
func calibrate() {
	const sampleSize = 4000

	for i := 0; i < sampleSize; i++ {
		readLSMData()
	}

	accelBiasX = accelXSum / sampleSize
	accelBiasY = accelYSum / sampleSize
	accelBiasZ = accelZSum / sampleSize
	println("Accel calibration complete. Bias X:", accelBiasX, "Bias Y:", accelBiasY, "Bias Z:", accelBiasZ)
	gyroBiasX = gyroXSum / sampleSize
	gyroBiasY = gyroYSum / sampleSize
	gyroBiasZ = gyroZSum / sampleSize
	println("Gyro calibration complete. Bias X:", gyroBiasX, "Bias Y:", gyroBiasY, "Bias Z:", gyroBiasZ)
}


// Helper function to constrain a value within min and max bounds.
func constrain(value, min, max float64) float64 {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}

// Helper function to map a value from one range to another.
func mapRange[T constraints.Float](value, fromMin, fromMax, toMin, toMax T) T {
	return (value-fromMin)/(fromMax-fromMin)*(toMax-toMin) + toMin
}

// Helper function to set servo PWM outputs using global variables.
func setServoPWM(leftPulse, rightPulse uint32) {
	// Set the PWM duty cycle for left elevon (CH1)
	pwm0.Set(pwmCh1, leftPulse)
	// Set the PWM duty cycle for right elevon (CH2)
	pwm0.Set(pwmCh2, rightPulse)
}

// Helper function to set the ESC's PWM output using a global variable.
func setESC(pulseWidth uint32) {
	// The ESC's channel is typically CH3
	pwm1.Set(pwmCh3, pulseWidth)
}

// Helper functions for reading IMU data, processing it, and calibration.
func readLSMData() {
	// Read raw sensor data from the IMU
	rawAccelX, rawAccelY, rawAccelZ, err := lsm.ReadAcceleration()
	if err != nil {
		println("Error reading acceleration:", err)
	}
	rawGyroX, rawGyroY, rawGyroZ, err := lsm.ReadRotation()
	if err != nil {
		println("Error reading rotation:", err)
	}

	// Convert raw sensor readings to standard units (rad/s and m/s^2)
	imuData.AccelX = float64(rawAccelX) * microGToMS2
	imuData.AccelY = float64(rawAccelY) * microGToMS2
	imuData.AccelZ = float64(rawAccelZ) * microGToMS2
	imuData.GyroX = float64(rawGyroX) * microDPSToRadS
	imuData.GyroY = float64(rawGyroY) * microDPSToRadS
	imuData.GyroZ = float64(rawGyroZ) * microDPSToRadS
}

// Process IMU data by applying calibration offsets and computing angles.
func processLSMData() {
	imuData.AccelX -= accelBiasX
	imuData.AccelY -= accelBiasY
	imuData.AccelZ -= accelBiasZ
	imuData.GyroX -= gyroBiasX
	imuData.GyroY -= gyroBiasY
	imuData.GyroZ -= gyroBiasZ
	imuData.Roll = imuData.rollAccel()
	imuData.Pitch = imuData.pitchAccel()
}

// Calibrate the IMU by averaging a number of samples to determine biases.
func calibrate() {
	const sampleSize = 4000

	for i := 0; i < sampleSize; i++ {
		readLSMData()
	}

	accelBiasX = accelXSum / sampleSize
	accelBiasY = accelYSum / sampleSize
	accelBiasZ = accelZSum / sampleSize
	println("Accel calibration complete. Bias X:", accelBiasX, "Bias Y:", accelBiasY, "Bias Z:", accelBiasZ)
	gyroBiasX = gyroXSum / sampleSize
	gyroBiasY = gyroYSum / sampleSize
	gyroBiasZ = gyroZSum / sampleSize
	println("Gyro calibration complete. Bias X:", gyroBiasX, "Bias Y:", gyroBiasY, "Bias Z:", gyroBiasZ)
}
