package main

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
	imuData.Pitch = imuData.pitchAccel()
	imuData.Roll = imuData.rollAccel()
}

// Calibrate the IMU by averaging a number of samples to determine bias offsets.
// This function should be called when the aircraft is stationary and level.
func calibrate() {
	const sampleSize = 10000
	for i := sampleSize; i > 0; i-- {
		readLSMData()
		accelXSum += imuData.AccelX
		accelYSum += imuData.AccelY
		accelZSum += imuData.AccelZ
		gyroXSum += imuData.GyroX
		gyroYSum += imuData.GyroY
		gyroZSum += imuData.GyroZ
		if i%1000 == 0 {
			println(i / 1000)
		}
	}
	// println(accelXSum, accelYSum, accelZSum, gyroXSum, gyroYSum, gyroZSum)

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
func mapRange[T uint16 | uint32 | float64](value, fromMin, fromMax, toMin, toMax T) T {
	return (value-fromMin)/(fromMax-fromMin)*(toMax-toMin) + toMin
}

// setServo sets the PWM duty cycle for the aileron and elevator servos.
// It converts a pulse width in microseconds to a value relative to the PWM period.
func setServo(leftPulse, rightPulse uint32) {
	// The Period() function is not available. We use the saved period instead.
	top_value := pwm0.Top()

	// Calculate the duty cycle for the left servo.
	duty_left := uint32(uint64(leftPulse) * 1000 * uint64(top_value) / uint64(servoPeriodNs))
	pwm0.Set(pwmCh1, duty_left)

	// Calculate the duty cycle for the right servo.
	duty_right := uint32(uint64(rightPulse) * 1000 * uint64(top_value) / uint64(servoPeriodNs))
	pwm0.Set(pwmCh2, duty_right)
}

// setESC sets the PWM duty cycle for the ESC.
// It converts a pulse width in microseconds to a value relative to the PWM period.
func setESC(pulseWidth uint32) {
	// The Period() function is not available. We use the saved period instead.
	top_value := pwm1.Top()

	// Calculate the duty cycle for the ESC.
	duty := uint32(uint64(pulseWidth) * 1000 * uint64(top_value) / uint64(escPeriodNs))
	pwm1.Set(pwmCh3, duty)
}
