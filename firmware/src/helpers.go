package main

// Read raw IMU data from the LSM6DS3TR sensor and apply a low-pass filter.
func readLSMData() {
	if hw == nil || hw.IMU == nil {
		return
	}
	// Read raw sensor data from the IMU
	rawAccelX, rawAccelY, rawAccelZ, err := hw.IMU.ReadAccel()
	if err != nil {
		println("Error reading acceleration:", err)
	}
	rawGyroX, rawGyroY, rawGyroZ, err := hw.IMU.ReadGyro()
	if err != nil {
		println("Error reading rotation:", err)
	}

	var (
		filteredAccelX, filteredAccelY, filteredAccelZ int32
		filteredGyroX, filteredGyroY, filteredGyroZ    int32
	)
	
	switch LPF_BITSHIFT_LEVEL {
    case 0:
        // No filtering
        filteredAccelX = int32(rawAccelX)
        filteredAccelY = int32(rawAccelY)
		filteredAccelZ = int32(rawAccelZ)
		filteredGyroX = int32(rawGyroX)
		filteredGyroY = int32(rawGyroY)
		filteredGyroZ = int32(rawGyroZ)
    case 1:
        // >> 2 shift
        filteredAccelX = filteredAccelX - (filteredAccelX >> 2) + (int32(rawAccelX) >> 2)
        filteredAccelY = filteredAccelY - (filteredAccelY >> 2) + (int32(rawAccelY) >> 2)
		filteredAccelZ = filteredAccelZ - (filteredAccelZ >> 2) + (int32(rawAccelZ) >> 2)
		filteredGyroX = filteredGyroX - (filteredGyroX >> 2) + (int32(rawGyroX) >> 2)
		filteredGyroY = filteredGyroY - (filteredGyroY >> 2) + (int32(rawGyroY) >> 2)
		filteredGyroZ = filteredGyroZ - (filteredGyroZ >> 2) + (int32(rawGyroZ) >> 2)
	case 2:
		// >> 3 shift
		filteredAccelX = filteredAccelX - (filteredAccelX >> 3) + (int32(rawAccelX) >> 3)
		filteredAccelY = filteredAccelY - (filteredAccelY >> 3) + (int32(rawAccelY) >> 3)
		filteredAccelZ = filteredAccelZ - (filteredAccelZ >> 3) + (int32(rawAccelZ) >> 3)
		filteredGyroX = filteredGyroX - (filteredGyroX >> 3) + (int32(rawGyroX) >> 3)
		filteredGyroY = filteredGyroY - (filteredGyroY >> 3) + (int32(rawGyroY) >> 3)
		filteredGyroZ = filteredGyroZ - (filteredGyroZ >> 3) + (int32(rawGyroZ) >> 3)
	
    }


	// Low-pass filter
	 // Only convert to float at the end
    imuData.AccelX = float32(filteredAccelX) * microGToMS2
	imuData.AccelY = float32(filteredAccelY) * microGToMS2
	imuData.AccelZ = float32(filteredAccelZ) * microGToMS2
	imuData.GyroX = float32(filteredGyroX) * microDPSToRadS
	imuData.GyroY = float32(filteredGyroY) * microDPSToRadS
	imuData.GyroZ = float32(filteredGyroZ) * microDPSToRadS
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

	accelBiasX = (accelXSum / sampleSize) * microGToMS2
	accelBiasY = (accelYSum / sampleSize) * microGToMS2
	accelBiasZ = (accelZSum / sampleSize) * microGToMS2
	println("Accel calibration complete. Bias X:", accelBiasX, "Bias Y:", accelBiasY, "Bias Z:", accelBiasZ)
	gyroBiasX = (gyroXSum / sampleSize) * microDPSToRadS
	gyroBiasY = (gyroYSum / sampleSize) * microDPSToRadS
	gyroBiasZ = (gyroZSum / sampleSize) * microDPSToRadS
	println("Gyro calibration complete. Bias X:", gyroBiasX, "Bias Y:", gyroBiasY, "Bias Z:", gyroBiasZ)
}

// Helper function to constrain a value within min and max bounds.
func constrain[T uint16 | uint32 | float32](value, min, max T) T {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}

// Helper function to map a value from one range to another.
func mapRange[T uint16 | uint32 | float32](value, fromMin, fromMax, toMin, toMax T) T {
	return (value-fromMin)/(fromMax-fromMin)*(toMax-toMin) + toMin
}

// setServo sets the PWM duty cycle for the aileron and elevator servos.
// It converts a pulse width in microseconds to a value relative to the PWM period.
func setServo(leftPulse, rightPulse uint32) {
	if hw == nil || hw.ServoPWM == nil {
		return
	}
	// The Period() function is not available. We use the saved period instead.
	top_value := hw.ServoPWM.Top()

	// Calculate the duty cycle for the left servo.
	duty_left := uint32(uint64(leftPulse) * 1000 * uint64(top_value) / uint64(hw.ServoPeriod))
	hw.ServoPWM.Set(hw.pwmCh1, duty_left)

	// Calculate the duty cycle for the right servo.
	duty_right := uint32(uint64(rightPulse) * 1000 * uint64(top_value) / uint64(hw.ServoPeriod))
	hw.ServoPWM.Set(hw.pwmCh2, duty_right)
}

// setESC sets the PWM duty cycle for the ESC.
// It converts a pulse width in microseconds to a value relative to the PWM period.
func setESC(pulseWidth uint32) {
	if hw == nil || hw.ESCPWM == nil {
		return
	}
	// The Period() function is not available. We use the saved period instead.
	top_value := hw.ESCPWM.Top()

	// Calculate the duty cycle for the ESC.
	duty := uint32(uint64(pulseWidth) * 1000 * uint64(top_value) / uint64(hw.ESCPeriod))
	hw.ESCPWM.Set(hw.pwmCh3, duty)
}
