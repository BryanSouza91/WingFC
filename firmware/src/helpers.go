package main

// Read raw IMU data from the LSM6DS3TR sensor and apply a low-pass filter.
func readIMUData() {
	if hw == nil || hw.IMU == nil {
		return
	}

	var (
		rawAccelX, rawAccelY, rawAccelZ int16
		rawGyroX, rawGyroY, rawGyroZ    int16

		err error
	)

	// Read raw sensor data from the IMU
	rawAccelX, rawAccelY, rawAccelZ, err = hw.IMU.ReadAccel()
	if err != nil {
		println("Error reading acceleration:", err)
	}
	rawGyroX, rawGyroY, rawGyroZ, err = hw.IMU.ReadGyro()
	if err != nil {
		println("Error reading rotation:", err)
	}

	imuData.rawAccelX = rawAccelX
	imuData.rawAccelY = rawAccelY
	imuData.rawAccelZ = rawAccelZ
	imuData.rawGyroX = rawGyroX
	imuData.rawGyroY = rawGyroY
	imuData.rawGyroZ = rawGyroZ
}

// applyLPF applies a simple low-pass filter to the raw IMU data to reduce noise.
func applyLPF(rawAccelXout, rawAccelYout, rawAccelZout, rawGyroXout, rawGyroYout, rawGyroZout int16) (filteredAccelX, filteredAccelY, filteredAccelZ int32, filteredGyroX, filteredGyroY, filteredGyroZ int32) {
	// Low-pass filter
	switch LPF_BITSHIFT_LEVEL {
	case 0:
		// No filtering
		filteredAccelX = int32(rawAccelXout)
		filteredAccelY = int32(rawAccelYout)
		filteredAccelZ = int32(rawAccelZout)
		filteredGyroX = int32(rawGyroXout)
		filteredGyroY = int32(rawGyroYout)
		filteredGyroZ = int32(rawGyroZout)
	case 1:
		// >> 2 shift
		filteredAccelX = filteredAccelX - (filteredAccelX >> 2) + (int32(rawAccelXout) >> 2)
		filteredAccelY = filteredAccelY - (filteredAccelY >> 2) + (int32(rawAccelYout) >> 2)
		filteredAccelZ = filteredAccelZ - (filteredAccelZ >> 2) + (int32(rawAccelZout) >> 2)
		filteredGyroX = filteredGyroX - (filteredGyroX >> 2) + (int32(rawGyroXout) >> 2)
		filteredGyroY = filteredGyroY - (filteredGyroY >> 2) + (int32(rawGyroYout) >> 2)
		filteredGyroZ = filteredGyroZ - (filteredGyroZ >> 2) + (int32(rawGyroZout) >> 2)
	case 2:
		// >> 3 shift
		filteredAccelX = filteredAccelX - (filteredAccelX >> 3) + (int32(rawAccelXout) >> 3)
		filteredAccelY = filteredAccelY - (filteredAccelY >> 3) + (int32(rawAccelYout) >> 3)
		filteredAccelZ = filteredAccelZ - (filteredAccelZ >> 3) + (int32(rawAccelZout) >> 3)
		filteredGyroX = filteredGyroX - (filteredGyroX >> 3) + (int32(rawGyroXout) >> 3)
		filteredGyroY = filteredGyroY - (filteredGyroY >> 3) + (int32(rawGyroYout) >> 3)
		filteredGyroZ = filteredGyroZ - (filteredGyroZ >> 3) + (int32(rawGyroZout) >> 3)
	}
	return
}

// applyOrientation maps the raw IMU data to the correct axes based on the board orientation configuration.
func applyOrientation(rawAccelX, rawAccelY, rawAccelZ, rawGyroX, rawGyroY, rawGyroZ int16) (rawAccelXout, rawAccelYout, rawAccelZout, rawGyroXout, rawGyroYout, rawGyroZout int16) {
	// Map imu data based on board orientation configuration
	switch ORIENTATION {
	case 1: // CW90
		rawAccelXout, rawAccelYout, rawAccelZout = rawAccelY, -rawAccelX, rawAccelZ
		rawGyroXout, rawGyroYout, rawGyroZout = rawGyroY, -rawGyroX, rawGyroZ
	case 2: // CW180
		rawAccelXout, rawAccelYout, rawAccelZout = -rawAccelX, -rawAccelY, rawAccelZ
		rawGyroXout, rawGyroYout, rawGyroZout = -rawGyroX, -rawGyroY, rawGyroZ
	case 3: // CW270
		rawAccelXout, rawAccelYout, rawAccelZout = -rawAccelY, rawAccelX, rawAccelZ
		rawGyroXout, rawGyroYout, rawGyroZout = -rawGyroY, rawGyroX, rawGyroZ
	case 4: // flip
		rawAccelXout, rawAccelYout, rawAccelZout = rawAccelX, -rawAccelY, -rawAccelZ
		rawGyroXout, rawGyroYout, rawGyroZout = rawGyroX, -rawGyroY, -rawGyroZ
	case 5: // flipCW90
		rawAccelXout, rawAccelYout, rawAccelZout = -rawAccelY, -rawAccelX, -rawAccelZ
		rawGyroXout, rawGyroYout, rawGyroZout = -rawGyroY, -rawGyroX, -rawGyroZ
	case 6: // flipCW180
		rawAccelXout, rawAccelYout, rawAccelZout = -rawAccelX, rawAccelY, -rawAccelZ
		rawGyroXout, rawGyroYout, rawGyroZout = -rawGyroX, rawGyroY, -rawGyroZ
	case 7: // flipCW270
		rawAccelXout, rawAccelYout, rawAccelZout = rawAccelY, rawAccelX, -rawAccelZ
		rawGyroXout, rawGyroYout, rawGyroZout = rawGyroY, rawGyroX, -rawGyroZ
	default: // default
		rawAccelXout, rawAccelYout, rawAccelZout = rawAccelX, rawAccelY, rawAccelZ
		rawGyroXout, rawGyroYout, rawGyroZout = rawGyroX, rawGyroY, rawGyroZ
	}
	return
}

// Process the raw IMU data by applying calibration offsets and computing roll/pitch angles.
func processIMUData() {
	// 1. Read raw data (already done in readIMUData)
	rawAccelX := imuData.rawAccelX
	rawAccelY := imuData.rawAccelY
	rawAccelZ := imuData.rawAccelZ
	rawGyroX := imuData.rawGyroX
	rawGyroY := imuData.rawGyroY
	rawGyroZ := imuData.rawGyroZ

	// 2. Apply orientation mapping first
	accelXout, accelYout, accelZout, gyroXout, gyroYout, gyroZout := applyOrientation(
		rawAccelX, rawAccelY, rawAccelZ, rawGyroX, rawGyroY, rawGyroZ)

	// 3. Apply LPF to oriented data
	filteredAccelX, filteredAccelY, filteredAccelZ, filteredGyroX, filteredGyroY, filteredGyroZ := applyLPF(
		accelXout, accelYout, accelZout, gyroXout, gyroYout, gyroZout)

	// 4. Convert to float with scale factors
	imuData.AccelX = float32(filteredAccelX) * microGToMS2
	imuData.AccelY = float32(filteredAccelY) * microGToMS2
	imuData.AccelZ = float32(filteredAccelZ) * microGToMS2
	imuData.GyroX = float32(filteredGyroX) * microDPSToRadS
	imuData.GyroY = float32(filteredGyroY) * microDPSToRadS
	imuData.GyroZ = float32(filteredGyroZ) * microDPSToRadS

	// 5. Apply calibration offsets and compute angles
	imuData.AccelX -= imuData.AccelXBias
	imuData.AccelY -= imuData.AccelYBias
	imuData.AccelZ -= imuData.AccelZBias
	imuData.GyroX -= imuData.GyroXBias
	imuData.GyroY -= imuData.GyroYBias
	imuData.GyroZ -= imuData.GyroZBias
	imuData.Pitch = imuData.pitchAccel()
	imuData.Roll = imuData.rollAccel()
}

// Calibrate the IMU by averaging a number of samples to determine bias offsets.
// This function should be called when the aircraft is stationary and level.
func calibrateIMU() {
	const sampleSize = 10000
	var (
		accelXSum, accelYSum, accelZSum float32 = 0, 0, 0
		gyroXSum, gyroYSum, gyroZSum    float32 = 0, 0, 0
	)
	for i := sampleSize; i > 0; i-- {
		readIMUData()
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

	imuData.AccelXBias = float32(accelXSum/sampleSize) * microGToMS2
	imuData.AccelYBias = float32(accelYSum/sampleSize) * microGToMS2
	imuData.AccelZBias = float32(accelZSum/sampleSize) * microGToMS2
	println("Accel calibration complete. Bias X:", imuData.AccelXBias, "Bias Y:", imuData.AccelYBias, "Bias Z:", imuData.AccelZBias)
	imuData.GyroXBias = float32(gyroXSum/sampleSize) * microDPSToRadS
	imuData.GyroYBias = float32(gyroYSum/sampleSize) * microDPSToRadS
	imuData.GyroZBias = float32(gyroZSum/sampleSize) * microDPSToRadS
	println("Gyro calibration complete. Bias X:", imuData.GyroXBias, "Bias Y:", imuData.GyroYBias, "Bias Z:", imuData.GyroZBias)
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
