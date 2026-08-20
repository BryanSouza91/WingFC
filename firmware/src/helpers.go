package main

// Read raw IMU data from the LSM6DS3TR sensor and apply a low-pass filter.
func readIMUData() {
	if hw == nil || hw.IMU == nil {
		return
	}

	var (
		rawAccelX, rawAccelY, rawAccelZ int32
		rawGyroX, rawGyroY, rawGyroZ    int32

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

// applyLPF applies independent exponential moving-average filters to accel and gyro.
// Accel uses LPF_ACCEL_LEVEL (heavier, vibration is high-frequency noise).
// Gyro uses LPF_GYRO_LEVEL (lighter/none, gyros are low-noise and need fast response).
var (
	lpfAccelX, lpfAccelY, lpfAccelZ int32
	lpfGyroX, lpfGyroY, lpfGyroZ    int32
)

func applyLPF(rawAccelXout, rawAccelYout, rawAccelZout, rawGyroXout, rawGyroYout, rawGyroZout int32) (filteredAccelX, filteredAccelY, filteredAccelZ int32, filteredGyroX, filteredGyroY, filteredGyroZ int32) {
	// --- Accelerometer filter ---
	switch LPF_ACCEL_LEVEL {
	case 0:
		filteredAccelX = rawAccelXout
		filteredAccelY = rawAccelYout
		filteredAccelZ = rawAccelZout
	case 1: // α ≈ 0.25 (>> 2)
		lpfAccelX = lpfAccelX - (lpfAccelX >> 2) + (rawAccelXout >> 2)
		lpfAccelY = lpfAccelY - (lpfAccelY >> 2) + (rawAccelYout >> 2)
		lpfAccelZ = lpfAccelZ - (lpfAccelZ >> 2) + (rawAccelZout >> 2)
		filteredAccelX, filteredAccelY, filteredAccelZ = lpfAccelX, lpfAccelY, lpfAccelZ
	case 2: // α ≈ 0.125 (>> 3)
		lpfAccelX = lpfAccelX - (lpfAccelX >> 3) + (rawAccelXout >> 3)
		lpfAccelY = lpfAccelY - (lpfAccelY >> 3) + (rawAccelYout >> 3)
		lpfAccelZ = lpfAccelZ - (lpfAccelZ >> 3) + (rawAccelZout >> 3)
		filteredAccelX, filteredAccelY, filteredAccelZ = lpfAccelX, lpfAccelY, lpfAccelZ
	default:
		filteredAccelX = rawAccelXout
		filteredAccelY = rawAccelYout
		filteredAccelZ = rawAccelZout
	}

	// --- Gyroscope filter ---
	switch LPF_GYRO_LEVEL {
	case 0:
		filteredGyroX = rawGyroXout
		filteredGyroY = rawGyroYout
		filteredGyroZ = rawGyroZout
	case 1: // α ≈ 0.25 (>> 2)
		lpfGyroX = lpfGyroX - (lpfGyroX >> 2) + (rawGyroXout >> 2)
		lpfGyroY = lpfGyroY - (lpfGyroY >> 2) + (rawGyroYout >> 2)
		lpfGyroZ = lpfGyroZ - (lpfGyroZ >> 2) + (rawGyroZout >> 2)
		filteredGyroX, filteredGyroY, filteredGyroZ = lpfGyroX, lpfGyroY, lpfGyroZ
	case 2: // α ≈ 0.125 (>> 3)
		lpfGyroX = lpfGyroX - (lpfGyroX >> 3) + (rawGyroXout >> 3)
		lpfGyroY = lpfGyroY - (lpfGyroY >> 3) + (rawGyroYout >> 3)
		lpfGyroZ = lpfGyroZ - (lpfGyroZ >> 3) + (rawGyroZout >> 3)
		filteredGyroX, filteredGyroY, filteredGyroZ = lpfGyroX, lpfGyroY, lpfGyroZ
	default:
		filteredGyroX = rawGyroXout
		filteredGyroY = rawGyroYout
		filteredGyroZ = rawGyroZout
	}

	return
}

// applyOrientation maps the raw IMU data to the correct axes based on the board orientation configuration.
func applyOrientation(rawAccelX, rawAccelY, rawAccelZ, rawGyroX, rawGyroY, rawGyroZ int32) (rawAccelXout, rawAccelYout, rawAccelZout, rawGyroXout, rawGyroYout, rawGyroZout int32) {
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
	imuData.Yaw = imuData.yawGyro()
}

// Calibrate the IMU by averaging a number of samples to determine bias offsets.
// This function should be called when the aircraft is stationary and level.
// Accumulates fully-processed (oriented, filtered, scaled) values so that bias
// offsets are in the same units as the values used during flight.
func calibrateIMU() {
	const sampleSize = 10000
	var (
		gyroXSum, gyroYSum, gyroZSum float32
	)

	// Clear any previous biases before sampling so they do not skew the average
	imuData.AccelXBias = 0
	imuData.AccelYBias = 0
	imuData.AccelZBias = 0
	imuData.GyroXBias = 0
	imuData.GyroYBias = 0
	imuData.GyroZBias = 0

	for i := sampleSize; i > 0; i-- {
		// readIMUData + processIMUData gives fully oriented, filtered, and
		// scaled AccelX/GyroX values (in m/s² and rad/s respectively).
		readIMUData()
		processIMUData()
		gyroXSum += imuData.GyroX
		gyroYSum += imuData.GyroY
		gyroZSum += imuData.GyroZ
		if i%1000 == 0 {
			println(i / 1000)
		}
	}

	imuData.GyroXBias = gyroXSum / sampleSize
	imuData.GyroYBias = gyroYSum / sampleSize
	imuData.GyroZBias = gyroZSum / sampleSize
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

// setAllServos sets the PWM duty cycle for all 5 servo channels using their dedicated instances.
func setAllServos(servo1, servo2, servo4, servo5, servo6 uint32) {
	if hw == nil {
		return
	}

	// PWM1 handles Servos 1, 2, 4, 5
	topPWM1 := hw.PWM1.Top()
	duty1 := uint32(uint64(servo1) * 1000 * uint64(topPWM1) / uint64(hw.PeriodPWM1))
	duty2 := uint32(uint64(servo2) * 1000 * uint64(topPWM1) / uint64(hw.PeriodPWM1))
	duty4 := uint32(uint64(servo4) * 1000 * uint64(topPWM1) / uint64(hw.PeriodPWM1))
	duty5 := uint32(uint64(servo5) * 1000 * uint64(topPWM1) / uint64(hw.PeriodPWM1))

	hw.PWM1.Set(hw.pwmCh1, duty1)
	hw.PWM1.Set(hw.pwmCh2, duty2)
	hw.PWM1.Set(hw.pwmCh4, duty4)
	hw.PWM1.Set(hw.pwmCh5, duty5)

	// PWM2 handles Servo 6
	topPWM2 := hw.PWM2.Top()
	duty6 := uint32(uint64(servo6) * 1000 * uint64(topPWM2) / uint64(hw.PeriodPWM2))

	hw.PWM2.Set(hw.pwmCh6, duty6)
}

// setESC isolates ESC control to PWM0 (enabling 400Hz support)
func setESC(pulseWidth uint32) {
	if hw == nil || hw.PWM0 == nil {
		return
	}

	topPWM0 := hw.PWM0.Top()
	duty := uint32(uint64(pulseWidth) * 1000 * uint64(topPWM0) / uint64(hw.PeriodPWM0))
	hw.PWM0.Set(hw.pwmCh3, duty)
}

// applyReversal flips a servo value within its min/max range if configured.
func applyReversal(pulse float32, reverse bool, min float32, max float32) float32 {
	if reverse {
		return min + max - pulse
	}
	return pulse
}
