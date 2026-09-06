diff --git a/firmware/src/config.go b/firmware/src/config.go
index ed46493..070e785 100644
--- a/firmware/src/config.go
+++ b/firmware/src/config.go
@@ -31,10 +31,10 @@ const (
 	ThrottleChannel   = 2 // CH3 (Throttle)
 	YawChannel        = 3 // CH4 (Yaw)
 	ArmChannel        = 4 // CH5
-	ManualModeChannel = 5 // CH6
+	ManualModeChannel = 7 // CH6
 
-	TuningChannelA = 6 // CH7 for tuning parameter A
-	TuningChannelB = 7 // CH8 for tuning parameter B
+	TuningChannelA = 5 // CH7 for tuning parameter A
+	TuningChannelB = 6 // CH8 for tuning parameter B
 	TuningChannelC = 8 // CH9 for tuning parameter C
 	TuningChannelD = 9 // CH10 for tuning parameter D
 )
@@ -43,11 +43,11 @@ const (
 const (
 	// Use DShot for ESC throttle control, so no traditional PWM channel for throttle is needed.
 	// DSHOT_MODE can be 150, 300, or 600 (representing DShot150, DShot300, DShot600)
-	DSHOT      = true
+	DSHOT      = false
 	DSHOT_MODE = 600
 
 	// PWM Frequencies
-	SERVO_PWM_FREQUENCY = 50  // 50Hz for analog servos
+	SERVO_PWM_FREQUENCY = 100 // 50Hz for analog servos
 	ESC_PWM_FREQUENCY   = 400 // 400Hz for high-speed ESC
 
 	DEADBAND      = 10
@@ -98,16 +98,27 @@ const (
 	MAX_ROLL_RATE_DEG  = 600
 	MAX_YAW_RATE_DEG   = 100
 
+	// Maximum stabilized-mode angle commands from stick (degrees)
+	// Stick at full deflection commands this many degrees of desired tilt.
+	MAX_PITCH_ANGLE_DEG float32 = 45
+	MAX_ROLL_ANGLE_DEG  float32 = 60
+
 	PID_WEIGHT float32 = 0.5
 
-	// Low-pass filter level for gyro data (higher = smoother but more lag)
-	// 0 = no filtering, 1 = mild (>> 2), 2 = stronger (>> 3)
-	LPF_BITSHIFT_LEVEL = 1
+	// Low-pass filter levels (applied independently to accel and gyro).
+	// 0 = no filtering, 1 = mild (α≈0.25, >>2), 2 = stronger (α≈0.125, >>3)
+	//
+	// Gyros are low-noise — keep at 0 for maximum stabilization responsiveness.
+	// Accelerometers are vibration-prone — heavier filtering is appropriate.
+	LPF_ACCEL_LEVEL = 2
+	LPF_GYRO_LEVEL  = 0
 
 	// PID Gains
-	pP, pI, pD float32 = 1.0, 0.1, 0.01
-	rP, rI, rD float32 = 1.0, 0.1, 0.01
-	yP, yI, yD float32 = 0.5, 0.05, 0.01 // Yaw PID gains
+	// D gain is intentionally 0: the derivative term amplifies noise by 1/dt (×200 at 200Hz).
+	// Tune P and I first; only add D if oscillation persists after P/I are dialled in.
+	pP, pI, pD float32 = 1.0, 0.05, 0.0
+	rP, rI, rD float32 = 1.0, 0.05, 0.0
+	yP, yI, yD float32 = 0.5, 0.02, 0.0 // Yaw PID gains
 )
 
 // --- Tuning Parameters ---
diff --git a/firmware/src/hardware_adapters.go b/firmware/src/hardware_adapters.go
index 01587fc..f6d5b5e 100644
--- a/firmware/src/hardware_adapters.go
+++ b/firmware/src/hardware_adapters.go
@@ -144,12 +144,12 @@ func (a *LSM6DS3TRAdapter) Connected() bool {
 	return a.dev.Connected()
 }
 
-func (a *LSM6DS3TRAdapter) ReadAccel() (x, y, z int16, err error) {
+func (a *LSM6DS3TRAdapter) ReadAccel() (x, y, z int32, err error) {
 	rawX, rawY, rawZ, errAccel := a.dev.ReadAcceleration()
-	return int16(rawX >> 8), int16(rawY >> 8), int16(rawZ >> 8), errAccel
+	return rawX, rawY, rawZ, errAccel
 }
 
-func (a *LSM6DS3TRAdapter) ReadGyro() (x, y, z int16, err error) {
+func (a *LSM6DS3TRAdapter) ReadGyro() (x, y, z int32, err error) {
 	rawX, rawY, rawZ, errGyro := a.dev.ReadRotation()
-	return int16(rawX >> 8), int16(rawY >> 8), int16(rawZ >> 8), errGyro
+	return rawX, rawY, rawZ, errGyro
 }
diff --git a/firmware/src/helpers.go b/firmware/src/helpers.go
index c2f5482..a0c3c5b 100644
--- a/firmware/src/helpers.go
+++ b/firmware/src/helpers.go
@@ -7,8 +7,8 @@ func readIMUData() {
 	}
 
 	var (
-		rawAccelX, rawAccelY, rawAccelZ int16
-		rawGyroX, rawGyroY, rawGyroZ    int16
+		rawAccelX, rawAccelY, rawAccelZ int32
+		rawGyroX, rawGyroY, rawGyroZ    int32
 
 		err error
 	)
@@ -31,40 +31,64 @@ func readIMUData() {
 	imuData.rawGyroZ = rawGyroZ
 }
 
-// applyLPF applies a simple low-pass filter to the raw IMU data to reduce noise.
-func applyLPF(rawAccelXout, rawAccelYout, rawAccelZout, rawGyroXout, rawGyroYout, rawGyroZout int16) (filteredAccelX, filteredAccelY, filteredAccelZ int32, filteredGyroX, filteredGyroY, filteredGyroZ int32) {
-	// Low-pass filter
-	switch LPF_BITSHIFT_LEVEL {
+// applyLPF applies independent exponential moving-average filters to accel and gyro.
+// Accel uses LPF_ACCEL_LEVEL (heavier, vibration is high-frequency noise).
+// Gyro uses LPF_GYRO_LEVEL (lighter/none, gyros are low-noise and need fast response).
+var (
+	lpfAccelX, lpfAccelY, lpfAccelZ int32
+	lpfGyroX, lpfGyroY, lpfGyroZ    int32
+)
+
+func applyLPF(rawAccelXout, rawAccelYout, rawAccelZout, rawGyroXout, rawGyroYout, rawGyroZout int32) (filteredAccelX, filteredAccelY, filteredAccelZ int32, filteredGyroX, filteredGyroY, filteredGyroZ int32) {
+	// --- Accelerometer filter ---
+	switch LPF_ACCEL_LEVEL {
 	case 0:
-		// No filtering
-		filteredAccelX = int32(rawAccelXout)
-		filteredAccelY = int32(rawAccelYout)
-		filteredAccelZ = int32(rawAccelZout)
-		filteredGyroX = int32(rawGyroXout)
-		filteredGyroY = int32(rawGyroYout)
-		filteredGyroZ = int32(rawGyroZout)
-	case 1:
-		// >> 2 shift
-		filteredAccelX = filteredAccelX - (filteredAccelX >> 2) + (int32(rawAccelXout) >> 2)
-		filteredAccelY = filteredAccelY - (filteredAccelY >> 2) + (int32(rawAccelYout) >> 2)
-		filteredAccelZ = filteredAccelZ - (filteredAccelZ >> 2) + (int32(rawAccelZout) >> 2)
-		filteredGyroX = filteredGyroX - (filteredGyroX >> 2) + (int32(rawGyroXout) >> 2)
-		filteredGyroY = filteredGyroY - (filteredGyroY >> 2) + (int32(rawGyroYout) >> 2)
-		filteredGyroZ = filteredGyroZ - (filteredGyroZ >> 2) + (int32(rawGyroZout) >> 2)
-	case 2:
-		// >> 3 shift
-		filteredAccelX = filteredAccelX - (filteredAccelX >> 3) + (int32(rawAccelXout) >> 3)
-		filteredAccelY = filteredAccelY - (filteredAccelY >> 3) + (int32(rawAccelYout) >> 3)
-		filteredAccelZ = filteredAccelZ - (filteredAccelZ >> 3) + (int32(rawAccelZout) >> 3)
-		filteredGyroX = filteredGyroX - (filteredGyroX >> 3) + (int32(rawGyroXout) >> 3)
-		filteredGyroY = filteredGyroY - (filteredGyroY >> 3) + (int32(rawGyroYout) >> 3)
-		filteredGyroZ = filteredGyroZ - (filteredGyroZ >> 3) + (int32(rawGyroZout) >> 3)
+		filteredAccelX = rawAccelXout
+		filteredAccelY = rawAccelYout
+		filteredAccelZ = rawAccelZout
+	case 1: // α ≈ 0.25 (>> 2)
+		lpfAccelX = lpfAccelX - (lpfAccelX >> 2) + (rawAccelXout >> 2)
+		lpfAccelY = lpfAccelY - (lpfAccelY >> 2) + (rawAccelYout >> 2)
+		lpfAccelZ = lpfAccelZ - (lpfAccelZ >> 2) + (rawAccelZout >> 2)
+		filteredAccelX, filteredAccelY, filteredAccelZ = lpfAccelX, lpfAccelY, lpfAccelZ
+	case 2: // α ≈ 0.125 (>> 3)
+		lpfAccelX = lpfAccelX - (lpfAccelX >> 3) + (rawAccelXout >> 3)
+		lpfAccelY = lpfAccelY - (lpfAccelY >> 3) + (rawAccelYout >> 3)
+		lpfAccelZ = lpfAccelZ - (lpfAccelZ >> 3) + (rawAccelZout >> 3)
+		filteredAccelX, filteredAccelY, filteredAccelZ = lpfAccelX, lpfAccelY, lpfAccelZ
+	default:
+		filteredAccelX = rawAccelXout
+		filteredAccelY = rawAccelYout
+		filteredAccelZ = rawAccelZout
 	}
+
+	// --- Gyroscope filter ---
+	switch LPF_GYRO_LEVEL {
+	case 0:
+		filteredGyroX = rawGyroXout
+		filteredGyroY = rawGyroYout
+		filteredGyroZ = rawGyroZout
+	case 1: // α ≈ 0.25 (>> 2)
+		lpfGyroX = lpfGyroX - (lpfGyroX >> 2) + (rawGyroXout >> 2)
+		lpfGyroY = lpfGyroY - (lpfGyroY >> 2) + (rawGyroYout >> 2)
+		lpfGyroZ = lpfGyroZ - (lpfGyroZ >> 2) + (rawGyroZout >> 2)
+		filteredGyroX, filteredGyroY, filteredGyroZ = lpfGyroX, lpfGyroY, lpfGyroZ
+	case 2: // α ≈ 0.125 (>> 3)
+		lpfGyroX = lpfGyroX - (lpfGyroX >> 3) + (rawGyroXout >> 3)
+		lpfGyroY = lpfGyroY - (lpfGyroY >> 3) + (rawGyroYout >> 3)
+		lpfGyroZ = lpfGyroZ - (lpfGyroZ >> 3) + (rawGyroZout >> 3)
+		filteredGyroX, filteredGyroY, filteredGyroZ = lpfGyroX, lpfGyroY, lpfGyroZ
+	default:
+		filteredGyroX = rawGyroXout
+		filteredGyroY = rawGyroYout
+		filteredGyroZ = rawGyroZout
+	}
+
 	return
 }
 
 // applyOrientation maps the raw IMU data to the correct axes based on the board orientation configuration.
-func applyOrientation(rawAccelX, rawAccelY, rawAccelZ, rawGyroX, rawGyroY, rawGyroZ int16) (rawAccelXout, rawAccelYout, rawAccelZout, rawGyroXout, rawGyroYout, rawGyroZout int16) {
+func applyOrientation(rawAccelX, rawAccelY, rawAccelZ, rawGyroX, rawGyroY, rawGyroZ int32) (rawAccelXout, rawAccelYout, rawAccelZout, rawGyroXout, rawGyroYout, rawGyroZout int32) {
 	// Map imu data based on board orientation configuration
 	switch ORIENTATION {
 	case 1: // CW90
@@ -135,17 +159,27 @@ func processIMUData() {
 
 // Calibrate the IMU by averaging a number of samples to determine bias offsets.
 // This function should be called when the aircraft is stationary and level.
+// Accumulates fully-processed (oriented, filtered, scaled) values so that bias
+// offsets are in the same units as the values used during flight.
 func calibrateIMU() {
 	const sampleSize = 10000
 	var (
-		accelXSum, accelYSum, accelZSum float32 = 0, 0, 0
-		gyroXSum, gyroYSum, gyroZSum    float32 = 0, 0, 0
+		gyroXSum, gyroYSum, gyroZSum float32
 	)
+
+	// Clear any previous biases before sampling so they do not skew the average
+	imuData.AccelXBias = 0
+	imuData.AccelYBias = 0
+	imuData.AccelZBias = 0
+	imuData.GyroXBias = 0
+	imuData.GyroYBias = 0
+	imuData.GyroZBias = 0
+
 	for i := sampleSize; i > 0; i-- {
+		// readIMUData + processIMUData gives fully oriented, filtered, and
+		// scaled AccelX/GyroX values (in m/s² and rad/s respectively).
 		readIMUData()
-		accelXSum += imuData.AccelX
-		accelYSum += imuData.AccelY
-		accelZSum += imuData.AccelZ
+		processIMUData()
 		gyroXSum += imuData.GyroX
 		gyroYSum += imuData.GyroY
 		gyroZSum += imuData.GyroZ
@@ -153,15 +187,10 @@ func calibrateIMU() {
 			println(i / 1000)
 		}
 	}
-	// println(accelXSum, accelYSum, accelZSum, gyroXSum, gyroYSum, gyroZSum)
-
-	imuData.AccelXBias = float32(accelXSum/sampleSize) * microGToMS2
-	imuData.AccelYBias = float32(accelYSum/sampleSize) * microGToMS2
-	imuData.AccelZBias = float32(accelZSum/sampleSize) * microGToMS2
-	println("Accel calibration complete. Bias X:", imuData.AccelXBias, "Bias Y:", imuData.AccelYBias, "Bias Z:", imuData.AccelZBias)
-	imuData.GyroXBias = float32(gyroXSum/sampleSize) * microDPSToRadS
-	imuData.GyroYBias = float32(gyroYSum/sampleSize) * microDPSToRadS
-	imuData.GyroZBias = float32(gyroZSum/sampleSize) * microDPSToRadS
+
+	imuData.GyroXBias = gyroXSum / sampleSize
+	imuData.GyroYBias = gyroYSum / sampleSize
+	imuData.GyroZBias = gyroZSum / sampleSize
 	println("Gyro calibration complete. Bias X:", imuData.GyroXBias, "Bias Y:", imuData.GyroYBias, "Bias Z:", imuData.GyroZBias)
 }
 
diff --git a/firmware/src/imu.go b/firmware/src/imu.go
index aaacd11..bdc47cd 100644
--- a/firmware/src/imu.go
+++ b/firmware/src/imu.go
@@ -6,12 +6,12 @@ import (
 
 // IMU Struct
 type IMU struct {
-	rawAccelX int16
-	rawAccelY int16
-	rawAccelZ int16
-	rawGyroX  int16
-	rawGyroY  int16
-	rawGyroZ  int16
+	rawAccelX int32
+	rawAccelY int32
+	rawAccelZ int32
+	rawGyroX  int32
+	rawGyroY  int32
+	rawGyroZ  int32
 
 	AccelXBias float32
 	AccelYBias float32
@@ -34,7 +34,7 @@ type IMU struct {
 
 // pitchAccel() calculates the pitch angle in radians from accelerometer data.
 func (i *IMU) pitchAccel() float32 {
-	return math.Atan2(-i.AccelX, math.Sqrt(i.AccelY*i.AccelY+i.AccelZ*i.AccelZ))
+	return math.Atan2(i.AccelX, math.Sqrt(i.AccelY*i.AccelY+i.AccelZ*i.AccelZ))
 }
 
 // rollAccel() calculates the roll angle in radians from accelerometer data.
@@ -42,12 +42,11 @@ func (i *IMU) rollAccel() float32 {
 	return math.Atan2(i.AccelY, i.AccelZ)
 }
 
-// yawGyro() calculates the yaw ratee in m/s^2 from gyroscope data.
+// yawGyro() calculates the yaw rate in rad/s from gyroscope data.
 func (i *IMU) yawGyro() float32 {
-	// No Motion No Integration (NMNI) - if gyroZ is below bias, treat as zero to prevent drift when stationary
 	// Apply deadzone to prevent drift when stationary
 	if math.Abs(i.GyroZ) < math.Abs(i.GyroZBias) {
 		return 0
 	}
-	return i.GyroZ * dt
+	return i.GyroZ
 }
diff --git a/firmware/src/interfaces.go b/firmware/src/interfaces.go
index ed0bbd0..a98e8ac 100644
--- a/firmware/src/interfaces.go
+++ b/firmware/src/interfaces.go
@@ -52,6 +52,6 @@ type LEDUpdater interface {
 type IMUDevice interface {
 	Configure(config interface{}) error
 	Connected() bool
-	ReadAccel() (x, y, z int16, err error)
-	ReadGyro() (x, y, z int16, err error)
+	ReadAccel() (x, y, z int32, err error)
+	ReadGyro() (x, y, z int32, err error)
 }
diff --git a/firmware/src/kalman.go b/firmware/src/kalman.go
index cbac72e..39cc136 100644
--- a/firmware/src/kalman.go
+++ b/firmware/src/kalman.go
@@ -29,15 +29,20 @@ func NewKalmanFilter(dt float32) *KalmanFilter {
 	r.Set(0, 0, 0.5) // Pitch Accel noise
 	r.Set(1, 1, 0.5) // Roll Accel noise
 
+	// H maps the 3-element state [pitch, roll, yaw] to the 2-element
+	// accelerometer measurement [accelPitch, accelRoll].
+	// H[0,0]=1 observes pitch; H[1,1]=1 observes roll.
+	h := NewMatrix2x3()
+	h.Set(0, 0, 1) // accelPitch  ← pitch state
+	h.Set(1, 1, 1) // accelRoll   ← roll state
+
 	return &KalmanFilter{
 		X: x,
 		P: Identity3x3(),
 		Q: q,
 		R: r,
 		F: Identity3x3(),
-		// H maps [pitch, roll, yaw] -> [accelPitch, accelRoll]
-		// Pitch is observed at index 0, Roll at index 1.
-		H:  NewMatrix2x3(),
+		H:  h,
 		dt: dt,
 	}
 }
diff --git a/firmware/src/main.go b/firmware/src/main.go
index 102aff7..97e9ec8 100644
--- a/firmware/src/main.go
+++ b/firmware/src/main.go
@@ -8,7 +8,7 @@ import (
 	"tinygo.org/x/drivers/lsm6ds3tr"
 )
 
-const Version = "0.3.0"
+const Version = "0.4.0"
 
 var (
 	watchdog = machine.Watchdog
@@ -43,11 +43,15 @@ const (
 	MIN_PULSE_WIDTH_US = 1000
 	MAX_PULSE_WIDTH_US = 2000
 
-	// Calculated constants for PID control
+	// Calculated rate constants for manual/rate mode
 	MAX_ROLL_RATE  float32 = MAX_ROLL_RATE_DEG * math.Pi / 180
 	MAX_PITCH_RATE float32 = MAX_PITCH_RATE_DEG * math.Pi / 180
 	MAX_YAW_RATE   float32 = MAX_YAW_RATE_DEG * math.Pi / 180
 
+	// Calculated angle constants for stabilized mode (radians)
+	MAX_PITCH_ANGLE float32 = MAX_PITCH_ANGLE_DEG * math.Pi / 180
+	MAX_ROLL_ANGLE  float32 = MAX_ROLL_ANGLE_DEG * math.Pi / 180
+
 	// Fail-safe constants
 	FAILSAFE_TIMEOUT_MS = 500
 
@@ -104,126 +108,143 @@ func main() {
 	watchdog.Start()
 
 	for {
-		select {
-		case packet := <-packetChan:
-			LastPacketTime = time.Now()
-			Channels = processReceiverPacket(packet)
-
-		default:
-			<-ticker.C
-
-			if time.Since(LastPacketTime).Milliseconds() > FAILSAFE_TIMEOUT_MS && flightState != FAILSAFE {
-				flightState = FAILSAFE
+		// Draining all pending receiver packets non-blockingly to update channel inputs
+	drainLoop:
+		for {
+			select {
+			case packet := <-packetChan:
+				LastPacketTime = time.Now()
+				Channels = processReceiverPacket(packet)
+			default:
+				break drainLoop
 			}
+		}
 
-			readIMUData()
-			processIMUData()
-
-			switch flightState {
-			case ESC_CALIBRATION:
-				// Pass throttle directly to ESC for calibration
-				// ESC stays in 'high' mode until user drops stick
-				currentThrottle := uint32(Channels[ThrottleChannel])
-				setESC(currentThrottle)
-
-				if currentThrottle < (MIN_PULSE_WIDTH_US + 50) {
-					flightState = IMU_CALIBRATION
-				}
+		<-ticker.C
 
-			case IMU_CALIBRATION:
-				hw.LED.SetState(CALIBRATE)
-				hw.LED.Update()
-				setAllServos(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
-				setESC(MIN_PULSE_WIDTH_US)
+		if time.Since(LastPacketTime).Milliseconds() > FAILSAFE_TIMEOUT_MS && flightState != FAILSAFE {
+			flightState = FAILSAFE
+		}
 
-				time.Sleep(time.Second)
-				calibrateIMU()
-				flightState = FLIGHT_MODE
+		readIMUData()
+		processIMUData()
 
-			case FLIGHT_MODE:
-				armed = Channels[ArmChannel] > HIGH_RX_VALUE
+		switch flightState {
+		case ESC_CALIBRATION:
+			// Pass throttle directly to ESC for calibration
+			// ESC stays in 'high' mode until user drops stick
+			currentThrottle := uint32(Channels[ThrottleChannel])
+			setESC(currentThrottle)
 
-				// Map RC inputs to Rates
-				pitchInput := mapRange(float32(Channels[ElevatorChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_PITCH_RATE, MAX_PITCH_RATE)
-				rollInput := mapRange(float32(Channels[AileronChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_ROLL_RATE, MAX_ROLL_RATE)
-				yawInput := mapRange(float32(Channels[YawChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_YAW_RATE, MAX_YAW_RATE)
+			if currentThrottle < (MIN_PULSE_WIDTH_US + 50) {
+				flightState = IMU_CALIBRATION
+			}
 
-				var pitchMix, rollMix, yawMix float32
+		case IMU_CALIBRATION:
+			hw.LED.SetState(CALIBRATE)
+			hw.LED.Update()
+			setAllServos(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
+			setESC(MIN_PULSE_WIDTH_US)
 
-				if Channels[ManualModeChannel] > HIGH_RX_VALUE {
-					// Manual Mode: Bypass PID, feed stick rates directly into Mixer
-					pitchMix = pitchInput
-					rollMix = rollInput
-					yawMix = yawInput
+			time.Sleep(time.Second)
+			calibrateIMU()
+			flightState = FLIGHT_MODE
+
+		case FLIGHT_MODE:
+			armed = Channels[ArmChannel] > HIGH_RX_VALUE
+
+			// Map RC inputs to normalized [-1.0, 1.0] range
+			pitchStick := mapRange(float32(Channels[ElevatorChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -1.0, 1.0)
+			rollStick := mapRange(float32(Channels[AileronChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -1.0, 1.0)
+			yawStick := mapRange(float32(Channels[YawChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -1.0, 1.0)
+
+			var pitchMix, rollMix, yawMix float32
+
+			if Channels[ManualModeChannel] > HIGH_RX_VALUE {
+				// Manual / Direct Pass-through Mode: feed stick directly into mixer [-1.0, 1.0]
+				pitchMix = pitchStick
+				rollMix = rollStick
+				yawMix = yawStick
+
+				pitchPID.Reset()
+				rollPID.Reset()
+				yawPID.Reset()
+			} else {
+				// Stabilized / Angle Mode:
+				//   1. Predict state with gyro
+				//   2. Update state with accel
+				//   3. Compute angle errors (desired_angle - estimated_angle)
+				kf.Predict(imuData.GyroX, imuData.GyroY, imuData.GyroZ)
+				kf.Update(imuData.Pitch, imuData.Roll)
+
+				// Desired angles mapped from stick inputs [-MAX_ANGLE, +MAX_ANGLE]
+				desiredPitch := pitchStick * MAX_PITCH_ANGLE
+				desiredRoll := rollStick * MAX_ROLL_ANGLE
+
+				// Read Kalman angle estimates [pitch=0, roll=1, yaw=2]
+				estPitch := kf.X.At(0, 0)
+				estRoll := kf.X.At(1, 0)
+
+				// Angle errors normalized by max angles so full error = 1.0
+				pitchError := (desiredPitch - estPitch) / MAX_PITCH_ANGLE
+				rollError := (desiredRoll - estRoll) / MAX_ROLL_ANGLE
+				// Yaw remains rate-based (desired rate - gyro rate) normalized by MAX_YAW_RATE
+				yawError := (yawStick*MAX_YAW_RATE - imuData.GyroZ) / MAX_YAW_RATE
+
+				pitchMix = constrain(pitchPID.Update(pitchError, dt)*PID_WEIGHT, -1.0, 1.0)
+				rollMix = constrain(rollPID.Update(rollError, dt)*PID_WEIGHT, -1.0, 1.0)
+				yawMix = constrain(yawPID.Update(yawError, dt)*PID_WEIGHT, -1.0, 1.0)
+			}
 
-					pitchPID.Reset()
-					rollPID.Reset()
-					yawPID.Reset()
+			// 1. Process Airframe Mixer (outputs in [-1.0, 1.0] range)
+			s1, s2, s4, s5, s6 := ApplyMixer(pitchMix, rollMix, yawMix)
+
+			// 2. Map Output Scale to Pulse Widths [-1.0, 1.0] -> [1000us, 2000us]
+			s1 = mapRange(s1, -1.0, 1.0, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
+			s2 = mapRange(s2, -1.0, 1.0, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
+			s4 = mapRange(s4, -1.0, 1.0, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
+			s5 = mapRange(s5, -1.0, 1.0, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
+			s6 = mapRange(s6, -1.0, 1.0, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
+
+			// 3. Apply Reversals
+			s1 = applyReversal(s1, SERVO1_REVERSE, float32(SERVO1_MIN), float32(SERVO1_MAX))
+			s2 = applyReversal(s2, SERVO2_REVERSE, float32(SERVO2_MIN), float32(SERVO2_MAX))
+			s4 = applyReversal(s4, SERVO4_REVERSE, float32(SERVO4_MIN), float32(SERVO4_MAX))
+			s5 = applyReversal(s5, SERVO5_REVERSE, float32(SERVO5_MIN), float32(SERVO5_MAX))
+			s6 = applyReversal(s6, SERVO6_REVERSE, float32(SERVO6_MIN), float32(SERVO6_MAX))
+
+			// 4. Add Trims and Constrain
+			pulse1 := uint32(constrain(s1+float32(SERVO1_SUBTRIM), float32(SERVO1_MIN), float32(SERVO1_MAX)))
+			pulse2 := uint32(constrain(s2+float32(SERVO2_SUBTRIM), float32(SERVO2_MIN), float32(SERVO2_MAX)))
+			pulse4 := uint32(constrain(s4+float32(SERVO4_SUBTRIM), float32(SERVO4_MIN), float32(SERVO4_MAX)))
+			pulse5 := uint32(constrain(s5+float32(SERVO5_SUBTRIM), float32(SERVO5_MIN), float32(SERVO5_MAX)))
+			pulse6 := uint32(constrain(s6+float32(SERVO6_SUBTRIM), float32(SERVO6_MIN), float32(SERVO6_MAX)))
+
+			setAllServos(pulse1, pulse2, pulse4, pulse5, pulse6)
+
+			if armed {
+				if DSHOT {
+					dshotThrottle := MapThrottle(Channels[ThrottleChannel])
+					hw.DShot.SendThrottle(dshotThrottle, false)
 				} else {
-					// Stabilized Mode: 3-Axis Kalman & PID
-					kf.Predict(imuData.GyroX, imuData.GyroY, imuData.GyroZ)
-					kf.Update(imuData.Pitch, imuData.Roll)
-
-					// Calculate errors between desired rates and actual rates for PID
-					pitchError := pitchInput - imuData.Pitch
-					rollError := rollInput - imuData.Roll
-					yawError := yawInput - imuData.Yaw
-
-					pitchMix = pitchPID.Update(pitchError, dt) * PID_WEIGHT
-					rollMix = rollPID.Update(rollError, dt) * PID_WEIGHT
-					yawMix = yawPID.Update(yawError, dt) * PID_WEIGHT
+					setESC(uint32(Channels[ThrottleChannel]))
 				}
-
-				// 1. Process Airframe Mixer
-				s1, s2, s4, s5, s6 := ApplyMixer(pitchMix, rollMix, yawMix)
-
-				// 2. Map Output Scale to Pulse Widths (Assuming Mixer outputs roughly -MAX_ROLL to +MAX_ROLL)
-				s1 = mapRange(s1, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
-				s2 = mapRange(s2, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
-				s4 = mapRange(s4, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
-				s5 = mapRange(s5, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
-				s6 = mapRange(s6, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
-
-				// 3. Apply Reversals
-				s1 = applyReversal(s1, SERVO1_REVERSE, float32(SERVO1_MIN), float32(SERVO1_MAX))
-				s2 = applyReversal(s2, SERVO2_REVERSE, float32(SERVO2_MIN), float32(SERVO2_MAX))
-				s4 = applyReversal(s4, SERVO4_REVERSE, float32(SERVO4_MIN), float32(SERVO4_MAX))
-				s5 = applyReversal(s5, SERVO5_REVERSE, float32(SERVO5_MIN), float32(SERVO5_MAX))
-				s6 = applyReversal(s6, SERVO6_REVERSE, float32(SERVO6_MIN), float32(SERVO6_MAX))
-
-				// 4. Add Trims and Constrain
-				pulse1 := uint32(constrain(s1+float32(SERVO1_SUBTRIM), float32(SERVO1_MIN), float32(SERVO1_MAX)))
-				pulse2 := uint32(constrain(s2+float32(SERVO2_SUBTRIM), float32(SERVO2_MIN), float32(SERVO2_MAX)))
-				pulse4 := uint32(constrain(s4+float32(SERVO4_SUBTRIM), float32(SERVO4_MIN), float32(SERVO4_MAX)))
-				pulse5 := uint32(constrain(s5+float32(SERVO5_SUBTRIM), float32(SERVO5_MIN), float32(SERVO5_MAX)))
-				pulse6 := uint32(constrain(s6+float32(SERVO6_SUBTRIM), float32(SERVO6_MIN), float32(SERVO6_MAX)))
-
-				setAllServos(pulse1, pulse2, pulse4, pulse5, pulse6)
-
-				if armed {
-					if DSHOT {
-						dshotThrottle := MapThrottle(Channels[ThrottleChannel])
-						hw.DShot.SendThrottle(dshotThrottle, false)
-					} else {
-						setESC(uint32(Channels[ThrottleChannel]))
-					}
+			} else {
+				if DSHOT {
+					hw.DShot.SendThrottle(0, false) // Send zero throttle command to disarm ESC safely
 				} else {
-					if DSHOT {
-						hw.DShot.SendThrottle(0, false) // Send zero throttle command to disarm ESC safely
-					} else {
-						setESC(MIN_PULSE_WIDTH_US)
-					}
-				}
-
-			case FAILSAFE:
-				setAllServos(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
-				setESC(MIN_PULSE_WIDTH_US)
-				if time.Since(LastPacketTime).Milliseconds() <= FAILSAFE_TIMEOUT_MS {
-					flightState = FLIGHT_MODE
+					setESC(MIN_PULSE_WIDTH_US)
 				}
 			}
 
-			watchdog.Update()
+		case FAILSAFE:
+			setAllServos(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
+			setESC(MIN_PULSE_WIDTH_US)
+			if time.Since(LastPacketTime).Milliseconds() <= FAILSAFE_TIMEOUT_MS {
+				flightState = FLIGHT_MODE
+			}
 		}
+
+		watchdog.Update()
 	}
 }
