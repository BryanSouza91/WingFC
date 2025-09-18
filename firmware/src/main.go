package main

import (
	// "fmt"
	"machine"
	"math"
	"time"

	"tinygo.org/x/drivers/lsm6ds3tr"
)

// Version of the flight controller software.
const Version = "0.1.2"

// Global variables for hardware interfaces, controllers, and filters.
var (
	// Hardware interfaces
	uart = machine.DefaultUART
	i2c  = machine.I2C0
	lsm  *lsm6ds3tr.Device

	// PWM controllers and channels
	pwm0   = machine.PWM0
	pwm1   = machine.PWM1
	pwmCh1 uint8
	pwmCh2 uint8
	pwmCh3 uint8

	// Control system components
	rollPID  *PIDController
	pitchPID *PIDController
	kf       *KalmanFilter
	imuData  IMU

	// RC Channels
	Channels [NumChannels]uint16

	err error
)

// Define constants for sensor value conversions and PWM.
const (
	// Convert sensor values to radians for calculations
	microGToMS2    = 9.80665 / 1e6
	microDPSToRadS = math.Pi / (180 * 1e6)

	// PWM pulse width constants
	MIN_PULSE_WIDTH_US = 1000
	MAX_PULSE_WIDTH_US = 2000

	// RC Receiver channel value constants
	MIN_RX_VALUE     = 988
	MAX_RX_VALUE     = 2012
	NEUTRAL_RX_VALUE = 1500

	// Calculated constants for PID control
	MAX_ROLL_RATE  = MAX_ROLL_RATE_DEG * math.Pi / 180
	MAX_PITCH_RATE = MAX_PITCH_RATE_DEG * math.Pi / 180

	// --- Hardware Mappings ---
	PWM_CH1_PIN = machine.D0 // Aileron Servo
	PWM_CH2_PIN = machine.D1 // Elevator Servo
	PWM_CH3_PIN = machine.D7 // ESC (Electronic Speed Controller)
)

// main is the entry point for the TinyGo program.
func main() {
	time.Sleep(2 * time.Second) // Wait for hardware to stabilize
	println("WingFC Flight Controller - Version", Version)

	// --- Hardware Setup ---
	uart.Configure(machine.UARTConfig{
		BaudRate: 115200,
		TX:       machine.NoPin,
		RX:       machine.UART_RX_PIN,
	})
	println("UART configured for iBus receiver.")

	servoPWMConfig := machine.PWMConfig{
		Period: machine.GHz * 1 / SERVO_PWM_FREQUENCY,
	}
	if err := pwm0.Configure(servoPWMConfig); err != nil {
		println("could not configure PWM:", err)
		return
	}
	pwmCh1, err = pwm0.Channel(PWM_CH1_PIN)
	if err != nil {
		println("could not get PWM channel:", err)
		return
	}
	pwmCh2, err = pwm0.Channel(PWM_CH2_PIN)
	if err != nil {
		println("could not get PWM channel:", err)
		return
	}
	println("PWM channels for servos initialized.")

	escPWMConfig := machine.PWMConfig{
		Period: machine.GHz * 1 / ESC_PWM_FREQUENCY,
	}
	if err = pwm1.Configure(escPWMConfig); err != nil {
		println("could not configure PWM for ESC:", err)
		return
	}
	println("PWM configured for ESC.")
	pwmCh3, err = pwm1.Channel(PWM_CH3_PIN)
	if err != nil {
		println("could not get PWM channel for pin D7:", err)
		return
	}
	println("PWM configured for ESC.")

	i2c.Configure(machine.I2CConfig{
		Frequency: 400 * machine.KHz,
	})
	println("I2C configured for IMU.")

	// --- IMU Setup ---

	lsm = lsm6ds3tr.New(i2c)
	err = lsm.Configure(lsm6ds3tr.Configuration{
		AccelRange:      lsm6ds3tr.ACCEL_8G,
		AccelSampleRate: lsm6ds3tr.ACCEL_SR_104,
		GyroRange:       lsm6ds3tr.GYRO_1000DPS,
		GyroSampleRate:  lsm6ds3tr.GYRO_SR_104,
	})
	if err != nil {
		for {
			println("Failed to configure LSM6DS3TR:", err.Error())
			time.Sleep(time.Second)
		}
	}
	if !lsm.Connected() {
		println("LSM6DS3TR not connected")
		time.Sleep(time.Second)
		return
	}
	println("LSM6DS3TR initialized.")

	// Calibrate gyro to find bias
	println("Calibrating Gyro... Keep gyro still!")
	var gyroXSum, gyroYSum, gyroBiasX, gyroBiasY float64 = 0., 0., 0., 0.
	var xG, yG int32
	const sampleSize = 1000

	for i := 0; i < sampleSize; i++ {
		xG, yG, _, err = lsm.ReadRotation()
		if err != nil {
			println("Error reading gyro during calibration:", err)
			continue
		}
		gyroXSum += float64(xG) * microDPSToRadS
		gyroYSum += float64(yG) * microDPSToRadS
	}
	gyroBiasX = gyroXSum / sampleSize
	gyroBiasY = gyroYSum / sampleSize
	println("Gyro calibration complete. Bias X:", gyroBiasX, "Bias Y:", gyroBiasY)

	// --- Filter and Controller Setup ---
	dt := 0.01
	kf = NewKalmanFilter(dt)
	rollPID = NewPIDController(P, I, D)
	pitchPID = NewPIDController(P, I, D)
	println("Control system initialized.")

	// Create a channel to receive iBus packets.
	packetChan := make(chan [IBUS_PACKET_SIZE]byte)

	// Start the goroutine to read iBus packets asynchronously.
	go readIBus(packetChan)

	// Main application loop using select.
	// --- Main Loop ---
	for {
		select {
		case packet := <-packetChan:
			// A complete packet has been received.
			processIBusPacket(packet)
			// println("Received and processed a new iBus packet.")

		default:
			// Read raw sensor data from the IMU
			rawAccelX, rawAccelY, rawAccelZ, err := lsm.ReadAcceleration()
			if err != nil {
				println("Error reading acceleration:", err)
				continue
			}
			rawGyroX, rawGyroY, rawGyroZ, err := lsm.ReadRotation()
			if err != nil {
				println("Error reading rotation:", err)
				continue
			}

			// Convert raw sensor readings to standard units (rad/s and m/s^2)
			imuData.AccelX = float64(rawAccelX) * microGToMS2
			imuData.AccelY = float64(rawAccelY) * microGToMS2
			imuData.AccelZ = float64(rawAccelZ) * microGToMS2
			imuData.GyroX = float64(rawGyroX)*microDPSToRadS - gyroBiasX
			imuData.GyroY = float64(rawGyroY)*microDPSToRadS - gyroBiasY
			imuData.GyroZ = float64(rawGyroZ) * microDPSToRadS

			// Use the Kalman filter to fuse sensor data and get a stable attitude estimate.
			kf.Predict(imuData.GyroX, imuData.GyroY)
			kf.Update(imuData.pitchAccel(), imuData.rollAccel())

			// Extract the fused roll and pitch from the Kalman filter
			// currentRoll := kf.X.At(1, 0)
			// currentPitch := kf.X.At(0, 0)

			// Get desired roll and pitch rates from the RC receiver.
			desiredRollRate := mapRange(float64(Channels[0]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_ROLL_RATE, MAX_ROLL_RATE)
			desiredPitchRate := mapRange(float64(Channels[1]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_PITCH_RATE, MAX_PITCH_RATE)

			// Apply deadband to avoid small unwanted movements
			if math.Abs(desiredRollRate) < DEADBAND*math.Pi/180 {
				desiredRollRate = 0
			}
			if math.Abs(desiredPitchRate) < DEADBAND*math.Pi/180 {
				desiredPitchRate = 0
			}

			// Calculate the error for PID controllers.
			rollError := desiredRollRate - imuData.GyroX
			pitchError := desiredPitchRate - imuData.GyroY

			// Update PID controllers and get the control outputs.
			rollOutput := rollPID.Update(rollError, dt)
			pitchOutput := pitchPID.Update(pitchError, dt)

			// Combine PID outputs with a mix of raw RC input.
			leftElevon := rollOutput + pitchOutput
			rightElevon := rollOutput - pitchOutput

			// Convert control outputs to PWM pulse widths.
			leftElevon = mapRange(float64(leftElevon), -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
			rightElevon = mapRange(float64(rightElevon), -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
			leftPulse := uint32(constrain(leftElevon, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US))
			rightPulse := uint32(constrain(rightElevon, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US))

			// Set the PWM signals for the servos.
			setServoPWM(leftPulse, rightPulse)

			// Handle ESC signal from CH3
			escPulse := uint32(mapRange(float64(Channels[2]), MIN_RX_VALUE, MAX_RX_VALUE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US))
			setESC(escPulse)

			// // Print status and sensor data for debugging
			// println(desiredRollRate, rollOutput, desiredPitchRate, pitchOutput)
			// println(Channels[0], Channels[1], Channels[2])
			// println(leftPulse, rightPulse, escPulse)
			// Wait for the next loop iteration.
			time.Sleep(10 * time.Millisecond)
		}
	}
}
