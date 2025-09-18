package main

import (
	"fmt"
	"machine"
	"math"
	"time"

	"tinygo.org/x/drivers/lsm6ds3tr"
)

const Version = "0.1.0"

// Define constants
const (
	// Convert sensor values to radians for calculations
	// The LSM6DS3TR driver returns values in micro-g for accel and micro-dps for gyro.
	// Convert to m/s^2 and rad/s respectively.
	microGToMS2    = 9.80665 / 1e6
	microDPSToRadS = math.Pi / (180 * 1e6)

	MIN_PULSE_WIDTH_US = 1000 // 1ms pulse for full negative deflection
	MAX_PULSE_WIDTH_US = 2000 // 2ms pulse for full positive deflection

	MIN_RX_VALUE     = 988  // Minimum Rx channel value
	MAX_RX_VALUE     = 2012 // Maximum Rx channel value
	NEUTRAL_RX_VALUE = 1500 // Neutral Rx channel value

	// Calculated constants
	MAX_ROLL_RATE  = MAX_ROLL_RATE_DEG * microDPSToRadS  // radians/sec
	MAX_PITCH_RATE = MAX_PITCH_RATE_DEG * microDPSToRadS // radians/sec
)

var (
	// Global hardware instances
	i2c    = machine.I2C0
	lsm    *lsm6ds3tr.Device
	uart   = machine.DefaultUART
	pwm0   = machine.PWM0
	pwm1   = machine.PWM1
	pwmCh1 uint8
	pwmCh2 uint8
	pwmCh3 uint8

	err error

	// Global filter and controller instances
	kf       *KalmanFilter
	imu      IMU
	rollPID  *PIDController
	pitchPID *PIDController
)

func main() {
	println("WingFC Flight Controller - Version", Version)

	// --- Hardware Setup ---
	uart.Configure(machine.UARTConfig{
		BaudRate: 115200,
		TX:       machine.UART_TX_PIN,
		RX:       machine.UART_RX_PIN,
	})

	// Configure Servos PWM
	servoPWMConfig := machine.PWMConfig{
		Period: machine.GHz * 1 / SERVO_PWM_FREQUENCY,
	}
	if err := pwm0.Configure(servoPWMConfig); err != nil {
		println("could not configure PWM for servos:", err)
		return
	}
	pwmCh1, err = pwm0.Channel(PWM_CH1_PIN)
	if err != nil {
		println("could not get PWM channel for pin D0:", err)
		return
	}
	pwmCh2, err = pwm0.Channel(PWM_CH2_PIN)
	if err != nil {
		println("could not get PWM channel for pin D1:", err)
		return
	}

	// Configure ESC PWM
	escPWMConfig := machine.PWMConfig{
		Period: machine.GHz * 1 / ESC_PWM_FREQUENCY,
	}
	if err := pwm1.Configure(escPWMConfig); err != nil {
		println("could not configure PWM for ESC:", err)
		return
	}
	pwmCh3, err = pwm1.Channel(PWM_CH3_PIN)
	if err != nil {
		println("could not get PWM channel for pin D7:", err)
		return
	}
	println("PWM configured for servos and ESC.")

	// I2C setup for the IMU
	i2c.Configure(machine.I2CConfig{
		Frequency: 400 * machine.KHz,
	})

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
	// --- End Hardware Setup ---

	// --- Filter and Controller Setup ---
	dt := 0.01 // Time step in seconds
	kf = NewKalmanFilter(dt)
	rollPID = NewPIDController(P, I, D)
	pitchPID = NewPIDController(P, I, D)

	println("Kalman Filter and PID controllers initialized.")
	// --- End Filter and Controller Setup ---

	// Start the receiver input handler in a separate goroutine.
	// This makes the main loop non-blocking.
	go HandleReceiverInput()

	// Main application loop
	for {
		// Use a select statement to either handle a new RC packet or
		// continue with the flight control loop. This makes the system
		// responsive to new RC commands without blocking.
		select {
		case <-packetReady:
			// A new, valid RC packet has been received and processed.
			// This signals that the Channels array has been updated.
			println("New RC packet received and processed.")
		default:
			// No new RC packet is available, so continue with the flight control logic.

			// Measure IMU data
			ax, ay, az, err := lsm.ReadAcceleration()
			if err != nil {
				println("Error reading acceleration:", err)
				continue
			}
			gx, gy, gz, err := lsm.ReadRotation()
			if err != nil {
				println("Error reading rotation:", err)
				continue
			}
			imu.AccelX = float64(ax) * microGToMS2
			imu.AccelY = float64(ay) * microGToMS2
			imu.AccelZ = float64(az) * microGToMS2
			imu.GyroX = float64(gx) * microDPSToRadS
			imu.GyroY = float64(gy) * microDPSToRadS
			imu.GyroZ = float64(gz) * microDPSToRadS

			// Get the accelerometer-based pitch and roll.
			accelPitch := imu.pitchAccel()
			accelRoll := imu.rollAccel()

			// Update the Kalman filter with new sensor data.
			kf.Predict(imu.GyroX, imu.GyroY)
			kf.Update(accelPitch, accelRoll)

			// Get filtered angles.
			imu.Pitch = kf.X.At(0, 0)
			imu.Roll = kf.X.At(1, 0)

			// Get the target roll/pitch from the RC transmitter (CH0 and CH1).
			// We map the raw channel value to a desired rate.
			// The RC channel value is typically between 988 and 2012.
			// The desired rate is between -MAX_RATE and +MAX_RATE.
			desiredRollRate := mapRange(float64(Channels[0]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_ROLL_RATE, MAX_ROLL_RATE)
			desiredPitchRate := mapRange(float64(Channels[1]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_PITCH_RATE, MAX_PITCH_RATE)

			// Get the current roll/pitch rate from the gyro.
			currentRollRate := imu.GyroX
			currentPitchRate := imu.GyroY

			// Calculate the error for PID controllers.
			rollError := desiredRollRate - currentRollRate
			pitchError := desiredPitchRate - currentPitchRate

			// Update PID controllers and get the control outputs.
			rollOutput := rollPID.Update(rollError, dt)
			pitchOutput := pitchPID.Update(pitchError, dt)

			// Combine PID outputs with a mix of raw RC input.
			// This is a simplified mixing model for an elevon-controlled aircraft.
			leftElevon := rollOutput + pitchOutput
			rightElevon := rollOutput - pitchOutput

			// Convert control outputs to PWM pulse widths.
			// Constrain the final values to the valid PWM range.
			leftPulse := uint32(constrain(leftElevon, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US))
			rightPulse := uint32(constrain(rightElevon, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US))

			// Set the PWM signals for the servos.
			setServoPWM(leftPulse, rightPulse)

			// Handle ESC signal from CH2
			escPulse := uint32(mapRange(float64(Channels[2]), MIN_RX_VALUE, MAX_RX_VALUE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US))
			setESC(escPulse)

			// Print status and sensor data
			fmt.Printf("Roll: %.2f deg, Pitch: %.2f deg, Desired Roll Rate: %.2f, Output: %.2f\n", imu.Roll*180/math.Pi, imu.Pitch*180/math.Pi, desiredRollRate, rollOutput)
			fmt.Printf("CH1: %d, CH2: %d, CH3: %d\n", Channels[0], Channels[1], Channels[2])

			// A small delay to keep the loop from running too fast.
			time.Sleep(10 * time.Millisecond)
		}
	}
}
