package main

import (
	// "fmt"
	"machine"
	"math"
	"time"

	"tinygo.org/x/drivers/lsm6ds3tr"
)

// Version of the flight controller software.
const Version = "0.2.0"

// Global variables for hardware interfaces, controllers, and filters.
var (
	// Hardware interfaces
	uart     = machine.DefaultUART
	i2c      = machine.I2C0
	lsm      *lsm6ds3tr.Device
	watchdog = machine.Watchdog

	// PWM controllers and channels
	pwm0   = machine.PWM0
	pwm1   = machine.PWM1
	pwmCh1 uint8
	pwmCh2 uint8
	pwmCh3 uint8

	// Control system components
	pitchPID *PIDController
	rollPID  *PIDController
	dt       = 0.01
	kf       *KalmanFilter
	imuData  IMU

	// IMU calibration
	accelXSum, accelYSum, accelZSum, accelBiasX, accelBiasY, accelBiasZ float64 = 0., 0., 0., 0., 0., 0.
	gyroXSum, gyroYSum, gyroZSum, gyroBiasX, gyroBiasY, gyroBiasZ       float64 = 0., 0., 0., 0., 0., 0.
	xA, yA, zA, xG, yG, zG                                              int32
	desiredPitchRate, desiredRollRate                                   float64

	// RC Channels
	Channels        [NumChannels]uint16
	LastPacketTime  time.Time
	lastFlightState flightState
	err             error
	calibStartTime  time.Time
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
	PWM_CH3_PIN = machine.D2 // ESC (Electronic Speed Controller)

	// Fail-safe constants
	FAILSAFE_TIMEOUT_MS = 500

	// State machine states
	INITIALIZATION flightState = iota
	WAITING
	CALIBRATING
	FLIGHT_MODE
	FAILSAFE
)

type flightState int

// main is the entry point for the TinyGo program.
func main() {
	time.Sleep(2 * time.Second) // Wait for hardware to stabilize
	println("WingFC Flight Controller - Version", Version)
	println("A TinyGo Flight Controller for Flying Wing Aircraft")
	println("Source: github.com/BryanSouza91/WingFC")
	println("Author: Bryan Souza (github.com/BryanSouza91)")

	flightState := INITIALIZATION
	lastFlightState = INITIALIZATION

	// Start the goroutine to read receiver packets asynchronously.
	go readReceiver(packetChan)

	interval := 10 * time.Millisecond
	ticker := time.NewTicker(interval)
	defer ticker.Stop()

	// Main application loop using select.
	// --- Main Loop ---
	for {
		select {
		case packet := <-packetChan:
			LastPacketTime = time.Now()
			// A complete packet has been received.
			processReceiverPacket(packet)
			// println("Received and processed a new receiver packet.")

		default:
			// Control loop at fixed intervals
			<-ticker.C

			// Always check for failsafe condition before the state machine logic
			// This provides a quick response to signal loss
			if time.Since(LastPacketTime).Milliseconds() > FAILSAFE_TIMEOUT_MS && flightState != FAILSAFE && flightState != WAITING {
				flightState = FAILSAFE
			}

			// The state machine from previous versions is now the default case
			switch flightState {
			case INITIALIZATION:

				// --- Hardware Setup ---
				uart.Configure(machine.UARTConfig{
					BaudRate: 115200,
					TX:       machine.NoPin,
					RX:       machine.UART_RX_PIN,
				})
				println("UART configured for receiver.")

				servoPWMConfig := machine.PWMConfig{
					Period: machine.GHz * 1 / SERVO_PWM_FREQUENCY,
				}
				if err := pwm0.Configure(servoPWMConfig); err != nil {
					println("could not configure PWM for servos:", err)
					return
				}
				pwmCh1, err = pwm0.Channel(PWM_CH1_PIN)
				if err != nil {
					println("could not get PWM channel 1:", err)
					return
				}
				pwm0.Set(pwmCh1, NEUTRAL_RX_VALUE)
				pwmCh2, err = pwm0.Channel(PWM_CH2_PIN)
				if err != nil {
					println("could not get PWM channel 2:", err)
					return
				}
				pwm0.Set(pwmCh2, NEUTRAL_RX_VALUE)
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
					println("could not get PWM channel for ESC:", err)
					return
				}
				pwm1.Set(pwmCh3, MIN_PULSE_WIDTH_US)
				println("PWM configured for ESC.")

				i2c.Configure(machine.I2CConfig{
					Frequency: 400 * machine.KHz,
				})
				println("I2C configured for IMU.")

				// --- IMU Setup ---
				lsm = lsm6ds3tr.New(i2c)
				err = lsm.Configure(lsm6ds3tr.Configuration{
					AccelRange:      lsm6ds3tr.ACCEL_8G,
					AccelSampleRate: lsm6ds3tr.ACCEL_SR_416,
					GyroRange:       lsm6ds3tr.GYRO_1000DPS,
					GyroSampleRate:  lsm6ds3tr.GYRO_SR_416,
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
				calibrate()

				// --- Filter and Controller Setup ---
				kf = NewKalmanFilter(dt)
				pitchPID = NewPIDController(pP, pI, pD)
				rollPID = NewPIDController(rP, rI, rD)
				println("Control system initialized.")

				// --- Watchdog Setup ---
				watchdog.Configure(machine.WatchdogConfig{
					TimeoutMillis: 500, // 500ms timeout
				})
				watchdog.Start()

				lastFlightState = flightState
				flightState = WAITING

			case WAITING:
				// Keep outputs at neutral and ESC at zero
				setServoPWM(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
				setESC(MIN_PULSE_WIDTH_US)

				// Check for calibration
				if Channels[CalibrateChannel] >= HIGH_RX_VALUE {
					calibStartTime = time.Now()
					lastFlightState = flightState
					flightState = CALIBRATING
					break
				}

				// Check for arming
				if Channels[ArmChannel] >= HIGH_RX_VALUE {
					println("Armed!")
					lastFlightState = flightState
					flightState = FLIGHT_MODE
				}

			case CALIBRATING:
				if Channels[CalibrateChannel] <= HIGH_RX_VALUE {
					println("Calibration Aborted.")
					lastFlightState = flightState
					flightState = WAITING
					break
				}

				println("Calibrating Gyro... Keep the wing still!")
				var gyroXSum, gyroYSum float64
				const sampleSize = 1000
				for i := 0; i < sampleSize; i++ {
					xG, yG, _, _ := lsm.ReadRotation()
					gyroXSum += float64(xG)
					gyroYSum += float64(yG)
					time.Sleep(time.Millisecond)
				}
				gyroBiasX = gyroXSum / sampleSize
				gyroBiasY = gyroYSum / sampleSize
				println("Calibration complete!")

				lastFlightState = flightState
				flightState = WAITING

			case FLIGHT_MODE:
				// Check for disarm
				if Channels[ArmChannel] <= HIGH_RX_VALUE {
					println("Disarmed.")
					lastFlightState = flightState
					flightState = WAITING
					break
				}

				// Handle failsafe and manual mode checks within the flight loop
				if time.Since(LastPacketTime).Milliseconds() > FAILSAFE_TIMEOUT_MS {
					flightState = FAILSAFE
					break
				}

				if Channels[ManualModeChannel] > HIGH_RX_VALUE {
					// Manual mode
					leftPulse := uint32(Channels[AileronChannel])
					rightPulse := uint32(Channels[ElevatorChannel])
					setServoPWM(leftPulse, rightPulse)
					setESC(uint32(Channels[ThrottleChannel]))
					break
				}

				// ---- Failsafe and Mode Handling ----
				// If no valid packet received recently, set servos and ESC to safe values
				// and skip the rest of the loop.
				if time.Since(LastPacketTime) > 500*time.Millisecond {
					setServoPWM(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
					setESC(MIN_PULSE_WIDTH_US)
					continue // Skip the rest of the loop if no valid signal
				}

				// In stabilized mode, use PID controllers to stabilize the aircraft.

				// Read and process IMU data.
				readLSMData()
				processLSMData()

				// Use the Kalman filter to fuse sensor data and get a stable attitude estimate.
				kf.Predict(imuData.GyroX, imuData.GyroY)
				kf.Update(imuData.Pitch, imuData.Roll)

				// In armed mode, use RC inputs to set desired rates.
				if Channels[4] < HIGH_RX_VALUE { // Switch to armed mode if CH5 is high
					// This is disarmed mode, set desired rates to zero
					desiredPitchRate = mapRange(NEUTRAL_RX_VALUE, MIN_RX_VALUE, MAX_RX_VALUE, -MAX_PITCH_RATE, MAX_PITCH_RATE)
					desiredRollRate = mapRange(NEUTRAL_RX_VALUE, MIN_RX_VALUE, MAX_RX_VALUE, -MAX_ROLL_RATE, MAX_ROLL_RATE)
				} else {
					// Get desired roll and pitch rates from the RC receiver.
					desiredPitchRate = mapRange(float64(Channels[1]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_PITCH_RATE, MAX_PITCH_RATE)
					desiredRollRate = mapRange(float64(Channels[0]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_ROLL_RATE, MAX_ROLL_RATE)
				}
				// Apply deadband to avoid small unwanted movements
				if math.Abs(desiredPitchRate) < DEADBAND*math.Pi/180 {
					desiredPitchRate = 0
				}
				if math.Abs(desiredRollRate) < DEADBAND*math.Pi/180 {
					desiredRollRate = 0
				}

				// Calculate the error for PID controllers.
				pitchError := desiredPitchRate - imuData.GyroY
				rollError := desiredRollRate - imuData.GyroX

				// Update PID controllers and get the control outputs.
				pitchOutput := pitchPID.Update(pitchError, dt) * PID_WEIGHT
				rollOutput := rollPID.Update(rollError, dt) * PID_WEIGHT

				// Combine PID outputs with a mix of raw RC input.
				leftElevon := pitchOutput + rollOutput
				rightElevon := pitchOutput - rollOutput

				// Convert control outputs to PWM pulse widths.
				leftElevon = mapRange(float64(leftElevon), -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
				rightElevon = mapRange(float64(rightElevon), -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)

				// Constrain pulse widths to a valid range.
				leftPulse := uint32(constrain(leftElevon, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US))
				rightPulse := uint32(constrain(rightElevon, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US))

				// Set the PWM signals for the servos.
				setServoPWM(leftPulse, rightPulse)

				// In armed mode, set the ESC from CH3
				if Channels[4] < HIGH_RX_VALUE { // Switch to armed mode if CH5 is high
					// This is disarmed mode, set ESC to minimum
					setESC(MIN_PULSE_WIDTH_US)
					continue // Skip ESC setting in disarmed mode
				}
				// Handle ESC signal from CH3
				escPulse := uint32(Channels[2])
				setESC(escPulse)

				// // Print status and sensor data for debugging
				println(desiredPitchRate, pitchOutput, desiredRollRate, rollOutput)
				println()
				println(Channels[0], Channels[1], Channels[2])
				println(leftPulse, rightPulse)

			case FAILSAFE:
				setServoPWM(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
				setESC(MIN_PULSE_WIDTH_US)

				if time.Since(LastPacketTime).Milliseconds() <= FAILSAFE_TIMEOUT_MS {
					lastFlightState = flightState
					flightState = WAITING
				}
			}

			// Keep the watchdog happy
			watchdog.Update()
		}
	}
}
