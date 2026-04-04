package main

import (
	"machine"
	"math"
	"time"
)

// Version of the flight controller software.
const Version = "0.3.0"

// Global variables for hardware interfaces, controllers, and filters.
var (
	watchdog = machine.Watchdog

	// Global hardware instance
	hw *FC_Hardware

	// Control system components
	pitchPID *PIDController
	rollPID  *PIDController
	dt       = 0.005 // 5ms loop time for 200Hz control loop
	kf       *KalmanFilter
	imuData  IMU

	// IMU calibration
	accelXSum, accelYSum, accelZSum, accelBiasX, accelBiasY, accelBiasZ float64 = 0., 0., 0., 0., 0., 0.
	gyroXSum, gyroYSum, gyroZSum, gyroBiasX, gyroBiasY, gyroBiasZ       float64 = 0., 0., 0., 0., 0., 0.
	xA, yA, zA, xG, yG, zG                                              int32
	desiredPitchRate, desiredRollRate                                   float64

	// RC Channels
	Channels        [NumChannels]uint16
	lastFlightState flightState
	LastPacketTime  time.Time
	calibStartTime  time.Time
	armed           bool
	err             error
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

	// Fail-safe constants
	FAILSAFE_TIMEOUT_MS = 500

	// State machine states
	CALIBRATION flightState = iota
	FLIGHT_MODE
	FAILSAFE
)

type flightState int

// main is the entry point for the TinyGo program.
func main() {
	// Delay may be necessary for USB serial connection
	// time.Sleep(2 * time.Second) // Wait for hardware to stabilize

	println("WingFC Flight Controller - Version", Version)
	println("A TinyGo Flight Controller for Flying Wing Aircraft")
	println("Source: github.com/BryanSouza91/WingFC")
	println("Author: Bryan Souza (github.com/BryanSouza91)")

	println("Initializing...")

	// Create concrete implementations and wire them into FC_Hardware
	hw = &FC_Hardware{
		I2C:         NewMachineI2C(machine.I2C0),
		UART:        NewMachineUART(machine.DefaultUART),
		ServoPWM:    NewMachinePWM(machine.PWM0),
		ESCPWM:      NewMachinePWM(machine.PWM1),
		LED:         NewLEDController(machine.LED_RED, machine.LED_GREEN, machine.LED_BLUE),
		PWM_CH1_PIN: machine.D0, // Aileron Servo
		PWM_CH2_PIN: machine.D1, // Elevator Servo
		PWM_CH3_PIN: machine.D2, // ESC
	}

	// Initialize all hardware
	if err := InitHardware(hw); err != nil {
		println("CRITICAL: Hardware initialization failed:", err)
		// Enter error loop - LED will show error state
		for {
			time.Sleep(time.Second)
		}
	}

	// --- Filter and Controller Setup ---
	kf = NewKalmanFilter(dt)
	pitchPID = NewPIDController(pP, pI, pD)
	rollPID = NewPIDController(rP, rI, rD)
	println("Control system initialized.")

	// --- Watchdog Setup ---
	watchdog.Configure(machine.WatchdogConfig{
		TimeoutMillis: 1000, // 1s timeout
	})

	flightState := CALIBRATION
	lastFlightState = CALIBRATION

	// Start the goroutine to read receiver packets asynchronously.
	go readReceiver(hw.UART, packetChan)

	// ticker to run the control loop at a fixed frequency matching Kalman filter.
	ticker := time.NewTicker(time.Duration(dt * float64(time.Second)))
	defer ticker.Stop()

	watchdog.Start()

	// Main application loop using select.
	// --- Main Loop ---
	for {

		select {
		case packet := <-packetChan:
			LastPacketTime = time.Now()
			// A complete packet has been received.
			Channels = processReceiverPacket(packet)

		default:
			// Control loop at fixed intervals
			<-ticker.C

			// Always check for failsafe condition before the state machine logic
			// This provides a quick response to signal loss
			if time.Since(LastPacketTime).Milliseconds() > FAILSAFE_TIMEOUT_MS && flightState != FAILSAFE {
				flightState = FAILSAFE
			}

			// Read and process IMU data every loop to have the freshest data available.
			readLSMData()
			processLSMData()

			// The state machine from previous versions is now the default case
			switch flightState {
			case CALIBRATION:
				hw.LED.SetState(CALIBRATE)
				hw.LED.Update()
				// Keep outputs at neutral and ESC at zero
				setServo(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
				setESC(MIN_PULSE_WIDTH_US)

				// Calibrate gyro to find bias
				println("Initial calibration")
				println("Calibrating Gyro... Keep gyro still!")
				time.Sleep(time.Second)
				calibrate()

				lastFlightState = flightState
				flightState = FLIGHT_MODE
				hw.LED.SetState(LEDOFF)
				hw.LED.Update()

			case FLIGHT_MODE:
				// Switch to armed mode if CH5 is high
				// Check for arm/disarm first every loop
				if Channels[ArmChannel] <= HIGH_RX_VALUE {
					// println("Disarmed.")
					hw.LED.SetState(DISARMED)
					hw.LED.Update()
					armed = false
				} else {
					// println("Armed!")
					hw.LED.SetState(ARMED)
					hw.LED.Update()
					armed = true
				}

				// Handle failsafe and manual mode checks within the flight loop
				if time.Since(LastPacketTime).Milliseconds() > FAILSAFE_TIMEOUT_MS {
					flightState = FAILSAFE
					break
				}

				// Handle manual mode
				if Channels[ManualModeChannel] > HIGH_RX_VALUE {
					// Manual mode
					leftPulse := uint32(Channels[AileronChannel])
					rightPulse := uint32(Channels[ElevatorChannel])
					setServo(leftPulse, rightPulse)
					setESC(uint32(Channels[ThrottleChannel]))
					pitchPID.Reset()
					rollPID.Reset()
					break
				}
				// In stabilized mode, use IMU, Kalman filter and PID controllers to stabilize the aircraft.

				// Use the Kalman filter to fuse sensor data and get a stable attitude estimate.
				kf.Predict(imuData.GyroX, imuData.GyroY)
				kf.Update(imuData.Pitch, imuData.Roll)

				// Get desired roll and pitch rates from the RC receiver.
				desiredPitchRate = mapRange(float64(Channels[ElevatorChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_PITCH_RATE, MAX_PITCH_RATE)
				desiredRollRate = mapRange(float64(Channels[AileronChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_ROLL_RATE, MAX_ROLL_RATE)

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
				setServo(leftPulse, rightPulse)

				// Arming engages throttle control Disarming disengages throttle control
				// Stabilization takes place regardless
				// In armed mode, set the ESC from ThrottleChannel
				if armed {
					// Handle ESC signal from ThrottleChannel
					escPulse := uint32(Channels[ThrottleChannel])
					setESC(escPulse)
				} else {
					// This is disarmed mode, set ESC to minimum
					setESC(MIN_PULSE_WIDTH_US)
				}

			case FAILSAFE:
				setServo(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
				setESC(MIN_PULSE_WIDTH_US)
				hw.LED.SetState(FAILSAFED)
				hw.LED.Update()
				if time.Since(LastPacketTime).Milliseconds() <= FAILSAFE_TIMEOUT_MS {
					lastFlightState = flightState
					flightState = FLIGHT_MODE
				}
			}

			// Keep the watchdog happy
			watchdog.Update()
		}
	}
}
