package main

import (
	// "fmt"
	"machine"
	"math"
	"time"

	"tinygo.org/x/drivers/lsm6ds3tr"
)

const Version = "0.0.1"

// Define constants
const (
	SERVO_PWM_FREQUENCY = 50   // Standard servo frequency (200Hz for digital servos)
	ESC_PWM_FREQUENCY   = 500  // Non-Standard ESC frequency
	MIN_PULSE_WIDTH_US  = 1000 // 1ms pulse for full negative deflection
	MAX_PULSE_WIDTH_US  = 2000 // 2ms pulse for full positive deflection
	MIN_RX_VALUE        = 988  // Minimum iBus channel value
	MAX_RX_VALUE        = 2012 // Maximum iBus channel value
	HIGH_RX_VALUE       = 1800 // High iBus channel value for arming/calibration
	NEUTRAL_RX_VALUE    = 1500 // Neutral iBus channel value
	DEADBAND            = 20   // Deadband around neutral

	// Maximum rotational rates
	MAX_ROLL_RATE_DEG  = 600 // degrees/sec
	MAX_PITCH_RATE_DEG = 200 // degrees/sec

	// Calculated constants
	MAX_ROLL_RATE  = MAX_ROLL_RATE_DEG * (math.Pi / 180)  // radians/sec
	MAX_PITCH_RATE = MAX_PITCH_RATE_DEG * (math.Pi / 180) // radians/sec

	FAILSAFE_TIMEOUT_MS = 500
	PID_WEIGHT          = 0.5 // Weighting factor for combining gyro and accel data with input

	// PWM output pins
	PWM_CH1_PIN = machine.D0
	PWM_CH2_PIN = machine.D1
	PWM_CH3_PIN = machine.D2

	// State machine states
	INITIALIZATION flightState = iota
	WAITING
	CALIBRATING
	FLIGHT_MODE
	FAILSAFE
)

var (
	watchdog        = machine.Watchdog
	pwm0            = machine.PWM0
	pwm1            = machine.PWM1
	pwmCh1          uint8
	pwmCh2          uint8
	pwmCh3          uint8
	err             error
	lsm             lsm6ds3tr.Device
	kf              *KalmanFilter
	controller      *PIDController
	imu             *IMU
	lastFlightState flightState

	calibStartTime time.Time
	gyroBiasX      float64
	gyroBiasY      float64
)

type flightState int

// Main program loop
func main() {
	// Print startup message
	println("WingFC - Version", Version)
	println("A TinyGo Flight Controller for Flying Wing Aircraft")
	println("Source: github.com/BryanSouza91/WingFC")
	println("Author: Bryan Souza (github.com/BryanSouza91)")

	// Initial state
	flightState := INITIALIZATION
	for {
		// Attempt to parse iBus data every loop iteration.
		ParseIBus()

		// Check for failsafe condition before the main state machine
		if time.Since(lastPacketTime).Milliseconds() > FAILSAFE_TIMEOUT_MS && flightState == FLIGHT_MODE {
			flightState = FAILSAFE
		}

		switch flightState {
		case INITIALIZATION:
			// --- Hardware Setup ---
			uart := machine.DefaultUART
			uart.Configure(machine.UARTConfig{
				BaudRate: 115200,
				TX:       machine.NoPin,
				RX:       machine.UART_RX_PIN, // iBus in
			})

			servoPWMConfig := machine.PWMConfig{Period: machine.GHz * 1 / SERVO_PWM_FREQUENCY}
			if err := pwm0.Configure(servoPWMConfig); err != nil {
				println("could not configure PWM:", err)
				return
			}
			pwmCh1, err = pwm0.Channel(PWM_CH1_PIN)
			if err != nil {
				println("could not get PWM channel for pin D2:", err)
				return
			}
			pwmCh2, err = pwm0.Channel(PWM_CH2_PIN)
			if err != nil {
				println("could not get PWM channel for pin D6:", err)
				return
			}
			escPWMConfig := machine.PWMConfig{Period: machine.GHz * 1 / ESC_PWM_FREQUENCY}
			if err := pwm1.Configure(escPWMConfig); err != nil {
				println("could not configure PWM for ESC:", err)
				return
			}
			pwmCh3, err = pwm1.Channel(PWM_CH3_PIN)
			if err != nil {
				println("could not get PWM channel for pin D7:", err)
				return
			}

			i2c := machine.I2C0
			i2c.Configure(machine.I2CConfig{
				Frequency: 400 * machine.KHz,
			})
			lsm := lsm6ds3tr.New(i2c)
			err := lsm.Configure(lsm6ds3tr.Configuration{
				AccelRange:      lsm6ds3tr.ACCEL_16G,
				AccelSampleRate: lsm6ds3tr.ACCEL_SR_6664,
				GyroRange:       lsm6ds3tr.GYRO_2000DPS,
				GyroSampleRate:  lsm6ds3tr.GYRO_SR_6664,
			})
			if err != nil {
				for {
					println("Failed to configure LSM6DS3TR:", err.Error())
					time.Sleep(time.Second)
				}
			}

			// --- Filter and Controller Setup ---
			dt := 0.01 // Time step in seconds
			kf = NewKalmanFilter(dt)
			controller = NewPIDController(0.5, 0.1, 0.2)
			imu = new(IMU)

			// Initial neutral PWM output
			setServoPWM(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
			setESC(MIN_PULSE_WIDTH_US) // Set ESC to zero throttle

			// Small delay to allow ESC to initialize
			time.Sleep(2 * time.Second)

			// Configuring Watchdog Timer
			watchdog.Configure(machine.WatchdogConfig{
				TimeoutMillis: 500, // 500ms timeout
			})
			watchdog.Start() // Start the watchdog timer

			lastFlightState = flightState
			flightState = WAITING

		case WAITING:
			// Output neutral PWM signals
			setServoPWM(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
			setESC(MIN_PULSE_WIDTH_US) // Keep ESC at zero

			// Check if we just exited failsafe
			// If so, wait for disarm before allowing re-arming
			// This prevents immediate re-arming after a failsafe event
			// which could be dangerous
			if lastFlightState == FAILSAFE {
				for ch5 > HIGH_RX_VALUE {
					continue // Wait for disarm
				}
			}

			// Check if pilot is calibrating the system
			if ch6 >= HIGH_RX_VALUE {
				calibStartTime = time.Now()
				lastFlightState = flightState
				flightState = CALIBRATING
			}

			// Check if the system is armed
			if ch5 > HIGH_RX_VALUE {
				lastFlightState = flightState
				flightState = FLIGHT_MODE
			}

		case CALIBRATING:
			// Check if calibration period is over
			if time.Since(calibStartTime).Seconds() > 10 {
				lastFlightState = flightState
				flightState = WAITING
				break
			}

			println("Calibrating Gyro... Keep the wing still!")
			// Use an array to store gyro readings for averaging
			var gyroXSum, gyroYSum float64
			const sampleSize = 1000

			// Perform the calibration loop
			for i := 0; i < sampleSize; i++ {
				xG, yG, _, _ := lsm.ReadRotation()
				gyroXSum += float64(xG)
				gyroYSum += float64(yG)
				time.Sleep(time.Millisecond) // Wait to get unique readings
			}

			// Calculate the average bias
			gyroBiasX = gyroXSum / sampleSize
			gyroBiasY = gyroYSum / sampleSize

			println("Calibration complete!")
			// println(fmt.Sprintf("Gyro Bias X: %.4f", gyroBiasX))
			// println(fmt.Sprintf("Gyro Bias Y: %.4f", gyroBiasY))

			// After calibration, set a neutral output
			setServoPWM(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)

			// Update last state
			lastFlightState = flightState
			// Transition to WAITING once calibration is done
			flightState = WAITING

		case FLIGHT_MODE:
			// Check if the system is disarmed
			if ch5 <= HIGH_RX_VALUE {
				lastFlightState = flightState
				flightState = WAITING
				break
			}
			// Read the raw iBus values for aileron and elevator
			rawAileron := float64(ch1)
			rawElevator := float64(ch2)

			// Apply deadband around neutral
			if (rawAileron > NEUTRAL_RX_VALUE-DEADBAND) && (rawAileron < NEUTRAL_RX_VALUE+DEADBAND) {
				rawAileron = NEUTRAL_RX_VALUE
			}
			if (rawElevator > NEUTRAL_RX_VALUE-DEADBAND) && (rawElevator < NEUTRAL_RX_VALUE+DEADBAND) {
				rawElevator = NEUTRAL_RX_VALUE
			}

			// Directly map the raw input to a desired rotational rate
			// This simplifies the logic by removing an intermediate step
			// The pilot's input now directly commands the desired rate of rotation
			desiredRollRate := mapRange(rawAileron, MIN_RX_VALUE, MAX_RX_VALUE, -MAX_ROLL_RATE, MAX_ROLL_RATE)
			desiredPitchRate := mapRange(rawElevator, MIN_RX_VALUE, MAX_RX_VALUE, -MAX_PITCH_RATE, MAX_PITCH_RATE)

			// Read IMU data
			xA, yA, zA, _ := lsm.ReadAcceleration()
			imu.AccelX, imu.AccelY, imu.AccelZ = float64(xA), float64(yA), float64(zA)
			xG, yG, _, _ := lsm.ReadRotation()

			// Subtract the calibrated gyro bias
			imu.GyroX = float64(xG) - gyroBiasX
			imu.GyroY = float64(yG) - gyroBiasY

			// --- PID Control ---
			// The 'error' is now the difference between the desired rate and the current rate
			pitchError := desiredPitchRate - imu.GyroY
			rollError := desiredRollRate - imu.GyroX

			pitchCorrection := controller.Update(pitchError, kf.dt)
			rollCorrection := controller.Update(rollError, kf.dt)

			// --- Mixing Logic ---
			// The final output is the sum of the pilot's desired input and the PID correction
			finalPitch := (imu.pitchAccel() * PID_WEIGHT) + pitchCorrection
			finalRoll := (imu.rollAccel() * PID_WEIGHT) + rollCorrection

			finalPitch = mapRange(finalPitch, -MAX_PITCH_RATE, MAX_PITCH_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
			finalRoll = mapRange(finalRoll, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)

			// Combine the final pitch and roll values for elevon mixing
			leftElevonOutput := finalPitch + finalRoll
			rightElevonOutput := finalPitch - finalRoll

			// Convert normalized outputs to PWM microseconds
			leftPulseWidth := mapRange(leftElevonOutput, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
			rightPulseWidth := mapRange(rightElevonOutput, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)

			// Set the servo PWM
			setServoPWM(uint32(leftPulseWidth), uint32(rightPulseWidth))

			// Set the ESC PWM based on throttle input (ch3)
			throttlePulse := mapRange(float64(ch3), MIN_RX_VALUE, MAX_RX_VALUE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
			setESC(uint32(throttlePulse))

		case FAILSAFE:
			// On signal loss, set servos to neutral
			setServoPWM(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
			setESC(MIN_PULSE_WIDTH_US) // Cut throttle

			// Optionally, you could implement a gradual descent or other safety measures here
			// Remain in FAILSAFE until signal is regained
			if time.Since(lastPacketTime).Milliseconds() <= FAILSAFE_TIMEOUT_MS {
				lastFlightState = flightState
				flightState = WAITING // Re-arm the system
			}
		default:
			flightState = WAITING // Fallback to a safe state
		}

		// Keep the watchdog happy
		watchdog.Update()

		// Control loop frequency
		// necessary to ensure stable operation
		// match to dt
		time.Sleep(time.Millisecond * 10) // 100Hz loop
	}
}

// Helper function to map a value from one range to another.
func mapRange(value, fromMin, fromMax, toMin, toMax float64) float64 {
	return (value-fromMin)/(fromMax-fromMin)*(toMax-toMin) + toMin
}

// Helper function to set servo PWM outputs
func setServoPWM(leftPulse, rightPulse uint32) {
	// Set the PWM duty cycle for left elevon (CH1)
	pwm0.Set(pwmCh1, pwm0.Top()/uint32(1e6)*leftPulse)
	// Set the PWM duty cycle for right elevon (CH2)
	pwm0.Set(pwmCh2, pwm0.Top()/uint32(1e6)*rightPulse)
}

// Helper function to set the ESC's PWM output
func setESC(pulseWidth uint32) {
	// The ESC's channel is typically CH3
	pwm1.Set(pwmCh3, pwm1.Top()/uint32(1e6)*pulseWidth)
}
