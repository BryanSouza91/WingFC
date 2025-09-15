package main

import (
	// "fmt"
	"machine"
	"math"
	"time"

	"tinygo.org/x/drivers/lsm6ds3tr"
)

const Version = "0.0.2"

// Define constants
const (
	MIN_PULSE_WIDTH_US = 1000 // 1ms pulse for full negative deflection
	MAX_PULSE_WIDTH_US = 2000 // 2ms pulse for full positive deflection

	MIN_RX_VALUE     = 988  // Minimum Rx channel value
	MAX_RX_VALUE     = 2012 // Maximum Rx channel value
	NEUTRAL_RX_VALUE = 1500 // Neutral Rx channel value

	// Calculated constants
	MAX_ROLL_RATE  = MAX_ROLL_RATE_DEG * (math.Pi / 180)  // radians/sec
	MAX_PITCH_RATE = MAX_PITCH_RATE_DEG * (math.Pi / 180) // radians/sec

	FAILSAFE_TIMEOUT_MS = 500

	// State machine states
	INITIALIZATION flightState = iota
	WAITING
	CALIBRATING
	FLIGHT_MODE
	FAILSAFE
)

var (
	// Channels holds the RC channel values for all protocols (iBus, CRSF, ELRS)
	Channels [NumChannels]uint16

	watchdog        = machine.Watchdog
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
		// Unified receiver handler for multi-protocol support
		HandleReceiverInput()

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
				RX:       machine.UART_RX_PIN, // iBus/CRSF/ELRS in
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
			controller = NewPIDController(P, I, D)
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
			if lastFlightState == FAILSAFE && armCh > HIGH_RX_VALUE {
				break
			}

			// Check if pilot is calibrating the system
			if calCh >= HIGH_RX_VALUE {
				calibStartTime = time.Now()
				lastFlightState = flightState
				flightState = CALIBRATING
				break
			}

			// Check if the system is armed
			if armCh > HIGH_RX_VALUE {
				lastFlightState = flightState
				flightState = FLIGHT_MODE
				break
			}

		case CALIBRATING:
			// Check if calibration switch has been turned off
			if calCh <= HIGH_RX_VALUE {
				lastFlightState = flightState
				flightState = WAITING
				break
			}

			// During calibration, set a neutral output
			setServoPWM(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)

			// Pause to let airframe settle
			time.Sleep(500 * time.Millisecond)

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

		case FLIGHT_MODE:
			// --- Read and Process RC Inputs ---

			// Check if the system is disarmed
			if armCh <= HIGH_RX_VALUE {
				lastFlightState = flightState
				flightState = WAITING
				break
			}

			// Convert raw RC channel values to float64 for calculations
			rawAileron := float64(aileronCh)
			rawElevator := float64(elevatorCh)

			// --- Apply Deadband Around Neutral ---
			// This prevents small stick movements or noise from affecting control
			if rawAileron > float64(NEUTRAL_RX_VALUE-DEADBAND) && rawAileron < float64(NEUTRAL_RX_VALUE+DEADBAND) {
				rawAileron = float64(NEUTRAL_RX_VALUE)
			}
			if rawElevator > float64(NEUTRAL_RX_VALUE-DEADBAND) && rawElevator < float64(NEUTRAL_RX_VALUE+DEADBAND) {
				rawElevator = float64(NEUTRAL_RX_VALUE)
			}

			// --- Map RC Inputs to Desired Rotational Rates ---
			// The pilot's stick input directly commands the desired rate of rotation
			desiredRollRate := mapRange(
				rawAileron,
				float64(MIN_RX_VALUE),
				float64(MAX_RX_VALUE),
				-float64(MAX_ROLL_RATE),
				float64(MAX_ROLL_RATE),
			)
			desiredPitchRate := mapRange(
				rawElevator,
				float64(MIN_RX_VALUE),
				float64(MAX_RX_VALUE),
				-float64(MAX_PITCH_RATE),
				float64(MAX_PITCH_RATE),
			)

			// --- Read IMU Data ---
			// Get acceleration and rotation from the IMU sensor
			xAccel, yAccel, zAccel, _ := lsm.ReadAcceleration()
			imu.AccelX = float64(xAccel)
			imu.AccelY = float64(yAccel)
			imu.AccelZ = float64(zAccel)

			xGyro, yGyro, _, _ := lsm.ReadRotation()

			// --- Subtract Calibrated Gyro Bias ---
			// This removes any offset from the gyro readings
			imu.GyroX = float64(xGyro) - gyroBiasX
			imu.GyroY = float64(yGyro) - gyroBiasY

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

			// --- Convert normalized outputs to PWM microseconds ---
			finalPitch = mapRange(
				float64(finalPitch),
				-float64(MAX_PITCH_RATE),
				float64(MAX_PITCH_RATE),
				float64(MIN_PULSE_WIDTH_US),
				float64(MAX_PULSE_WIDTH_US),
			)
			finalRoll = mapRange(
				float64(finalRoll),
				-float64(MAX_ROLL_RATE),
				float64(MAX_ROLL_RATE),
				float64(MIN_PULSE_WIDTH_US),
				float64(MAX_PULSE_WIDTH_US),
			)

			// --- Elevon Mixing ---
			// Combine the final pitch and roll values for elevon mixing
			leftElevonOutput := float64(finalPitch) + float64(finalRoll)
			rightElevonOutput := float64(finalPitch) - float64(finalRoll)

			// Define the possible output range for mixing
			const (
				elevonMixMin = -2 * MIN_PULSE_WIDTH_US // -2000
				elevonMixMax = 2 * MAX_PULSE_WIDTH_US  // 4000
			)

			// --- Remap mixed outputs to valid PWM pulse width range ---
			leftPulseWidth := mapRange(
				leftElevonOutput,
				float64(elevonMixMin),
				float64(elevonMixMax),
				float64(MIN_PULSE_WIDTH_US),
				float64(MAX_PULSE_WIDTH_US),
			)
			rightPulseWidth := mapRange(
				rightElevonOutput,
				float64(elevonMixMin),
				float64(elevonMixMax),
				float64(MIN_PULSE_WIDTH_US),
				float64(MAX_PULSE_WIDTH_US),
			)

			// --- Set the servo PWM ---
			setServoPWM(uint32(leftPulseWidth), uint32(rightPulseWidth))

			// --- Set the ESC PWM based on throttle input (ch3) ---
			throttlePulse := mapRange(
				float64(throttleCh),
				float64(MIN_RX_VALUE),
				float64(MAX_RX_VALUE),
				float64(MIN_PULSE_WIDTH_US),
				float64(MAX_PULSE_WIDTH_US),
			)
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
