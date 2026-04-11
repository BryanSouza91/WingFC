package main

import (
	math "github.com/orsinium-labs/tinymath"
	"machine"
	"time"
)

const Version = "0.3.0"

var (
	watchdog = machine.Watchdog
	hw       *FC_Hardware

	// 3-Axis PID
	pitchPID *PIDController
	rollPID  *PIDController
	yawPID   *PIDController

	dt      float32 = 0.005
	kf      *KalmanFilter
	imuData IMU

	desiredPitchRate, desiredRollRate, desiredYawRate float32

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
	microGToMS2    float32 = 9.80665 / 1e6
	microDPSToRadS float32 = math.Pi / (180 * 1e6)

	// PWM pulse width constants
	MIN_PULSE_WIDTH_US = 1000
	MAX_PULSE_WIDTH_US = 2000

	// Calculated constants for PID control
	MAX_ROLL_RATE  float32 = MAX_ROLL_RATE_DEG * math.Pi / 180
	MAX_PITCH_RATE float32 = MAX_PITCH_RATE_DEG * math.Pi / 180

	// Fail-safe constants
	FAILSAFE_TIMEOUT_MS = 500

	// Flight states
	ESC_CALIBRATION flightState = iota // New state for PWM range setting
	IMU_CALIBRATION                    // Renamed from CALIBRATION
	FLIGHT_MODE
	FAILSAFE
)

type flightState int

func main() {
	println("WingFC Flight Controller - Version", Version)
	println("A TinyGo Flight Controller for Radio-Control Aircraft")
	println("Source: github.com/BryanSouza91/WingFC")
	println("Author: Bryan Souza (github.com/BryanSouza91)")

	// Route Seeed XIAO nRF52840 hardware pins
	hw = &FC_Hardware{
		I2C:         NewMachineI2C(machine.I2C0),
		UART:        NewMachineUART(machine.DefaultUART),
		PWM0:        NewMachinePWM(machine.PWM0), // ESC
		PWM1:        NewMachinePWM(machine.PWM1), // Servos 1,2,4,5
		PWM2:        NewMachinePWM(machine.PWM2), // Servo 6
		PWM_CH1_PIN: machine.D0,
		PWM_CH2_PIN: machine.D1,
		PWM_CH3_PIN: machine.D3, // Moved to D3 for ESC/PWM0 grouping
		PWM_CH4_PIN: machine.D2,
		PWM_CH5_PIN: machine.D4,
		PWM_CH6_PIN: machine.D5,
	}

	if err := InitHardware(hw); err != nil {
		for {
			time.Sleep(time.Second)
		} // Critical failure loop
	}

	kf = NewKalmanFilter(dt)
	pitchPID = NewPIDController(pP, pI, pD)
	rollPID = NewPIDController(rP, rI, rD)
	yawPID = NewPIDController(yP, yI, yD)

	watchdog.Configure(machine.WatchdogConfig{TimeoutMillis: 1000})

	flightState := ESC_CALIBRATION
	lastFlightState = flightState

	go readReceiver(hw.UART, packetChan)
	ticker := time.NewTicker(time.Duration(dt * float32(time.Second)))
	defer ticker.Stop()
	watchdog.Start()

	for {
		select {
		case packet := <-packetChan:
			LastPacketTime = time.Now()
			Channels = processReceiverPacket(packet)

		default:
			<-ticker.C

			if time.Since(LastPacketTime).Milliseconds() > FAILSAFE_TIMEOUT_MS && flightState != FAILSAFE {
				flightState = FAILSAFE
			}

			readIMUData()
			processIMUData()

			switch flightState {
			case ESC_CALIBRATION:
				// Pass throttle directly to ESC for calibration
				// ESC stays in 'high' mode until user drops stick
				currentThrottle := uint32(Channels[ThrottleChannel])
				setESC(currentThrottle)

				if currentThrottle < (MIN_PULSE_WIDTH_US + 50) {
					flightState = IMU_CALIBRATION
				}

			case IMU_CALIBRATION:
				hw.LED.SetState(CALIBRATE)
				hw.LED.Update()
				setAllServos(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
				setESC(MIN_PULSE_WIDTH_US)

				time.Sleep(time.Second)
				calibrateIMU()
				flightState = FLIGHT_MODE

			case FLIGHT_MODE:
				armed = Channels[ArmChannel] > HIGH_RX_VALUE

				// Map RC inputs to Rates
				pitchInput := mapRange(float32(Channels[ElevatorChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_PITCH_RATE_DEG, MAX_PITCH_RATE_DEG)
				rollInput := mapRange(float32(Channels[AileronChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_ROLL_RATE_DEG, MAX_ROLL_RATE_DEG)
				yawInput := mapRange(float32(Channels[YawChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_YAW_RATE_DEG, MAX_YAW_RATE_DEG)

				var pitchMix, rollMix, yawMix float32

				if Channels[ManualModeChannel] > HIGH_RX_VALUE {
					// Manual Mode: Bypass PID, feed stick rates directly into Mixer
					pitchMix = pitchInput
					rollMix = rollInput
					yawMix = yawInput

					pitchPID.Reset()
					rollPID.Reset()
					yawPID.Reset()
				} else {
					// Stabilized Mode: 3-Axis Kalman & PID
					kf.Predict(imuData.GyroX, imuData.GyroY, imuData.GyroZ)
					kf.Update(imuData.Pitch, imuData.Roll)

					// Calculate errors (convert input degrees to radians if necessary, matching your IMU)
					pitchError := (pitchInput * math.Pi / 180) - imuData.GyroY
					rollError := (rollInput * math.Pi / 180) - imuData.GyroX
					yawError := (yawInput * math.Pi / 180) - imuData.GyroZ

					pitchMix = pitchPID.Update(pitchError, dt) * PID_WEIGHT
					rollMix = rollPID.Update(rollError, dt) * PID_WEIGHT
					yawMix = yawPID.Update(yawError, dt) * PID_WEIGHT
				}

				// 1. Process Airframe Mixer
				s1, s2, s4, s5, s6 := ApplyMixer(pitchMix, rollMix, yawMix)

				// 2. Map Output Scale to Pulse Widths (Assuming Mixer outputs roughly -MAX_ROLL to +MAX_ROLL)
				s1 = mapRange(s1, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
				s2 = mapRange(s2, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
				s4 = mapRange(s4, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
				s5 = mapRange(s5, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
				s6 = mapRange(s6, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)

				// 3. Apply Reversals
				s1 = applyReversal(s1, SERVO1_REVERSE, float32(SERVO1_MIN), float32(SERVO1_MAX))
				s2 = applyReversal(s2, SERVO2_REVERSE, float32(SERVO2_MIN), float32(SERVO2_MAX))
				s4 = applyReversal(s4, SERVO4_REVERSE, float32(SERVO4_MIN), float32(SERVO4_MAX))
				s5 = applyReversal(s5, SERVO5_REVERSE, float32(SERVO5_MIN), float32(SERVO5_MAX))
				s6 = applyReversal(s6, SERVO6_REVERSE, float32(SERVO6_MIN), float32(SERVO6_MAX))

				// 4. Add Trims and Constrain
				pulse1 := uint32(constrain(s1+float32(SERVO1_SUBTRIM), float32(SERVO1_MIN), float32(SERVO1_MAX)))
				pulse2 := uint32(constrain(s2+float32(SERVO2_SUBTRIM), float32(SERVO2_MIN), float32(SERVO2_MAX)))
				pulse4 := uint32(constrain(s4+float32(SERVO4_SUBTRIM), float32(SERVO4_MIN), float32(SERVO4_MAX)))
				pulse5 := uint32(constrain(s5+float32(SERVO5_SUBTRIM), float32(SERVO5_MIN), float32(SERVO5_MAX)))
				pulse6 := uint32(constrain(s6+float32(SERVO6_SUBTRIM), float32(SERVO6_MIN), float32(SERVO6_MAX)))

				setAllServos(pulse1, pulse2, pulse4, pulse5, pulse6)

				if armed {
					setESC(uint32(Channels[ThrottleChannel]))
				} else {
					setESC(MIN_PULSE_WIDTH_US)
				}

			case FAILSAFE:
				setAllServos(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
				setESC(MIN_PULSE_WIDTH_US)
				if time.Since(LastPacketTime).Milliseconds() <= FAILSAFE_TIMEOUT_MS {
					flightState = FLIGHT_MODE
				}
			}

			watchdog.Update()
		}
	}
}
