package main

import (
	"machine"
	"time"

	math "github.com/orsinium-labs/tinymath"
)

const Version = "0.4.0"

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
	gpsData GPSData // GPS data updated asynchronously

	desiredPitchRate, desiredRollRate, desiredYawRate float32

	Channels        [NumChannels]uint16
	lastFlightState flightState
	LastPacketTime  time.Time
	calibStartTime  time.Time
	armed           bool
	err             error

	// Return-to-Home navigation state
	navState            *NavigationState
	rthActive           bool
	homeNotCaptured     bool
	signalLostStartTime time.Time
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
	MAX_YAW_RATE   float32 = MAX_YAW_RATE_DEG * math.Pi / 180

	// Fail-safe constants
	FAILSAFE_TIMEOUT_MS = 500

	// Flight states
	ESC_CALIBRATION flightState = iota // New state for PWM range setting
	IMU_CALIBRATION                    // Renamed from CALIBRATION
	FLIGHT_MODE
	FAILSAFE
	RETURN_TO_HOME // Return-to-home after GPS acquisition
)

type flightState int

// conditionalGPSTicker returns a channel that receives on ticker ticks, or nil if ticker is nil.
// This allows using a nil ticker in a select statement without panicking.
func conditionalGPSTicker(ticker *time.Ticker) <-chan time.Time {
	if ticker == nil {
		return nil
	}
	return ticker.C
}

func main() {
	println("WingFC Flight Controller - Version", Version)
	println("A TinyGo Flight Controller for Radio-Control Aircraft")
	println("Source: github.com/BryanSouza91/WingFC")
	println("Author: Bryan Souza (github.com/BryanSouza91)")

	// Route Seeed XIAO nRF52840 hardware pins
	hw = &FC_Hardware{
		I2C:         NewMachineI2C(machine.I2C0),
		UART0:       NewMachineUART(machine.DefaultUART), // Main UART for receiver
		UART1:       NewMachineUART(machine.UART1),       // GPS UART
		PWM0:        NewMachinePWM(machine.PWM0),         // ESC
		PWM1:        NewMachinePWM(machine.PWM1),         // Servos 1,2,4,5
		PWM2:        NewMachinePWM(machine.PWM2),         // Servo 6
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

	// Initialize navigation state for RTH
	navState = NewNavigationState()
	homeNotCaptured = true
	rthActive = false

	watchdog.Configure(machine.WatchdogConfig{TimeoutMillis: 1000})

	flightState := ESC_CALIBRATION
	lastFlightState = flightState

	go readReceiver(hw.UART0, packetChan)

	// Main flight loop ticker (200Hz)
	ticker := time.NewTicker(time.Duration(dt * float32(time.Second)))
	defer ticker.Stop()

	// GPS read ticker (5Hz)
	var gpsTicker *time.Ticker
	if GPSEnabled {
		gpsTicker = time.NewTicker(time.Duration(1000/GPSReadRate) * time.Millisecond)
		defer gpsTicker.Stop()
	}

	watchdog.Start()

	for {
		select {
		case packet := <-packetChan:
			LastPacketTime = time.Now()
			Channels = processReceiverPacket(packet)

		case <-conditionalGPSTicker(gpsTicker):
			if GPSEnabled && flightState == FLIGHT_MODE {
				newGPSData, err := hw.GPS.ReadGPS()
				if err == nil && newGPSData.Valid {
					gpsData = newGPSData

					// Capture home on first valid fix in FLIGHT_MODE
					if homeNotCaptured {
						if err := hw.GPS.CaptureHome(); err == nil {
							// Store home in navigation state for RTH
							gpsAdapter := hw.GPS // Type assert to GPSAdapter
							navState.RTHTargetLat = gpsAdapter.config.HomeLatitude
							navState.RTHTargetLon = gpsAdapter.config.HomeLongitude
							navState.RTHHoldAltitude = gpsAdapter.config.HomeAltitude
							homeNotCaptured = false
						}
					}

					// Debug output
					print("GPS: Lat=")
					print(gpsData.Latitude)
					print(", Lon=")
					print(gpsData.Longitude)
					print(", Alt=")
					print(gpsData.Altitude)
					print("m, Sats=")
					print(gpsData.Satellites)
					print(", Valid=")
					println(gpsData.Valid)
				}
			}

		default:
			<-ticker.C

			if time.Since(LastPacketTime).Milliseconds() > FAILSAFE_TIMEOUT_MS && flightState != FAILSAFE && flightState != RETURN_TO_HOME {
				// Check if RTH is enabled and GPS has a valid fix
				if RTHEnabled && gpsData.Valid && !homeNotCaptured {
					flightState = RETURN_TO_HOME
					rthActive = true
					signalLostStartTime = time.Now()
					navState.RTHHoldAltitude = gpsData.Altitude // Hold current altitude
					println("RTH: Failsafe triggered, activating Return-to-Home")
				} else {
					flightState = FAILSAFE
				}
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
				pitchInput := mapRange(float32(Channels[ElevatorChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_PITCH_RATE, MAX_PITCH_RATE)
				rollInput := mapRange(float32(Channels[AileronChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_ROLL_RATE, MAX_ROLL_RATE)
				yawInput := mapRange(float32(Channels[YawChannel]), MIN_RX_VALUE, MAX_RX_VALUE, -MAX_YAW_RATE, MAX_YAW_RATE)

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

					// Calculate errors between desired rates and actual rates for PID
					pitchError := pitchInput - imuData.Pitch
					rollError := rollInput - imuData.Roll
					yawError := yawInput - imuData.Yaw

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
					if DSHOT {
						dshotThrottle := MapThrottle(Channels[ThrottleChannel])
						hw.DShot.SendThrottle(dshotThrottle, false)
					} else {
						setESC(uint32(Channels[ThrottleChannel]))
					}
				} else {
					if DSHOT {
						hw.DShot.SendThrottle(0, false) // Send zero throttle command to disarm ESC safely
					} else {
						setESC(MIN_PULSE_WIDTH_US)
					}
				}

			case FAILSAFE:
				setAllServos(NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE, NEUTRAL_RX_VALUE)
				setESC(MIN_PULSE_WIDTH_US)
				if time.Since(LastPacketTime).Milliseconds() <= FAILSAFE_TIMEOUT_MS {
					flightState = FLIGHT_MODE
				}

			case RETURN_TO_HOME:
				armed = Channels[ArmChannel] > HIGH_RX_VALUE

				// Update navigation state with current GPS position
				if gpsData.Valid {
					UpdateRTHNavigation(navState, gpsData, imuData, gpsData.Altitude)

					// Generate navigation control inputs
					// Yaw control: bearing to home
					// Convert heading error (radians) to rate-like error for PID (scale by Kp factor)
					yawHeadingError := CalculateHeadingError(navState.RTHDesiredHeading, imuData.Yaw)
					yawError := yawHeadingError * 2.0 // Scale factor to convert heading error to rate-like command
					yawMix := yawPID.Update(yawError, dt) * PID_WEIGHT

					// Pitch control: altitude hold
					altitudeError := CalculateAltitudeError(gpsData.Altitude, navState.RTHHoldAltitude)
					var pitchMix, rollMix float32
					if altitudeError > RTHAltitudeDeadzone {
						// Need to climb
						pitchMix = 0.2 // Climb pitch
					} else if altitudeError < -RTHAltitudeDeadzone {
						// Need to descend
						pitchMix = -0.2 // Descend pitch
					}
					// Roll control: wings level for RTH (no lateral correction yet)
					rollMix = 0.0

					// Apply mixer
					s1, s2, s4, s5, s6 := ApplyMixer(pitchMix, rollMix, yawMix)

					// Map outputs and apply servos
					s1 = mapRange(s1, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
					s2 = mapRange(s2, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
					s4 = mapRange(s4, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
					s5 = mapRange(s5, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
					s6 = mapRange(s6, -MAX_ROLL_RATE, MAX_ROLL_RATE, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)

					// Apply reversals
					s1 = applyReversal(s1, SERVO1_REVERSE, float32(SERVO1_MIN), float32(SERVO1_MAX))
					s2 = applyReversal(s2, SERVO2_REVERSE, float32(SERVO2_MIN), float32(SERVO2_MAX))
					s4 = applyReversal(s4, SERVO4_REVERSE, float32(SERVO4_MIN), float32(SERVO4_MAX))
					s5 = applyReversal(s5, SERVO5_REVERSE, float32(SERVO5_MIN), float32(SERVO5_MAX))
					s6 = applyReversal(s6, SERVO6_REVERSE, float32(SERVO6_MIN), float32(SERVO6_MAX))

					// Add trims and constrain
					pulse1 := uint32(constrain(s1+float32(SERVO1_SUBTRIM), float32(SERVO1_MIN), float32(SERVO1_MAX)))
					pulse2 := uint32(constrain(s2+float32(SERVO2_SUBTRIM), float32(SERVO2_MIN), float32(SERVO2_MAX)))
					pulse4 := uint32(constrain(s4+float32(SERVO4_SUBTRIM), float32(SERVO4_MIN), float32(SERVO4_MAX)))
					pulse5 := uint32(constrain(s5+float32(SERVO5_SUBTRIM), float32(SERVO5_MIN), float32(SERVO5_MAX)))
					pulse6 := uint32(constrain(s6+float32(SERVO6_SUBTRIM), float32(SERVO6_MIN), float32(SERVO6_MAX)))

					setAllServos(pulse1, pulse2, pulse4, pulse5, pulse6)

					// Maintain cruise throttle for RTH (use mid-range throttle)
					if armed {
						if DSHOT {
							dshotThrottle := uint16(mapRange(1500, MIN_RX_VALUE, MAX_RX_VALUE, 0, 2000)) // Neutral throttle
							hw.DShot.SendThrottle(dshotThrottle, false)
						} else {
							setESC(1500) // Mid-range PWM
						}
					} else {
						if DSHOT {
							hw.DShot.SendThrottle(0, false)
						} else {
							setESC(MIN_PULSE_WIDTH_US)
						}
					}

					// Check if RTH is complete or signal recovered
					if IsRTHComplete(navState) {
						println("RTH: Reached home, switching to FAILSAFE")
						flightState = FAILSAFE
						rthActive = false
					} else if time.Since(LastPacketTime).Milliseconds() <= FAILSAFE_TIMEOUT_MS {
						// Signal recovered, return to FLIGHT_MODE
						println("RTH: Signal recovered, returning to FLIGHT_MODE")
						flightState = FLIGHT_MODE
						rthActive = false
					}
				} else {
					// GPS lost during RTH, switch to FAILSAFE
					println("RTH: GPS signal lost, switching to FAILSAFE")
					flightState = FAILSAFE
					rthActive = false
				}
			}

			watchdog.Update()
		}
	}
}
