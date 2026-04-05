package main

// WingFC Configuration
// All user-configurable parameters and hardware mappings
//
// All the configurable values are defined here, making it easy to tune the
// flight controller without changing the main application logic.

// --- Protocol Settings ---
const (
	// Number of supported RC channels
	NumChannels = 18
)

// --- Receiver Configuration ---
// --- Channel Mappings ---

const (
	AileronChannel    = 0 // CH1
	ElevatorChannel   = 1 // CH2
	ThrottleChannel   = 2 // CH3
	ArmChannel        = 4 // CH5
	ManualModeChannel = 7 // CH8
)

// --- PWM Configuration ---
const (
	// Analog servo frequency 50Hz
	// Digital servo frequency 100Hz 250Hz 333Hz etc.
	SERVO_PWM_FREQUENCY = 50

	// ESC frequency set at analog servo frequency 50Hz
	// another common ESC frequency is 400Hz
	ESC_PWM_FREQUENCY = 50

	// Deadband around neutral for stick input
	DEADBAND = 10

	// High Rx channel value for arming/calibration
	HIGH_RX_VALUE = 1800

	// Left Elevon (Aileron) - servo travel limits and trim
	LEFT_SERVO_MIN     = 1100 // Minimum pulse width (µs)
	LEFT_SERVO_MAX     = 1900 // Maximum pulse width (µs)
	LEFT_SERVO_SUBTRIM = 0    // Neutral offset (µs), e.g., +50 moves servo toward max

	// Right Elevon (Elevator) - servo travel limits and trim
	RIGHT_SERVO_MIN     = 1100
	RIGHT_SERVO_MAX     = 1900
	RIGHT_SERVO_SUBTRIM = 0
)

// --- Flight Control Parameters ---
const (
	// Maximum desired pitch rate in degrees/sec
	MAX_PITCH_RATE_DEG = 200

	// Maximum desired roll rate in degrees/sec
	MAX_ROLL_RATE_DEG = 600

	// Weighting for combining gyro/accel with input
	PID_WEIGHT float32 = 0.5

	// LPF bit-shift levels for integer filtering
	// Level 0: No filtering (full weight to new value)
	// Level 1: >> 2 shift (alpha ≈ 0.25, more responsive)
	// Level 2: >> 3 shift (alpha ≈ 0.125, more smoothing)
	LPF_BITSHIFT_LEVEL = 1 // Choose: 0, 1, or 2

	// PID gains (P, I, D) for the pitch and roll controllers
	pP, pI, pD float32 = 1., 0.1, 0.01
	rP, rI, rD float32 = 1., 0.1, 0.01
)
