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
	AileronChannel  = 0 // CH1
	ElevatorChannel = 1 // CH2
	ThrottleChannel = 2 // CH3
	ArmChannel      = 4 // CH5
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
)

// --- Flight Control Parameters ---
const (
	// Maximum desired pitch rate in degrees/sec
	MAX_PITCH_RATE_DEG = 200

	// Maximum desired roll rate in degrees/sec
	MAX_ROLL_RATE_DEG = 600

	// Weighting for combining gyro/accel with input
	PID_WEIGHT = 0.5

	// LPF alpha for gyro/accel fusion
	LPF_ALPHA = 0.2

	// PID gains (P, I, D) for the pitch and roll controllers
	pP, pI, pD = 1., 0.1, 0.01
	rP, rI, rD = 1., 0.1, 0.01
)
