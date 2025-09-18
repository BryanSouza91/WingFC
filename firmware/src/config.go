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

// --- PWM Configuration ---
const (
	// Standard servo frequency (Hz)
	SERVO_PWM_FREQUENCY = 50

	// ESC frequency (Hz)
	ESC_PWM_FREQUENCY = 50

	// Deadband around neutral for stick input
	DEADBAND = 5

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

	// PID gains (P, I, D) for the pitch and roll controllers
	pP, pI, pD = 1., 0.1, 0.01
	rP, rI, rD = 1., 0.1, 0.01
)
