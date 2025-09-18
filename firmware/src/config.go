package main

import "machine"
// WingFC Configuration
// All user-configurable parameters and hardware mappings

// --- Protocol Selection ---
const (
	NumChannels    = 18
	activeProtocol = PROTOCOL_IBUS // Set protocol: PROTOCOL_IBUS, PROTOCOL_CRSF, PROTOCOL_ELRS
)

// --- PWM Configuration ---
const (
	SERVO_PWM_FREQUENCY = 50   // Standard servo frequency (Hz)
	ESC_PWM_FREQUENCY   = 500  // ESC frequency (Hz)
	DEADBAND            = 20   // Deadband around neutral
	HIGH_RX_VALUE       = 1800 // High Rx channel value for arming/calibration
)

// --- Flight Control Parameters ---
const (
	MAX_ROLL_RATE_DEG  = 600           // degrees/sec
	MAX_PITCH_RATE_DEG = 200           // degrees/sec
	PID_WEIGHT         = 0.5           // Weighting for combining gyro/accel with input
	P, I, D            = 0.5, 0.1, 0.2 // PID gains
)

// --- Hardware Mappings ---
// The following pins are specific to the Xiao BLE
const (
	PWM_CH1_PIN = machine.D0
	PWM_CH2_PIN = machine.D1
	PWM_CH3_PIN = machine.D2 // The ESC needs its own pin and PWM peripheral
)
