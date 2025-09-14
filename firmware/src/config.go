package main

// WingFC Configuration
// All user-configurable parameters and hardware mappings

// --- Protocol Selection ---
const (
	NumChannels      = 18 // Number of supported RC channels
	activeProtocol   = PROTOCOL_IBUS // Set protocol: PROTOCOL_IBUS, PROTOCOL_CRSF, PROTOCOL_ELRS
)

// --- PWM Configuration ---
const (
	SERVO_PWM_FREQUENCY = 200 // Standard servo frequency (Hz)
	ESC_PWM_FREQUENCY   = 500 // ESC frequency (Hz)
	DEADBAND            = 20  // Deadband around neutral
	HIGH_RX_VALUE       = 1800 // High Rx channel value for arming/calibration
)

// --- Flight Control Parameters ---
const (
	MAX_ROLL_RATE_DEG  = 600 // degrees/sec
	MAX_PITCH_RATE_DEG = 200 // degrees/sec
	PID_WEIGHT         = 0.5 // Weighting for combining gyro/accel with input
	P, I, D            = 0.5, 0.1, 0.2 // PID gains
)

// --- Hardware Mappings ---
const (
	PWM_CH1_PIN = machine.D0 // Aileron Servo
	PWM_CH2_PIN = machine.D1 // Elevator Servo
	PWM_CH3_PIN = machine.D2 // Throttle (ESC)
)

// --- Channel Mapping ---
var (
	aileronCh  = Channels[0] // Rx channel 1
	elevatorCh = Channels[1] // Rx channel 2
	throttleCh = Channels[2] // Rx channel 3
	armCh      = Channels[4] // Rx channel 5
	calCh      = Channels[5] // Rx channel 6
)

// --- Hardware Interfaces ---
var (
	pwm0 = machine.PWM0 // Servo PWM
	pwm1 = machine.PWM1 // ESC PWM
)
