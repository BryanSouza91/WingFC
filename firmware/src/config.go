package main

// WingFC Configuration
// All user-configurable parameters and hardware mappings
//
// All the configurable values are defined here, making it easy to tune the
// flight controller without changing the main application logic.

// --- Protocol Settings ---
const (
	NumChannels = 18
)

// --- Board Orientation ---
// 0: Default, 1: CW90, 2: CW180, 3: CW270, 4: flip, 5: flipCW90, 6: flipCW180, 7: flipCW270
const ORIENTATION = 0

// --- Airframe Configuration ---
const (
	AIRFRAME_ELEVON = iota
	AIRFRAME_SINGLE_AILERON_T_TAIL
	AIRFRAME_DUAL_AILERON_T_TAIL
	AIRFRAME_SINGLE_AILERON_V_TAIL
	AIRFRAME_DUAL_AILERON_V_TAIL
)

// Set your aircraft type here
const AIRCRAFT_TYPE = AIRFRAME_ELEVON

// --- Receiver Configuration ---
const (
	AileronChannel    = 0 // CH1 (Roll)
	ElevatorChannel   = 1 // CH2 (Pitch)
	ThrottleChannel   = 2 // CH3 (Throttle)
	YawChannel        = 3 // CH4 (Yaw)
	ArmChannel        = 4 // CH5
	ManualModeChannel = 5 // CH6

	TuningChannelA = 6 // CH7 for tuning parameter A
	TuningChannelB = 7 // CH8 for tuning parameter B
	TuningChannelC = 8 // CH9 for tuning parameter C
	TuningChannelD = 9 // CH10 for tuning parameter D
)

// --- PWM & Hardware Configuration ---
const (
	// PWM Frequencies
	SERVO_PWM_FREQUENCY = 50  // 50Hz for analog servos
	ESC_PWM_FREQUENCY   = 400 // 400Hz for high-speed ESC

	DEADBAND      = 10
	HIGH_RX_VALUE = 1800

	// RC Receiver channel value constants
	MIN_RX_VALUE     = 988
	MAX_RX_VALUE     = 2012
	NEUTRAL_RX_VALUE = 1500
)

// --- Servo Configuration (Limits, Trims, & Reversals) ---
const (
	// Servo 1: Primary Aileron / Left Elevon
	SERVO1_MIN     = 1100
	SERVO1_MAX     = 1900
	SERVO1_SUBTRIM = 0
	SERVO1_REVERSE = false

	// Servo 2: Primary Elevator / Right Elevon
	SERVO2_MIN     = 1100
	SERVO2_MAX     = 1900
	SERVO2_SUBTRIM = 0
	SERVO2_REVERSE = false

	// Servo 4: Rudder / V-Tail
	SERVO4_MIN     = 1100
	SERVO4_MAX     = 1900
	SERVO4_SUBTRIM = 0
	SERVO4_REVERSE = false

	// Servo 5: Secondary Aileron
	SERVO5_MIN     = 1100
	SERVO5_MAX     = 1900
	SERVO5_SUBTRIM = 0
	SERVO5_REVERSE = false

	// Servo 6: Aux
	SERVO6_MIN     = 1100
	SERVO6_MAX     = 1900
	SERVO6_SUBTRIM = 0
	SERVO6_REVERSE = false
)

// --- Flight Control Parameters ---
const (
	MAX_PITCH_RATE_DEG = 200
	MAX_ROLL_RATE_DEG  = 600
	MAX_YAW_RATE_DEG   = 100

	PID_WEIGHT float32 = 0.5

	// Low-pass filter level for gyro data (higher = smoother but more lag)
	// 0 = no filtering, 1 = mild (>> 2), 2 = stronger (>> 3)
	LPF_BITSHIFT_LEVEL = 1

	// PID Gains
	pP, pI, pD float32 = 1.0, 0.1, 0.01
	rP, rI, rD float32 = 1.0, 0.1, 0.01
	yP, yI, yD float32 = 0.5, 0.05, 0.01 // Yaw PID gains
)

// --- Tuning Parameters ---
// Set TuneParameter to 0 to disable tuning, or 1-6 to enable
const (
	// Parameter selection (0 = disabled)
	// 1: Pitch P, 2: Roll P
	// 3: Pitch I, 4: Roll I
	// 5: Pitch D, 6: Roll D
	TuneParameterA = 0 // Disabled by default
	TuneParameterB = 0
	TuneParameterC = 0
	TuneParameterD = 0

	// Parameter ranges (adjusted via stick position 988-2012)
	TuneParameterAmin float32 = 0.1
	TuneParameterAmax float32 = 2.0

	TuneParameterBmin float32 = 0.01
	TuneParameterBmax float32 = 0.5

	TuneParameterCmin float32 = 0.001
	TuneParameterCmax float32 = 0.1

	TuneParameterDmin float32 = 0.001
	TuneParameterDmax float32 = 0.1
)
