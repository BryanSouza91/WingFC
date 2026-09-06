package main

import (
	"bytes"
	"flag"
	"fmt"
	"go/format"
	"os"
	"path/filepath"
	"strings"
	"text/template"

	"gopkg.in/yaml.v3"
)

type Config struct {
	Version       string              `yaml:"version"`
	Protocol      ProtocolConfig      `yaml:"protocol"`
	Board         BoardConfig         `yaml:"board"`
	Airframe      AirframeConfig      `yaml:"airframe"`
	Receiver      ReceiverConfig      `yaml:"receiver"`
	PWM           PWMConfig           `yaml:"pwm"`
	Servos        ServosConfig        `yaml:"servos"`
	FlightControl FlightControlConfig `yaml:"flight_control"`
	Tuning        TuningConfig        `yaml:"tuning"`
}

type ProtocolConfig struct {
	NumChannels int `yaml:"num_channels"`
}

type BoardConfig struct {
	Orientation yaml.Node `yaml:"orientation"`
}

type AirframeConfig struct {
	Type yaml.Node `yaml:"type"`
}

type ReceiverConfig struct {
	Channels       ReceiverChannels `yaml:"channels"`
	Deadband       int              `yaml:"deadband"`
	HighRxValue    int              `yaml:"high_rx_value"`
	MinRxValue     int              `yaml:"min_rx_value"`
	MaxRxValue     int              `yaml:"max_rx_value"`
	NeutralRxValue int              `yaml:"neutral_rx_value"`
}

type ReceiverChannels struct {
	Aileron    int `yaml:"aileron"`
	Elevator   int `yaml:"elevator"`
	Throttle   int `yaml:"throttle"`
	Yaw        int `yaml:"yaw"`
	Arm        int `yaml:"arm"`
	ManualMode int `yaml:"manual_mode"`
	TuningA    int `yaml:"tuning_a"`
	TuningB    int `yaml:"tuning_b"`
	TuningC    int `yaml:"tuning_c"`
	TuningD    int `yaml:"tuning_d"`
}

type PWMConfig struct {
	Dshot            bool `yaml:"dshot"`
	DshotMode        int  `yaml:"dshot_mode"`
	ServoFrequencyHz int  `yaml:"servo_frequency_hz"`
	EscFrequencyHz   int  `yaml:"esc_frequency_hz"`
}

type ServoConfig struct {
	Min     int  `yaml:"min"`
	Max     int  `yaml:"max"`
	Subtrim int  `yaml:"subtrim"`
	Reverse bool `yaml:"reverse"`
}

type ServosConfig struct {
	Servo1 ServoConfig `yaml:"servo1"`
	Servo2 ServoConfig `yaml:"servo2"`
	Servo4 ServoConfig `yaml:"servo4"`
	Servo5 ServoConfig `yaml:"servo5"`
	Servo6 ServoConfig `yaml:"servo6"`
}

type FlightControlConfig struct {
	MaxRatesDeg  MaxRatesConfig  `yaml:"max_rates_deg"`
	MaxAnglesDeg MaxAnglesConfig `yaml:"max_angles_deg"`
	PIDWeight    float32         `yaml:"pid_weight"`
	Filters      FiltersConfig   `yaml:"filters"`
	PID          PIDConfig       `yaml:"pid"`
}

type MaxRatesConfig struct {
	Pitch int `yaml:"pitch"`
	Roll  int `yaml:"roll"`
	Yaw   int `yaml:"yaw"`
}

type MaxAnglesConfig struct {
	Pitch float32 `yaml:"pitch"`
	Roll  float32 `yaml:"roll"`
}

type FiltersConfig struct {
	AccelLevel int `yaml:"accel_level"`
	GyroLevel  int `yaml:"gyro_level"`
}

type PIDGains struct {
	P float32 `yaml:"p"`
	I float32 `yaml:"i"`
	D float32 `yaml:"d"`
}

type PIDConfig struct {
	Pitch PIDGains `yaml:"pitch"`
	Roll  PIDGains `yaml:"roll"`
	Yaw   PIDGains `yaml:"yaw"`
}

type TuningParamConfig struct {
	Parameter yaml.Node `yaml:"parameter"`
	Min       float32   `yaml:"min"`
	Max       float32   `yaml:"max"`
}

type TuningConfig struct {
	ParamA TuningParamConfig `yaml:"param_a"`
	ParamB TuningParamConfig `yaml:"param_b"`
	ParamC TuningParamConfig `yaml:"param_c"`
	ParamD TuningParamConfig `yaml:"param_d"`
}

// ValidatedConfig contains resolved enum and validated parameters ready for code generation.
type ValidatedConfig struct {
	SourceFile          string
	NumChannels         int
	Orientation         int
	OrientationComment  string
	AirframeTypeConst   string
	AileronChannel      int
	ElevatorChannel     int
	ThrottleChannel     int
	YawChannel          int
	ArmChannel          int
	ManualModeChannel   int
	TuningChannelA      int
	TuningChannelB      int
	TuningChannelC      int
	TuningChannelD      int
	DSHOT               bool
	DSHOT_MODE          int
	SERVO_PWM_FREQUENCY int
	ESC_PWM_FREQUENCY   int
	DEADBAND            int
	HIGH_RX_VALUE       int
	MIN_RX_VALUE        int
	MAX_RX_VALUE        int
	NEUTRAL_RX_VALUE    int
	Servo1              ServoConfig
	Servo2              ServoConfig
	Servo4              ServoConfig
	Servo5              ServoConfig
	Servo6              ServoConfig
	MaxPitchRateDeg     int
	MaxRollRateDeg      int
	MaxYawRateDeg       int
	MaxPitchAngleDeg    float32
	MaxRollAngleDeg     float32
	PIDWeight           float32
	LPFAccelLevel       int
	LPFGyroLevel        int
	PitchPID            PIDGains
	RollPID             PIDGains
	YawPID              PIDGains
	TuneParamA          int
	TuneParamAmin       float32
	TuneParamAmax       float32
	TuneParamB          int
	TuneParamBmin       float32
	TuneParamBmax       float32
	TuneParamC          int
	TuneParamCmin       float32
	TuneParamCmax       float32
	TuneParamD          int
	TuneParamDmin       float32
	TuneParamDmax       float32
}

var orientationMap = map[string]int{
	"DEFAULT":    0,
	"CW90":       1,
	"CW180":      2,
	"CW270":      3,
	"FLIP":       4,
	"FLIP_CW90":  5,
	"FLIP_CW180": 6,
	"FLIP_CW270": 7,
}

var airframeTypes = map[string]string{
	"AIRFRAME_ELEVON":                "AIRFRAME_ELEVON",
	"ELEVON":                         "AIRFRAME_ELEVON",
	"AIRFRAME_SINGLE_AILERON_T_TAIL": "AIRFRAME_SINGLE_AILERON_T_TAIL",
	"SINGLE_AILERON_T_TAIL":          "AIRFRAME_SINGLE_AILERON_T_TAIL",
	"T_TAIL_SINGLE":                  "AIRFRAME_SINGLE_AILERON_T_TAIL",
	"AIRFRAME_DUAL_AILERON_T_TAIL":   "AIRFRAME_DUAL_AILERON_T_TAIL",
	"DUAL_AILERON_T_TAIL":            "AIRFRAME_DUAL_AILERON_T_TAIL",
	"T_TAIL_DUAL":                    "AIRFRAME_DUAL_AILERON_T_TAIL",
	"AIRFRAME_SINGLE_AILERON_V_TAIL": "AIRFRAME_SINGLE_AILERON_V_TAIL",
	"SINGLE_AILERON_V_TAIL":          "AIRFRAME_SINGLE_AILERON_V_TAIL",
	"V_TAIL_SINGLE":                  "AIRFRAME_SINGLE_AILERON_V_TAIL",
	"AIRFRAME_DUAL_AILERON_V_TAIL":   "AIRFRAME_DUAL_AILERON_V_TAIL",
	"DUAL_AILERON_V_TAIL":            "AIRFRAME_DUAL_AILERON_V_TAIL",
	"V_TAIL_DUAL":                    "AIRFRAME_DUAL_AILERON_V_TAIL",
}

var tuningParamMap = map[string]int{
	"DISABLED": 0,
	"NONE":     0,
	"PITCH_P":  1,
	"ROLL_P":   2,
	"PITCH_I":  3,
	"ROLL_I":   4,
	"PITCH_D":  5,
	"ROLL_D":   6,
}

func parseOrientation(node yaml.Node) (int, error) {
	var intVal int
	if err := node.Decode(&intVal); err == nil {
		if intVal < 0 || intVal > 7 {
			return 0, fmt.Errorf("board orientation must be between 0 and 7, got %d", intVal)
		}
		return intVal, nil
	}
	var strVal string
	if err := node.Decode(&strVal); err == nil {
		key := strings.ToUpper(strings.TrimSpace(strVal))
		if val, ok := orientationMap[key]; ok {
			return val, nil
		}
		return 0, fmt.Errorf("unknown board orientation '%s'", strVal)
	}
	return 0, fmt.Errorf("invalid orientation format: %v", node.Value)
}

func parseAirframeType(node yaml.Node) (string, error) {
	var intVal int
	if err := node.Decode(&intVal); err == nil {
		switch intVal {
		case 0:
			return "AIRFRAME_ELEVON", nil
		case 1:
			return "AIRFRAME_SINGLE_AILERON_T_TAIL", nil
		case 2:
			return "AIRFRAME_DUAL_AILERON_T_TAIL", nil
		case 3:
			return "AIRFRAME_SINGLE_AILERON_V_TAIL", nil
		case 4:
			return "AIRFRAME_DUAL_AILERON_V_TAIL", nil
		default:
			return "", fmt.Errorf("airframe type integer must be 0-4, got %d", intVal)
		}
	}
	var strVal string
	if err := node.Decode(&strVal); err == nil {
		key := strings.ToUpper(strings.TrimSpace(strVal))
		if val, ok := airframeTypes[key]; ok {
			return val, nil
		}
		return "", fmt.Errorf("unknown airframe type '%s'", strVal)
	}
	return "", fmt.Errorf("invalid airframe type format: %v", node.Value)
}

func parseTuningParam(node yaml.Node) (int, error) {
	if node.Kind == 0 && node.Value == "" {
		return 0, nil
	}
	var intVal int
	if err := node.Decode(&intVal); err == nil {
		if intVal < 0 || intVal > 6 {
			return 0, fmt.Errorf("tuning parameter ID must be between 0 and 6, got %d", intVal)
		}
		return intVal, nil
	}
	var strVal string
	if err := node.Decode(&strVal); err == nil {
		key := strings.ToUpper(strings.TrimSpace(strVal))
		if val, ok := tuningParamMap[key]; ok {
			return val, nil
		}
		return 0, fmt.Errorf("unknown tuning parameter '%s' (valid: 0-6, DISABLED, PITCH_P, ROLL_P, PITCH_I, ROLL_I, PITCH_D, ROLL_D)", strVal)
	}
	return 0, fmt.Errorf("invalid tuning parameter format: %v", node.Value)
}

func validateServo(name string, s ServoConfig) error {
	if s.Min < 500 || s.Min > 2500 {
		return fmt.Errorf("%s min (%d) is outside safe pulse range [500, 2500]", name, s.Min)
	}
	if s.Max < 500 || s.Max > 2500 {
		return fmt.Errorf("%s max (%d) is outside safe pulse range [500, 2500]", name, s.Max)
	}
	if s.Min >= s.Max {
		return fmt.Errorf("%s min (%d) must be less than max (%d)", name, s.Min, s.Max)
	}
	return nil
}

func ValidateAndResolve(cfg *Config, sourceName string) (*ValidatedConfig, error) {
	if cfg.Protocol.NumChannels <= 0 || cfg.Protocol.NumChannels > 32 {
		return nil, fmt.Errorf("protocol.num_channels must be between 1 and 32, got %d", cfg.Protocol.NumChannels)
	}

	orientation, err := parseOrientation(cfg.Board.Orientation)
	if err != nil {
		return nil, err
	}

	airframe, err := parseAirframeType(cfg.Airframe.Type)
	if err != nil {
		return nil, err
	}

	// Validate channels
	numCh := cfg.Protocol.NumChannels
	chMap := map[string]int{
		"aileron":     cfg.Receiver.Channels.Aileron,
		"elevator":    cfg.Receiver.Channels.Elevator,
		"throttle":    cfg.Receiver.Channels.Throttle,
		"yaw":         cfg.Receiver.Channels.Yaw,
		"arm":         cfg.Receiver.Channels.Arm,
		"manual_mode": cfg.Receiver.Channels.ManualMode,
		"tuning_a":    cfg.Receiver.Channels.TuningA,
		"tuning_b":    cfg.Receiver.Channels.TuningB,
		"tuning_c":    cfg.Receiver.Channels.TuningC,
		"tuning_d":    cfg.Receiver.Channels.TuningD,
	}

	for chName, chIdx := range chMap {
		if chIdx < 0 || chIdx >= numCh {
			return nil, fmt.Errorf("receiver channel '%s' (%d) is out of range [0, %d)", chName, chIdx, numCh)
		}
	}

	// Validate RX pulse bounds
	if cfg.Receiver.MinRxValue >= cfg.Receiver.NeutralRxValue || cfg.Receiver.NeutralRxValue >= cfg.Receiver.MaxRxValue {
		return nil, fmt.Errorf("receiver pulses must satisfy min (%d) < neutral (%d) < max (%d)",
			cfg.Receiver.MinRxValue, cfg.Receiver.NeutralRxValue, cfg.Receiver.MaxRxValue)
	}

	// Validate PWM
	if cfg.PWM.Dshot {
		if cfg.PWM.DshotMode != 150 && cfg.PWM.DshotMode != 300 && cfg.PWM.DshotMode != 600 {
			return nil, fmt.Errorf("pwm.dshot_mode must be 150, 300, or 600, got %d", cfg.PWM.DshotMode)
		}
	}
	if cfg.PWM.ServoFrequencyHz < 50 || cfg.PWM.ServoFrequencyHz > 400 {
		return nil, fmt.Errorf("pwm.servo_frequency_hz (%d) must be between 50 and 400", cfg.PWM.ServoFrequencyHz)
	}
	if cfg.PWM.EscFrequencyHz < 50 || cfg.PWM.EscFrequencyHz > 1000 {
		return nil, fmt.Errorf("pwm.esc_frequency_hz (%d) must be between 50 and 1000", cfg.PWM.EscFrequencyHz)
	}

	// Validate Servos
	if err := validateServo("servo1", cfg.Servos.Servo1); err != nil {
		return nil, err
	}
	if err := validateServo("servo2", cfg.Servos.Servo2); err != nil {
		return nil, err
	}
	if err := validateServo("servo4", cfg.Servos.Servo4); err != nil {
		return nil, err
	}
	if err := validateServo("servo5", cfg.Servos.Servo5); err != nil {
		return nil, err
	}
	if err := validateServo("servo6", cfg.Servos.Servo6); err != nil {
		return nil, err
	}

	// Validate Flight Control
	fc := cfg.FlightControl
	if fc.MaxRatesDeg.Pitch <= 0 || fc.MaxRatesDeg.Roll <= 0 || fc.MaxRatesDeg.Yaw <= 0 {
		return nil, fmt.Errorf("max_rates_deg must be positive values, got pitch=%d roll=%d yaw=%d",
			fc.MaxRatesDeg.Pitch, fc.MaxRatesDeg.Roll, fc.MaxRatesDeg.Yaw)
	}
	if fc.MaxAnglesDeg.Pitch <= 0 || fc.MaxAnglesDeg.Pitch > 90 || fc.MaxAnglesDeg.Roll <= 0 || fc.MaxAnglesDeg.Roll > 90 {
		return nil, fmt.Errorf("max_angles_deg must be between 0 and 90 degrees, got pitch=%.1f roll=%.1f",
			fc.MaxAnglesDeg.Pitch, fc.MaxAnglesDeg.Roll)
	}
	if fc.Filters.AccelLevel < 0 || fc.Filters.AccelLevel > 2 {
		return nil, fmt.Errorf("filters.accel_level must be 0, 1, or 2, got %d", fc.Filters.AccelLevel)
	}
	if fc.Filters.GyroLevel < 0 || fc.Filters.GyroLevel > 2 {
		return nil, fmt.Errorf("filters.gyro_level must be 0, 1, or 2, got %d", fc.Filters.GyroLevel)
	}

	// Validate Tuning
	tA, err := parseTuningParam(cfg.Tuning.ParamA.Parameter)
	if err != nil {
		return nil, fmt.Errorf("tuning.param_a: %w", err)
	}
	tB, err := parseTuningParam(cfg.Tuning.ParamB.Parameter)
	if err != nil {
		return nil, fmt.Errorf("tuning.param_b: %w", err)
	}
	tC, err := parseTuningParam(cfg.Tuning.ParamC.Parameter)
	if err != nil {
		return nil, fmt.Errorf("tuning.param_c: %w", err)
	}
	tD, err := parseTuningParam(cfg.Tuning.ParamD.Parameter)
	if err != nil {
		return nil, fmt.Errorf("tuning.param_d: %w", err)
	}

	return &ValidatedConfig{
		SourceFile:          sourceName,
		NumChannels:         cfg.Protocol.NumChannels,
		Orientation:         orientation,
		AirframeTypeConst:   airframe,
		AileronChannel:      cfg.Receiver.Channels.Aileron,
		ElevatorChannel:     cfg.Receiver.Channels.Elevator,
		ThrottleChannel:     cfg.Receiver.Channels.Throttle,
		YawChannel:          cfg.Receiver.Channels.Yaw,
		ArmChannel:          cfg.Receiver.Channels.Arm,
		ManualModeChannel:   cfg.Receiver.Channels.ManualMode,
		TuningChannelA:      cfg.Receiver.Channels.TuningA,
		TuningChannelB:      cfg.Receiver.Channels.TuningB,
		TuningChannelC:      cfg.Receiver.Channels.TuningC,
		TuningChannelD:      cfg.Receiver.Channels.TuningD,
		DSHOT:               cfg.PWM.Dshot,
		DSHOT_MODE:          cfg.PWM.DshotMode,
		SERVO_PWM_FREQUENCY: cfg.PWM.ServoFrequencyHz,
		ESC_PWM_FREQUENCY:   cfg.PWM.EscFrequencyHz,
		DEADBAND:            cfg.Receiver.Deadband,
		HIGH_RX_VALUE:       cfg.Receiver.HighRxValue,
		MIN_RX_VALUE:        cfg.Receiver.MinRxValue,
		MAX_RX_VALUE:        cfg.Receiver.MaxRxValue,
		NEUTRAL_RX_VALUE:    cfg.Receiver.NeutralRxValue,
		Servo1:              cfg.Servos.Servo1,
		Servo2:              cfg.Servos.Servo2,
		Servo4:              cfg.Servos.Servo4,
		Servo5:              cfg.Servos.Servo5,
		Servo6:              cfg.Servos.Servo6,
		MaxPitchRateDeg:     fc.MaxRatesDeg.Pitch,
		MaxRollRateDeg:      fc.MaxRatesDeg.Roll,
		MaxYawRateDeg:       fc.MaxRatesDeg.Yaw,
		MaxPitchAngleDeg:    fc.MaxAnglesDeg.Pitch,
		MaxRollAngleDeg:     fc.MaxAnglesDeg.Roll,
		PIDWeight:           fc.PIDWeight,
		LPFAccelLevel:       fc.Filters.AccelLevel,
		LPFGyroLevel:        fc.Filters.GyroLevel,
		PitchPID:            fc.PID.Pitch,
		RollPID:             fc.PID.Roll,
		YawPID:              fc.PID.Yaw,
		TuneParamA:          tA,
		TuneParamAmin:       cfg.Tuning.ParamA.Min,
		TuneParamAmax:       cfg.Tuning.ParamA.Max,
		TuneParamB:          tB,
		TuneParamBmin:       cfg.Tuning.ParamB.Min,
		TuneParamBmax:       cfg.Tuning.ParamB.Max,
		TuneParamC:          tC,
		TuneParamCmin:       cfg.Tuning.ParamC.Min,
		TuneParamCmax:       cfg.Tuning.ParamC.Max,
		TuneParamD:          tD,
		TuneParamDmin:       cfg.Tuning.ParamD.Min,
		TuneParamDmax:       cfg.Tuning.ParamD.Max,
	}, nil
}

const configTemplate = `// Code generated by configgen from {{ .SourceFile }}; DO NOT EDIT.
// To regenerate: go generate ./...
//
// WingFC Configuration
// All user-configurable parameters and hardware mappings
//
// All the configurable values are defined here, making it easy to tune the
// flight controller without changing the main application logic.

package main

//go:generate go run ../../tools/configgen -config ../../configs/default.yaml -out config.go

// --- Protocol Settings ---
const (
	NumChannels = {{ .NumChannels }}
)

// --- Board Orientation ---
// 0: Default, 1: CW90, 2: CW180, 3: CW270, 4: flip, 5: flipCW90, 6: flipCW180, 7: flipCW270
const ORIENTATION = {{ .Orientation }}

// Set your aircraft type here
// Options:
// AIRFRAME_ELEVON
// AIRFRAME_SINGLE_AILERON_T_TAIL
// AIRFRAME_DUAL_AILERON_T_TAIL
// AIRFRAME_SINGLE_AILERON_V_TAIL
// AIRFRAME_DUAL_AILERON_V_TAIL
const AIRCRAFT_TYPE = {{ .AirframeTypeConst }}

// --- Receiver Configuration ---
const (
	AileronChannel    = {{ .AileronChannel }} // CH1 (Roll)
	ElevatorChannel   = {{ .ElevatorChannel }} // CH2 (Pitch)
	ThrottleChannel   = {{ .ThrottleChannel }} // CH3 (Throttle)
	YawChannel        = {{ .YawChannel }} // CH4 (Yaw)
	ArmChannel        = {{ .ArmChannel }} // CH5
	ManualModeChannel = {{ .ManualModeChannel }} // CH6

	TuningChannelA = {{ .TuningChannelA }} // CH7 for tuning parameter A
	TuningChannelB = {{ .TuningChannelB }} // CH8 for tuning parameter B
	TuningChannelC = {{ .TuningChannelC }} // CH9 for tuning parameter C
	TuningChannelD = {{ .TuningChannelD }} // CH10 for tuning parameter D
)

// --- PWM & Hardware Configuration ---
const (
	// Use DShot for ESC throttle control, so no traditional PWM channel for throttle is needed.
	// DSHOT_MODE can be 150, 300, or 600 (representing DShot150, DShot300, DShot600)
	DSHOT      = {{ .DSHOT }}
	DSHOT_MODE = {{ .DSHOT_MODE }}

	// PWM Frequencies
	SERVO_PWM_FREQUENCY = {{ .SERVO_PWM_FREQUENCY }} // 50Hz for analog servos
	ESC_PWM_FREQUENCY   = {{ .ESC_PWM_FREQUENCY }} // 400Hz for high-speed ESC

	DEADBAND      = {{ .DEADBAND }}
	HIGH_RX_VALUE = {{ .HIGH_RX_VALUE }}

	// RC Receiver channel value constants
	MIN_RX_VALUE     = {{ .MIN_RX_VALUE }}
	MAX_RX_VALUE     = {{ .MAX_RX_VALUE }}
	NEUTRAL_RX_VALUE = {{ .NEUTRAL_RX_VALUE }}
)

// --- Servo Configuration (Limits, Trims, & Reversals) ---
const (
	// Servo 1: Primary Aileron / Left Elevon
	SERVO1_MIN     = {{ .Servo1.Min }}
	SERVO1_MAX     = {{ .Servo1.Max }}
	SERVO1_SUBTRIM = {{ .Servo1.Subtrim }}
	SERVO1_REVERSE = {{ .Servo1.Reverse }}

	// Servo 2: Primary Elevator / Right Elevon
	SERVO2_MIN     = {{ .Servo2.Min }}
	SERVO2_MAX     = {{ .Servo2.Max }}
	SERVO2_SUBTRIM = {{ .Servo2.Subtrim }}
	SERVO2_REVERSE = {{ .Servo2.Reverse }}

	// Servo 4: Rudder / V-Tail
	SERVO4_MIN     = {{ .Servo4.Min }}
	SERVO4_MAX     = {{ .Servo4.Max }}
	SERVO4_SUBTRIM = {{ .Servo4.Subtrim }}
	SERVO4_REVERSE = {{ .Servo4.Reverse }}

	// Servo 5: Secondary Aileron
	SERVO5_MIN     = {{ .Servo5.Min }}
	SERVO5_MAX     = {{ .Servo5.Max }}
	SERVO5_SUBTRIM = {{ .Servo5.Subtrim }}
	SERVO5_REVERSE = {{ .Servo5.Reverse }}

	// Servo 6: Aux
	SERVO6_MIN     = {{ .Servo6.Min }}
	SERVO6_MAX     = {{ .Servo6.Max }}
	SERVO6_SUBTRIM = {{ .Servo6.Subtrim }}
	SERVO6_REVERSE = {{ .Servo6.Reverse }}
)

// --- Flight Control Parameters ---
const (
	MAX_PITCH_RATE_DEG = {{ .MaxPitchRateDeg }}
	MAX_ROLL_RATE_DEG  = {{ .MaxRollRateDeg }}
	MAX_YAW_RATE_DEG   = {{ .MaxYawRateDeg }}

	// Maximum stabilized-mode angle commands from stick (degrees)
	// Stick at full deflection commands this many degrees of desired tilt.
	MAX_PITCH_ANGLE_DEG float32 = {{ printf "%.1f" .MaxPitchAngleDeg }}
	MAX_ROLL_ANGLE_DEG  float32 = {{ printf "%.1f" .MaxRollAngleDeg }}

	PID_WEIGHT float32 = {{ printf "%.1f" .PIDWeight }}

	// Low-pass filter levels (applied independently to accel and gyro).
	// 0 = no filtering, 1 = mild (α≈0.25, >>2), 2 = stronger (α≈0.125, >>3)
	//
	// Gyros are low-noise — keep at 0 for maximum stabilization responsiveness.
	// Accelerometers are vibration-prone — heavier filtering is appropriate.
	LPF_ACCEL_LEVEL = {{ .LPFAccelLevel }}
	LPF_GYRO_LEVEL  = {{ .LPFGyroLevel }}

	// PID Gains
	// D gain is intentionally 0: the derivative term amplifies noise by 1/dt (×200 at 200Hz).
	// Tune P and I first; only add D if oscillation persists after P/I are dialled in.
	pP, pI, pD float32 = {{ printf "%.2f" .PitchPID.P }}, {{ printf "%.3f" .PitchPID.I }}, {{ printf "%.2f" .PitchPID.D }}
	rP, rI, rD float32 = {{ printf "%.2f" .RollPID.P }}, {{ printf "%.3f" .RollPID.I }}, {{ printf "%.2f" .RollPID.D }}
	yP, yI, yD float32 = {{ printf "%.2f" .YawPID.P }}, {{ printf "%.3f" .YawPID.I }}, {{ printf "%.2f" .YawPID.D }} // Yaw PID gains
)

// --- Tuning Parameters ---
// Set TuneParameter to 0 to disable tuning, or 1-6 to enable
const (
	// Parameter selection (0 = disabled)
	// 1: Pitch P, 2: Roll P
	// 3: Pitch I, 4: Roll I
	// 5: Pitch D, 6: Roll D
	TuneParameterA = {{ .TuneParamA }} // Disabled by default
	TuneParameterB = {{ .TuneParamB }}
	TuneParameterC = {{ .TuneParamC }}
	TuneParameterD = {{ .TuneParamD }}

	// Parameter ranges (adjusted via stick position 988-2012)
	TuneParameterAmin float32 = {{ printf "%.4f" .TuneParamAmin }}
	TuneParameterAmax float32 = {{ printf "%.4f" .TuneParamAmax }}

	TuneParameterBmin float32 = {{ printf "%.4f" .TuneParamBmin }}
	TuneParameterBmax float32 = {{ printf "%.4f" .TuneParamBmax }}

	TuneParameterCmin float32 = {{ printf "%.4f" .TuneParamCmin }}
	TuneParameterCmax float32 = {{ printf "%.4f" .TuneParamCmax }}

	TuneParameterDmin float32 = {{ printf "%.4f" .TuneParamDmin }}
	TuneParameterDmax float32 = {{ printf "%.4f" .TuneParamDmax }}
)
`

func GenerateCode(vc *ValidatedConfig) ([]byte, error) {
	tmpl, err := template.New("config").Parse(configTemplate)
	if err != nil {
		return nil, fmt.Errorf("failed to parse template: %w", err)
	}

	var buf bytes.Buffer
	if err := tmpl.Execute(&buf, vc); err != nil {
		return nil, fmt.Errorf("failed to execute template: %w", err)
	}

	formatted, err := format.Source(buf.Bytes())
	if err != nil {
		return nil, fmt.Errorf("gofmt failed on generated code: %w\n%s", err, buf.String())
	}

	return formatted, nil
}

// findConfigsDir locates the configs directory from various working directories.
func findConfigsDir() string {
	candidates := []string{"configs", "../configs", "../../configs"}
	for _, c := range candidates {
		if info, err := os.Stat(c); err == nil && info.IsDir() {
			return c
		}
	}
	return "configs"
}

// findDefaultOutputFile locates the target config.go location.
func findDefaultOutputFile() string {
	candidates := []string{"firmware/src/config.go", "src/config.go", "config.go"}
	for _, c := range candidates {
		dir := filepath.Dir(c)
		if info, err := os.Stat(dir); err == nil && info.IsDir() {
			return c
		}
	}
	return "firmware/src/config.go"
}

func listProfiles(configsDir string) {
	entries, err := os.ReadDir(configsDir)
	if err != nil {
		fmt.Fprintf(os.Stderr, "Error reading configs directory %s: %v\n", configsDir, err)
		os.Exit(1)
	}

	fmt.Println("Available WingFC Aircraft Profiles:")
	for _, entry := range entries {
		if entry.IsDir() || (!strings.HasSuffix(entry.Name(), ".yaml") && !strings.HasSuffix(entry.Name(), ".yml")) {
			continue
		}
		path := filepath.Join(configsDir, entry.Name())
		data, err := os.ReadFile(path)
		profileName := strings.TrimSuffix(strings.TrimSuffix(entry.Name(), ".yaml"), ".yml")
		if err == nil {
			var cfg Config
			if err := yaml.Unmarshal(data, &cfg); err == nil {
				airframeStr := cfg.Airframe.Type.Value
				if airframeStr == "" {
					airframeStr = "AIRFRAME_ELEVON"
				}
				fmt.Printf("  • %-16s [%s]\n", profileName, airframeStr)
				continue
			}
		}
		fmt.Printf("  • %s\n", profileName)
	}
}

func main() {
	configPath := flag.String("config", "", "Path to YAML configuration file")
	profileName := flag.String("profile", "", "Profile name in configs/ directory (e.g. elevon_wing, t_tail, v_tail)")
	outputPath := flag.String("out", "", "Path to generated Go file (defaults to firmware/src/config.go)")
	validateOnly := flag.Bool("validate-only", false, "Validate YAML config without generating Go file")
	listFlag := flag.Bool("list", false, "List available profiles in configs/ directory")
	flag.Parse()

	configsDir := findConfigsDir()

	if *listFlag {
		listProfiles(configsDir)
		return
	}

	// Resolve input config path
	selectedConfig := *configPath
	if *profileName != "" {
		candidates := []string{
			filepath.Join(configsDir, *profileName+".yaml"),
			filepath.Join(configsDir, *profileName+".yml"),
			filepath.Join(configsDir, *profileName),
		}
		found := false
		for _, c := range candidates {
			if _, err := os.Stat(c); err == nil {
				selectedConfig = c
				found = true
				break
			}
		}
		if !found {
			fmt.Fprintf(os.Stderr, "Profile '%s' not found in %s\n", *profileName, configsDir)
			listProfiles(configsDir)
			os.Exit(1)
		}
	}

	if selectedConfig == "" {
		defaultCandidates := []string{
			filepath.Join(configsDir, "default.yaml"),
			filepath.Join(configsDir, "default.yml"),
		}
		for _, c := range defaultCandidates {
			if _, err := os.Stat(c); err == nil {
				selectedConfig = c
				break
			}
		}
		if selectedConfig == "" {
			selectedConfig = "configs/default.yaml"
		}
	}

	// Resolve output path
	targetOut := *outputPath
	if targetOut == "" {
		targetOut = findDefaultOutputFile()
	}

	data, err := os.ReadFile(selectedConfig)
	if err != nil {
		fmt.Fprintf(os.Stderr, "Error reading config file %s: %v\n", selectedConfig, err)
		os.Exit(1)
	}

	var cfg Config
	if err := yaml.Unmarshal(data, &cfg); err != nil {
		fmt.Fprintf(os.Stderr, "Error parsing YAML file %s: %v\n", selectedConfig, err)
		os.Exit(1)
	}

	relSource, err := filepath.Rel(filepath.Dir(targetOut), selectedConfig)
	if err != nil {
		relSource = selectedConfig
	}

	vc, err := ValidateAndResolve(&cfg, relSource)
	if err != nil {
		fmt.Fprintf(os.Stderr, "Config validation error in %s: %v\n", selectedConfig, err)
		os.Exit(1)
	}

	if *validateOnly {
		fmt.Printf("Configuration %s is valid.\n", selectedConfig)
		return
	}

	code, err := GenerateCode(vc)
	if err != nil {
		fmt.Fprintf(os.Stderr, "Code generation error: %v\n", err)
		os.Exit(1)
	}

	// Ensure destination directory exists
	outDir := filepath.Dir(targetOut)
	if err := os.MkdirAll(outDir, 0755); err != nil {
		fmt.Fprintf(os.Stderr, "Failed to create directory %s: %v\n", outDir, err)
		os.Exit(1)
	}

	if err := os.WriteFile(targetOut, code, 0644); err != nil {
		fmt.Fprintf(os.Stderr, "Error writing to %s: %v\n", targetOut, err)
		os.Exit(1)
	}

	fmt.Printf("Successfully generated %s from %s\n", targetOut, selectedConfig)
}
