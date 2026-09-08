package main

import (
	"strings"
	"testing"

	"gopkg.in/yaml.v3"
)

const sampleValidYAML = `
version: "1.0"
protocol:
  num_channels: 18

board:
  orientation: 0

airframe:
  type: AIRFRAME_ELEVON

receiver:
  channels:
    aileron: 0
    elevator: 1
    throttle: 2
    yaw: 3
    arm: 4
    manual_mode: 7
    tuning_a: 5
    tuning_b: 6
    tuning_c: 8
    tuning_d: 9
  deadband: 10
  high_rx_value: 1800
  min_rx_value: 988
  max_rx_value: 2012
  neutral_rx_value: 1500

pwm:
  dshot: false
  dshot_mode: 600
  servo_frequency_hz: 100
  esc_frequency_hz: 400

servos:
  servo1: { min: 1100, max: 1900, subtrim: 0, reverse: false }
  servo2: { min: 1100, max: 1900, subtrim: 0, reverse: false }
  servo4: { min: 1100, max: 1900, subtrim: 0, reverse: false }
  servo5: { min: 1100, max: 1900, subtrim: 0, reverse: false }
  servo6: { min: 1100, max: 1900, subtrim: 0, reverse: false }

flight_control:
  max_rates_deg:
    pitch: 200
    roll: 600
    yaw: 100
  max_angles_deg:
    pitch: 45.0
    roll: 60.0
  pid_weight: 0.5
  filters:
    accel_level: 2
    gyro_level: 0
  pid:
    pitch: { p: 1.0, i: 0.05, d: 0.0 }
    roll:  { p: 1.0, i: 0.05, d: 0.0 }
    yaw:   { p: 0.5, i: 0.02, d: 0.0 }

tuning:
  param_a: { parameter: 0, min: 0.1, max: 2.0 }
  param_b: { parameter: 0, min: 0.01, max: 0.5 }
  param_c: { parameter: 0, min: 0.001, max: 0.1 }
  param_d: { parameter: 0, min: 0.001, max: 0.1 }
`

func TestValidYAML(t *testing.T) {
	var cfg Config
	if err := yaml.Unmarshal([]byte(sampleValidYAML), &cfg); err != nil {
		t.Fatalf("Unmarshal failed: %v", err)
	}

	vc, err := ValidateAndResolve(&cfg, "configs/default.yaml")
	if err != nil {
		t.Fatalf("Validation failed: %v", err)
	}

	if vc.NumChannels != 18 {
		t.Errorf("expected NumChannels=18, got %d", vc.NumChannels)
	}
	if vc.AirframeTypeConst != "AIRFRAME_ELEVON" {
		t.Errorf("expected AIRFRAME_ELEVON, got %s", vc.AirframeTypeConst)
	}

	code, err := GenerateCode(vc)
	if err != nil {
		t.Fatalf("GenerateCode failed: %v", err)
	}

	codeStr := string(code)
	if !strings.Contains(codeStr, "NumChannels = 18") {
		t.Errorf("generated code missing NumChannels = 18")
	}
	if !strings.Contains(codeStr, "const AIRCRAFT_TYPE = AIRFRAME_ELEVON") {
		t.Errorf("generated code missing const AIRCRAFT_TYPE = AIRFRAME_ELEVON")
	}
}

func TestStringEnums(t *testing.T) {
	yamlWithStrings := strings.Replace(sampleValidYAML, "orientation: 0", "orientation: CW90", 1)
	yamlWithStrings = strings.Replace(yamlWithStrings, "type: AIRFRAME_ELEVON", "type: dual_aileron_t_tail", 1)
	yamlWithStrings = strings.Replace(yamlWithStrings, "parameter: 0", "parameter: PITCH_P", 1)

	var cfg Config
	if err := yaml.Unmarshal([]byte(yamlWithStrings), &cfg); err != nil {
		t.Fatalf("Unmarshal failed: %v", err)
	}

	vc, err := ValidateAndResolve(&cfg, "test.yaml")
	if err != nil {
		t.Fatalf("Validation failed: %v", err)
	}

	if vc.Orientation != 1 {
		t.Errorf("expected Orientation=1 for CW90, got %d", vc.Orientation)
	}
	if vc.AirframeTypeConst != "AIRFRAME_DUAL_AILERON_T_TAIL" {
		t.Errorf("expected AIRFRAME_DUAL_AILERON_T_TAIL, got %s", vc.AirframeTypeConst)
	}
	if vc.TuneParamA != 1 {
		t.Errorf("expected TuneParamA=1 for PITCH_P, got %d", vc.TuneParamA)
	}
}

func TestValidationErrors(t *testing.T) {
	tests := []struct {
		name       string
		modifier   func(s string) string
		errSnippet string
	}{
		{
			name: "Invalid channel range",
			modifier: func(s string) string {
				return strings.Replace(s, "aileron: 0", "aileron: 25", 1)
			},
			errSnippet: "out of range",
		},
		{
			name: "Invalid servo range",
			modifier: func(s string) string {
				return strings.Replace(s, "min: 1100, max: 1900", "min: 1900, max: 1100", 1)
			},
			errSnippet: "must be less than max",
		},
		{
			name: "Invalid RX pulse ordering",
			modifier: func(s string) string {
				return strings.Replace(s, "min_rx_value: 988", "min_rx_value: 1600", 1)
			},
			errSnippet: "min (1600) < neutral (1500)",
		},
		{
			name: "Invalid DShot mode",
			modifier: func(s string) string {
				s = strings.Replace(s, "dshot: false", "dshot: true", 1)
				return strings.Replace(s, "dshot_mode: 600", "dshot_mode: 200", 1)
			},
			errSnippet: "pwm.dshot_mode must be 150, 300, or 600",
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			modYAML := tc.modifier(sampleValidYAML)
			var cfg Config
			if err := yaml.Unmarshal([]byte(modYAML), &cfg); err != nil {
				t.Fatalf("Unmarshal failed: %v", err)
			}
			_, err := ValidateAndResolve(&cfg, "test.yaml")
			if err == nil {
				t.Fatalf("expected error containing %q, got nil", tc.errSnippet)
			}
			if !strings.Contains(err.Error(), tc.errSnippet) {
				t.Errorf("expected error to contain %q, got: %v", tc.errSnippet, err)
			}
		})
	}
}
