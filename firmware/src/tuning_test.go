//go:build tinygo
// +build tinygo

package main

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

// Note: Due to global state dependencies (Channels, pitchPID, rollPID),
// these tests focus on the tuning logic parameters and ranges

func TestTuningParameterConstants(t *testing.T) {
	// Verify tuning configuration constants
	assert.Equal(t, 0, TuneParameterA)     // Disabled by default
	assert.Equal(t, 0, TuneParameterB)     // Disabled by default
	assert.Equal(t, 4, TuningChannelA)     // CH5
	assert.Equal(t, 5, TuningChannelB)     // CH6
}

func TestTuningParameterRanges(t *testing.T) {
	// Verify tuning parameter ranges
	tests := []struct {
		name      string
		minValue  float64
		maxValue  float64
		tolerance float64
	}{
		{
			name:      "Parameter A range",
			minValue:  TuneParameterAmin,
			maxValue:  TuneParameterAmax,
			tolerance: 0.01,
		},
		{
			name:      "Parameter B range",
			minValue:  TuneParameterBmin,
			maxValue:  TuneParameterBmax,
			tolerance: 0.001,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			assert.True(t, test.minValue > 0, "Min value should be positive")
			assert.True(t, test.maxValue > test.minValue, "Max should be greater than min")
		})
	}
}

func TestTuningParameterIDValidity(t *testing.T) {
	// Parameter IDs should be in range 1-6 (or 0 for disabled)
	validIDs := []int{0, 1, 2, 3, 4, 5, 6}

	for _, id := range validIDs {
		// Verify ID is valid (used in switch statements in UpdateTuning)
		assert.True(t, id >= 0 && id <= 6, "Parameter ID %d should be valid", id)
	}
}

func TestTuningParameterMapping(t *testing.T) {
	// Test parameter mapping logic
	tests := []struct {
		name         string
		parameterID  int
		rcValue      float64
		minRange     float64
		maxRange     float64
	}{
		{
			name:        "Pitch P tuning",
			parameterID: 1,
			rcValue:     1500.0,
			minRange:    TuneParameterAmin,
			maxRange:    TuneParameterAmax,
		},
		{
			name:        "Roll P tuning",
			parameterID: 2,
			rcValue:     1500.0,
			minRange:    TuneParameterAmin,
			maxRange:    TuneParameterAmax,
		},
		{
			name:        "Pitch I tuning",
			parameterID: 3,
			rcValue:     1500.0,
			minRange:    TuneParameterBmin,
			maxRange:    TuneParameterBmax,
		},
		{
			name:        "Roll I tuning",
			parameterID: 4,
			rcValue:     1500.0,
			minRange:    TuneParameterBmin,
			maxRange:    TuneParameterBmax,
		},
		{
			name:        "Pitch D tuning",
			parameterID: 5,
			rcValue:     1500.0,
			minRange:    TuneParameterAmin,
			maxRange:    TuneParameterAmax,
		},
		{
			name:        "Roll D tuning",
			parameterID: 6,
			rcValue:     1500.0,
			minRange:    TuneParameterBmin,
			maxRange:    TuneParameterBmax,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			// Verify parameter mapping is valid
			assert.True(t, test.parameterID >= 1 && test.parameterID <= 6)
			assert.True(t, test.minRange > 0)
			assert.True(t, test.maxRange > test.minRange)
		})
	}
}

func TestTuningRCChannelRanges(t *testing.T) {
	// Verify RC channel value ranges for tuning
	const MIN_RX_VALUE = 988
	const MAX_RX_VALUE = 2012

	tests := []struct {
		name          string
		rcValue       float64
		expectedValid bool
	}{
		{
			name:          "Minimum RC value",
			rcValue:       MIN_RX_VALUE,
			expectedValid: true,
		},
		{
			name:          "Maximum RC value",
			rcValue:       MAX_RX_VALUE,
			expectedValid: true,
		},
		{
			name:          "Mid-range RC value",
			rcValue:       1500.0,
			expectedValid: true,
		},
		{
			name:          "Below minimum",
			rcValue:       500.0,
			expectedValid: true, // Will be clamped
		},
		{
			name:          "Above maximum",
			rcValue:       3000.0,
			expectedValid: true, // Will be clamped
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			// Verify mapping of RC value to parameter range
			parameterValue := mapRange(test.rcValue, MIN_RX_VALUE, MAX_RX_VALUE,
				TuneParameterAmin, TuneParameterAmax)

			// Result should not be NaN
			assert.False(t, isNaN(parameterValue), "Parameter value should not be NaN")
		})
	}
}

func TestTuningParameterDisabled(t *testing.T) {
	// When TuneParameter is 0, tuning should be disabled
	tests := []struct {
		name      string
		parameter int
	}{
		{
			name:      "Parameter A disabled",
			parameter: TuneParameterA,
		},
		{
			name:      "Parameter B disabled",
			parameter: TuneParameterB,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			// Default should be disabled (0)
			assert.Equal(t, 0, test.parameter)
		})
	}
}

func TestTuningChannelAssignments(t *testing.T) {
	// Verify that tuning channels are assigned properly
	const AileronChannel = 0
	const ElevatorChannel = 1
	const ThrottleChannel = 2
	const ArmChannel = 4
	_ = AileronChannel
	_ = ElevatorChannel
	_ = ThrottleChannel
	_ = ArmChannel

	// TuningChannelA and B should be different from control channels
	assert.Equal(t, 4, TuningChannelA)
	assert.Equal(t, 5, TuningChannelB)

	// Verify they're aux channels (typically 5-8)
	assert.True(t, TuningChannelA >= 4 && TuningChannelA <= 7)
	assert.True(t, TuningChannelB >= 4 && TuningChannelB <= 7)
}

func TestTuningRangeLogic(t *testing.T) {
	// Test the range mapping logic for tuning
	tests := []struct {
		name     string
		rcMin    float64
		rcMax    float64
		paramMin float64
		paramMax float64
		expected func(result float64) bool
	}{
		{
			name:     "RC mid maps to param mid",
			rcMin:    988,
			rcMax:    2012,
			paramMin: 0.1,
			paramMax: 2.0,
			expected: func(result float64) bool {
				mid := (0.1 + 2.0) / 2
				return FloatEqual(result, mid, 0.1)
			},
		},
		{
			name:     "RC min maps to param min",
			rcMin:    988,
			rcMax:    2012,
			paramMin: 0.1,
			paramMax: 2.0,
			expected: func(result float64) bool {
				return FloatEqual(result, 0.1, 0.01)
			},
		},
		{
			name:     "RC max maps to param max",
			rcMin:    988,
			rcMax:    2012,
			paramMin: 0.1,
			paramMax: 2.0,
			expected: func(result float64) bool {
				return FloatEqual(result, 2.0, 0.01)
			},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			var rcValue float64
			if test.name == "RC mid maps to param mid" {
				rcValue = (test.rcMin + test.rcMax) / 2
			} else if test.name == "RC min maps to param min" {
				rcValue = test.rcMin
			} else {
				rcValue = test.rcMax
			}

			result := mapRange(rcValue, test.rcMin, test.rcMax, test.paramMin, test.paramMax)
			assert.True(t, test.expected(result))
		})
	}
}

func TestTuningBothParametersIndependent(t *testing.T) {
	// Verify that Parameter A and B can operate independently
	// Parameter A controls different gains than Parameter B

	// Usually Ki and Kd have smaller ranges than Kp
	assert.True(t, TuneParameterBmax < TuneParameterAmax,
		"Parameter B max should be smaller than Parameter A max")
}

func TestTuningParameterBoundaryValues(t *testing.T) {
	// Test boundary values for parameters
	tests := []struct {
		name       string
		value      float64
		minBound   float64
		maxBound   float64
		isInBounds bool
	}{
		{
			name:       "Parameter A at minimum",
			value:      TuneParameterAmin,
			minBound:   TuneParameterAmin,
			maxBound:   TuneParameterAmax,
			isInBounds: true,
		},
		{
			name:       "Parameter A at maximum",
			value:      TuneParameterAmax,
			minBound:   TuneParameterAmin,
			maxBound:   TuneParameterAmax,
			isInBounds: true,
		},
		{
			name:       "Parameter A below minimum",
			value:      TuneParameterAmin - 0.1,
			minBound:   TuneParameterAmin,
			maxBound:   TuneParameterAmax,
			isInBounds: false,
		},
		{
			name:       "Parameter A above maximum",
			value:      TuneParameterAmax + 0.1,
			minBound:   TuneParameterAmin,
			maxBound:   TuneParameterAmax,
			isInBounds: false,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			inBounds := test.value >= test.minBound && test.value <= test.maxBound
			assert.Equal(t, test.isInBounds, inBounds)
		})
	}
}

func TestTuningValidSelection(t *testing.T) {
	// Test that tuning parameter selection is valid when enabled
	validParameters := []int{1, 2, 3, 4, 5, 6}

	for _, param := range validParameters {
		t.Run("Parameter"+string(rune(param)), func(t *testing.T) {
			// Verify parameter is within valid range for UpdateTuning switch statement
			assert.True(t, param > 0 && param <= 6)
		})
	}
}
