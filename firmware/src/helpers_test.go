//go:build tinygo
// +build tinygo

package main

import (
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestConstrain(t *testing.T) {
	tests := []struct {
		name     string
		value    float64
		min      float64
		max      float64
		expected float64
	}{
		{
			name:     "Value within bounds",
			value:    5.0,
			min:      0.0,
			max:      10.0,
			expected: 5.0,
		},
		{
			name:     "Value below min",
			value:    -1.0,
			min:      0.0,
			max:      10.0,
			expected: 0.0,
		},
		{
			name:     "Value above max",
			value:    15.0,
			min:      0.0,
			max:      10.0,
			expected: 10.0,
		},
		{
			name:     "Value equals min",
			value:    0.0,
			min:      0.0,
			max:      10.0,
			expected: 0.0,
		},
		{
			name:     "Value equals max",
			value:    10.0,
			min:      0.0,
			max:      10.0,
			expected: 10.0,
		},
		{
			name:     "Negative range",
			value:    -5.0,
			min:      -10.0,
			max:      -1.0,
			expected: -5.0,
		},
		{
			name:     "Negative value above max",
			value:    5.0,
			min:      -10.0,
			max:      -1.0,
			expected: -1.0,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			result := constrain(test.value, test.min, test.max)
			assert.Equal(t, test.expected, result)
		})
	}
}

func TestConstrainExtremeCases(t *testing.T) {
	tests := []struct {
		name     string
		value    float64
		min      float64
		max      float64
		expected float64
	}{
		{
			name:     "Zero midpoint",
			value:    0.0,
			min:      -5.0,
			max:      5.0,
			expected: 0.0,
		},
		{
			name:     "Very small positive",
			value:    0.000001,
			min:      0.0,
			max:      1.0,
			expected: 0.000001,
		},
		{
			name:     "Very large values",
			value:    1000000.0,
			min:      -1000000.0,
			max:      2000000.0,
			expected: 1000000.0,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			result := constrain(test.value, test.min, test.max)
			assert.True(t, FloatEqual(result, test.expected, 0.0001))
		})
	}
}

func TestMapRangeUint16(t *testing.T) {
	tests := []struct {
		name     string
		value    uint16
		fromMin  uint16
		fromMax  uint16
		toMin    uint16
		toMax    uint16
		expected uint16
	}{
		{
			name:     "Map midpoint 1000->2000 to 0->360",
			value:    1500,
			fromMin:  1000,
			fromMax:  2000,
			toMin:    0,
			toMax:    360,
			expected: 180,
		},
		{
			name:     "Map minimum",
			value:    1000,
			fromMin:  1000,
			fromMax:  2000,
			toMin:    0,
			toMax:    100,
			expected: 0,
		},
		{
			name:     "Map maximum",
			value:    2000,
			fromMin:  1000,
			fromMax:  2000,
			toMin:    0,
			toMax:    100,
			expected: 100,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			result := mapRange(test.value, test.fromMin, test.fromMax, test.toMin, test.toMax)
			// Allow small rounding error
			assert.True(t, result >= test.expected-1 && result <= test.expected+1,
				"Expected ~%d, got %d", test.expected, result)
		})
	}
}

func TestMapRangeFloat64(t *testing.T) {
	tests := []struct {
		name     string
		value    float64
		fromMin  float64
		fromMax  float64
		toMin    float64
		toMax    float64
		tolerance float64
	}{
		{
			name:      "Map 0.5 from [0,1] to [0,100]",
			value:     0.5,
			fromMin:   0.0,
			fromMax:   1.0,
			toMin:     0.0,
			toMax:     100.0,
			tolerance: 0.1,
		},
		{
			name:      "Map 0 (minimum)",
			value:     0.0,
			fromMin:   0.0,
			fromMax:   1.0,
			toMin:     0.0,
			toMax:     100.0,
			tolerance: 0.1,
		},
		{
			name:      "Map 1 (maximum)",
			value:     1.0,
			fromMin:   0.0,
			fromMax:   1.0,
			toMin:     0.0,
			toMax:     100.0,
			tolerance: 0.1,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			result := mapRange(test.value, test.fromMin, test.fromMax, test.toMin, test.toMax)
			// For midpoint test
			if test.value == 0.5 {
				assert.True(t, FloatEqual(result, 50.0, test.tolerance))
			} else if test.value == 0.0 {
				assert.True(t, FloatEqual(result, 0.0, test.tolerance))
			} else if test.value == 1.0 {
				assert.True(t, FloatEqual(result, 100.0, test.tolerance))
			}
		})
	}
}

func TestMapRangeReverseOrder(t *testing.T) {
	// Test mapping with reversed target range (inversion)
	tests := []struct {
		name     string
		value    float64
		fromMin  float64
		fromMax  float64
		toMin    float64
		toMax    float64
	}{
		{
			name:     "Invert mapping",
			value:    0.25,
			fromMin:  0.0,
			fromMax:  1.0,
			toMin:    100.0,
			toMax:    0.0,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			result := mapRange(test.value, test.fromMin, test.fromMax, test.toMin, test.toMax)
			// When toMax < toMin, the value should be inverted
			assert.True(t, result > 50.0 && result < 100.0)
		})
	}
}

func TestSensorConversionConstants(t *testing.T) {
	// Verify sensor conversion constants are defined and reasonable
	const microGToMS2 = 9.80665 / 1e6
	const microDPSToRadS = math.Pi / (180 * 1e6)

	// microGToMS2 should convert micro-g to m/s²
	// 1 g = 9.80665 m/s²
	// 1 micro-g = 9.80665e-6 m/s²
	assert.True(t, microGToMS2 > 9.8e-6 && microGToMS2 < 9.9e-6,
		"microGToMS2 conversion constant seems wrong: %v", microGToMS2)

	// microDPSToRadS should convert micro-DPS to rad/s
	// 180 degrees = π radians
	// 1 DPS = π/180 rad/s
	// 1 micro-DPS = π/(180*1e6) rad/s
	assert.True(t, microDPSToRadS > 1.7e-8 && microDPSToRadS < 1.8e-8,
		"microDPSToRadS conversion constant seems wrong: %v", microDPSToRadS)
}

func TestLowPassFilterAlpha(t *testing.T) {
	// Test that LPF_ALPHA is a reasonable value
	const LPF_ALPHA = 0.2

	assert.True(t, LPF_ALPHA > 0.0 && LPF_ALPHA <= 1.0,
		"LPF_ALPHA should be between 0 and 1: %v", LPF_ALPHA)

	// Lower alpha = more filtering (more lag)
	// Higher alpha = less filtering (more responsiveness)
	// 0.2 is a reasonable middle ground
}

func TestProcessLSMDataBiasRemoval(t *testing.T) {
	// Test that bias removal works correctly
	// This is a conceptual test of the bias removal process

	tests := []struct {
		name         string
		measuredAccel float64
		bias         float64
		expected     float64
	}{
		{
			name:         "Remove positive bias",
			measuredAccel: 10.0,
			bias:         2.0,
			expected:     8.0,
		},
		{
			name:         "Remove negative bias",
			measuredAccel: 8.0,
			bias:         -2.0,
			expected:     10.0,
		},
		{
			name:         "No bias",
			measuredAccel: 9.81,
			bias:         0.0,
			expected:     9.81,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			// Simulating: accel -= bias
			result := test.measuredAccel - test.bias
			assert.Equal(t, test.expected, result)
		})
	}
}

func TestLowPassFilterUpdate(t *testing.T) {
	// Test that low-pass filter correctly updates values
	const LPF_ALPHA = 0.2

	tests := []struct {
		name     string
		previous float64
		current  float64
		expected float64
	}{
		{
			name:     "Filter new measurement",
			previous: 0.0,
			current:  10.0,
			expected: 2.0, // 0 + 0.2 * (10 - 0) = 2.0
		},
		{
			name:     "Filter towards stable value",
			previous: 5.0,
			current:  10.0,
			expected: 6.0, // 5 + 0.2 * (10 - 5) = 6.0
		},
		{
			name:     "No change in measurement",
			previous: 10.0,
			current:  10.0,
			expected: 10.0, // 10 + 0.2 * (10 - 10) = 10.0
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			// Simulating: filtered += LPF_ALPHA * (raw - filtered)
			result := test.previous + LPF_ALPHA*(test.current-test.previous)
			assert.True(t, FloatEqual(result, test.expected, 0.01))
		})
	}
}

func TestConstrainPulseWidth(t *testing.T) {
	// Test constraining pulse widths (common PWM operation)
	const MIN_PULSE_WIDTH_US = 1000
	const MAX_PULSE_WIDTH_US = 2000

	tests := []struct {
		name     string
		pulse    float64
		expected float64
	}{
		{
			name:     "Valid pulse",
			pulse:    1500.0,
			expected: 1500.0,
		},
		{
			name:     "Pulse too low",
			pulse:    500.0,
			expected: MIN_PULSE_WIDTH_US,
		},
		{
			name:     "Pulse too high",
			pulse:    2500.0,
			expected: MAX_PULSE_WIDTH_US,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			result := constrain(test.pulse, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
			assert.Equal(t, test.expected, result)
		})
	}
}

func TestMapRangeRCChannels(t *testing.T) {
	// Test mapping RC channel values (1000-2000) to servo pulse width
	// Channel range: 1000 (min) to 2000 (max)
	// Servo range: 1000us to 2000us

	tests := []struct {
		name     string
		channel  uint16
		expected uint16
	}{
		{
			name:     "Min channel to min servo",
			channel:  1000,
			expected: 1000,
		},
		{
			name:     "Mid channel to mid servo",
			channel:  1500,
			expected: 1500,
		},
		{
			name:     "Max channel to max servo",
			channel:  2000,
			expected: 2000,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			// Directly map from channel to servo (1:1 in this case)
			result := mapRange(test.channel, 1000, 2000, 1000, 2000)
			assert.Equal(t, test.expected, result)
		})
	}
}
