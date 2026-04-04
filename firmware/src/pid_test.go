package main

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestNewPIDController(t *testing.T) {
	tests := []struct {
		name string
		kp   float64
		ki   float64
		kd   float64
	}{
		{
			name: "Standard gains",
			kp:   1.0,
			ki:   0.1,
			kd:   0.05,
		},
		{
			name: "Zero gains",
			kp:   0.0,
			ki:   0.0,
			kd:   0.0,
		},
		{
			name: "High gains",
			kp:   5.0,
			ki:   2.0,
			kd:   1.0,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			pid := NewPIDController(test.kp, test.ki, test.kd)

			assert.NotNil(t, pid)
			assert.Equal(t, test.kp, pid.Kp)
			assert.Equal(t, test.ki, pid.Ki)
			assert.Equal(t, test.kd, pid.Kd)
			assert.Equal(t, 0.0, pid.prevError)
			assert.Equal(t, 0.0, pid.integral)
		})
	}
}

func TestPIDControllerProportionalTerm(t *testing.T) {
	tests := []struct {
		name     string
		kp       float64
		error    float64
		dt       float64
		expected float64 // Expected output contains at least Kp * error
	}{
		{
			name:     "Positive error",
			kp:       1.0,
			error:    0.5,
			dt:       0.005,
			expected: 0.5, // Kp * error = 1.0 * 0.5 = 0.5
		},
		{
			name:     "Negative error",
			kp:       1.0,
			error:    -0.3,
			dt:       0.005,
			expected: -0.3,
		},
		{
			name:     "High proportional gain",
			kp:       2.0,
			error:    0.4,
			dt:       0.005,
			expected: 0.8,
		},
		{
			name:     "Zero error",
			kp:       1.0,
			error:    0.0,
			dt:       0.005,
			expected: 0.0,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			pid := NewPIDController(test.kp, 0.0, 0.0)
			output := pid.Update(test.error, test.dt)

			// Proportional term should be Kp * error
			assert.True(t, FloatEqual(output, test.expected, 0.00001),
				"Expected %f, got %f", test.expected, output)
		})
	}
}

func TestPIDControllerIntegralTerm(t *testing.T) {
	tests := []struct {
		name      string
		ki        float64
		errors    []float64 // Sequence of errors
		dt        float64
		tolerance float64
	}{
		{
			name:      "Accumulating error",
			ki:        0.1,
			errors:    []float64{0.5, 0.5, 0.5}, // Should accumulate over 3 steps
			dt:        0.01,
			tolerance: 0.0001,
		},
		{
			name:      "Zero integral gain",
			ki:        0.0,
			errors:    []float64{1.0, 1.0, 1.0},
			dt:        0.01,
			tolerance: 0.0001,
		},
		{
			name:      "Positive and negative errors canceling",
			ki:        0.1,
			errors:    []float64{0.5, -0.5, 0.0},
			dt:        0.01,
			tolerance: 0.0001,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			pid := NewPIDController(0.0, test.ki, 0.0)

			for _, err := range test.errors {
				pid.Update(err, test.dt)
			}

			// After updates, integral should have accumulated
			// Verify that integral accumulation is correct by checking final output
			if test.ki > 0 {
				expectedIntegral := 0.0
				for _, err := range test.errors {
					expectedIntegral += err * test.dt
				}
				expectedOutput := test.ki * expectedIntegral
				// Get final output with zero error to see just the integral
				lastOutput := pid.Update(0.0, test.dt)
				// With zero error now, output should be just Ki * integral
				assert.True(t, FloatEqual(lastOutput, expectedOutput, test.tolerance),
					"Expected integral contribution ~%f, got %f", expectedOutput, lastOutput)
			}
		})
	}
}

func TestPIDControllerDerivativeTerm(t *testing.T) {
	tests := []struct {
		name          string
		kd            float64
		errorSequence []float64 // Sequence of errors to compute derivative
		dt            float64
		tolerance     float64
	}{
		{
			name:          "Constant rate of change",
			kd:            0.3,
			errorSequence: []float64{0.0, 0.1, 0.2, 0.3}, // Delta = 0.1 each step
			dt:            0.01,
			tolerance:     0.0001,
		},
		{
			name:          "Zero derivative gain",
			kd:            0.0,
			errorSequence: []float64{0.0, 0.5, 1.0},
			dt:            0.01,
			tolerance:     0.0001,
		},
		{
			name:          "Decreasing error (negative derivative)",
			kd:            0.2,
			errorSequence: []float64{1.0, 0.8, 0.6},
			dt:            0.01,
			tolerance:     0.0001,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			pid := NewPIDController(0.0, 0.0, test.kd)

			for i, err := range test.errorSequence {
				output := pid.Update(err, test.dt)

				if i > 0 {
					// After first update, derivative term should be present
					expectedDerivative := (err - test.errorSequence[i-1]) / test.dt
					expectedOutput := test.kd * expectedDerivative
					if test.kd > 0 {
						assert.True(t, FloatEqual(output, expectedOutput, test.tolerance),
							"Step %d: Expected derivative output ~%f, got %f", i, expectedOutput, output)
					}
				}
			}
		})
	}
}

func TestPIDControllerReset(t *testing.T) {
	tests := []struct {
		name   string
		kp     float64
		ki     float64
		kd     float64
		errors []float64
	}{
		{
			name:   "Reset clears integral",
			kp:     1.0,
			ki:     0.5,
			kd:     0.0, // No D term to avoid spikes
			errors: []float64{0.5, 0.3, 0.2},
		},
		{
			name:   "Reset on zero gains",
			kp:     0.0,
			ki:     0.0,
			kd:     0.0,
			errors: []float64{1.0},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			pid := NewPIDController(test.kp, test.ki, test.kd)

			// Feed some errors
			for _, err := range test.errors {
				pid.Update(err, 0.005)
			}

			// Reset the controller
			pid.Reset()

			// Verify reset state variables
			assert.Equal(t, 0.0, pid.prevError, "prevError not reset")
			assert.Equal(t, 0.0, pid.integral, "integral not reset")

			// After reset with zero error input, output should be zero (no accumulated state)
			output := pid.Update(0.0, 0.005)
			assert.True(t, FloatEqual(output, 0.0, 0.00001),
				"After reset with zero error, expected 0, got %f", output)
		})
	}
}

func TestPIDControllerEdgeCases(t *testing.T) {
	tests := []struct {
		name  string
		kp    float64
		ki    float64
		kd    float64
		error float64
		dt    float64
	}{
		{
			name:  "Very small time step",
			kp:    1.0,
			ki:    0.1,
			kd:    0.05,
			error: 0.1,
			dt:    0.0001,
		},
		{
			name:  "Large time step",
			kp:    1.0,
			ki:    0.1,
			kd:    0.05,
			error: 0.1,
			dt:    0.1,
		},
		{
			name:  "Very large error",
			kp:    1.0,
			ki:    0.1,
			kd:    0.05,
			error: 100.0,
			dt:    0.005,
		},
		{
			name:  "Very small error",
			kp:    1.0,
			ki:    0.1,
			kd:    0.05,
			error: 0.00001,
			dt:    0.005,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			pid := NewPIDController(test.kp, test.ki, test.kd)
			// Just ensure no panic occurs
			output := pid.Update(test.error, test.dt)
			// Verify output is a valid number (notNaN, not Inf)
			assert.False(t, output != output) // NaN check (NaN != NaN is true)
		})
	}
}
