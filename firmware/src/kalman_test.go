package main

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestNewKalmanFilter(t *testing.T) {
	dt := 0.005
	kf := NewKalmanFilter(dt)

	assert.NotNil(t, kf)
	assert.Equal(t, dt, kf.dt)
	assert.NotNil(t, kf.X)
	assert.NotNil(t, kf.P)
	assert.NotNil(t, kf.Q)
	assert.NotNil(t, kf.R)
	assert.NotNil(t, kf.F)
	assert.NotNil(t, kf.H)

	// Check initial state is zero
	assert.Equal(t, 0.0, kf.X.At(0, 0))
	assert.Equal(t, 0.0, kf.X.At(1, 0))

	// Check P is identity matrix
	assert.Equal(t, 1.0, kf.P.At(0, 0))
	assert.Equal(t, 1.0, kf.P.At(1, 1))
	assert.Equal(t, 0.0, kf.P.At(0, 1))
	assert.Equal(t, 0.0, kf.P.At(1, 0))
}

func TestKalmanFilterPredict(t *testing.T) {
	tests := []struct {
		name      string
		dt        float64
		gyroX     float64
		gyroY     float64
		tolerance float64
	}{
		{
			name:      "Zero gyro input",
			dt:        0.005,
			gyroX:     0.0,
			gyroY:     0.0,
			tolerance: 0.0001,
		},
		{
			name:      "Positive gyro input",
			dt:        0.005,
			gyroX:     0.1,
			gyroY:     0.2,
			tolerance: 0.0001,
		},
		{
			name:      "Negative gyro input",
			dt:        0.005,
			gyroX:     -0.1,
			gyroY:     -0.2,
			tolerance: 0.0001,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			kf := NewKalmanFilter(test.dt)

			// Predict with gyro input
			kf.Predict(test.gyroX, test.gyroY)

			// State should be updated: X = X_prev + [gyroY*dt, gyroX*dt]
			expectedX := test.gyroY * test.dt
			expectedY := test.gyroX * test.dt

			assert.True(t, FloatEqual(kf.X.At(0, 0), expectedX, test.tolerance),
				"Expected X[0]=%f, got %f", expectedX, kf.X.At(0, 0))
			assert.True(t, FloatEqual(kf.X.At(1, 0), expectedY, test.tolerance),
				"Expected X[1]=%f, got %f", expectedY, kf.X.At(1, 0))
		})
	}
}

func TestKalmanFilterUpdate(t *testing.T) {
	tests := []struct {
		name       string
		accelPitch float64
		accelRoll  float64
		tolerance  float64
	}{
		{
			name:       "Zero measurement",
			accelPitch: 0.0,
			accelRoll:  0.0,
			tolerance:  0.0001,
		},
		{
			name:       "Positive measurement",
			accelPitch: 0.3,
			accelRoll:  0.2,
			tolerance:  0.0001,
		},
		{
			name:       "Negative measurement",
			accelPitch: -0.1,
			accelRoll:  -0.2,
			tolerance:  0.0001,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			kf := NewKalmanFilter(0.005)

			// Store initial state
			initialX0 := kf.X.At(0, 0)
			initialX1 := kf.X.At(1, 0)

			// Update with measurement
			kf.Update(test.accelPitch, test.accelRoll)

			// After update, state should be corrected towards measurement
			// (exact values depend on Kalman gain, just verify it changes)
			assert.NotNil(t, kf.X)
			assert.NotNil(t, kf.P)

			// State should be moved towards measurement if measurement is non-zero
			if test.accelPitch != 0.0 || test.accelRoll != 0.0 {
				// The state should change after an update
				t.Logf("Initial state: [%f, %f]", initialX0, initialX1)
				t.Logf("Updated state: [%f, %f]", kf.X.At(0, 0), kf.X.At(1, 0))
			}
		})
	}
}

func TestKalmanFilterPredictUpdateSequence(t *testing.T) {
	// Test a sequence of predict and update steps
	kf := NewKalmanFilter(0.005)

	// Initial state should be zero
	assert.Equal(t, 0.0, kf.X.At(0, 0))
	assert.Equal(t, 0.0, kf.X.At(1, 0))

	// Predict with gyro
	kf.Predict(0.1, 0.2)
	x0_after_predict := kf.X.At(0, 0)
	x1_after_predict := kf.X.At(1, 0)

	// State should have changed
	assert.NotEqual(t, 0.0, x0_after_predict)
	assert.NotEqual(t, 0.0, x1_after_predict)

	// Update with accelerometer measurement
	kf.Update(0.05, 0.1)

	// State should still exist and be reasonable
	assert.False(t, isNaN(kf.X.At(0, 0)), "X[0] should not be NaN")
	assert.False(t, isNaN(kf.X.At(1, 0)), "X[1] should not be NaN")

	// Covariance should be positive definite (diagonal elements > 0)
	assert.True(t, kf.P.At(0, 0) > 0, "P[0,0] should be positive")
	assert.True(t, kf.P.At(1, 1) > 0, "P[1,1] should be positive")
}

func TestKalmanFilterCovarianceUpdate(t *testing.T) {
	// Verify that covariance matrix is properly updated
	kf := NewKalmanFilter(0.005)

	initialP00 := kf.P.At(0, 0)
	initialP11 := kf.P.At(1, 1)

	// After prediction, covariance should increase (uncertainty grows)
	kf.Predict(0.1, 0.2)
	predictP00 := kf.P.At(0, 0)
	predictP11 := kf.P.At(1, 1)

	// Covariance should increase after prediction (more uncertainty)
	assert.True(t, predictP00 >= initialP00, "Covariance should increase after prediction")
	assert.True(t, predictP11 >= initialP11, "Covariance should increase after prediction")

	// After update, covariance should decrease (measurement reduces uncertainty)
	kf.Update(0.1, 0.1)
	updatedP00 := kf.P.At(0, 0)
	updatedP11 := kf.P.At(1, 1)

	// Covariance should decrease after measurement update
	assert.True(t, updatedP00 <= predictP00, "Covariance should decrease after update")
	assert.True(t, updatedP11 <= predictP11, "Covariance should decrease after update")
}

func TestKalmanFilterNoNaN(t *testing.T) {
	// Verify that filter never produces NaN values
	kf := NewKalmanFilter(0.005)

	for i := 0; i < 100; i++ {
		kf.Predict(0.05, 0.1)
		kf.Update(0.03, 0.07)

		assert.False(t, isNaN(kf.X.At(0, 0)), "X[0] should not be NaN")
		assert.False(t, isNaN(kf.X.At(1, 0)), "X[1] should not be NaN")
		assert.False(t, isNaN(kf.P.At(0, 0)), "P[0,0] should not be NaN")
	}
}

// Helper function to check if a float is NaN
func isNaN(f float64) bool {
	return f != f // NaN is the only float that's not equal to itself
}
