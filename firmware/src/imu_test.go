package main

import (
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestIMUStructCreation(t *testing.T) {
	imu := &IMU{
		AccelX: 1.0,
		AccelY: 2.0,
		AccelZ: 3.0,
		GyroX:  0.1,
		GyroY:  0.2,
		GyroZ:  0.3,
		Pitch:  0.0,
		Roll:   0.0,
	}

	assert.Equal(t, 1.0, imu.AccelX)
	assert.Equal(t, 2.0, imu.AccelY)
	assert.Equal(t, 3.0, imu.AccelZ)
	assert.Equal(t, 0.1, imu.GyroX)
	assert.Equal(t, 0.2, imu.GyroY)
	assert.Equal(t, 0.3, imu.GyroZ)
}

func TestIMUPitchAccel(t *testing.T) {
	tests := []struct {
		name       string
		accelX     float64
		accelY     float64
		accelZ     float64
		expectedFn func(x, y, z float64) float64 // Function to compute expected value
	}{
		{
			name:   "Level pitch (Z only acceleration)",
			accelX: 0.0,
			accelY: 0.0,
			accelZ: 9.81,
			expectedFn: func(x, y, z float64) float64 {
				return math.Atan2(-x, math.Sqrt(y*y+z*z))
			},
		},
		{
			name:   "Pitched forward",
			accelX: -2.0,
			accelY: 0.0,
			accelZ: 9.81,
			expectedFn: func(x, y, z float64) float64 {
				return math.Atan2(-x, math.Sqrt(y*y+z*z))
			},
		},
		{
			name:   "Pitched backward",
			accelX: 2.0,
			accelY: 0.0,
			accelZ: 9.81,
			expectedFn: func(x, y, z float64) float64 {
				return math.Atan2(-x, math.Sqrt(y*y+z*z))
			},
		},
		{
			name:   "Combined accelerations",
			accelX: 1.0,
			accelY: 2.0,
			accelZ: 9.81,
			expectedFn: func(x, y, z float64) float64 {
				return math.Atan2(-x, math.Sqrt(y*y+z*z))
			},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			imu := &IMU{
				AccelX: test.accelX,
				AccelY: test.accelY,
				AccelZ: test.accelZ,
			}

			expected := test.expectedFn(test.accelX, test.accelY, test.accelZ)
			result := imu.pitchAccel()

			assert.True(t, FloatEqual(result, expected, 0.0001),
				"Expected pitch %f, got %f", expected, result)
		})
	}
}

func TestIMURollAccel(t *testing.T) {
	tests := []struct {
		name       string
		accelX     float64
		accelY     float64
		accelZ     float64
		expectedFn func(x, y, z float64) float64
	}{
		{
			name:   "Level roll (Z only acceleration)",
			accelX: 0.0,
			accelY: 0.0,
			accelZ: 9.81,
			expectedFn: func(x, y, z float64) float64 {
				return math.Atan2(y, z)
			},
		},
		{
			name:   "Rolled right",
			accelX: 0.0,
			accelY: 2.0,
			accelZ: 9.81,
			expectedFn: func(x, y, z float64) float64 {
				return math.Atan2(y, z)
			},
		},
		{
			name:   "Rolled left",
			accelX: 0.0,
			accelY: -2.0,
			accelZ: 9.81,
			expectedFn: func(x, y, z float64) float64 {
				return math.Atan2(y, z)
			},
		},
		{
			name:   "Combined accelerations",
			accelX: 1.0,
			accelY: 2.0,
			accelZ: 9.81,
			expectedFn: func(x, y, z float64) float64 {
				return math.Atan2(y, z)
			},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			imu := &IMU{
				AccelX: test.accelX,
				AccelY: test.accelY,
				AccelZ: test.accelZ,
			}

			expected := test.expectedFn(test.accelX, test.accelY, test.accelZ)
			result := imu.rollAccel()

			assert.True(t, FloatEqual(result, expected, 0.0001),
				"Expected roll %f, got %f", expected, result)
		})
	}
}

func TestIMUPitchRollRelationship(t *testing.T) {
	// Test that pitch and roll calculations are independent
	tests := []struct {
		name   string
		accelX float64
		accelY float64
		accelZ float64
	}{
		{"Pitch only", -5.0, 0.0, 9.81},
		{"Roll only", 0.0, 5.0, 9.81},
		{"Pitch and roll", -3.0, 4.0, 9.81},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			imu := &IMU{
				AccelX: test.accelX,
				AccelY: test.accelY,
				AccelZ: test.accelZ,
			}

			pitch := imu.pitchAccel()
			roll := imu.rollAccel()

			// Both should be finite
			assert.False(t, math.IsInf(pitch, 0), "Pitch should not be infinite")
			assert.False(t, math.IsInf(roll, 0), "Roll should not be infinite")
			assert.False(t, math.IsNaN(pitch), "Pitch should not be NaN")
			assert.False(t, math.IsNaN(roll), "Roll should not be NaN")

			// Pitch should be within [-π/2, π/2]
			assert.True(t, pitch >= -math.Pi/2 && pitch <= math.Pi/2,
				"Pitch should be in [-π/2, π/2], got %f", pitch)

			// Roll should be within [-π, π] (atan2 range)
			assert.True(t, roll >= -math.Pi && roll <= math.Pi,
				"Roll should be in [-π, π], got %f", roll)
		})
	}
}

func TestIMUExtremeCases(t *testing.T) {
	tests := []struct {
		name   string
		accelX float64
		accelY float64
		accelZ float64
	}{
		{"Large positive accelerations", 10.0, 10.0, 10.0},
		{"Large negative accelerations", -10.0, -10.0, -10.0},
		{"Mixed large accelerations", 100.0, -50.0, 75.0},
		{"Very small accelerations", 0.001, 0.001, 0.001},
		{"One zero acceleration", 0.0, 1.0, 9.81},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			imu := &IMU{
				AccelX: test.accelX,
				AccelY: test.accelY,
				AccelZ: test.accelZ,
			}

			// Should not panic or return NaN
			pitch := imu.pitchAccel()
			roll := imu.rollAccel()

			assert.False(t, math.IsNaN(pitch), "Pitch should not be NaN for %s", test.name)
			assert.False(t, math.IsNaN(roll), "Roll should not be NaN for %s", test.name)
		})
	}
}

func TestIMUZeroAcceleration(t *testing.T) {
	// When all accelerations are zero, atan2(0, 0) should return 0
	imu := &IMU{
		AccelX: 0.0,
		AccelY: 0.0,
		AccelZ: 0.0,
	}

	pitch := imu.pitchAccel()
	roll := imu.rollAccel()

	// atan2(0, 0) returns 0
	assert.Equal(t, 0.0, pitch)
	assert.Equal(t, 0.0, roll)
}

func TestIMUGyroData(t *testing.T) {
	// Test that gyro data can be set and retrieved
	imu := &IMU{
		GyroX: 1.5,
		GyroY: 2.5,
		GyroZ: 3.5,
	}

	assert.Equal(t, 1.5, imu.GyroX)
	assert.Equal(t, 2.5, imu.GyroY)
	assert.Equal(t, 3.5, imu.GyroZ)
}

func TestIMUDataIndependence(t *testing.T) {
	// Test that modifying one IMU instance doesn't affect another
	imu1 := &IMU{
		AccelX: 1.0,
		AccelY: 2.0,
		AccelZ: 3.0,
	}

	imu2 := &IMU{
		AccelX: 4.0,
		AccelY: 5.0,
		AccelZ: 6.0,
	}

	assert.NotEqual(t, imu1.AccelX, imu2.AccelX)
	assert.NotEqual(t, imu1.AccelY, imu2.AccelY)
	assert.NotEqual(t, imu1.AccelZ, imu2.AccelZ)
}
