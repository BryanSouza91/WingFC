package main

import (
	math "github.com/orsinium-labs/tinymath"
)

// IMU Struct
type IMU struct {
	rawAccelX int16
	rawAccelY int16
	rawAccelZ int16
	rawGyroX  int16
	rawGyroY  int16
	rawGyroZ  int16

	AccelXBias float32
	AccelYBias float32
	AccelZBias float32
	GyroXBias  float32
	GyroYBias  float32
	GyroZBias  float32

	AccelX float32
	AccelY float32
	AccelZ float32
	GyroX  float32
	GyroY  float32
	GyroZ  float32

	Pitch float32
	Roll  float32
	Yaw   float32
}

// pitchAccel() calculates the pitch angle in radians from accelerometer data.
func (i *IMU) pitchAccel() float32 {
	return math.Atan2(-i.AccelX, math.Sqrt(i.AccelY*i.AccelY+i.AccelZ*i.AccelZ))
}

// rollAccel() calculates the roll angle in radians from accelerometer data.
func (i *IMU) rollAccel() float32 {
	return math.Atan2(i.AccelY, i.AccelZ)
}

// yawGyro() calculates the yaw ratee in m/s^2 from gyroscope data.
func (i *IMU) yawGyro() float32 {
	// No Motion No Integration (NMNI) - if gyroZ is below bias, treat as zero to prevent drift when stationary
	// Apply deadzone to prevent drift when stationary
	if math.Abs(i.GyroZ) < math.Abs(i.GyroZBias) {
		return 0
	}
	return i.GyroZ * dt
}
