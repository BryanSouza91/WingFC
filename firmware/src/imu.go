package main

import (
	math "github.com/orsinium-labs/tinymath"
)

const (
	microGToMS2    float32 = 9.80665 / 1e6
	microDPSToRadS float32 = 3.141592653589793 / (180 * 1e6)
)

// IMU Struct
type IMU struct {
	rawAccelX int32
	rawAccelY int32
	rawAccelZ int32
	rawGyroX  int32
	rawGyroY  int32
	rawGyroZ  int32

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

var imuData IMU

// pitchAccel() calculates the pitch angle in radians from accelerometer data.
func (i *IMU) pitchAccel() float32 {
	return math.Atan2(i.AccelX, math.Sqrt(i.AccelY*i.AccelY+i.AccelZ*i.AccelZ))
}

// rollAccel() calculates the roll angle in radians from accelerometer data.
func (i *IMU) rollAccel() float32 {
	return math.Atan2(i.AccelY, i.AccelZ)
}

// GYRO_NOISE_DEADBAND defines the minimum angular rate threshold in rad/s (~0.57 deg/s)
// below which stationary gyro noise is suppressed to prevent integration drift.
const GYRO_NOISE_DEADBAND float32 = 0.01

// yawGyro() calculates the yaw rate in rad/s from gyroscope data.
func (i *IMU) yawGyro() float32 {
	// Apply deadzone to prevent drift when stationary
	if math.Abs(i.GyroZ) < GYRO_NOISE_DEADBAND {
		return 0
	}
	return i.GyroZ
}
