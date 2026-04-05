package main

import (
	math "github.com/orsinium-labs/tinymath"
)

// IMU Struct
type IMU struct {
	AccelX float32
	AccelY float32
	AccelZ float32
	GyroX  float32
	GyroY  float32
	GyroZ  float32

	Pitch float32
	Roll  float32
}

// pitchAccel() calculates the pitch angle in radians from accelerometer data.
func (i *IMU) pitchAccel() float32 {
	return math.Atan2(-i.AccelX, math.Sqrt(i.AccelY*i.AccelY+i.AccelZ*i.AccelZ))
}

// rollAccel() calculates the roll angle in radians from accelerometer data.
func (i *IMU) rollAccel() float32 {
	return math.Atan2(i.AccelY, i.AccelZ)
}
