package main

import (
	"math"
)

// IMU Struct
type IMU struct {
	AccelX float64
	AccelY float64
	AccelZ float64
	GyroX  float64
	GyroY  float64
	GyroZ  float64

	Pitch float64
	Roll  float64
}

// pitchAccel() calculates the pitch angle in radians from accelerometer data.
func (i *IMU) pitchAccel() float64 {
	return math.Atan2(-i.AccelX, math.Sqrt(i.AccelY*i.AccelY+i.AccelZ*i.AccelZ))
}

// rollAccel() calculates the roll angle in radians from accelerometer data.
func (i *IMU) rollAccel() float64 {
	return math.Atan2(i.AccelY, i.AccelZ)
}
