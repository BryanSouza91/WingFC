// +build !tinygo
// This file provides test-safe type definitions for FC_Hardware when not compiling for TinyGo

package main

// FC_Hardware holds all hardware interfaces and their state.
// This is a minimal definition for testing when hardware.go is excluded.
type FC_Hardware struct {
	I2C      I2C
	UART     UART
	ServoPWM PWM
	ESCPWM   PWM
	IMU      IMUDevice
	LED      LEDUpdater

	// PWM channel IDs
	pwmCh1 uint8
	pwmCh2 uint8
	pwmCh3 uint8

	// Tracked periods for PWM pulse width calculations
	ServoPeriod uint64
	ESCPeriod   uint64
}
