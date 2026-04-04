// +build !tinygo
// This file provides test-safe type definitions when not compiling for TinyGo

package main

// TestPinConfig is a test-safe version of machine.PinConfig
type TestPinConfig struct {
	Mode uint8
}

// TestUARTConfig is a test-safe version of machine.UARTConfig
type TestUARTConfig struct {
	BaudRate uint32
}

// TestPWMConfig is a test-safe version of machine.PWMConfig
type TestPWMConfig struct {
	Frequency uint32
}

// TestI2CConfig is a test-safe version of machine.I2CConfig
type TestI2CConfig struct {
	Frequency uint32
}

// LEDState is the LED status enumeration
type LEDState int

const (
	LEDOFF LEDState = iota
	PWMCONFIG
	PWMERROR
	SERVOINIT
	SERVOERROR
	ESCINIT
	ESCERROR
	IMUCONFIG
	IMUINIT
	IMUERROR
	CALIBRATE
	DISARMED
	ARMED
	FAILSAFED
)
