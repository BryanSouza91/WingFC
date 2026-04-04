// +build !tinygo
// This file provides test-safe interface definitions when not compiling for TinyGo

package main

// DigitalPin defines the interface for basic GPIO operations.
type DigitalPin interface {
	Configure(config interface{})
	High()
	Low()
}

// UART defines the interface for serial communication.
type UART interface {
	Configure(config interface{}) error
	Read() (byte, error)
	Write(b byte) error
}

// PWM defines the interface for PWM control.
type PWM interface {
	Configure(config interface{}) error
	Channel(pin interface{}) (uint8, error)
	Set(channel uint8, value uint32)
	Top() uint32
}

// I2C defines the interface for I2C communication.
type I2C interface {
	Configure(config interface{}) error
	Tx(addr uint16, tx []byte, rx []byte) error
}

// LEDUpdater defines the interface for LED state management.
type LEDUpdater interface {
	SetState(state LEDState)
	Update()
}

// IMUDevice defines the interface for IMU sensor operations.
type IMUDevice interface {
	Configure(config interface{}) error
	Connected() bool
	ReadAccel() (x, y, z int16, err error)
	ReadGyro() (x, y, z int16, err error)
}
