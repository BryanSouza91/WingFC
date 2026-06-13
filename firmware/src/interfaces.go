package main

import "machine"

// DigitalPin defines the interface for basic GPIO operations.
type DigitalPin interface {
	Configure(config machine.PinConfig)
	High()
	Low()
}

// UART defines the interface for serial communication.
type UART interface {
	Configure(machine.UARTConfig) error
	Read() (byte, error)
	Write(b byte) error
}

// SPI defines the interface for SPI communication.
type SPI interface {
	Configure(machine.SPIConfig) error
	Tx(w, r []byte) error
}

// DShot defines the interface for digital ESC communication.
type DShot interface {
	Configure() error
	SendThrottle(throttle uint16, telemetry bool)
}

// PWM defines the interface for PWM control.
type PWM interface {
	Configure(machine.PWMConfig) error
	Channel(pin machine.Pin) (uint8, error)
	Set(channel uint8, value uint32)
	Top() uint32
}

// I2C defines the interface for I2C communication.
type I2C interface {
	Configure(machine.I2CConfig) error
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

// GPSDevice defines the interface for GPS operations.
type GPSDevice interface {
	Configure() error
	Connected() bool
	Read() (gpsData GPSData, err error)
}
