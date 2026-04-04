//go:build tinygo
// +build tinygo

package main

import (
	"machine"

	"tinygo.org/x/drivers/lsm6ds3tr"
)

// MachinePin adapts machine.Pin to the DigitalPin interface.
type MachinePin struct {
	pin machine.Pin
}

func NewMachinePin(pin machine.Pin) *MachinePin {
	return &MachinePin{pin: pin}
}

func (m *MachinePin) Configure(cfg machine.PinConfig) {
	m.pin.Configure(cfg)
}

func (m *MachinePin) High() {
	m.pin.High()
}

func (m *MachinePin) Low() {
	m.pin.Low()
}

// === UART Adapters ===

// MachineUART adapts machine.UART to the UART interface.
type MachineUART struct {
	uart *machine.UART
}

func NewMachineUART(uart *machine.UART) *MachineUART {
	return &MachineUART{uart: uart}
}

func (m *MachineUART) Configure(cfg machine.UARTConfig) error {
	m.uart.Configure(cfg)
	return nil
}

func (m *MachineUART) Read() (byte, error) {
	return m.uart.ReadByte()
}

func (m *MachineUART) Write(b byte) error {
	return m.uart.WriteByte(b)
}

// === PWM Adapters ===

// MachinePWM0 adapts *machine.PWM to the PWM interface.
type MachinePWM0 struct {
	pwm *machine.PWM
}

func NewMachinePWM0(pwm *machine.PWM) *MachinePWM0 {
	return &MachinePWM0{pwm: pwm}
}

func (m *MachinePWM0) Configure(cfg machine.PWMConfig) error {
	return m.pwm.Configure(cfg)
}

func (m *MachinePWM0) Channel(pin machine.Pin) (uint8, error) {
	ch, err := m.pwm.Channel(pin)
	if err != nil {
		return 0, err
	}
	return ch, nil
}

func (m *MachinePWM0) Set(channel uint8, value uint32) {
	m.pwm.Set(channel, value)
}

func (m *MachinePWM0) Top() uint32 {
	return m.pwm.Top()
}

// MachinePWM1 adapts *machine.PWM to the PWM interface.
type MachinePWM1 struct {
	pwm *machine.PWM
}

func NewMachinePWM1(pwm *machine.PWM) *MachinePWM1 {
	return &MachinePWM1{pwm: pwm}
}

func (m *MachinePWM1) Configure(cfg machine.PWMConfig) error {
	return m.pwm.Configure(cfg)
}

func (m *MachinePWM1) Channel(pin machine.Pin) (uint8, error) {
	ch, err := m.pwm.Channel(pin)
	if err != nil {
		return 0, err
	}
	return ch, nil
}

func (m *MachinePWM1) Set(channel uint8, value uint32) {
	m.pwm.Set(channel, value)
}

func (m *MachinePWM1) Top() uint32 {
	return m.pwm.Top()
}

// === I2C Adapters ===

// MachineI2C adapts *machine.I2C to the I2C interface.
type MachineI2C struct {
	i2c *machine.I2C
}

func NewMachineI2C(i2c *machine.I2C) *MachineI2C {
	return &MachineI2C{i2c: i2c}
}

func (m *MachineI2C) Configure(cfg machine.I2CConfig) error {
	m.i2c.Configure(cfg)
	return nil
}

func (m *MachineI2C) Tx(addr uint16, tx []byte, rx []byte) error {
	return m.i2c.Tx(addr, tx, rx)
}

// === IMU Adapters ===

// LSM6DS3TRAdapter adapts *lsm6ds3tr.Device to the IMUDevice interface.
type LSM6DS3TRAdapter struct {
	dev *lsm6ds3tr.Device
}

func NewLSM6DS3TRAdapter(dev *lsm6ds3tr.Device) *LSM6DS3TRAdapter {
	return &LSM6DS3TRAdapter{dev: dev}
}

func (a *LSM6DS3TRAdapter) Configure(config interface{}) error {
	cfg, ok := config.(lsm6ds3tr.Configuration)
	if !ok {
		return nil // Try to cast if necessary
	}
	return a.dev.Configure(cfg)
}

func (a *LSM6DS3TRAdapter) Connected() bool {
	return a.dev.Connected()
}

func (a *LSM6DS3TRAdapter) ReadAccel() (x, y, z int16, err error) {
	rawX, rawY, rawZ, errAccel := a.dev.ReadAcceleration()
	return int16(rawX >> 8), int16(rawY >> 8), int16(rawZ >> 8), errAccel
}

func (a *LSM6DS3TRAdapter) ReadGyro() (x, y, z int16, err error) {
	rawX, rawY, rawZ, errGyro := a.dev.ReadRotation()
	return int16(rawX >> 8), int16(rawY >> 8), int16(rawZ >> 8), errGyro
}
