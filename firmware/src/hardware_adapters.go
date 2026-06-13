package main

import (
	"machine"

	"tinygo.org/x/drivers/gps"
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

// === SPI Adapters ===

// MachineSPI adapts machine.SPI to the SPI interface.
type MachineSPI struct {
	spi *machine.SPI
}

func NewMachineSPI(spi *machine.SPI) *MachineSPI {
	return &MachineSPI{spi: spi}
}

func (m *MachineSPI) Configure(cfg machine.SPIConfig) error {
	return m.spi.Configure(cfg)
}

func (m *MachineSPI) Tx(w, r []byte) error {
	return m.spi.Tx(w, r)
}

// === PWM Adapters ===

// MachinePWM adapts *machine.PWM to the PWM interface.
type MachinePWM struct {
	pwm *machine.PWM
}

func NewMachinePWM(pwm *machine.PWM) *MachinePWM {
	return &MachinePWM{pwm: pwm}
}

func (m *MachinePWM) Configure(cfg machine.PWMConfig) error {
	return m.pwm.Configure(cfg)
}

func (m *MachinePWM) Channel(pin machine.Pin) (uint8, error) {
	ch, err := m.pwm.Channel(pin)
	if err != nil {
		return 0, err
	}
	return ch, nil
}

func (m *MachinePWM) Set(channel uint8, value uint32) {
	m.pwm.Set(channel, value)
}

func (m *MachinePWM) Top() uint32 {
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

// === GPS Adapters ===
type GPSAdapter struct {
	Device gps.Device
	Data   GPSData
}

func NewGPS(uart *MachineUART) *GPSAdapter {
	return &GPSAdapter{
		Device: gps.NewUART(uart),
	}
}

func (g *GPSAdapter) Configure() error {
	return g.Device.Configure()
}

func (g *GPSAdapter) Connected() bool {	
	return g.Device.Connected()
}

func (g *GPSAdapter) Read() (g.Data GPSData, err error) {
	err = g.Device.Read()
	if err != nil {
		return GPSData{}, err
	}
	return GPSData{
		Latitude:   g.Data.Latitude,
		Longitude:  g.Data.Longitude,
		Altitude:   g.Data.Altitude,
		Satellites: g.Data.Satellites,
		Fix:        g.Data.Fix,
		Speed:      g.Data.Speed,
		Course:     g.Data.Course,
		Time:       g.Data.Time,
	}, nil
}
