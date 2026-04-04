package main

import (
	"bytes"
)

// ===== Mock UART for testing =====

// mockUART implements the UART interface with a buffer for testing.
type mockUART struct {
	readBuffer  *bytes.Buffer
	writeBuffer *bytes.Buffer
}

// NewMockUART creates a new mock UART instance.
func NewMockUART() *mockUART {
	return &mockUART{
		readBuffer:  &bytes.Buffer{},
		writeBuffer: &bytes.Buffer{},
	}
}

// Configure implements UART interface (no-op for mock).
func (m *mockUART) Configure(config interface{}) error {
	return nil
}

// Write implements UART interface.
func (m *mockUART) Write(b byte) error {
	return m.writeBuffer.WriteByte(b)
}

// Read implements UART interface - reads one byte at a time.
func (m *mockUART) Read() (byte, error) {
	return m.readBuffer.ReadByte()
}

// ReadByte reads one byte from the buffer (for compatibility).
func (m *mockUART) ReadByte() (byte, error) {
	return m.readBuffer.ReadByte()
}

// WriteData puts data into the read buffer for the mock to consume.
func (m *mockUART) WriteData(data []byte) {
	m.readBuffer.Write(data)
}

// ===== Mock DigitalPin for testing =====

// mockPin implements the DigitalPin interface for testing.
type mockPin struct {
	state  uint8 // 0 = Low, 1 = High
	config interface{}
}

// NewMockPin creates a new mock pin.
func NewMockPin() *mockPin {
	return &mockPin{state: 1} // Default to High
}

// Configure implements DigitalPin interface.
func (p *mockPin) Configure(config interface{}) {
	p.config = config
}

// High implements DigitalPin interface.
func (p *mockPin) High() {
	p.state = 1
}

// Low implements DigitalPin interface.
func (p *mockPin) Low() {
	p.state = 0
}

// Get implements DigitalPin interface.
func (p *mockPin) Get() uint8 {
	return p.state
}

// GetState returns the current state for testing.
func (p *mockPin) GetState() uint8 {
	return p.state
}

// ===== Mock PWM for testing =====

// mockPWM implements the PWM interface for testing.
type mockPWM struct {
	channels map[uint8]uint32
	top      uint32
}

// NewMockPWM creates a new mock PWM instance.
func NewMockPWM() *mockPWM {
	return &mockPWM{
		channels: make(map[uint8]uint32),
		top:      1000,
	}
}

// Configure implements PWM interface (no-op for mock).
func (m *mockPWM) Configure(config interface{}) error {
	return nil
}

// Channel implements PWM interface - returns a mock channel number.
func (m *mockPWM) Channel(pin interface{}) (uint8, error) {
	return 0, nil
}

// Set implements PWM interface - stores the value for a channel.
func (m *mockPWM) Set(channel uint8, value uint32) {
	m.channels[channel] = value
}

// Top implements PWM interface - returns the PWM period.
func (m *mockPWM) Top() uint32 {
	return m.top
}

// GetChannelValue returns the stored value for a channel (for testing).
func (m *mockPWM) GetChannelValue(channel uint8) uint32 {
	return m.channels[channel]
}

// ===== Mock I2C for testing =====

// mockI2C implements the I2C interface for testing.
type mockI2C struct {
	readData map[uint8][]byte // Map of register addr to data
}

// NewMockI2C creates a new mock I2C instance.
func NewMockI2C() *mockI2C {
	return &mockI2C{
		readData: make(map[uint8][]byte),
	}
}

// Configure implements I2C interface (no-op for mock).
func (m *mockI2C) Configure(config interface{}) error {
	return nil
}

// Tx implements I2C interface - handles register reads for IMU.
func (m *mockI2C) Tx(addr uint16, w []byte, r []byte) error {
	// For testing purposes, if reading, return predefined data
	if len(w) > 0 && len(r) > 0 {
		registerAddr := w[0]
		if data, ok := m.readData[registerAddr]; ok {
			copy(r, data)
		}
	}
	return nil
}

// SetRegisterData sets what data should be returned for a register read.
func (m *mockI2C) SetRegisterData(registerAddr uint8, data []byte) {
	m.readData[registerAddr] = make([]byte, len(data))
	copy(m.readData[registerAddr], data)
}

// ===== Mock LED Updater for testing =====

// mockLEDUpdater implements the LEDUpdater interface for testing.
type mockLEDUpdater struct {
	lastState LEDState
}

// NewMockLEDUpdater creates a new mock LED updater.
func NewMockLEDUpdater() *mockLEDUpdater {
	return &mockLEDUpdater{}
}

// SetState implements LEDUpdater interface.
func (m *mockLEDUpdater) SetState(state LEDState) {
	m.lastState = state
}

// Update implements LEDUpdater interface.
func (m *mockLEDUpdater) Update() {
	// No-op for mock
}

// GetLastState returns the last state set (for testing).
func (m *mockLEDUpdater) GetLastState() LEDState {
	return m.lastState
}

// ===== Mock IMU Device for testing =====

// mockIMUDevice implements the IMUDevice interface for testing.
type mockIMUDevice struct {
	gyroX, gyroY, gyroZ   int16
	accelX, accelY, accelZ int16
	connected             bool
}

// NewMockIMUDevice creates a new mock IMU device.
func NewMockIMUDevice() *mockIMUDevice {
	return &mockIMUDevice{
		connected: true,
	}
}

// Configure implements IMUDevice interface.
func (m *mockIMUDevice) Configure(config interface{}) error {
	return nil
}

// Connected implements IMUDevice interface.
func (m *mockIMUDevice) Connected() bool {
	return m.connected
}

// ReadGyro implements IMUDevice interface.
func (m *mockIMUDevice) ReadGyro() (x, y, z int16, err error) {
	return m.gyroX, m.gyroY, m.gyroZ, nil
}

// ReadAccel implements IMUDevice interface.
func (m *mockIMUDevice) ReadAccel() (x, y, z int16, err error) {
	return m.accelX, m.accelY, m.accelZ, nil
}

// SetGyroData sets the gyro values to return.
func (m *mockIMUDevice) SetGyroData(x, y, z int16) {
	m.gyroX = x
	m.gyroY = y
	m.gyroZ = z
}

// SetAccelData sets the accelerometer values to return.
func (m *mockIMUDevice) SetAccelData(x, y, z int16) {
	m.accelX = x
	m.accelY = y
	m.accelZ = z
}

// ===== Test Data Helpers =====

// FloatEqual compares two floats with tolerance for testing.
func FloatEqual(a, b, tolerance float64) bool {
	if a > b {
		return (a - b) < tolerance
	}
	return (b - a) < tolerance
}
