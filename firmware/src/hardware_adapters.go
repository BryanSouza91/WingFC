package main

import (
	"fmt"
	"machine"
	"time"

	math "github.com/orsinium-labs/tinymath"
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

// GPSAdapter wraps the GPS driver for UART-based communication.
type GPSAdapter struct {
	device  gps.Device
	parser  gps.Parser
	data    GPSData
	config  GPSConfig
	lastFix gps.Fix
}

// NewGPS creates a new GPS adapter wrapping the  GPS driver.
func NewGPS(uart *MachineUART) GPSAdapter {
	parser := gps.NewParser()
	device := gps.NewUART(uart.uart)
	return GPSAdapter{
		device:  device,
		parser:  parser,
		data:    GPSData{},
		config:  GPSConfig{Enabled: GPSEnabled, BaudRate: GPSBaudRate, StaleDataTimeout: 2 * time.Second, HomeNotCaptured: true},
		lastFix: gps.Fix{},
	}
}

// Configure initializes the GPS device (UART already configured in hardware.go).
func (g *GPSAdapter) Configure() error {
	// GPS device is already initialized with UART connection
	return nil
}

// ReadGPS reads and parses NMEA sentences from the GPS device.
// Returns the most recent valid fix or stale data depending on timeout.
func (g *GPSAdapter) ReadGPS() (GPSData, error) {
	// Read NMEA sentences in a loop until we get one we can parse or we've tried enough.
	// This prevents blocking on the main flight loop.
	for attempt := 0; attempt < 10; attempt++ {
		sentence, err := g.device.NextSentence()

		// Handle EOF or no data available (non-blocking read)
		if err != nil || sentence == "" {
			break // No more data available
		}

		// Parse the sentence
		fix, parseErr := g.parser.Parse(sentence)
		if parseErr != nil {
			// Log parse errors but continue to next sentence
			println("GPS parse error:", parseErr)
			continue
		}

		// Update the last received fix
		g.lastFix = fix

		// Check if this is a valid fix
		if fix.Valid {
			g.data.Latitude = fix.Latitude
			g.data.Longitude = fix.Longitude
			g.data.Altitude = fix.Altitude
			g.data.Satellites = int(fix.Satellites)
			g.data.Fix = 3 // Assuming valid fix is 3D (GGA sentence)
			g.data.Speed = fix.Speed
			g.data.Course = fix.Heading
			g.data.Time = fix.Time.Format("15:04:05")
			g.data.Valid = true
			g.data.LastUpdateTime = time.Now()
			g.data.HDOP = 1.0 // Default HDOP value ( GPS doesn't expose HDOP)
		}
	}

	// Check for stale data (no update for > StaleDataTimeout)
	if !g.data.LastUpdateTime.IsZero() && time.Since(g.data.LastUpdateTime) > g.config.StaleDataTimeout {
		g.data.Valid = false
	}

	return g.data, nil
}

// CaptureHome stores the current position as the home location.
func (g *GPSAdapter) CaptureHome() error {
	if !g.data.Valid {
		return fmt.Errorf("cannot capture home: GPS fix is not valid")
	}
	g.config.HomeLatitude = g.data.Latitude
	g.config.HomeLongitude = g.data.Longitude
	g.config.HomeAltitude = g.data.Altitude
	g.config.HomeNotCaptured = false
	println("GPS: Home captured at", g.data.Latitude, ",", g.data.Longitude, "Altitude:", g.data.Altitude)
	return nil
}

// IsValid returns whether the GPS fix is currently valid.
func (g *GPSAdapter) IsValid() bool {
	return g.data.Valid && !g.data.LastUpdateTime.IsZero() && time.Since(g.data.LastUpdateTime) <= g.config.StaleDataTimeout
}

// GetDistanceTo calculates the distance to a given lat/lon using Haversine formula (meters).
func (g *GPSAdapter) GetDistanceTo(latitude, longitude float32) float32 {
	if !g.IsValid() {
		return -1 // Invalid fix
	}

	const earthRadius float32 = 6371000.0 // meters
	const degreesToRadians float32 = math.Pi / 180.0

	lat1 := g.data.Latitude * degreesToRadians
	lat2 := latitude * degreesToRadians
	deltaLat := (latitude - g.data.Latitude) * degreesToRadians
	deltaLon := (longitude - g.data.Longitude) * degreesToRadians

	a := math.Sin(deltaLat/2)*math.Sin(deltaLat/2) +
		math.Cos(lat1)*math.Cos(lat2)*math.Sin(deltaLon/2)*math.Sin(deltaLon/2)

	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))
	distance := earthRadius * c

	return distance
}
