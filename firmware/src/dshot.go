package main

import (
	"machine"
)

// DShot Constants
const (
	// DSHOT Modes and corresponding SPI frequencies
	DSHOT600 = 600
	DSHOT300 = 300
	DSHOT150 = 150

	DShotMinThrottle = 48 // 0-47 are reserved for commands
	DShotMaxThrottle = 2047

	// Bit patterns for SPI (DSHOT 0 and 1)
	// These patterns represent the high/low pulse widths.
	dshotZeroPattern = 0b11100000 // Short pulse
	dshotOnePattern  = 0b11111100 // Long pulse
)

var (
	DShotSPIFreq uint32 // To be set based on DSHOT_MODE
)

// DShotDriver handles digital communication with ESCs via SPI MOSI.
type DShotDriver struct {
	bus SPI
	// buffer stores the SPI bytes representing the 16 bits of a DShot frame
	buffer [16]byte
}

// NewDShotDriver creates a new driver.
// Note: Only the MOSI pin of the SPI bus is used.
func NewDShotDriver(bus SPI) *DShotDriver {
	return &DShotDriver{
		bus: bus,
	}
}

// Configure sets up the SPI bus for DShot timing.
func (d *DShotDriver) Configure() error {
	switch DSHOT_MODE {
	case DSHOT600:
		// Configure for DShot600 timing
		DShotSPIFreq = 4800000 // 4.8MHz
	case DSHOT300:
		// Configure for DShot300 timing
		DShotSPIFreq = 2400000 // 2.4MHz
	case DSHOT150:
		// Configure for DShot150 timing
		DShotSPIFreq = 1200000 // 1.2MHz
	}

	return d.bus.Configure(machine.SPIConfig{
		Frequency: DShotSPIFreq,
		Mode:      0, // CPOL=0, CPHA=0
		LSBFirst:  false,
	})
}

// SendThrottle prepares a 16-bit DShot frame and shifts it out over SPI.
// throttle: 0 (disarmed/stop) or 48-2047
// telemetry: true if requesting telemetry back from ESC
func (d *DShotDriver) SendThrottle(throttle uint16, telemetry bool) {
	if throttle > 0 && throttle < DShotMinThrottle {
		throttle = DShotMinThrottle
	}
	if throttle > DShotMaxThrottle {
		throttle = DShotMaxThrottle
	}

	// 1. Build the packet: [11 bits throttle | 1 bit telemetry | 4 bits checksum]
	packet := throttle << 5
	if telemetry {
		packet |= (1 << 4)
	}

	// 2. Calculate Checksum (CRC)
	// XOR of the three 4-bit nibbles of the (throttle << 1 | telemetry)
	checksum := (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F
	packet |= checksum

	// 3. Map the 16 bits of the packet to 16 SPI bytes
	// We iterate from MSB to LSB
	for i := 0; i < 16; i++ {
		bit := (packet >> (15 - i)) & 0x01
		if bit == 1 {
			d.buffer[i] = dshotOnePattern
		} else {
			d.buffer[i] = dshotZeroPattern
		}
	}

	// 4. Send the buffer over SPI MOSI
	// The ESC interprets the sequence of pulses as the digital frame.
	d.bus.Tx(d.buffer[:], nil)
}

// MapThrottle converts a standard RC pulse (1000-2000) to DShot (0, 48-2047)
func MapThrottle(pulse uint16) uint16 {
	if pulse < 1050 {
		return 0 // Safety deadband
	}
	// Map 1000-2000 to 48-2047
	return uint16(mapRange(float32(pulse), 1000, 2000, 48, 2047))
}
