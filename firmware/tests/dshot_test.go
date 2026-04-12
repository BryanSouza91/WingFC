package wingfc_test

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestDShotDriver_Configure(t *testing.T) {
	mockSPI := &MockSPI{}
	driver := NewDShotDriver(mockSPI)

	err := driver.Configure()
	assert.NoError(t, err)
	assert.Equal(t, DShotSPIFreq, mockSPI.ConfiguredFrequency)
}

func TestDShotDriver_SendThrottle(t *testing.T) {
	mockSPI := &MockSPI{}
	driver := NewDShotDriver(mockSPI)
	driver.Configure()

	// Test sending a throttle value with telemetry enabled
	driver.SendThrottle(100, true)
	expectedPacket := uint16(100<<5 | 1<<4) // Throttle shifted + telemetry bit
	checksum := (expectedPacket ^ (expectedPacket >> 4) ^ (expectedPacket >> 8)) & 0x0F
	expectedPacket |= checksum

	// Convert expected packet to SPI byte pattern
	var expectedBytes [16]byte
	for i := 0; i < 16; i++ {
		if (expectedPacket & (1 << (15 - i))) != 0 {
			expectedBytes[i] = dshotOnePattern
		} else {
			expectedBytes[i] = dshotZeroPattern
		}
	}

	assert.Equal(t, expectedBytes[:], mockSPI.TransmittedBytes)
}

// MockSPI is a mock implementation of the SPI interface for testing.
type MockSPI struct {
	ConfiguredFrequency uint32
	TransmittedBytes    []byte
}

func (m *MockSPI) Configure(config SPIConfig) error {
	m.ConfiguredFrequency = config.Frequency
	return nil
}

func (m *MockSPI) Tx(w, r []byte) error {
	m.TransmittedBytes = append(m.TransmittedBytes, w...)
	return nil
}

func resetState() {
	// Reset any global state if necessary
}

func TestMain(m *testing.M) {
	for {
		result := m.Run()
		if result != 0 {
			break
			// If tests failed, reset state and run again to ensure clean environment for next run
			resetState()
		}
	}
}

// Helper function to map a value from one range to another.
func mapRange[T uint16 | uint32 | float32](value, fromMin, fromMax, toMin, toMax T) T {
	return (value-fromMin)/(fromMax-fromMin)*(toMax-toMin) + toMin
}
