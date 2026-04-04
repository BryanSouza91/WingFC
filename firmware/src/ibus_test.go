//go:build ibus
// +build ibus

package main

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestIBusProcessReceiverPacket(t *testing.T) {
	tests := []struct {
		name     string
		packet   [IBUS_PACKET_SIZE]byte
		expected [NumChannels]uint16
	}{
		{
			name:   "All channels zero",
			packet: [IBUS_PACKET_SIZE]byte{},
			expected: func() [NumChannels]uint16 {
				var arr [NumChannels]uint16
				return arr
			}(),
		},
		{
			name: "First channel 1500 (mid)",
			packet: func() [IBUS_PACKET_SIZE]byte {
				var arr [IBUS_PACKET_SIZE]byte
				// 1500 = 0x05DC, little endian: DC 05
				arr[0] = 0xDC
				arr[1] = 0x05
				return arr
			}(),
			expected: func() [NumChannels]uint16 {
				var arr [NumChannels]uint16
				arr[0] = 1500
				return arr
			}(),
		},
		{
			name: "Multiple channels",
			packet: func() [IBUS_PACKET_SIZE]byte {
				var arr [IBUS_PACKET_SIZE]byte
				// Channel 0: 1500
				arr[0] = 0xDC
				arr[1] = 0x05
				// Channel 1: 1000
				arr[2] = 0xE8
				arr[3] = 0x03
				// Channel 2: 2000
				arr[4] = 0xD0
				arr[5] = 0x07
				return arr
			}(),
			expected: func() [NumChannels]uint16 {
				var arr [NumChannels]uint16
				arr[0] = 1500
				arr[1] = 1000
				arr[2] = 2000
				return arr
			}(),
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			result := processReceiverPacket(test.packet)

			for i := 0; i < NumChannels; i++ {
				assert.Equal(t, test.expected[i], result[i],
					"Channel %d mismatch", i)
			}
		})
	}
}

func TestIBusPacketSize(t *testing.T) {
	// Verify that the packet size is correct
	// Header (2) + Channels (18 * 2) + Checksum (2) = 40 bytes
	expectedSize := 2 + (IBUS_NUM_CHANNELS * 2) + 2
	assert.Equal(t, 40, expectedSize)
	assert.Equal(t, expectedSize, IBUS_PACKET_SIZE)
}

func TestIBusHeaderConstants(t *testing.T) {
	// Verify iBus protocol header constants
	assert.Equal(t, byte(0x20), IBUS_HEADER1)
	assert.Equal(t, byte(0x40), IBUS_HEADER2)
}

func TestIBusNumberOfChannels(t *testing.T) {
	// Verify that iBus uses correct number of channels
	assert.Equal(t, 18, IBUS_NUM_CHANNELS)
	assert.Equal(t, NumChannels, IBUS_NUM_CHANNELS)
}

func TestIBusChannelExtraction(t *testing.T) {
	// Test extracting specific channel values
	tests := []struct {
		name         string
		channelIndex int
		lowByte      byte
		highByte     byte
		expectedVal  uint16
	}{
		{
			name:         "Mid value",
			channelIndex: 0,
			lowByte:      0xDC,
			highByte:     0x05,
			expectedVal:  1500,
		},
		{
			name:         "Min value",
			channelIndex: 0,
			lowByte:      0xE8,
			highByte:     0x03,
			expectedVal:  1000,
		},
		{
			name:         "Max value",
			channelIndex: 0,
			lowByte:      0xD0,
			highByte:     0x07,
			expectedVal:  2000,
		},
		{
			name:         "Zero value",
			channelIndex: 0,
			lowByte:      0x00,
			highByte:     0x00,
			expectedVal:  0,
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			var packet [IBUS_PACKET_SIZE]byte
			packet[2*test.channelIndex] = test.lowByte
			packet[2*test.channelIndex+1] = test.highByte

			result := processReceiverPacket(packet)
			assert.Equal(t, test.expectedVal, result[test.channelIndex])
		})
	}
}

func TestIBusAllChannelsExtraction(t *testing.T) {
	// Test that all 18 channels can be extracted properly
	var packet [IBUS_PACKET_SIZE]byte

	// Set each channel to a unique value
	for i := 0; i < IBUS_NUM_CHANNELS; i++ {
		val := uint16(1000 + i*50) // 1000, 1050, 1100, ..., 1850
		packet[2*i] = byte(val & 0xFF)
		packet[2*i+1] = byte((val >> 8) & 0xFF)
	}

	result := processReceiverPacket(packet)

	// Verify each channel
	for i := 0; i < IBUS_NUM_CHANNELS; i++ {
		expected := uint16(1000 + i*50)
		assert.Equal(t, expected, result[i],
			"Channel %d: expected %d, got %d", i, expected, result[i])
	}
}

func TestIBusChannelDataIndependence(t *testing.T) {
	// Verify that channel data doesn't interfere with each other
	var packet [IBUS_PACKET_SIZE]byte

	// Set specific channels
	packet[0] = 0xDC  // Channel 0 low
	packet[1] = 0x05  // Channel 0 high (1500)
	packet[4] = 0xD0  // Channel 2 low
	packet[5] = 0x07  // Channel 2 high (2000)

	result := processReceiverPacket(packet)

	assert.Equal(t, uint16(1500), result[0])
	assert.Equal(t, uint16(0), result[1])    // Should be zero
	assert.Equal(t, uint16(2000), result[2])
	assert.Equal(t, uint16(0), result[3])    // Should be zero
}

func TestIBusLittleEndianEncoding(t *testing.T) {
	// Test little-endian byte order interpretation
	tests := []struct {
		name         string
		val          uint16
		lowExpected  byte
		highExpected byte
	}{
		{
			name:         "1500",
			val:          1500,
			lowExpected:  byte(1500 & 0xFF),
			highExpected: byte((1500 >> 8) & 0xFF),
		},
		{
			name:         "65535 (max uint16)",
			val:          65535,
			lowExpected:  byte(65535 & 0xFF),
			highExpected: byte((65535 >> 8) & 0xFF),
		},
		{
			name:         "256",
			val:          256,
			lowExpected:  byte(256 & 0xFF),
			highExpected: byte((256 >> 8) & 0xFF),
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			var packet [IBUS_PACKET_SIZE]byte
			packet[0] = test.lowExpected
			packet[1] = test.highExpected

			result := processReceiverPacket(packet)
			extracted := result[0]

			// Verify little-endian reconstruction
			expected := uint16(test.lowExpected) | uint16(test.highExpected)<<8
			assert.Equal(t, expected, extracted)
			assert.Equal(t, test.val, extracted)
		})
	}
}

func TestIBusArmChannel(t *testing.T) {
	// Test ARM channel (channel 4)
	var packet [IBUS_PACKET_SIZE]byte

	// Set ARM channel (channel 4) to HIGH value (1800+)
	armValue := uint16(1900)
	packet[2*ArmChannel] = byte(armValue & 0xFF)
	packet[2*ArmChannel+1] = byte((armValue >> 8) & 0xFF)

	result := processReceiverPacket(packet)

	assert.Equal(t, armValue, result[ArmChannel])
	assert.True(t, result[ArmChannel] >= uint16(HIGH_RX_VALUE))
}

func TestIBusThrottleChannel(t *testing.T) {
	// Test THROTTLE channel (channel 2)
	var packet [IBUS_PACKET_SIZE]byte

	throttleValue := uint16(1200)
	packet[2*ThrottleChannel] = byte(throttleValue & 0xFF)
	packet[2*ThrottleChannel+1] = byte((throttleValue >> 8) & 0xFF)

	result := processReceiverPacket(packet)

	assert.Equal(t, throttleValue, result[ThrottleChannel])
}

func TestIBusAllZeroPacket(t *testing.T) {
	// Test processing a packet with all zeros
	var packet [IBUS_PACKET_SIZE]byte
	result := processReceiverPacket(packet)

	for i := 0; i < NumChannels; i++ {
		assert.Equal(t, uint16(0), result[i])
	}
}

func TestIBusMaxValuePacket(t *testing.T) {
	// Test processing a packet with max values
	var packet [IBUS_PACKET_SIZE]byte

	maxVal := uint16(2000)
	for i := 0; i < IBUS_NUM_CHANNELS; i++ {
		packet[2*i] = byte(maxVal & 0xFF)
		packet[2*i+1] = byte((maxVal >> 8) & 0xFF)
	}

	result := processReceiverPacket(packet)

	for i := 0; i < NumChannels; i++ {
		assert.Equal(t, maxVal, result[i])
	}
}
