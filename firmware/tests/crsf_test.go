package main

import (
	"fmt"
	"testing"
	"time"
)

// --- MOCK HARDWARE AND DEPENDENCIES FOR TESTING ---

// We need to mock the machine package's UART interface for testing.
// This is done by creating a new type that implements the ReadByte() method.
type mockUART struct {
	dataChan chan byte
}

func (m *mockUART) ReadByte() (byte, error) {
	select {
	case b := <-m.dataChan:
		return b, nil
	case <-time.After(10 * time.Millisecond): // Simulate a timeout
		return 0, nil
	}
}

func (m *mockUART) WriteByte(b byte) error {
	return nil
}

// Override the global uart variable for testing.
var uart *mockUART

// Define the constants and types from crsf.go and config.go
const (
	CRSF_PACKET_SIZE = 26
	NumChannels      = 16
	CRSF_SYNC_BYTE   = 0xC8

	CRSF_CHANNEL_VALUE_MIN = 172  // 987us
	CRSF_CHANNEL_VALUE_MAX = 1811 // 2012us
	MIN_RX_VALUE           = 988
	MAX_RX_VALUE           = 2012
)

type CRSFState int

const (
	WAITING_FOR_HEADER1 CRSFState = iota
	READING_LENGTH
	READING_TYPE_AND_PAYLOAD
	READING_CHECKSUM
)

// A simplified, test-only version of the global channel
var packetChan = make(chan [CRSF_PACKET_SIZE]byte)

// Define a simplified version of calculateCrc8 for the test file.
func calculateCrc8(data []byte) byte {
	crc := byte(0x00)
	for _, b := range data {
		crc ^= b
		for i := 0; i < 8; i++ {
			if (crc & 0x80) != 0 {
				crc = (crc << 1) ^ 0xD5
			} else {
				crc = crc << 1
			}
		}
	}
	return crc
}

// --- TEST THE readReceiver FUNCTION ---

// We need to include a simplified version of readReceiver to be tested.
func readReceiver(packetChan chan<- [CRSF_PACKET_SIZE]byte) {
	var crsfState CRSFState
	var packetIndex int
	var buffer [CRSF_PACKET_SIZE]byte
	var packetLength int

	for {
		data, err := uart.ReadByte()
		if err != nil {
			// If there's no data available, we can just continue.
			println("No data from CRSF RX")
			continue
		}

		switch crsfState {
		case WAITING_FOR_HEADER1:
			if data == CRSF_SYNC_BYTE {
				crsfState = READING_LENGTH
				packetIndex = 1
				buffer[0] = data
			}
		case READING_LENGTH:
			packetLength = int(data)
			buffer[1] = data
			packetIndex = 2
			crsfState = READING_TYPE_AND_PAYLOAD
		case READING_TYPE_AND_PAYLOAD:
			buffer[packetIndex] = data
			packetIndex++
			if packetIndex >= packetLength+1 { // We've read all payload bytes, next is checksum
				crsfState = READING_CHECKSUM
			}
		case READING_CHECKSUM:
			// The checksum byte is the last byte in the packet, at index 25.
			// Do not increment packetIndex after processing.
			buffer[packetIndex] = data

			// The CRC8 is calculated over the frame, from the length byte
			// at index 1 to the end of the payload at packetIndex-1.
			calculatedChecksum := calculateCrc8(buffer[2:packetIndex])
			crc := crc8(buffer[2:packetIndex])
			if crc == calculatedChecksum {
				println("MATCH:\t", crc, "\t", calculatedChecksum)
			} else {
				println("FAIL:\t", crc, "\t", calculatedChecksum)
			}

			if calculatedChecksum == data {
				println(fmt.Sprintf("%q", buffer[:]))
				packetChan <- buffer
			} else {
				println("Checksum mismatch. Discarding packet.")
			}

			// Reset the state machine for the next packet.
			crsfState = WAITING_FOR_HEADER1
			packetIndex = 0
		}
	}
}

// --- THE ACTUAL TEST CASE ---

func TestCRSFProtocol(t *testing.T) {
	// Create a mock UART and a channel to feed data.
	mockUart := &mockUART{
		dataChan: make(chan byte, CRSF_PACKET_SIZE),
	}

	// Override the global uart variable with our mock for the test.
	uart = mockUart

	// Create the packet channel for the receiver to send its output to.
	testPacketChan := make(chan [CRSF_PACKET_SIZE]byte, 1)

	// A valid CRSF packet to test with.
	// This packet has a length of 24 (0x18), type of 22 (0x16), 22 payload bytes, and CRC of 173 (0xad)
	// The full packet is 1(sync) + 1(length) + 1(type) + 22(payload) + 1(crc) = 26 bytes.

	// Valid
	packetData := []byte{
		0xc8, 0x18, 0x16, 0xe0, 0x03, 0x1f, 0xf8, 0xc0, 0x07, 0x3e, 0xf0, 0x81, 0x0f, 0x7c,
		0xe0, 0x03, 0x1f, 0xf8, 0xc0, 0x07, 0x3e, 0xf0, 0x81, 0x0f, 0x7c, 0xad,
	}

	// Valid
	// packetData := []byte{
	// 	0xC8, 0x18, 0x16, 0xE0, 0x03, 0x9F, 0x2B, 0xC0, 0xF7, 0x8B, 0x5F, 0xFC, 0xE2, 0x17,
	// 	0xBF, 0xF8, 0x85, 0xE6, 0x5C, 0x03, 0x00, 0x00, 0x4C, 0x3C, 0xD7, 0xBD,
	// }

	// Invalid
	// packetData := []byte{
	// 	0xC8, 0x18, 0x16, 0xAE, 0x70, 0x85, 0x2B, 0x68, 0xF1, 0x8B, 0x9F, 0xFC, 0xE2, 0x17,
	// 	0x7F, 0xF8, 0x05, 0xF8, 0x28, 0x08, 0x00, 0x00, 0x4C, 0x7C, 0xE2, 0x63,
	// }

	// Invalid
	// packetData := []byte{
	// 	0xC8, 0x18, 0x16, 0xC0, 0x03, 0x9F, 0x2B, 0x80, 0xF7, 0x8B, 0x5F, 0x94, 0x0F, 0xC0,
	// 	0x7F, 0x48, 0x4A, 0xF9, 0xCA, 0x07, 0x00, 0x00, 0x4C, 0x7C, 0xE2, 0x09,
	// }

	// Valid
	// packetData := []byte{
	// 	0xC8, // Address
	// 	0x18, // Length
	// 	0x16, // Type (RC Channels)
	// 	0x03, 0x1F, 0x58, 0xC0, 0x07, 0x16, 0xB0, 0x80, 0x05, 0x2C, 0x60, 0x01, 0x0B, 0xF8, 0xC0,
	// 	0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 252, // Packet
	// 	0x42, // Crc
	// }

	println(fmt.Sprintf("%v", packetData[:CRSF_PACKET_SIZE-1]))
	// Run the readReceiver function in a goroutine.
	go readReceiver(testPacketChan)

	// Feed the packet data byte by byte to the mock UART.
	for _, b := range packetData {
		mockUart.dataChan <- b
	}

	// Wait for a packet to be received from the receiver.
	select {
	case receivedPacket := <-testPacketChan:
		// Convert the byte slice to a fixed-size array for comparison.
		expectedPacket := [CRSF_PACKET_SIZE]byte{}
		copy(expectedPacket[:], packetData)

		// Check if the received packet matches the expected packet.
		if receivedPacket != expectedPacket {
			t.Errorf("Received packet does not match expected packet.\nExpected: %x\nGot: %x\n", expectedPacket, receivedPacket)
		} else {
			channels := processReceiverPacket(receivedPacket)
			for i := range channels {
				print("CH", i+1, "\t")
				println(channels[i])
			}
			t.Log("Successfully received and validated the CRSF packet.")
		}
	case <-time.After(50 * time.Millisecond):
		t.Fatal("Timeout: readReceiver did not produce a packet.")
	}
}

// processReceiverPacket unpacks the 11-bit channel values from a CRSF packet payload.
// This function is based on the robust bit-packing logic from BetaFlight.
func processReceiverPacket(payload [CRSF_PACKET_SIZE]byte) [NumChannels]uint16 {
	// The RC channel data starts at byte 3 of the packet
	const payloadStartIndex = 3
	// The payload is from index 3 to the checksum byte's index (25) - 1
	bitstream := payload[payloadStartIndex : CRSF_PACKET_SIZE-1]

	var channelValues [NumChannels]uint16
	var bitsMerged uint
	var readValue uint32
	var readByteIndex uint

	for n := 0; n < NumChannels; n++ {
		for bitsMerged < 11 {
			// Add a boundary check to prevent out of range access
			if readByteIndex >= uint(len(bitstream)) {
				return channelValues
			}
			readByte := bitstream[readByteIndex]
			readByteIndex++
			readValue |= uint32(readByte) << bitsMerged
			bitsMerged += 8
		}
		channelValues[n] = uint16(readValue & 0x07FF)
		readValue >>= 11
		bitsMerged -= 11
	}
	return channelValues
}

// crc8 calculates the 8-bit CRC for a given byte slice using a lookup table.
//
// Parameters:
//   - data: The byte slice to be processed.
//
// Returns:
//   - An 8-bit unsigned integer representing the calculated CRC.
func crc8(data []uint8) uint8 {
	var crsfcrc8tab = [256]uint8{
		0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
		0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
		0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
		0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
		0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
		0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
		0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
		0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
		0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
		0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
		0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
		0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
		0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
		0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
		0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
		0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9,
	}

	var crc uint8 = 0
	for _, b := range data {
		crc = crsfcrc8tab[crc^b]
	}
	return crc
}
