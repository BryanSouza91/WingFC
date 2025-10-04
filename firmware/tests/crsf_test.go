package main

import (
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

			if calculatedChecksum == data {
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
	packetData := []byte{
		0xc8, 0x18, 0x16, 0xe0, 0x03, 0x1f, 0xf8, 0xc0, 0x07, 0x3e, 0xf0, 0x81, 0x0f, 0x7c,
		0xe0, 0x03, 0x1f, 0xf8, 0xc0, 0x07, 0x3e, 0xf0, 0x81, 0x0f, 0x7c, 0xad,
	}

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
				print("CH:", i+1, "\t")
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