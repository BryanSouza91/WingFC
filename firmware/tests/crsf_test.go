package crsf

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

const NumChannels = 16
var Channels [NumChannels]uint16

// --- THE ACTUAL TEST CASE ---

// TestCRSFProtocol runs a suite of tests on the readReceiver function with a variety of packet data.
func TestCRSFProtocol(t *testing.T) {
	// Define the test cases in a slice of structs.
	testCases := []struct {
		name              string
		packetData        []byte
		expectedToSucceed bool
		expectedChannels  [NumChannels]uint16
	}{
		{
			name: "Valid RC Channels Packet 1 (Centered)",
			packetData: []byte{
				0xc8, 0x18, 0x16, 0xe0, 0x03, 0x1f, 0xf8, 0xc0, 0x07, 0x3e, 0xf0, 0x81, 0x0f, 0x7c,
				0xe0, 0x03, 0x1f, 0xf8, 0xc0, 0x07, 0x3e, 0xf0, 0x81, 0x0f, 0x7c, 0xad,
			},
			expectedToSucceed: true,
			expectedChannels:  [NumChannels]uint16{992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992},
		},
		{
			name: "Valid RC Channels Packet 2 (Random Values)",
			packetData: []byte{
				0xC8, 0x18, 0x16, 0xE0, 0x03, 0x9F, 0x2B, 0xC0, 0xF7, 0x8B, 0x5F, 0xFC, 0xE2, 0x17,
				0xBF, 0xF8, 0x85, 0xE6, 0x5C, 0x03, 0x00, 0x00, 0x4C, 0x3C, 0xD7, 0xBD,
			},
			expectedToSucceed: true,
			expectedChannels:  [NumChannels]uint16{992, 992, 174, 992, 191, 191, 191, 191, 191, 191, 922, 430, 0, 0, 1811, 1721},
		},
		{
			name: "Valid RC Channels Packet 3 (Random Values)",
			packetData: []byte{
				0xC8, // Address
				0x18, // Length
				0x16, // Type (RC Channels)
				0x03, 0x1F, 0x58, 0xC0, 0x07, 0x16, 0xB0, 0x80, 0x05, 0x2C, 0x60, 0x01, 0x0B, 0xF8, 0xC0,
				0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 252, // Packet
				0x42, // Crc
			},
			expectedToSucceed: true,
			expectedChannels:  [NumChannels]uint16{1795, 771, 1793, 771, 769, 769, 769, 769, 769, 1793, 1795, 3, 0, 0, 0, 2016},
		},
		{
			name: "Valid RC Channels Packet 4 (Random Values)",
			packetData: []byte{
				0xC8, 12, 0x14, 16, 19, 99, 151, 1, 2, 3, 8, 88, 148, 252, 0xC8, 24, 0x16, 0xE0, 0x03,
				0x1F, 0x58, 0xC0, 0x07, 0x16, 0xB0, 0x80, 0x05, 0x2C, 0x60, 0x01, 0x0B, 0xF8, 0xC0,
				0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 103,
			},
			expectedToSucceed: true,
			expectedChannels:  [NumChannels]uint16{992, 992, 352, 992, 352, 352, 352, 352, 352, 352, 992, 992, 0, 0, 0, 0},
		},
		{
			name: "Invalid Checksum Packet 1",
			packetData: []byte{
				0xC8, 0x18, 0x16, 0xAE, 0x70, 0x85, 0x2B, 0x68, 0xF1, 0x8B, 0x9F, 0xFC, 0xE2, 0x17,
				0x7F, 0xF8, 0x05, 0xF8, 0x28, 0x08, 0x00, 0x00, 0x4C, 0x7C, 0xE2, 0x63, // Incorrect CRC
			},
			expectedToSucceed: false, // The CRC is wrong, so the packet should be dropped.
		},
		{
			name: "Invalid Checksum Packet 2",
			packetData: []byte{
				0xC8, 0x18, 0x16, 0xC0, 0x03, 0x9F, 0x2B, 0x80, 0xF7, 0x8B, 0x5F, 0x94, 0x0F, 0xC0,
				0x7F, 0x48, 0x4A, 0xF9, 0xCA, 0x07, 0x00, 0x00, 0x4C, 0x7C, 0xE2, 0x09, // Incorrect CRC
			},
			expectedToSucceed: false, // The CRC is wrong, so the packet should be dropped.
		},
	}

	for _, tc := range testCases {
		t.Run(tc.name, func(t *testing.T) {
			// Create a new mock UART and a channel for each test case.
			mockUart := &mockUART{
				dataChan: make(chan byte, CRSF_PACKET_SIZE),
			}

			uart = mockUart
			testPacketChan := make(chan [CRSF_PACKET_SIZE]byte, 1)

			// Start the receiver goroutine.
			go readReceiver(testPacketChan)

			// Feed the packet data byte by byte to the mock UART.
			for _, b := range tc.packetData {
				mockUart.dataChan <- b
			}

			// Wait for a packet to be received from the receiver.
			select {
			case receivedPacket := <-testPacketChan:
				// Add the correct checksum to the packetData if it's expected to succeed.
				if tc.expectedToSucceed {
					// The CRC is calculated over the payload, starting from the type byte (index 2).
					crc := calculateCrc8(receivedPacket[2 : len(receivedPacket)-1])
					t.Log("Checksums:\t", crc, tc.packetData[len(tc.packetData)-1])
				} else {
					t.Errorf("Received an unexpected packet for a test case that should fail.")
				}

				// Check if the received packet matches the expected packet.
				processReceiverPacket(receivedPacket)
				if Channels != tc.expectedChannels {
					t.Errorf("Received channels do not match expected channels.\nExpected: %d\nGot: %d\n", tc.expectedChannels, Channels)
				}
				t.Log("Channels:\t", Channels)
			case <-time.After(10 * time.Millisecond):
				if tc.expectedToSucceed {
					t.Fatal("Timeout: readReceiver did not produce a packet as expected.")
				} else {
					t.Log("Successfully timed out, as expected for an invalid packet.")
				}
			}
		})
	}
}
