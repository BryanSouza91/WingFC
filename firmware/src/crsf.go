//go:build crsf
// +build crsf

package main

import "time"

// CRSF (Crossfire) protocol receiver implementation
// Used by TBS Crossfire and ExpressLRS for RC link

// Define constants for CRSF protocol
const (
	// CRSF uses 0xC8 as the address for the flight controller sync byte
	CRSF_SYNC_BYTE             = 0xC8
	CRSF_FRAMETYPE_RC_CHANNELS = 0x16

	// A standard RC channels packed packet is 26 bytes long.
	// 1 (sync) + 1 (length) + 1 (type) + 22 (payload) + 1 (CRC) = 26 bytes
	CRSF_PACKET_SIZE = 26

	// We'll use the number of channels from config.go
	CRSF_NUM_CHANNELS = NumChannels

	CRSF_CHANNEL_VALUE_MIN = 172  // 987us
	CRSF_CHANNEL_VALUE_MAX = 1811 // 2012us

	BAUD_RATE = 420000
)

// Create a channel to receive CRSF packets.
var packetChan = make(chan [CRSF_PACKET_SIZE]byte)

// CRSF State Machine States
type CRSFState int

const (
	WAITING_FOR_HEADER1 CRSFState = iota
	READING_LENGTH
	READING_TYPE_AND_PAYLOAD
	READING_CHECKSUM
)

// readReceiver is a goroutine that reads CRSF packets from the UART and sends them to a channel.
// This function uses a state machine to ensure a complete packet is received before
// being sent over the channel.
func readReceiver(packetChan chan<- [CRSF_PACKET_SIZE]byte) {
	var crsfState CRSFState
	var packetIndex int
	var buffer [CRSF_PACKET_SIZE]byte
	var packetLength int

	for {
		// Use a non-blocking read pattern
		// Wondering if this is the root of the problem...
		// iBus has a packet cycle time around 7ms 
		// I imagine CRSF has a packet cycle time around 2ms
		// This could cause the default case in the main loop from never being reached
		if uart.Buffered() == 0 {
			time.Sleep(1 * time.Millisecond) // Yield to other goroutines
			continue
		}

		data, err := uart.ReadByte()
		if err != nil {
			// If there's no data available, we can just continue
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
			if packetIndex >= packetLength+1 { 
				// We've read all payload bytes, next is checksum
				crsfState = READING_CHECKSUM
			}
		case READING_CHECKSUM:
			// The checksum byte is the last byte in the packet, at index 25.
			// Do not increment packetIndex after processing.
			buffer[packetIndex] = data

			// The CRC8 is calculated over the frame, from after the length 
			// byte at index 2 to the end of the payload at packetIndex.
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

// calculateCrc8 computes the CRC8 checksum for a CRSF packet.
// The CRC8 algorithm for CRSF is a specific implementation of CRC8-DVB-S2.
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
