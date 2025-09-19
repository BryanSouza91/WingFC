//go:build crsf
// +build crsf

package main

// CRSF (Crossfire) protocol receiver implementation
// Used by TBS Crossfire and ExpressLRS for RC link

// Define constants for CRSF protocol
const (
	CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
	CRSF_FRAMETYPE_RC_CHANNELS     = 0x16

	// We'll use the number of channels from config.go
	CRSF_NUM_CHANNELS = NumChannels

	CRSF_PACKET_SIZE = 24 // A standard RC channels packed packet is 24 bytes long
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
	crsfState := WAITING_FOR_HEADER1
	buffer := [CRSF_PACKET_SIZE]byte{}
	packetIndex := 0
	payloadLength := 0
	
	for {
		data, err := uart.ReadByte()
		if err != nil {
			// If there's no data available, we can just continue.
			continue
		}

		switch crsfState {
		case WAITING_FOR_HEADER1:
			// The first byte is the device address.
			if data == CRSF_ADDRESS_FLIGHT_CONTROLLER {
				buffer[0] = data
				packetIndex = 1
				crsfState = READING_LENGTH
			}
		case READING_LENGTH:
			// The second byte is the frame length.
			payloadLength = int(data)
			buffer[packetIndex] = data
			packetIndex++

			// The packet size is frame length + 2 (address and length bytes).
			// We validate the length here to prevent buffer overflow.
			if payloadLength > (CRSF_PACKET_SIZE-2) {
				crsfState = WAITING_FOR_HEADER1 // Invalid length, reset
			} else {
				crsfState = READING_TYPE_AND_PAYLOAD
			}
		case READING_TYPE_AND_PAYLOAD:
			buffer[packetIndex] = data
			packetIndex++

			// We have received the full frame length (including the type byte and checksum).
			// The CRSF packet length byte includes the frame type and payload, but not the sync/address byte or length byte itself.
			// The total packet length will be the payload length + 2 (sync + length bytes).
			if packetIndex >= payloadLength + 2 {
				crsfState = READING_CHECKSUM
			}
		case READING_CHECKSUM:
			buffer[packetIndex] = data
			
			// Validate the checksum
			calculatedChecksum := calculateCrc8(buffer[2:packetIndex])
			if calculatedChecksum == data {
				// Send the complete packet to the channel.
				packetChan <- buffer
			}

			// Reset the state machine for the next packet.
			crsfState = WAITING_FOR_HEADER1
			packetIndex = 0
		}
	}
}

// processReceiverPacket takes a CRSF packet and extracts the RC channel values.
// This function is called by the main loop.
func processReceiverPacket(packet [CRSF_PACKET_SIZE]byte) {
	// A CRSF RC Channels packet contains 16 channels packed into 22 bytes.
	// Each channel is an 11-bit value.
	// The payload starts at index 3 of the packet (after address, length, and type).
	payload := packet[3:]

	// RC Channel Encoding (Packed 11-bit)
	// Up to 16 channels, each 11 bits.
	// We'll use a local array to store the unpacked values.
	var channelValues [CRSF_NUM_CHANNELS]uint16

	// Loop through the 22-byte payload to unpack the 11-bit channel values.
	for i := 0; i < 16; i++ {
		firstByteIndex := i + (i / 8)
		secondByteIndex := i + 1 + (i / 8)
		thirdByteIndex := i + 2 + (i / 8)

		var value uint16
		// Channels are little-endian.
		if i%2 == 0 { // Even channel (0, 2, 4...)
			// Bits from byte 1 (least significant) and bits 0-2 from byte 2
			value = uint16(payload[firstByteIndex]) | (uint16(payload[secondByteIndex]) << 8)
		} else { // Odd channel (1, 3, 5...)
			// Bits from byte 2 (bits 3-7) and all of byte 3
			value = (uint16(payload[secondByteIndex]) >> 3) | (uint16(payload[thirdByteIndex]) << 5)
		}

		// Mask to get only the 11 bits.
		channelValues[i] = value & 0x07FF
	}

	// Update the global Channels array.
	for i := 0; i < CRSF_NUM_CHANNELS; i++ {
		Channels[i] = channelValues[i]
	}
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