//go:build crsf
// +build crsf

package main

// CRSF (Crossfire) protocol receiver implementation
// Used by TBS Crossfire and ExpressLRS for RC link

// Define constants for CRSF protocol
const (
	// CRSF uses 0xC8 as the address for the flight controller sync byte
	CRSF_SYNC_BYTE = 0xC8
	CRSF_FRAMETYPE_RC_CHANNELS     = 0x16

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
	crsfState := WAITING_FOR_HEADER1
	buffer := [CRSF_PACKET_SIZE]byte{}
	packetIndex := 0
	payloadLength := 0

	for {
		data, err := uart.ReadByte()
		if err != nil {
			// If there's no data available, we can just continue.
			println("No data from CRSF RX")
			continue
		}

		switch crsfState {
		case WAITING_FOR_HEADER1:
			// The first byte is the device address (sync byte).
			if data == CRSF_SYNC_BYTE {
				buffer[0] = data
				packetIndex = 1
				crsfState = READING_LENGTH
			}
		case READING_LENGTH:
			// The second byte is the frame length (frame_size from the C header).
			// For RC channels, this value should be 24 (1 byte type + 22 bytes payload + 1 byte CRC).
			payloadLength = int(data)
			buffer[packetIndex] = data
			packetIndex++

			// Validate the length to prevent buffer overflow. A standard RC packet has a frame_size of 24.
			if payloadLength != 24 {
				crsfState = WAITING_FOR_HEADER1 // Invalid length, reset
			} else {
				crsfState = READING_TYPE_AND_PAYLOAD
			}
		case READING_TYPE_AND_PAYLOAD:
			buffer[packetIndex] = data
			packetIndex++

			// We have received the full frame length (including the type byte and checksum).
			// The total bytes received in the buffer will be 1 (sync) + 1 (length) + payloadLength.
			if packetIndex >= 2+payloadLength {
				crsfState = READING_CHECKSUM
			}
		case READING_CHECKSUM:
			buffer[packetIndex] = data

			// Validate the checksum. The CRC is calculated over the frame, starting from the type byte.
			// The frame size (payloadLength) includes the type byte and CRC.
			// So, the data for CRC calculation is from buffer[2] up to the byte before the CRC.
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
func processReceiverPacket(packet [CRSF_PACKET_SIZE]byte) {
	// The payload starts at index 3 of the packet (after address, length, and type).
	payload := packet[3:]

	// RC Channel Encoding (Packed 11-bit)
	// Up to 16 channels, each 11 bits. The payload is 22 bytes long.
	// We'll unpack the values using a bitstream approach for clarity and correctness.
	var channelValues [CRSF_NUM_CHANNELS]uint16

	// Create a single slice of bytes for easier bit manipulation.
	bitstream := payload[:]

	for i := 0; i < CRSF_NUM_CHANNELS; i++ {
		// Calculate the start bit for the current channel. Each channel is 11 bits.
		startBit := i * 11

		// Calculate the byte and bit offsets.
		byteOffset := startBit / 8
		bitOffset := startBit % 8

		// The 11 bits are spread across 2 or 3 bytes.
		// Read the first byte.
		val := uint16(bitstream[byteOffset]) >> bitOffset

		// Read the second byte and shift it to the correct position.
		val |= uint16(bitstream[byteOffset+1]) << (8 - bitOffset)

		// If we need more bits, read the third byte.
		if bitOffset > 5 {
			val |= uint16(bitstream[byteOffset+2]) << (16 - bitOffset)
		}

		// Mask to get only the 11 bits.
		channelValues[i] = val & 0x07FF
	}

	// Update the global Channels array.
	for i := 0; i < CRSF_NUM_CHANNELS; i++ {
		// As a flight controller, you typically need to convert the raw
		// CRSF value to a standard PWM pulse width in microseconds (1000-2000).
		Channels[i] = mapRange(channelValues[i], CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, MIN_RX_VALUE, MAX_RX_VALUE)
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
