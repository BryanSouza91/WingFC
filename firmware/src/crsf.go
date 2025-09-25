//go:build crsf
// +build crsf

package main

// CRSF (Crossfire) protocol receiver implementation
// Used by TBS Crossfire and ExpressLRS for RC link

// Define constants for CRSF protocol
const (
	// CRSF uses 0xC8 as the address for the flight controller sync byte
	CRSF_FLIGHT_CONTROLLER     = 0xC8
	CRSF_FRAMETYPE_RC_CHANNELS = 0x16

	// A standard RC channels packed packet is 26 bytes long.
	// 1 (sync) + 1 (length) + 1 (type) + 22 (payload) + 1 (CRC) = 26 bytes
	CRSF_PACKET_SIZE = 26

	// We'll use the number of channels from config.go
	CRSF_NUM_CHANNELS = NumChannels

	CRSF_CHANNEL_VALUE_MIN = 172  // 987us
	CRSF_CHANNEL_VALUE_MAX = 1811 // 2012us

	// ELRS=420000 CRSF=416666 Radiomaster/ELRS=115200???
	BAUD_RATE = 420000
)

// --- CRSF Receiver Logic ---

// Create a channel to receive CRSF packets.
var packetChan = make(chan [CRSF_PACKET_SIZE]byte)

// CRSF State Machine States
type CRSFState int

const (
	DESTINATION CRSFState = iota
	TYPE
	LENGTH
	PAYLOAD
	CHECKSUM
)

// readReceiver is a goroutine that reads CRSF packets from the UART and sends them to a channel.
// This function uses a state machine to ensure a complete packet is received before being passed on.
// This function now accepts the channel as an argument, which is required for build-time selection.
func readReceiver(packetChan chan<- [CRSF_PACKET_SIZE]byte) {
	var packet [CRSF_PACKET_SIZE]byte
	var packetIndex uint8 = 0
	var state CRSFState = DESTINATION
	var length uint8

	resetState := func() {
		packet = [CRSF_PACKET_SIZE]byte{}
		packetIndex = 0
		state = DESTINATION
	}

	for {

		if uart.Buffered() <= CRSF_PACKET_SIZE {
			// wait for full packet in buffer
			// time.Sleep(time.Millisecond) // Not sure if we need to delay further if we wait for ~64 byte buffer
			continue
		}

		b, err := uart.ReadByte()
		if err != nil {
			// A non-blocking read returns a timeout error. We can simply continue.
			continue
		}

		switch state {
		case DESTINATION:

			// Wait for the destination byte.
			if b == CRSF_FLIGHT_CONTROLLER {
				packet[packetIndex] = b
				packetIndex = 1
				state = LENGTH
			}

		case LENGTH:

			length = b
			// The length byte includes the type and payload but not the destination byte.
			if length >= 2 && length <= 64 { // The minimum length is 2 (type + CRC). The maximum is 64. A standard RC packet is 24.
				packet[packetIndex] = length
				packetIndex++
				state = TYPE
			} else {
				resetState()
				state = DESTINATION // Invalid length, restart.
			}

		case TYPE:

			if b == CRSF_FRAMETYPE_RC_CHANNELS {
				packet[packetIndex] = b
				packetIndex++
				state = PAYLOAD
			} else {
				resetState()
				state = DESTINATION // Invalid frametype, restart.
			}

		case PAYLOAD:

			packet[packetIndex] = b
			packetIndex++
			if packetIndex >= length+1 {
				state = CHECKSUM
			}

		case CHECKSUM:

			// The CRC8 is calculated over the frame, from after the length
			// byte at index 2 to the end of the payload at packetIndex.
			calculatedChecksum := calculateCrc8(packet[2:packetIndex])
			// breakpoint here to check variables. specifically checksum and index and
			//  if at all possible try to capture an entire packet here as well
			if calculatedChecksum == b {
				packetChan <- packet
			} else {
				println("Checksum mismatch. Discarding packet.")
			}
			resetState()
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
