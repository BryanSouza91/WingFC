//go:build ibus
// +build ibus

package main

// iBus multi-protocol receiver implementation
// Supports FS-A8S (18 channels) and uses shared Channels array for integration with CRSF/ELRS

// Define constants for iBus protocol
const (
	IBUS_HEADER1 = 0x20
	IBUS_HEADER2 = 0x40
	// We'll use the number of channels from config.go
	IBUS_NUM_CHANNELS = NumChannels
	// Header (2) + Channels (18 * 2) + Checksum (2)
	IBUS_PACKET_SIZE = 2 + (IBUS_NUM_CHANNELS * 2) + 2

	BAUD_RATE = 115200
)

// Create a channel to receive iBus packets.
var packetChan = make(chan [IBUS_PACKET_SIZE]byte)

// iBus State Machine States
type IBusState int

const (
	WAITING_FOR_HEADER1 IBusState = iota
	WAITING_FOR_HEADER2
	READING_PAYLOAD
	READING_CHECKSUM_LOW
	READING_CHECKSUM_HIGH
)

// readIBus is a goroutine that reads iBus packets from the UART and sends them to a channel.
// This function uses a state machine to ensure a complete packet is received before
// being sent over the channel.
func readReceiver(packetChan chan<- [IBUS_PACKET_SIZE]byte) {
	ibusState := WAITING_FOR_HEADER1
	payloadBuffer := [IBUS_PACKET_SIZE]byte{}
	payloadIndex := 0

	for {
		data, err := uart.ReadByte()
		if err != nil {
			// If there's no data available, we can just continue.
			// The goroutine will not block here.
			continue
		}

		switch ibusState {
		case WAITING_FOR_HEADER1:
			if data == IBUS_HEADER1 {
				ibusState = WAITING_FOR_HEADER2
			}
		case WAITING_FOR_HEADER2:
			if data == IBUS_HEADER2 {
				payloadIndex = 0
				ibusState = READING_PAYLOAD
			} else {
				ibusState = WAITING_FOR_HEADER1 // Invalid header sequence, reset
			}
		case READING_PAYLOAD:
			payloadBuffer[payloadIndex] = data
			payloadIndex++
			if payloadIndex >= IBUS_PACKET_SIZE-2 {
				ibusState = READING_CHECKSUM_LOW
			}
		case READING_CHECKSUM_LOW:
			payloadBuffer[payloadIndex] = data
			payloadIndex++
			ibusState = READING_CHECKSUM_HIGH
		case READING_CHECKSUM_HIGH:
			payloadBuffer[payloadIndex] = data

			// Send the complete packet to the channel.
			// This will block until the main goroutine is ready to receive it.
			packetChan <- payloadBuffer

			// Reset the state machine for the next packet.
			ibusState = WAITING_FOR_HEADER1
			payloadIndex = 0
		}
	}
}

// Helper function to process the iBus packet and update the global Channels array.
func processReceiverPacket(packet [IBUS_PACKET_SIZE]byte) {
	// A simple checksum check can be added here
	for i := 0; i < IBUS_NUM_CHANNELS; i++ {
		Channels[i] = uint16(packet[2*i]) | uint16(packet[2*i+1])<<8
	}
}
