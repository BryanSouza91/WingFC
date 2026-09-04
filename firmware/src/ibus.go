//go:build ibus
// +build ibus

package main

import "time"

// iBus receiver implementation for FlySky protocol
// Standard frame: 32 bytes (2 byte header + 14 channels × 2 bytes + 2 byte checksum)

// Define constants for iBus protocol
const (
	IBUS_HEADER1      = 0x20 // Length of standard packet (32 bytes)
	IBUS_HEADER2      = 0x40 // Command byte
	IBUS_NUM_CHANNELS = 14
	IBUS_PACKET_SIZE  = 32

	BAUD_RATE = 115200
)

// Create a channel to receive iBus packets.
var packetChan = make(chan [IBUS_PACKET_SIZE]byte, 10) // Buffered channel to prevent blocking

// iBus State Machine States
type IBusState int

const (
	WAITING_FOR_HEADER1 IBusState = iota
	WAITING_FOR_HEADER2
	READING_PAYLOAD
)

// readReceiver is a goroutine that reads iBus packets from the UART and sends them to a channel.
func readReceiver(uart UART, packetChan chan<- [IBUS_PACKET_SIZE]byte) {
	if uart == nil {
		return
	}

	ibusState := WAITING_FOR_HEADER1
	payloadBuffer := [IBUS_PACKET_SIZE]byte{}
	payloadIndex := 0

	for {
		data, err := uart.Read()
		if err != nil {
			time.Sleep(250 * time.Microsecond)
			continue
		}

		switch ibusState {
		case WAITING_FOR_HEADER1:
			if data == IBUS_HEADER1 {
				payloadBuffer[0] = data
				payloadIndex = 1
				ibusState = WAITING_FOR_HEADER2
			}
		case WAITING_FOR_HEADER2:
			if data == IBUS_HEADER2 {
				payloadBuffer[1] = data
				payloadIndex = 2
				ibusState = READING_PAYLOAD
			} else {
				ibusState = WAITING_FOR_HEADER1 // Invalid header sequence, reset
			}
		case READING_PAYLOAD:
			payloadBuffer[payloadIndex] = data
			payloadIndex++
			if payloadIndex >= IBUS_PACKET_SIZE {
				// Verify 16-bit checksum: 0xFFFF - sum(bytes[0:30])
				var calculatedChecksum uint16 = 0xFFFF
				for i := 0; i < IBUS_PACKET_SIZE-2; i++ {
					calculatedChecksum -= uint16(payloadBuffer[i])
				}
				receivedChecksum := uint16(payloadBuffer[IBUS_PACKET_SIZE-2]) | (uint16(payloadBuffer[IBUS_PACKET_SIZE-1]) << 8)

				if calculatedChecksum == receivedChecksum {
					packetChan <- payloadBuffer
				}

				// Reset the state machine for the next packet.
				ibusState = WAITING_FOR_HEADER1
				payloadIndex = 0
			}
		}
	}
}

// Helper function to process the iBus packet and update the channel array.
func processReceiverPacket(packet [IBUS_PACKET_SIZE]byte) [NumChannels]uint16 {
	var channelValues [NumChannels]uint16
	for i := 0; i < IBUS_NUM_CHANNELS && i < NumChannels; i++ {
		channelValues[i] = uint16(packet[2+2*i]) | (uint16(packet[2+2*i+1]) << 8)
	}
	return channelValues
}
