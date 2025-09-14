package main

// iBus multi-protocol receiver implementation
// Supports FS-A8S (18 channels) and uses shared Channels array for integration with CRSF/ELRS

import (
	"machine"
	"time"
)

// Define constants for iBus protocol
const (
	IBUS_HEADER1      = 0x20
	IBUS_HEADER2      = 0x40
	IBUS_NUM_CHANNELS = NumChannels                     // FS-A8S supports 18 channels
	IBUS_PACKET_SIZE  = 2 + (IBUS_NUM_CHANNELS * 2) + 2 // Header (2) + Channels (18 * 2) + Checksum (2)
)

// State machine states for iBus parsing
type IBusState int

const (
	WAITING_FOR_HEADER1 IBusState = iota
	WAITING_FOR_HEADER2
	READING_PAYLOAD
	READING_CHECKSUM_LOW
	READING_CHECKSUM_HIGH
)

var (

	// Channel variables removed; use channels.Channels instead
	lastPacketTime time.Time

	ibusState     = WAITING_FOR_HEADER1
	payloadBuffer [IBUS_PACKET_SIZE]byte
	payloadIndex  int
	checksum      uint16

	uart = machine.DefaultUART
)

func ParseIBus() {
	// Read byte from UART
	data, err := uart.ReadByte()
	if err != nil {
		// UART read error, skip this cycle
		return
	}
	switch ibusState {
	case WAITING_FOR_HEADER1:
		// Wait for first header byte
		if data == IBUS_HEADER1 {
			ibusState = WAITING_FOR_HEADER2
			lastPacketTime = time.Now()
		}
	case WAITING_FOR_HEADER2:
		// Wait for second header byte
		if data == IBUS_HEADER2 {
			ibusState = READING_PAYLOAD
			payloadIndex = 0
			checksum = 0xFFFF - uint16(IBUS_HEADER1) - uint16(IBUS_HEADER2)
		} else {
			// Unexpected header, reset state machine
			ibusState = WAITING_FOR_HEADER1
		}
	case READING_PAYLOAD:
		// Read channel payload bytes
		if payloadIndex < IBUS_NUM_CHANNELS*2 {
			payloadBuffer[payloadIndex] = data
			checksum -= uint16(data)
			payloadIndex++
		} else {
			// All payload bytes read, move to checksum
			ibusState = READING_CHECKSUM_LOW
			payloadBuffer[payloadIndex] = data
			payloadIndex++
		}
	case READING_CHECKSUM_LOW:
		// Read checksum low byte
		payloadBuffer[payloadIndex] = data
		ibusState = READING_CHECKSUM_HIGH
		payloadIndex++
	case READING_CHECKSUM_HIGH:
		// Read checksum high byte and validate packet
		payloadBuffer[payloadIndex] = data
		receivedChecksum := uint16(payloadBuffer[IBUS_PACKET_SIZE-2]) | uint16(payloadBuffer[IBUS_PACKET_SIZE-1])<<8

		// Check for valid packet size before processing
		if payloadIndex != IBUS_PACKET_SIZE-1 {
			// Unexpected packet size, reset state
			ibusState = WAITING_FOR_HEADER1
			return
		}

		// If checksum is valid, process channels
		if receivedChecksum == checksum {
			// Extract channels (18 channels, 2 bytes each)
			for i := 0; i < IBUS_NUM_CHANNELS; i++ {
				Channels[i] = uint16(payloadBuffer[2*i]) | uint16(payloadBuffer[2*i+1])<<8
			}
			lastPacketTime = time.Now()
		} else {
			// Checksum mismatch, discard packet
			// Optionally log error here
		}
		// Reset state for next packet
		ibusState = WAITING_FOR_HEADER1
	}
}
