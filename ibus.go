package main

import (
	"machine"
	"time"
)

// Define constants for iBus protocol
const (
	IBUS_HEADER1     = 0x20
	IBUS_HEADER2     = 0x40
	IBUS_PACKET_SIZE = 32 // Header (2) + Channels (14 * 2) + Checksum (2)
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
	ch1            uint16
	ch2            uint16
	ch3            uint16
	ch4            uint16
	ch5            uint16
	ch6            uint16
	ch7            uint16
	ch8            uint16
	ch9            uint16
	ch10           uint16
	ch11           uint16
	ch12           uint16
	ch13           uint16
	ch14           uint16
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
		return
	}

	switch ibusState {
	case WAITING_FOR_HEADER1:
		if data == IBUS_HEADER1 {
			ibusState = WAITING_FOR_HEADER2
			lastPacketTime = time.Now()
		}
	case WAITING_FOR_HEADER2:
		if data == IBUS_HEADER2 {
			ibusState = READING_PAYLOAD
			payloadIndex = 0
			checksum = 0xFFFF - uint16(IBUS_HEADER1) - uint16(IBUS_HEADER2)
		} else {
			// If not the expected header, reset
			ibusState = WAITING_FOR_HEADER1
		}
	case READING_PAYLOAD:
		if payloadIndex < 28 {
			payloadBuffer[payloadIndex] = data
			checksum -= uint16(data)
			payloadIndex++
		} else {
			// All payload bytes read, time for checksum
			ibusState = READING_CHECKSUM_LOW
			payloadBuffer[payloadIndex] = data
			payloadIndex++
		}
	case READING_CHECKSUM_LOW:
		payloadBuffer[payloadIndex] = data
		ibusState = READING_CHECKSUM_HIGH
		payloadIndex++
	case READING_CHECKSUM_HIGH:
		payloadBuffer[payloadIndex] = data
		// Checksum high byte received, validate packet
		receivedChecksum := uint16(payloadBuffer[30]) | uint16(payloadBuffer[31])<<8

		// If checksum is valid, process channels
		if receivedChecksum == checksum {
			ch1 = uint16(payloadBuffer[0]) | uint16(payloadBuffer[1])<<8
			ch2 = uint16(payloadBuffer[2]) | uint16(payloadBuffer[3])<<8
			ch3 = uint16(payloadBuffer[4]) | uint16(payloadBuffer[5])<<8
			ch4 = uint16(payloadBuffer[6]) | uint16(payloadBuffer[7])<<8
			ch5 = uint16(payloadBuffer[8]) | uint16(payloadBuffer[9])<<8
			ch6 = uint16(payloadBuffer[10]) | uint16(payloadBuffer[11])<<8
			ch7 = uint16(payloadBuffer[12]) | uint16(payloadBuffer[13])<<8
			ch8 = uint16(payloadBuffer[14]) | uint16(payloadBuffer[15])<<8
			ch9 = uint16(payloadBuffer[16]) | uint16(payloadBuffer[17])<<8
			ch10 = uint16(payloadBuffer[18]) | uint16(payloadBuffer[19])<<8
			ch11 = uint16(payloadBuffer[20]) | uint16(payloadBuffer[21])<<8
			ch12 = uint16(payloadBuffer[22]) | uint16(payloadBuffer[23])<<8
			ch13 = uint16(payloadBuffer[24]) | uint16(payloadBuffer[25])<<8
			ch14 = uint16(payloadBuffer[26]) | uint16(payloadBuffer[27])<<8
			lastPacketTime = time.Now()
		}
		// Reset state for next packet
		ibusState = WAITING_FOR_HEADER1
	}
}
