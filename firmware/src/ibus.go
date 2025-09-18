package main

import (
	"encoding/binary"
)

// IBus protocol constants
const (
	IBUS_HEADER1 = 0x20
	IBUS_HEADER2 = 0x40
	// The number of channels is now defined in config.go and shared via channels.go
	IBUS_PACKET_SIZE = 2 + (NumChannels * 2) + 2 // Header (2) + Channels (18 * 2) + Checksum (2)
)

// IBus State Machine States
type IBusState int

const (
	IBusStateHeader1 IBusState = iota
	IBusStateHeader2
	IBusStatePayload
	IBusStateChecksum1
	IBusStateChecksum2
)

// IBus Frame structure
type IBusFrame struct {
	Payload  [IBUS_PACKET_SIZE - 4]byte // Header and checksum bytes are not included
	Checksum uint16
}

// IBus Parser/State Machine
type IBusParser struct {
	state  IBusState
	buffer [IBUS_PACKET_SIZE]byte
	index  int
	frame  IBusFrame
}

func NewIBusParser() *IBusParser {
	return &IBusParser{
		state: IBusStateHeader1,
	}
}

// Feed a byte into the parser
func (p *IBusParser) Feed(b byte) bool {
	var frameReady bool
	switch p.state {
	case IBusStateHeader1:
		if b == IBUS_HEADER1 {
			p.state = IBusStateHeader2
		} else {
			// Reset on incorrect header byte
			p.state = IBusStateHeader1
		}
	case IBusStateHeader2:
		if b == IBUS_HEADER2 {
			p.state = IBusStatePayload
			p.index = 0
		} else {
			// Reset on incorrect header byte
			p.state = IBusStateHeader1
		}
	case IBusStatePayload:
		p.buffer[p.index] = b
		p.index++
		if p.index == len(p.frame.Payload) {
			p.state = IBusStateChecksum1
			p.index = 0
		}
	case IBusStateChecksum1:
		p.frame.Checksum = uint16(b)
		p.state = IBusStateChecksum2
	case IBusStateChecksum2:
		p.frame.Checksum |= uint16(b) << 8
		p.state = IBusStateHeader1 // Reset for next frame

		// Validate frame checksum
		var calculatedChecksum uint16
		for i := 0; i < len(p.frame.Payload); i += 2 {
			calculatedChecksum += uint16(binary.LittleEndian.Uint16(p.buffer[i : i+2]))
		}

		// The iBus protocol uses a bitwise XOR of the calculated checksum with the magic number
		if calculatedChecksum == p.frame.Checksum {
			// Extract channels and store them in the global Channels array
			var newChannels [NumChannels]uint16
			for i := 0; i < NumChannels; i++ {
				offset := i * 2
				newChannels[i] = binary.LittleEndian.Uint16(p.buffer[offset : offset+2])
			}
			UpdateChannels(newChannels)
			frameReady = true
		}
	}
	return frameReady
}

// ParseIBus continuously reads bytes from the UART and feeds them to the IBus parser.
func ParseIBus() {
	p := NewIBusParser()
	for {
		b, err := uart.ReadByte()
		if err == nil {
			if p.Feed(b) {
				// New frame is ready, signal the main loop
				select {
				case PacketReady <- struct{}{}:
					// Sent signal
				default:
					// Channel is full, do nothing.
				}
			}
		}
	}
}