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

var (
	// Channel variables removed; use channels.Channels instead
	lastPacketTime time.Time
	uart           = machine.DefaultUART
)

func ParseIBus() {
	// Read the channel data from the UART
	// We only need to check for the first byte of the header
	data, err := uart.ReadByte()
	if err != nil {
		// If there's no data available, this will return an error
		// which is expected. We just continue the loop.
		return
	}
	// Check for the first start byte
	if data == IBUS_HEADER1 {
		// Check for the second start byte
		secondByte, err := uart.ReadByte()
		if err != nil {
			return
		}
		if secondByte == IBUS_HEADER2 {
			// Valid header found, now read the rest of the packet
			var buffer [IBUS_PACKET_SIZE - 2]byte
			bytesRead := 0
			for bytesRead < IBUS_PACKET_SIZE-2 {
				n, err := uart.Read(buffer[bytesRead:])
				if err != nil {
					//println("Error reading iBus data:", err)
					return // Exit the inner loop on error
				}
				bytesRead += n
			}

			if bytesRead == IBUS_PACKET_SIZE-2 {
				// All bytes have been read, process the data
				// Extract channels
				for i := 0; i < IBUS_NUM_CHANNELS; i++ {
					Channels[i] = uint16(buffer[2*i]) | uint16(buffer[2*i+1])<<8
				}

				// Ignoring checksum validation for simplicity
				lastPacketTime = time.Now()
			}
		}
	}
}
