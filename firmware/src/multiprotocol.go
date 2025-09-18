package main

import (
	"time"
)

// Supported protocols
const (
	PROTOCOL_IBUS = iota
	PROTOCOL_CRSF
	PROTOCOL_ELRS
)

var (
	// Global parser instances and state
	crsfParser     = NewCRSFParser()
	iBusParser     = NewIBusParser()
	elrsParser     = NewELRSParser()
	lastPacketTime time.Time

	// A channel to signal to the main loop that a new packet is ready
	// This buffered channel prevents the sender (goroutine) from blocking
	// if the receiver (main loop) is not ready.
	packetReady = make(chan struct{}, 1)
)

// HandleReceiverInput is a polling function that reads bytes from the UART
// and feeds them to the active protocol parser. This function should be
// called continuously from the main loop.
func HandleReceiverInput() {
	for {
		// Read a single byte from the UART.
		// This call is non-blocking on most TinyGo targets. If no data is
		// available, it will return an error immediately, allowing the
		// main loop to continue.
		b, err := uart.ReadByte()
		if err != nil {
			// No data available, so we sleep for a short time to yield.
			time.Sleep(1 * time.Millisecond)
			continue
		}

		// A byte was successfully read, now feed it to the correct parser.
		var frameReady bool
		switch activeProtocol {
		case PROTOCOL_IBUS:
			// println("Received byte:", b)
			frameReady = iBusParser.Feed(b)
		case PROTOCOL_CRSF:
			frameReady, err = crsfParser.Feed(b)
			if err != nil {
				// Handle parsing error if needed
				continue
			}
		case PROTOCOL_ELRS:
			//frameReady, _ = elrsParser.Feed(b)
		}

		if frameReady {
			lastPacketTime = time.Now()
			// Send a signal to the main loop that a new packet is ready.
			// Use a select with a default case to ensure this send is non-blocking.
			select {
			case packetReady <- struct{}{}:
			default:
				// Channel is full, meaning the main loop is still processing the last packet.
				// We can simply drop this signal without blocking.
			}
		}
	}
}
