package main

import (
	"machine"
)

// Supported receiver protocols
const (
	PROTOCOL_IBUS = iota
	PROTOCOL_CRSF
	PROTOCOL_ELRS
)

// CRSF/ELRS parser instance
var crsfParser = crsf.NewCRSFParser()

// HandleReceiverInput parses receiver data for the selected protocol
// Updates channels.Channels with latest values
func HandleReceiverInput() bool {
	switch activeProtocol {
	case PROTOCOL_IBUS:
		ParseIBus()
		// iBus updates Channels directly
		return true
	case PROTOCOL_CRSF:
		// Read bytes from UART and feed to CRSF parser
		for {
			b, err := uart.ReadByte()
			if err != nil {
				break
			}
			frameReady, frame := crsfParser.Feed(b)
			if frameReady {
				ExtractCRSFChannels(frame)
				return true
			}
		}
	case PROTOCOL_ELRS:
		// ELRS uses CRSF protocol
		for {
			b, err := uart.ReadByte()
			if err != nil {
				break
			}
			frameReady, frame := crsfParser.Feed(b)
			if frameReady {
				ExtractELRSChannels(frame)
				return true
			}
		}
	}
	return false
}
