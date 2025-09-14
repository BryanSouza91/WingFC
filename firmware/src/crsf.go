package main

// CRSF (Crossfire) protocol receiver implementation
// Used by TBS Crossfire and ExpressLRS for RC link
// Implements a state machine for frame parsing and channel extraction

// CRSF protocol constants
const (
	// CRSF_SYNC_BYTE: Start of frame marker
	// CRSF_MAX_PACKET_LEN: Maximum allowed packet length
	CRSF_SYNC_BYTE      = 0xC8
	CRSF_MAX_PACKET_LEN = 64
)

// CRSF State Machine States
type CRSFState int

// CRSFState: State machine for parsing CRSF frames

const (
	CRSFStateSync CRSFState = iota
	CRSFStateLen
	CRSFStateType
	CRSFStatePayload
	CRSFStateCRC
)

// CRSF Frame structure
type CRSFFrame struct {
	Type    byte   // Frame type (e.g., RC channels, telemetry)
	Payload []byte // Frame payload data
	CRC     byte   // Frame CRC for integrity check
}

// CRSF Parser/State Machine
// Similar to iBus, but for CRSF frames

type CRSFParser struct {
	state    CRSFState                 // Current state in the parser
	buffer   [CRSF_MAX_PACKET_LEN]byte // Temporary buffer for payload
	index    int                       // Current index in buffer
	frameLen int                       // Length of current frame
	frame    CRSFFrame                 // Parsed frame
}

func NewCRSFParser() *CRSFParser {
	return &CRSFParser{
		state: CRSFStateSync,
	}
}

// Feed a byte into the parser
func (p *CRSFParser) Feed(b byte) (frameReady bool, frame CRSFFrame) {
	// Feed a single byte into the CRSF state machine
	// Returns true and a valid frame when a complete, valid frame is parsed
	switch p.state {
	case CRSFStateSync:
		// Wait for sync byte to start a new frame
		if b == CRSF_SYNC_BYTE {
			p.state = CRSFStateLen
			p.index = 0
		}
	case CRSFStateLen:
		// Read frame length
		p.frameLen = int(b)
		if p.frameLen > CRSF_MAX_PACKET_LEN {
			// Invalid length, reset state machine
			p.state = CRSFStateSync
		} else {
			p.state = CRSFStateType
		}
	case CRSFStateType:
		// Read frame type
		p.frame.Type = b
		p.state = CRSFStatePayload
		p.index = 0
	case CRSFStatePayload:
		// Read payload bytes
		if p.index < p.frameLen-2 {
			p.buffer[p.index] = b
			p.index++
			if p.index == p.frameLen-2 {
				p.state = CRSFStateCRC
			}
		}
	case CRSFStateCRC:
		// Read CRC byte and validate frame
		p.frame.Payload = make([]byte, p.frameLen-2)
		copy(p.frame.Payload, p.buffer[:p.frameLen-2])
		p.frame.CRC = b
		// Calculate CRC over sync, length, type, and payload
		crc := calcCRSFCRC(append([]byte{CRSF_SYNC_BYTE, byte(p.frameLen), p.frame.Type}, p.frame.Payload...))
		frameValid := crc == p.frame.CRC
		p.state = CRSFStateSync
		if frameValid {
			// Valid frame received
			return true, p.frame
		}
		// CRC failed, discard frame
		return false, CRSFFrame{}
	}
	// Frame not complete yet
	return false, CRSFFrame{}
}

// Calculate CRSF CRC (8-bit)
func calcCRSFCRC(data []byte) byte {
	// Calculate 8-bit CRC for CRSF frame
	crc := byte(0)
	for _, b := range data {
		crc ^= b
	}
	return crc
}

// Extract channels from CRSF RC frame (Type 0x16)
func ExtractCRSFChannels(frame CRSFFrame) bool {
	// Extract RC channel data from CRSF frame (Type 0x16)
	// Returns true if extraction is successful
	if frame.Type != 0x16 {
		// Not an RC channel frame
		return false
	}
	if len(frame.Payload) < 22 {
		// Payload too short for 16 channels
		return false
	}
	for i := 0; i < NumChannels; i++ {
		// Each channel is 11 bits, packed into payload
		ch := uint16(frame.Payload[2*i]) | (uint16(frame.Payload[2*i+1]) << 8)
		Channels[i] = ch & 0x07FF // Mask to 11 bits
	}
	return true
}
