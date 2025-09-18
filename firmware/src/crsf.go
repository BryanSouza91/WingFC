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
	// CRSF channel values are 11-bit, max value is 2047
	CRSF_MAX_CHANNEL_VALUE uint16 = 0x7FF
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
func (p *CRSFParser) Feed(b byte) (bool, error) {
	var frameReady bool
	switch p.state {
	case CRSFStateSync:
		if b == CRSF_SYNC_BYTE {
			p.state = CRSFStateLen
			p.index = 0
		}
	case CRSFStateLen:
		p.frameLen = int(b)
		if p.frameLen > 0 && p.frameLen <= CRSF_MAX_PACKET_LEN {
			p.buffer[p.index] = b
			p.index++
			p.state = CRSFStateType
		} else {
			p.state = CRSFStateSync
		}
	case CRSFStateType:
		p.frame.Type = b
		p.buffer[p.index] = b
		p.index++
		if p.frameLen > 2 { // Check for payload
			p.state = CRSFStatePayload
		} else {
			p.state = CRSFStateCRC
		}
	case CRSFStatePayload:
		p.buffer[p.index] = b
		p.index++
		if p.index == p.frameLen-1 { // -1 for CRC
			p.frame.Payload = p.buffer[2 : p.frameLen-1]
			p.state = CRSFStateCRC
		}
	case CRSFStateCRC:
		p.frame.CRC = b
		p.state = CRSFStateSync // Reset for next frame

		// CRC check
		if p.frame.CRC == calculateCRC8(p.buffer[:p.frameLen-1]) {
			if p.frame.Type == 0x16 { // RC channel data
				DecodeCRSFChannels(p.frame.Payload)
				frameReady = true
			}
		}
	}
	return frameReady, nil
}

// Calculate the CRC8 checksum for a given data slice
func calculateCRC8(data []byte) byte {
	var crc byte = 0
	for _, b := range data {
		crc ^= b
		for i := 0; i < 8; i++ {
			if crc&0x80 != 0 {
				crc = (crc << 1) ^ 0x07
			} else {
				crc <<= 1
			}
		}
	}
	return crc
}

// Decode 11-bit CRSF channels from the payload
func DecodeCRSFChannels(rcData []byte) {
	var newChannels [NumChannels]uint16
	// CRSF is a bit-packed protocol. We need to manually extract the 11-bit channel values.
	// This is a simplified example, a full implementation would be more robust.
	if len(rcData) < 22 {
		return
	}
	newChannels[0] = ((uint16(rcData[0])) | (uint16(rcData[1]) << 8)) & CRSF_MAX_CHANNEL_VALUE
	newChannels[1] = ((uint16(rcData[1]) >> 3) | (uint16(rcData[2]) << 5)) & CRSF_MAX_CHANNEL_VALUE
	newChannels[2] = ((uint16(rcData[2]) >> 6) | (uint16(rcData[3]) << 2) | (uint16(rcData[4]) << 10)) & CRSF_MAX_CHANNEL_VALUE
	newChannels[3] = ((uint16(rcData[4]) >> 1) | (uint16(rcData[5]) << 7)) & CRSF_MAX_CHANNEL_VALUE
	newChannels[4] = ((uint16(rcData[5]) >> 4) | (uint16(rcData[6]) << 4)) & CRSF_MAX_CHANNEL_VALUE
	newChannels[5] = ((uint16(rcData[6]) >> 7) | (uint16(rcData[7]) << 1) | (uint16(rcData[8]) << 9)) & CRSF_MAX_CHANNEL_VALUE
	newChannels[6] = ((uint16(rcData[8]) >> 2) | (uint16(rcData[9]) << 6)) & CRSF_MAX_CHANNEL_VALUE
	newChannels[7] = ((uint16(rcData[9]) >> 5) | (uint16(rcData[10]) << 3)) & CRSF_MAX_CHANNEL_VALUE
	newChannels[8] = (uint16(rcData[11]) | (uint16(rcData[12]) << 8)) & CRSF_MAX_CHANNEL_VALUE
	newChannels[9] = ((uint16(rcData[12]) >> 3) | (uint16(rcData[13]) << 5)) & CRSF_MAX_CHANNEL_VALUE
	newChannels[10] = ((uint16(rcData[13]) >> 6) | (uint16(rcData[14]) << 2) | (uint16(rcData[15]) << 10)) & CRSF_MAX_CHANNEL_VALUE
	newChannels[11] = ((uint16(rcData[15]) >> 1) | (uint16(rcData[16]) << 7)) & CRSF_MAX_CHANNEL_VALUE
	newChannels[12] = ((uint16(rcData[16]) >> 4) | (uint16(rcData[17]) << 4)) & CRSF_MAX_CHANNEL_VALUE
	newChannels[13] = ((uint16(rcData[17]) >> 7) | (uint16(rcData[18]) << 1) | (uint16(rcData[19]) << 9)) & CRSF_MAX_CHANNEL_VALUE
	newChannels[14] = ((uint16(rcData[19]) >> 2) | (uint16(rcData[20]) << 6)) & CRSF_MAX_CHANNEL_VALUE
	newChannels[15] = ((uint16(rcData[20]) >> 5) | (uint16(rcData[21]) << 3)) & CRSF_MAX_CHANNEL_VALUE

	// Update the global Channels array
	UpdateChannels(newChannels)
}
