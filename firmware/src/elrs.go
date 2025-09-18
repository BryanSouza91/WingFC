package main

// ELRSParser is an alias for CRSFParser
type ELRSParser = CRSFParser

func NewELRSParser() *ELRSParser {
	return NewCRSFParser()
}

// Extract ELRS channels (uses CRSF extraction)
func ExtractELRSChannels(frame CRSFFrame) bool {
	// The CRSF and ELRS channel packing is identical.
	// We call the public CRSF decoding function.
	DecodeCRSFChannels(frame.Payload)
	return true
}
