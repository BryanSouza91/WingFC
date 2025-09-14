package main

// ELRSParser is an alias for CRSFParser
type ELRSParser = CRSFParser

func NewELRSParser() *ELRSParser {
	return NewCRSFParser()
}

// Extract ELRS channels (uses CRSF extraction)
func ExtractELRSChannels(frame CRSFFrame) bool {
	return ExtractCRSFChannels(frame)
}
