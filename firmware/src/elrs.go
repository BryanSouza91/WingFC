package main

// ELRSParser is an alias for CRSFParser
type ELRSParser = crsf.CRSFParser

func NewELRSParser() *ELRSParser {
	return crsf.NewCRSFParser()
}

// Extract ELRS channels (uses CRSF extraction)
func ExtractELRSChannels(frame crsf.CRSFFrame) bool {
	return crsf.ExtractCRSFChannels(frame)
}
