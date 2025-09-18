package main

import (
	"sync"
	"time"
)

var (
	// Channels is a globally shared array to store the RC receiver channel data.
	// It's accessed by all protocol parsers (iBus, CRSF, ELRS) and the main loop.
	// We use a mutex to protect it from concurrent access, although in this
	// single-threaded main loop model, it's more for good practice.
	Channels [NumChannels]uint16

	// A mutex to protect the Channels array from concurrent writes.
	channelsMutex sync.Mutex

	// A channel to signal that a new packet is ready to be processed.
	// This is a buffered channel so the sender (parser) doesn't block
	// waiting for the receiver (main loop).
	PacketReady = make(chan struct{}, 1)

	// LastPacketTime stores the timestamp of the last valid receiver packet.
	LastPacketTime time.Time
)

// UpdateChannels is a safe way to write to the global Channels array.
func UpdateChannels(newChannels [NumChannels]uint16) {
	channelsMutex.Lock()
	defer channelsMutex.Unlock()
	Channels = newChannels
}

// GetChannels is a safe way to read from the global Channels array.
func GetChannels() [NumChannels]uint16 {
	channelsMutex.Lock()
	defer channelsMutex.Unlock()
	return Channels
}
