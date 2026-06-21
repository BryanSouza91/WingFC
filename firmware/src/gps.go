package main

import "time"

const (
	GPSBaudRate = 9600
)

type GPSData struct {
	Latitude       float32
	Longitude      float32
	Altitude       int32
	Satellites     int
	Fix            int // 0=invalid, 2=2D, 3=3D
	Speed          float32
	Course         float32
	Time           string
	Valid          bool      // Whether this fix is current
	HDOP           float32   // Horizontal dilution of precision (signal quality)
	LastUpdateTime time.Time // When this fix was last updated
}

// GPSConfig holds GPS-specific configuration parameters.
type GPSConfig struct {
	Enabled          bool
	BaudRate         int
	HomeLatitude     float32
	HomeLongitude    float32
	HomeAltitude     int32
	HomeNotCaptured  bool          // Set to true until first valid fix in FLIGHT_MODE
	UpdateRate       int           // Hz (typically 1-5)
	StaleDataTimeout time.Duration // Mark GPS data as invalid if no update after this time
}
