package main

const (
	GPSBaudRate = 9600
)

type GPSData struct {
	Latitude   float64
	Longitude  float64
	Altitude   float64
	Satellites int
	Fix        int
	Speed      float64
	Course     float64
	Time       string
}
