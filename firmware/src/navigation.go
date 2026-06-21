package main

import math "github.com/orsinium-labs/tinymath"

const (
	// Earth's radius in meters (WGS84)
	EarthRadiusMeters = 6371000.0

	// Navigation constants for Return-to-Home
	RTHMinAltitude       int32   = 10  // Minimum safe altitude for RTH (meters)
	RTHClimbRate         int32   = 1   // Climb rate in m/s
	RTHDescentRate       float32 = 0.5 // Descent rate in m/s m/s
	RTHAltitudeDeadzone  int32   = 1   // Altitude error deadzone (meters)
	RTHDistanceThreshold float32 = 5.0 // Distance to home threshold for RTH completion (meters)
)

// NavigationState tracks the current navigation state for return-to-home.
type NavigationState struct {
	RTHActive          bool
	RTHTargetLat       float32
	RTHTargetLon       float32
	RTHHoldAltitude    int32
	RTHDesiredHeading  float32 // Bearing to home (0-360 degrees)
	RTHDistanceToHome  float32 // Distance to home in meters
	DistanceThreshold  float32 // Feet
	SignalLostTime     int64   // Time signal was lost (milliseconds)
	DescentStartHeight float32
}

// CalculateBearing calculates the bearing (direction) from one coordinate to another.
// Returns bearing in degrees (0-360), where:
// 0°   = North
// 90°  = East
// 180° = South
// 270° = West
func CalculateBearing(fromLat, fromLon, toLat, toLon float32) float32 {
	const degreesToRadians float32 = math.Pi / 180.0
	const radiansToDegrees float32 = 180.0 / math.Pi

	lat1 := fromLat * degreesToRadians
	lat2 := toLat * degreesToRadians
	deltaLon := (toLon - fromLon) * degreesToRadians

	y := math.Sin(deltaLon) * math.Cos(lat2)
	x := math.Cos(lat1)*math.Sin(lat2) - math.Sin(lat1)*math.Cos(lat2)*math.Cos(deltaLon)

	bearing := math.Atan2(y, x) * radiansToDegrees

	// Normalize to 0-360 range
	if bearing < 0 {
		bearing += 360.0
	}

	return bearing
}

// CalculateDistance calculates the distance between two geographic coordinates using Haversine formula.
// Returns distance in meters.
func CalculateDistance(lat1, lon1, lat2, lon2 float32) float32 {
	const degreesToRadians float32 = math.Pi / 180.0

	dlat := (lat2 - lat1) * degreesToRadians
	dlon := (lon2 - lon1) * degreesToRadians

	lat1Rad := lat1 * degreesToRadians
	lat2Rad := lat2 * degreesToRadians

	a := math.Sin(dlat/2)*math.Sin(dlat/2) +
		math.Cos(lat1Rad)*math.Cos(lat2Rad)*math.Sin(dlon/2)*math.Sin(dlon/2)

	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))
	distance := 6371000.0 * c

	return distance
}

// CalculateAltitudeError calculates the altitude correction needed.
// Returns the altitude error in meters (positive = need to climb, negative = need to descend).
func CalculateAltitudeError(currentAltitude, targetAltitude int32) int32 {
	return targetAltitude - currentAltitude
}

// CalculateHeadingError calculates the heading correction needed.
// Returns error in radians (-π to +π).
// Negative = need to turn left (CCW), Positive = need to turn right (CW).
func CalculateHeadingError(desiredHeading, currentHeading float32) float32 {
	const degreesToRadians float32 = math.Pi / 180.0

	// Convert degrees to radians
	desired := desiredHeading * degreesToRadians
	current := currentHeading * degreesToRadians

	// Calculate raw difference
	delta := desired - current

	// Normalize to [-π, π]
	for delta > math.Pi {
		delta -= 2 * math.Pi
	}
	for delta < -math.Pi {
		delta += 2 * math.Pi
	}

	return delta
}

// UpdateRTHNavigation updates navigation state for return-to-home.
// Call this during FLIGHT_MODE every control loop to calculate RTH corrections.
func UpdateRTHNavigation(navState *NavigationState, currentGPS GPSData, currentAttitude IMU, currentAltitude int32) {
	if !navState.RTHActive || !currentGPS.Valid {
		return
	}

	// Calculate distance and bearing to home
	navState.RTHDistanceToHome = CalculateDistance(
		currentGPS.Latitude, currentGPS.Longitude,
		navState.RTHTargetLat, navState.RTHTargetLon,
	)

	navState.RTHDesiredHeading = CalculateBearing(
		currentGPS.Latitude, currentGPS.Longitude,
		navState.RTHTargetLat, navState.RTHTargetLon,
	)

	// Debug output
	print("RTH: Distance=")
	print(int32(navState.RTHDistanceToHome))
	print("m, Bearing=")
	print(int32(navState.RTHDesiredHeading))
	print("°, AltError=")
	altError := CalculateAltitudeError(currentAltitude, navState.RTHHoldAltitude)
	print(altError)
	println("m")
}

// IsRTHComplete checks if the aircraft has reached home.
// Returns true if within RTHDistanceThreshold of home location.
func IsRTHComplete(navState *NavigationState) bool {
	return navState.RTHDistanceToHome <= navState.DistanceThreshold
}

// NewNavigationState creates a new navigation state with sensible defaults.
func NewNavigationState() *NavigationState {
	return &NavigationState{
		RTHActive:         false,
		DistanceThreshold: RTHDistanceThreshold,
		SignalLostTime:    0,
	}
}
