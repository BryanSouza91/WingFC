// tuning.go - On-the-fly PID parameter tuning via RC channels

package main

// Tuning Configuration
// Set TuneParameter to 0 to disable tuning, or 1-6 to enable
const (
	// Parameter selection (0 = disabled)
	// 1: Pitch P, 2: Roll P
	// 3: Pitch I, 4: Roll I
	// 5: Pitch D, 6: Roll D
	TuneParameterA = 0 // Disabled by default
	TuneParameterB = 0

	// RC channels used for tuning (typically aux channels)
	TuningChannelA = 4 // CH5 (aux channel)
	TuningChannelB = 5 // CH6 (aux channel)

	// Parameter ranges (adjusted via stick position 988-2012)
	TuneParameterAmin = 0.1
	TuneParameterAmax = 2.0

	TuneParameterBmin = 0.01
	TuneParameterBmax = 0.5
)

// UpdateTuning adjusts PID gains and other parameters based on RC input.
// Call this once per control loop when tuning is enabled.
//
// Usage:
//   1. Set TuneParameterA and TuneParameterB to desired parameter IDs (1-6)
//   2. Assign TuningChannelA/B to RC input channels
//   3. Call UpdateTuning() in main control loop
//   4. Adjust transmitter potentiometers to tune in real-time
//
// Parameter IDs:
//   1: Pitch P (Kp)      3: Pitch I (Ki)      5: Pitch D (Kd)
//   2: Roll P (Kp)       4: Roll I (Ki)       6: Roll D (Kd)
func UpdateTuning() {
	// Tune Parameter A via Channel A
	if TuneParameterA > 0 && TuneParameterA <= 6 {
		value := mapRange(float64(Channels[TuningChannelA]), MIN_RX_VALUE, MAX_RX_VALUE,
			TuneParameterAmin, TuneParameterAmax)

		switch TuneParameterA {
		case 1: // Pitch P
			pitchPID.Kp = value
		case 2: // Roll P
			rollPID.Kp = value
		case 3: // Pitch I
			pitchPID.Ki = value
		case 4: // Roll I
			rollPID.Ki = value
		case 5: // Pitch D
			pitchPID.Kd = value
		case 6: // Roll D
			rollPID.Kd = value
		}
	}

	// Tune Parameter B via Channel B
	if TuneParameterB > 0 && TuneParameterB <= 6 {
		value := mapRange(float64(Channels[TuningChannelB]), MIN_RX_VALUE, MAX_RX_VALUE,
			TuneParameterBmin, TuneParameterBmax)

		switch TuneParameterB {
		case 1: // Pitch P
			pitchPID.Kp = value
		case 2: // Roll P
			rollPID.Kp = value
		case 3: // Pitch I
			pitchPID.Ki = value
		case 4: // Roll I
			rollPID.Ki = value
		case 5: // Pitch D
			pitchPID.Kd = value
		case 6: // Roll D
			rollPID.Kd = value
		}
	}
}

// PrintTuningStatus outputs current gain values to serial for monitoring.
// Call this periodically (e.g., every 40 loops) when actively tuning.
func PrintTuningStatus() {
	println("[TUNING STATUS]")
	println("Pitch PID - Kp:", pitchPID.Kp, "Ki:", pitchPID.Ki, "Kd:", pitchPID.Kd)
	println("Roll PID  - Kp:", rollPID.Kp, "Ki:", rollPID.Ki, "Kd:", rollPID.Kd)
}
