// tuning.go - On-the-fly PID parameter tuning via RC channels

package main

// UpdateTuning adjusts PID gains and other parameters based on RC input.
// Call this once per control loop when tuning is enabled.
//
// Usage:
//  1. Set TuneParameterA and TuneParameterB to desired parameter IDs (1-6)
//  2. Assign TuningChannelA/B to RC input channels
//  3. Call UpdateTuning() in main control loop
//  4. Adjust transmitter potentiometers to tune in real-time
//
// Parameter IDs:
//
//	1: Pitch P (Kp)      3: Pitch I (Ki)      5: Pitch D (Kd)
//	2: Roll P (Kp)       4: Roll I (Ki)       6: Roll D (Kd)
func UpdateTuning() {
	// Tune Parameter A via Channel A
	if TuneParameterA > 0 && TuneParameterA <= 6 {
		value := mapRange(float32(Channels[TuningChannelA]), MIN_RX_VALUE, MAX_RX_VALUE,
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
		value := mapRange(float32(Channels[TuningChannelB]), MIN_RX_VALUE, MAX_RX_VALUE,
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

	if TuningChannelC >= 0 && TuningChannelC < NumChannels {
		// Future expansion: use Channel C for additional parameters (e.g., PID weight, filter settings)
	}

	if TuningChannelD >= 0 && TuningChannelD < NumChannels {
		// Future expansion: use Channel D for additional parameters (e.g., PID weight, filter settings)
	}
}

// PrintTuningStatus outputs current gain values to serial for monitoring.
// Call this periodically (e.g., every 40 loops) when actively tuning.
func PrintTuningStatus() {
	println("[TUNING STATUS]")
	println("Pitch PID - Kp:", pitchPID.Kp, "Ki:", pitchPID.Ki, "Kd:", pitchPID.Kd)
	println("Roll PID  - Kp:", rollPID.Kp, "Ki:", rollPID.Ki, "Kd:", rollPID.Kd)
}
