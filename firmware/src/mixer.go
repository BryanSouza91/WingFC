package main

// ApplyMixer takes the desired pitch, roll, and yaw outputs (from PID or manual input)
// and mixes them into specific servo channels based on the chosen airframe type.
// The output scale matches the input scale (e.g., expected -MAX_RATE to +MAX_RATE).
func ApplyMixer(pitchOutput, rollOutput, yawOutput float32) (servo1, servo2, servo4, servo5, servo6 float32) {
	// Initialize outputs to neutral/zero
	servo1, servo2, servo4, servo5, servo6 = 0, 0, 0, 0, 0

	switch AIRCRAFT_TYPE {
	case AIRFRAME_ELEVON:
		// Left Elevon = Roll + Pitch
		servo1 = rollOutput + pitchOutput
		// Right Elevon = -Roll + Pitch
		servo2 = -rollOutput + pitchOutput
		// Yaw mapping (if rudder exists on the wing)
		servo4 = yawOutput

	case AIRFRAME_SINGLE_AILERON_T_TAIL:
		servo1 = rollOutput  // Aileron
		servo2 = pitchOutput // Elevator
		servo4 = yawOutput   // Rudder

	case AIRFRAME_DUAL_AILERON_T_TAIL:
		servo1 = rollOutput  // Left Aileron
		servo5 = -rollOutput // Right Aileron
		servo2 = pitchOutput // Elevator
		servo4 = yawOutput   // Rudder

	case AIRFRAME_SINGLE_AILERON_V_TAIL:
		servo1 = rollOutput              // Aileron
		servo2 = pitchOutput + yawOutput // Right V-Tail surface
		servo4 = pitchOutput - yawOutput // Left V-Tail surface

	case AIRFRAME_DUAL_AILERON_V_TAIL:
		servo1 = rollOutput              // Left Aileron
		servo5 = -rollOutput             // Right Aileron
		servo2 = pitchOutput + yawOutput // Right V-Tail surface
		servo4 = pitchOutput - yawOutput // Left V-Tail surface
	}

	return servo1, servo2, servo4, servo5, servo6
}
