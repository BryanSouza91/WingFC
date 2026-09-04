package main

// --- Airframe Configuration ---
const (
	AIRFRAME_ELEVON = iota
	AIRFRAME_SINGLE_AILERON_T_TAIL
	AIRFRAME_DUAL_AILERON_T_TAIL
	AIRFRAME_SINGLE_AILERON_V_TAIL
	AIRFRAME_DUAL_AILERON_V_TAIL
)

// ApplyMixer takes the desired pitch, roll, and yaw outputs (from PID or manual input)
// and mixes them into specific servo channels based on the chosen airframe type.
// The output scale matches the input scale (e.g., expected -MAX_RATE to +MAX_RATE).
func ApplyMixer(pitchOutput, rollOutput, yawOutput float32) (servo1, servo2, servo4, servo5, servo6 float32) {
	// Initialize outputs to neutral/zero
	servo1, servo2, servo4, servo5, servo6 = 0, 0, 0, 0, 0

	switch AIRCRAFT_TYPE {
	case AIRFRAME_ELEVON:
		// Left Elevon = 0.5*Roll + 0.5*Pitch
		servo1 = 0.5*rollOutput + 0.5*pitchOutput
		// Right Elevon = -0.5*Roll + 0.5*Pitch
		servo2 = -0.5*rollOutput + 0.5*pitchOutput
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
		servo1 = rollOutput                    // Aileron
		servo2 = 0.5*pitchOutput + 0.5*yawOutput // Right V-Tail surface
		servo4 = 0.5*pitchOutput - 0.5*yawOutput // Left V-Tail surface

	case AIRFRAME_DUAL_AILERON_V_TAIL:
		servo1 = rollOutput                    // Left Aileron
		servo5 = -rollOutput                   // Right Aileron
		servo2 = 0.5*pitchOutput + 0.5*yawOutput // Right V-Tail surface
		servo4 = 0.5*pitchOutput - 0.5*yawOutput // Left V-Tail surface
	}

	return servo1, servo2, servo4, servo5, servo6
}
