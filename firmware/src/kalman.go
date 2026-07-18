package main

// KalmanFilter represents a multivariate Kalman Filter upgraded for 3-Axis control.
// State vector X: [pitch, roll, yaw]
// Measurement vector Z: [pitch_accel, roll_accel]
//
// All matrix fields are value types (see matrix.go). This keeps Predict/Update
// allocation-free: they run every control-loop tick, so heap-allocating a new
// matrix on every arithmetic operation would create continuous GC pressure.
type KalmanFilter struct {
	X  Matrix3x1 // (3x1) estimated state vector [pitch, roll, yaw]
	P  Matrix3x3 // (3x3) Estimate error covariance
	Q  Matrix3x3 // (3x3) Process noise covariance
	R  Matrix2x2 // (2x2) Measurement noise covariance (Accel provides Pitch/Roll only)
	F  Matrix3x3 // (3x3) State transition matrix
	H  Matrix2x3 // (2x3) Observation matrix (Corrected: 2 rows, 3 cols)
	dt float32
}

func NewKalmanFilter(dt float32) *KalmanFilter {
	x := NewMatrix3x1()

	q := Identity3x3()
	q.Set(0, 0, 0.01) // Pitch noise
	q.Set(1, 1, 0.01) // Roll noise
	q.Set(2, 2, 0.01) // Yaw noise (Gyro drift)

	r := Identity2x2()
	r.Set(0, 0, 0.5) // Pitch Accel noise
	r.Set(1, 1, 0.5) // Roll Accel noise

	return &KalmanFilter{
		X: x,
		P: Identity3x3(),
		Q: q,
		R: r,
		F: Identity3x3(),
		// H maps [pitch, roll, yaw] -> [accelPitch, accelRoll]
		// Pitch is observed at index 0, Roll at index 1.
		H:  NewMatrix2x3(),
		dt: dt,
	}
}

func (kf *KalmanFilter) Predict(gyroX, gyroY, gyroZ float32) {
	// Predict next state using all 3 gyro axes
	// gyroVector units: radians
	gyroVector := NewMatrix3x1()
	gyroVector.Set(0, 0, gyroY*kf.dt) // Pitch (typically Y axis)
	gyroVector.Set(1, 0, gyroX*kf.dt) // Roll (typically X axis)
	gyroVector.Set(2, 0, gyroZ*kf.dt) // Yaw (typically Z axis)

	kf.X = kf.X.Add(gyroVector)

	// Predict next covariance: P = F * P * F^T + Q
	// With F as Identity, this is P = P + Q
	kf.P = kf.P.Add(kf.Q)
}

func (kf *KalmanFilter) Update(accelPitch, accelRoll float32) {
	// Measurement update uses only Accel Pitch/Roll.
	z := NewMatrix2x1()
	z.Set(0, 0, accelPitch)
	z.Set(1, 0, accelRoll)

	// Innovation: y = z - Hx
	Hx := kf.H.Multiply3x1(kf.X) // Result is 2x1
	y := z.Subtract(Hx)

	// S = HPHt + R
	hT := kf.H.Transpose()       // Result is 3x2
	HP := kf.H.Multiply3x3(kf.P) // Result is 2x3
	HPHt := HP.Multiply3x2(hT)   // Result is 2x2
	S := HPHt.Add(kf.R)

	// K = P*Ht*Sinv
	Sinv := S.Inverse()
	PHt := kf.P.Multiply3x2(hT) // Result is 3x2
	K := PHt.Multiply2x2(Sinv)  // Result is 3x2 (Kalman Gain)

	// Update state: x = x + Ky
	Ky := K.Multiply2x1(y) // Result is 3x1
	kf.X = kf.X.Add(Ky)

	// Update covariance: P = (I - KH)P
	I := Identity3x3()
	KH := K.Multiply2x3(kf.H) // Result is 3x3
	IKH := I.Subtract(KH)
	kf.P = IKH.Multiply(kf.P)
}
