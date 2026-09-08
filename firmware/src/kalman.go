package main

// KalmanFilter represents a 2-state Kalman Filter for attitude estimation.
// State vector X: [pitch, roll]
// Measurement vector Z: [pitch_accel, roll_accel]
//
// All matrix fields are value types (see matrix.go). This keeps Predict/Update
// allocation-free: they run every control-loop tick, so heap-allocating a new
// matrix on every arithmetic operation would create continuous GC pressure.
type KalmanFilter struct {
	X  Matrix2x1 // (2x1) estimated state vector [pitch, roll]
	P  Matrix2x2 // (2x2) Estimate error covariance
	Q  Matrix2x2 // (2x2) Process noise covariance
	R  Matrix2x2 // (2x2) Measurement noise covariance (Accel provides Pitch/Roll)
	dt float32
}

func NewKalmanFilter(dt float32) *KalmanFilter {
	x := NewMatrix2x1()

	q := Identity2x2()
	q.Set(0, 0, 0.01) // Pitch process noise
	q.Set(1, 1, 0.01) // Roll process noise

	r := Identity2x2()
	r.Set(0, 0, 0.5) // Pitch Accel measurement noise
	r.Set(1, 1, 0.5) // Roll Accel measurement noise

	return &KalmanFilter{
		X:  x,
		P:  Identity2x2(),
		Q:  q,
		R:  r,
		dt: dt,
	}
}

func (kf *KalmanFilter) Predict(gyroX, gyroY float32) {
	// Predict next state using pitch and roll gyros
	// gyroVector units: radians
	gyroVector := NewMatrix2x1()
	gyroVector.Set(0, 0, gyroY*kf.dt) // Pitch (typically Y axis)
	gyroVector.Set(1, 0, gyroX*kf.dt) // Roll (typically X axis)

	kf.X = kf.X.Add(gyroVector)

	// Predict next covariance: P = F * P * F^T + Q
	// With F as Identity, this is P = P + Q
	kf.P = kf.P.Add(kf.Q)
}

func (kf *KalmanFilter) Update(accelPitch, accelRoll float32) {
	// Measurement update uses Accel Pitch/Roll.
	z := NewMatrix2x1()
	z.Set(0, 0, accelPitch)
	z.Set(1, 0, accelRoll)

	// Innovation: y = z - Hx (with H = I, Hx = X)
	y := z.Subtract(kf.X)

	// Innovation covariance: S = HPH^T + R = P + R
	S := kf.P.Add(kf.R)

	// Kalman gain: K = P * H^T * S^-1 = P * S^-1
	Sinv := S.Inverse()
	K := kf.P.Multiply(Sinv) // (2x2)

	// Update state estimate: X = X + K*y
	Ky := K.Multiply2x1(y)
	kf.X = kf.X.Add(Ky)

	// Update error covariance: P = (I - K*H)P = (I - K)P
	I := Identity2x2()
	IK := I.Subtract(K)
	kf.P = IK.Multiply(kf.P)
}
