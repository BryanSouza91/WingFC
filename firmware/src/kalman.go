package main

// KalmanFilter represents a multivariate Kalman Filter.
// State vector X: [pitch, roll]
// Measurement vector Z: [pitch_accel, roll_accel]
type KalmanFilter struct {
	// State Vector
	X *Matrix // (2x1) estimated state vector [pitch, roll]

	// Covariance Matrices
	P *Matrix // (2x2) Estimate error covariance
	Q *Matrix // (2x2) Process noise covariance
	R *Matrix // (2x2) Measurement noise covariance

	// System Matrices
	F *Matrix // (2x2) State transition matrix
	H *Matrix // (2x2) Observation matrix
	B *Matrix // (2x2) Control input matrix (we won't use this for a simple model, so it's a zero matrix)

	dt float64 // Time step
}

// NewKalmanFilter creates and initializes a new KalmanFilter.
func NewKalmanFilter(dt float64) *KalmanFilter {
	// State vector: [pitch, roll] (2x1)
	x := NewMatrix(2, 1)

	// Process Noise Covariance (Q): We trust our gyroscope, so Q is small.
	q := Identity(2)
	q.Set(0, 0, 0.01) // Pitch noise
	q.Set(1, 1, 0.01) // Roll noise

	// Measurement Noise Covariance (R): Accel is noisy.
	// Measurement measurements: pitch_accel, roll_accel
	r := Identity(2)
	r.Set(0, 0, 0.5) // Pitch noise
	r.Set(1, 1, 0.5) // Roll noise

	// State Transition Matrix (F)
	f := Identity(2)

	// Observation Matrix (H): We observe pitch and roll directly
	h := Identity(2)

	return &KalmanFilter{
		X:  x,
		P:  Identity(2), // P starts as an identity matrix
		Q:  q,
		R:  r,
		F:  f,
		H:  h,
		dt: dt,
	}
}

// Predict updates the state and covariance using the control inputs from the gyroscope.
func (kf *KalmanFilter) Predict(gyroX, gyroY float64) {
	// Update the state transition matrix F using gyro rates.
	// The new pitch is the old pitch + gyro_y * dt.
	// The new roll is the old roll + gyro_x * dt.
	kf.F.Set(0, 0, 1.0) // Pitch
	kf.F.Set(0, 1, 0.0)
	kf.F.Set(1, 0, 0.0) // Roll
	kf.F.Set(1, 1, 1.0)

	// Use gyro rates to predict the next state
	// x_pred = F * x_prev
	// F contains the gyro rates in this model, so we can multiply
	gyroVector := NewMatrix(2, 1)
	gyroVector.Set(0, 0, gyroY*kf.dt)
	gyroVector.Set(1, 0, gyroX*kf.dt)
	kf.X = kf.X.Add(gyroVector)

	// Predict the next covariance
	// P_pred = F * P_prev * F^T + Q
	fT := kf.F.Transpose()
	kf.P = kf.F.Multiply(kf.P).Multiply(fT).Add(kf.Q)
}

// Update corrects the state and covariance with a new measurement from the accelerometer.
func (kf *KalmanFilter) Update(accelPitch, accelRoll float64) {
	// Measurement vector Z
	z := NewMatrix(2, 1)
	z.Set(0, 0, accelPitch)
	z.Set(1, 0, accelRoll)

	// Innovation y = z - H * x_pred
	y := z.Subtract(kf.H.Multiply(kf.X))

	// Innovation covariance S = H * P_pred * H^T + R
	hT := kf.H.Transpose()
	S := kf.H.Multiply(kf.P).Multiply(hT).Add(kf.R)
	Sinv := S.Inverse()

	// Kalman gain K = P_pred * H^T * S^-1
	K := kf.P.Multiply(hT).Multiply(Sinv)

	// Updated state estimate x = x_pred + K * y
	kf.X = kf.X.Add(K.Multiply(y))

	// Updated estimate covariance P = (I - K * H) * P_pred
	I := Identity(2)
	kf.P = I.Subtract(K.Multiply(kf.H)).Multiply(kf.P)
}
