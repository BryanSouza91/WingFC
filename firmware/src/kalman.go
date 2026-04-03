package main

// KalmanFilter represents a multivariate Kalman Filter optimized for flight control.
// State vector X: [pitch, roll]
// Measurement vector Z: [pitch_accel, roll_accel]
// Uses Matrix2x2 and Matrix2x1 for performance in the control loop.
type KalmanFilter struct {
	// State Vector
	X *Matrix2x1 // (2x1) estimated state vector [pitch, roll]

	// Covariance Matrices
	P *Matrix2x2 // (2x2) Estimate error covariance
	Q *Matrix2x2 // (2x2) Process noise covariance
	R *Matrix2x2 // (2x2) Measurement noise covariance

	// System Matrices
	F *Matrix2x2 // (2x2) State transition matrix
	H *Matrix2x2 // (2x2) Observation matrix

	dt float64 // Time step
}

// NewKalmanFilter creates and initializes a new optimized KalmanFilter.
func NewKalmanFilter(dt float64) *KalmanFilter {
	// State vector: [pitch, roll] (2x1)
	x := NewMatrix2x1()

	// Process Noise Covariance (Q): We trust our gyroscope, so Q is small.
	q := Identity2x2()
	q.Set(0, 0, 0.01) // Pitch noise
	q.Set(1, 1, 0.01) // Roll noise

	// Measurement Noise Covariance (R): Accel is noisy.
	r := Identity2x2()
	r.Set(0, 0, 0.5) // Pitch noise
	r.Set(1, 1, 0.5) // Roll noise

	// State Transition Matrix (F)
	f := Identity2x2()

	// Observation Matrix (H): We observe pitch and roll directly
	h := Identity2x2()

	return &KalmanFilter{
		X:  x,
		P:  Identity2x2(), // P starts as an identity matrix
		Q:  q,
		R:  r,
		F:  f,
		H:  h,
		dt: dt,
	}
}

// Predict updates the state and covariance using the control inputs from the gyroscope.
func (kf *KalmanFilter) Predict(gyroX, gyroY float64) {
	// Update the state transition matrix F.
	// F is identity matrix in this model: x_new = x_old + gyro_rates * dt
	kf.F.Set(0, 0, 1.0) // Pitch
	kf.F.Set(0, 1, 0.0)
	kf.F.Set(1, 0, 0.0) // Roll
	kf.F.Set(1, 1, 1.0)

	// Use gyro rates to predict the next state
	// x_pred = x_prev + gyro_rates * dt
	gyroVector := NewMatrix2x1()
	gyroVector.Set(0, 0, gyroY*kf.dt)
	gyroVector.Set(1, 0, gyroX*kf.dt)
	kf.X = kf.X.Add(gyroVector)

	// Predict the next covariance
	// P_pred = F * P_prev * F^T + Q
	fT := kf.F.Transpose()
	fP := kf.F.Multiply(kf.P)
	fPfT := fP.Multiply(fT)
	kf.P = fPfT.Add(kf.Q)
}

// Update corrects the state and covariance with a new measurement from the accelerometer.
func (kf *KalmanFilter) Update(accelPitch, accelRoll float64) {
	// Measurement vector Z
	z := NewMatrix2x1()
	z.Set(0, 0, accelPitch)
	z.Set(1, 0, accelRoll)

	// Innovation y = z - H * x_pred
	Hx := kf.H.Multiply2x1(kf.X)
	y := z.Subtract(Hx)

	// Innovation covariance S = H * P_pred * H^T + R
	hT := kf.H.Transpose()
	HP := kf.H.Multiply(kf.P)
	HPHt := HP.Multiply(hT)
	S := HPHt.Add(kf.R)

	// Kalman gain K = P_pred * H^T * S^-1
	Sinv := S.Inverse()
	PH := kf.P.Multiply(hT)
	K := PH.Multiply(Sinv)

	// Updated state estimate x = x_pred + K * y
	Ky := K.Multiply2x1(y)
	kf.X = kf.X.Add(Ky)

	// Updated estimate covariance P = (I - K * H) * P_pred
	I := Identity2x2()
	KH := K.Multiply(kf.H)
	IKH := I.Subtract(KH)
	kf.P = IKH.Multiply(kf.P)
}
