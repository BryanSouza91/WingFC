package main

// --- Simple Matrix Library for the Kalman Filter Library ---

// Matrix represents a simple 2D float32 matrix.
type Matrix struct {
	rows, cols int
	data       []float32
}

// NewMatrix creates a new matrix of the given size.
func NewMatrix(rows, cols int) *Matrix {
	return &Matrix{
		rows: rows,
		cols: cols,
		data: make([]float32, rows*cols),
	}
}

// At returns the value at a specific row and column.
func (m *Matrix) At(r, c int) float32 {
	return m.data[r*m.cols+c]
}

// Set sets the value at a specific row and column.
func (m *Matrix) Set(r, c int, val float32) {
	m.data[r*m.cols+c] = val
}

// Identity returns a new identity matrix of size n x n.
func Identity(n int) *Matrix {
	m := NewMatrix(n, n)
	for i := 0; i < n; i++ {
		m.Set(i, i, 1.0)
	}
	return m
}

// Add returns the sum of two matrices.
func (m *Matrix) Add(other *Matrix) *Matrix {
	res := NewMatrix(m.rows, m.cols)
	for i := 0; i < len(m.data); i++ {
		res.data[i] = m.data[i] + other.data[i]
	}
	return res
}

// Subtract returns the difference of two matrices.
func (m *Matrix) Subtract(other *Matrix) *Matrix {
	res := NewMatrix(m.rows, m.cols)
	for i := 0; i < len(m.data); i++ {
		res.data[i] = m.data[i] - other.data[i]
	}
	return res
}

// Multiply returns the product of two matrices.
func (m *Matrix) Multiply(other *Matrix) *Matrix {
	if m.cols != other.rows {
		panic("Matrix dimensions for multiplication are incompatible")
	}
	res := NewMatrix(m.rows, other.cols)
	for i := 0; i < m.rows; i++ {
		for j := 0; j < other.cols; j++ {
			var sum float32 = 0.0
			for k := 0; k < m.cols; k++ {
				sum += m.At(i, k) * other.At(k, j)
			}
			res.Set(i, j, sum)
		}
	}
	return res
}

// Transpose returns the transpose of the matrix.
func (m *Matrix) Transpose() *Matrix {
	res := NewMatrix(m.cols, m.rows)
	for i := 0; i < m.rows; i++ {
		for j := 0; j < m.cols; j++ {
			res.Set(j, i, m.At(i, j))
		}
	}
	return res
}

// Inverse returns the inverse of a 2x2 or 3x3 matrix.
func (m *Matrix) Inverse() *Matrix {
	if m.rows == 2 && m.cols == 2 {
		return m.inverse2x2()
	} else if m.rows == 3 && m.cols == 3 {
		return m.inverse3x3()
	}
	panic("Inverse is only implemented for 2x2 and 3x3 matrices")
}

// inverse2x2 returns the inverse of a 2x2 matrix.
func (m *Matrix) inverse2x2() *Matrix {
	a := m.At(0, 0)
	b := m.At(0, 1)
	c := m.At(1, 0)
	d := m.At(1, 1)

	det := a*d - b*c
	if det == 0 {
		panic("Matrix is singular and cannot be inverted")
	}

	invDet := 1.0 / det

	res := NewMatrix(2, 2)
	res.Set(0, 0, d*invDet)
	res.Set(0, 1, -b*invDet)
	res.Set(1, 0, -c*invDet)
	res.Set(1, 1, a*invDet)

	return res
}

// inverse3x3 returns the inverse of a 3x3 matrix using the adjugate method.
func (m *Matrix) inverse3x3() *Matrix {
	// Calculate the determinant using the rule of Sarrus
	det := m.At(0, 0)*(m.At(1, 1)*m.At(2, 2)-m.At(1, 2)*m.At(2, 1)) -
		m.At(0, 1)*(m.At(1, 0)*m.At(2, 2)-m.At(1, 2)*m.At(2, 0)) +
		m.At(0, 2)*(m.At(1, 0)*m.At(2, 1)-m.At(1, 1)*m.At(2, 0))

	if det == 0 {
		panic("Matrix is singular and cannot be inverted")
	}

	// Calculate the matrix of minors (adjugate matrix)
	res := NewMatrix(3, 3)

	res.Set(0, 0, (m.At(1, 1)*m.At(2, 2)-m.At(1, 2)*m.At(2, 1))/det)
	res.Set(0, 1, -(m.At(0, 1)*m.At(2, 2)-m.At(0, 2)*m.At(2, 1))/det)
	res.Set(0, 2, (m.At(0, 1)*m.At(1, 2)-m.At(0, 2)*m.At(1, 1))/det)

	res.Set(1, 0, -(m.At(1, 0)*m.At(2, 2)-m.At(1, 2)*m.At(2, 0))/det)
	res.Set(1, 1, (m.At(0, 0)*m.At(2, 2)-m.At(0, 2)*m.At(2, 0))/det)
	res.Set(1, 2, -(m.At(0, 0)*m.At(1, 2)-m.At(0, 2)*m.At(1, 0))/det)

	res.Set(2, 0, (m.At(1, 0)*m.At(2, 1)-m.At(1, 1)*m.At(2, 0))/det)
	res.Set(2, 1, -(m.At(0, 0)*m.At(2, 1)-m.At(0, 1)*m.At(2, 0))/det)
	res.Set(2, 2, (m.At(0, 0)*m.At(1, 1)-m.At(0, 1)*m.At(1, 0))/det)

	return res
}

// --- Optimized 2x2 Matrix for Kalman Filter ---
//
// NOTE: Matrix2x2, Matrix2x1, Matrix3x3, Matrix3x2, Matrix3x1 and Matrix2x3
// below are all value types (not pointer types). They are used in the
// Kalman filter's Predict/Update hot path, which runs every control-loop
// tick. Returning them by value (instead of `*MatrixNxM`) lets TinyGo keep
// them on the stack/in registers instead of heap-allocating a new object on
// every arithmetic operation. Only Set uses a pointer receiver, since it
// needs to mutate the caller's copy; this does not allocate because the
// receiver's address never escapes the call.

// Matrix2x2 represents a specialized 2x2 matrix for Kalman filter operations.
// Uses fixed array storage for better cache locality and zero heap allocation.
type Matrix2x2 struct {
	data [4]float32 // Row-major order: [0,0], [0,1], [1,0], [1,1]
}

// NewMatrix2x2 creates a new zero-initialized 2x2 matrix.
func NewMatrix2x2() Matrix2x2 {
	return Matrix2x2{}
}

// At returns the value at a specific row and column.
func (m Matrix2x2) At(r, c int) float32 {
	return m.data[r*2+c]
}

// Set sets the value at a specific row and column.
func (m *Matrix2x2) Set(r, c int, val float32) {
	m.data[r*2+c] = val
}

// Identity2x2 returns a new 2x2 identity matrix.
func Identity2x2() Matrix2x2 {
	m := NewMatrix2x2()
	m.Set(0, 0, 1.0)
	m.Set(1, 1, 1.0)
	return m
}

// Add2x2 returns the sum of two 2x2 matrices.
func (m Matrix2x2) Add(other Matrix2x2) Matrix2x2 {
	var res Matrix2x2
	for i := 0; i < 4; i++ {
		res.data[i] = m.data[i] + other.data[i]
	}
	return res
}

// Subtract2x2 returns the difference of two 2x2 matrices.
func (m Matrix2x2) Subtract(other Matrix2x2) Matrix2x2 {
	var res Matrix2x2
	for i := 0; i < 4; i++ {
		res.data[i] = m.data[i] - other.data[i]
	}
	return res
}

// Multiply2x2 returns the product of two 2x2 matrices.
func (m Matrix2x2) Multiply(other Matrix2x2) Matrix2x2 {
	var res Matrix2x2
	res.Set(0, 0, m.At(0, 0)*other.At(0, 0)+m.At(0, 1)*other.At(1, 0))
	res.Set(0, 1, m.At(0, 0)*other.At(0, 1)+m.At(0, 1)*other.At(1, 1))
	res.Set(1, 0, m.At(1, 0)*other.At(0, 0)+m.At(1, 1)*other.At(1, 0))
	res.Set(1, 1, m.At(1, 0)*other.At(0, 1)+m.At(1, 1)*other.At(1, 1))
	return res
}

// Transpose2x2 returns the transpose of the 2x2 matrix.
func (m Matrix2x2) Transpose() Matrix2x2 {
	var res Matrix2x2
	res.Set(0, 0, m.At(0, 0))
	res.Set(0, 1, m.At(1, 0))
	res.Set(1, 0, m.At(0, 1))
	res.Set(1, 1, m.At(1, 1))
	return res
}

// Inverse2x2 returns the inverse of this 2x2 matrix.
func (m Matrix2x2) Inverse() Matrix2x2 {
	det := m.data[0]*m.data[3] - m.data[1]*m.data[2]
	if det == 0 {
		panic("Matrix2x2 is singular and cannot be inverted")
	}

	var res Matrix2x2
	invDet := 1.0 / det
	res.data[0] = m.data[3] * invDet
	res.data[1] = -m.data[1] * invDet
	res.data[2] = -m.data[2] * invDet
	res.data[3] = m.data[0] * invDet
	return res
}

// MultiplyVector2x2 multiplies this 2x2 matrix by a 2-element vector and returns the result.
func (m Matrix2x2) MultiplyVector(v [2]float32) [2]float32 {
	return [2]float32{
		m.data[0]*v[0] + m.data[1]*v[1],
		m.data[2]*v[0] + m.data[3]*v[1],
	}
}

// Multiply2x1 returns the product of this 2x2 matrix by a 2x1 vector (column matrix).
func (m Matrix2x2) Multiply2x1(v Matrix2x1) Matrix2x1 {
	var res Matrix2x1
	res.Set(0, 0, m.At(0, 0)*v.At(0, 0)+m.At(0, 1)*v.At(1, 0))
	res.Set(1, 0, m.At(1, 0)*v.At(0, 0)+m.At(1, 1)*v.At(1, 0))
	return res
}

// --- Optimized 2x1 Matrix (Vector) for Kalman Filter ---

// Matrix2x1 represents a specialized 2x1 matrix (column vector) for Kalman filter state vectors.
type Matrix2x1 struct {
	data [2]float32 // [row0, row1]
}

// NewMatrix2x1 creates a new zero-initialized 2x1 matrix.
func NewMatrix2x1() Matrix2x1 {
	return Matrix2x1{}
}

// At returns the value at a specific row.
func (m Matrix2x1) At(r, c int) float32 {
	if c != 0 {
		panic("Column index for 2x1 matrix must be 0")
	}
	return m.data[r]
}

// Set sets the value at a specific row.
func (m *Matrix2x1) Set(r, c int, val float32) {
	if c != 0 {
		panic("Column index for 2x1 matrix must be 0")
	}
	m.data[r] = val
}

// Add2x1 returns the sum of two 2x1 matrices.
func (m Matrix2x1) Add(other Matrix2x1) Matrix2x1 {
	var res Matrix2x1
	for i := 0; i < 2; i++ {
		res.data[i] = m.data[i] + other.data[i]
	}
	return res
}

// Subtract2x1 returns the difference of two 2x1 matrices.
func (m Matrix2x1) Subtract(other Matrix2x1) Matrix2x1 {
	var res Matrix2x1
	for i := 0; i < 2; i++ {
		res.data[i] = m.data[i] - other.data[i]
	}
	return res
}

// --- Optimized 3x3 Matrix for Flight Control ---

// Matrix3x3 represents a specialized 3x3 matrix for performance-critical operations.
// Uses fixed array storage instead of slices for better cache locality and speed.
type Matrix3x3 struct {
	data [9]float32 // Row-major order: [0,0], [0,1], [0,2], [1,0], [1,1], [1,2], [2,0], [2,1], [2,2]
}

// NewMatrix3x3 creates a new zero-initialized 3x3 matrix.
func NewMatrix3x3() Matrix3x3 {
	return Matrix3x3{}
}

// At returns the value at a specific row and column.
func (m Matrix3x3) At(r, c int) float32 {
	return m.data[r*3+c]
}

// Set sets the value at a specific row and column.
func (m *Matrix3x3) Set(r, c int, val float32) {
	m.data[r*3+c] = val
}

// Identity3x3 returns a new 3x3 identity matrix.
func Identity3x3() Matrix3x3 {
	m := NewMatrix3x3()
	m.Set(0, 0, 1.0)
	m.Set(1, 1, 1.0)
	m.Set(2, 2, 1.0)
	return m
}

// Add3x3 returns the sum of two 3x3 matrices.
func (m Matrix3x3) Add(other Matrix3x3) Matrix3x3 {
	var res Matrix3x3
	for i := 0; i < 9; i++ {
		res.data[i] = m.data[i] + other.data[i]
	}
	return res
}

// Subtract3x3 returns the difference of two 3x3 matrices.
func (m Matrix3x3) Subtract(other Matrix3x3) Matrix3x3 {
	var res Matrix3x3
	for i := 0; i < 9; i++ {
		res.data[i] = m.data[i] - other.data[i]
	}
	return res
}

// Multiply3x3 returns the product of two 3x3 matrices.
func (m Matrix3x3) Multiply(other Matrix3x3) Matrix3x3 {
	var res Matrix3x3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			var sum float32 = 0.0
			for k := 0; k < 3; k++ {
				sum += m.At(i, k) * other.At(k, j)
			}
			res.Set(i, j, sum)
		}
	}
	return res
}

// Transpose3x3 returns the transpose of the 3x3 matrix.
func (m Matrix3x3) Transpose() Matrix3x3 {
	var res Matrix3x3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			res.Set(j, i, m.At(i, j))
		}
	}
	return res
}

// Inverse3x3 returns the inverse of this 3x3 matrix using the adjugate method.
func (m Matrix3x3) Inverse() Matrix3x3 {
	// Calculate the determinant using cofactor expansion on first row
	det := m.data[0]*(m.data[4]*m.data[8]-m.data[5]*m.data[7]) -
		m.data[1]*(m.data[3]*m.data[8]-m.data[5]*m.data[6]) +
		m.data[2]*(m.data[3]*m.data[7]-m.data[4]*m.data[6])

	if det == 0 {
		panic("Matrix3x3 is singular and cannot be inverted")
	}

	var res Matrix3x3
	invDet := 1.0 / det

	// Calculate adjugate matrix and divide by determinant
	res.data[0] = (m.data[4]*m.data[8] - m.data[5]*m.data[7]) * invDet
	res.data[1] = -(m.data[1]*m.data[8] - m.data[2]*m.data[7]) * invDet
	res.data[2] = (m.data[1]*m.data[5] - m.data[2]*m.data[4]) * invDet

	res.data[3] = -(m.data[3]*m.data[8] - m.data[5]*m.data[6]) * invDet
	res.data[4] = (m.data[0]*m.data[8] - m.data[2]*m.data[6]) * invDet
	res.data[5] = -(m.data[0]*m.data[5] - m.data[2]*m.data[3]) * invDet

	res.data[6] = (m.data[3]*m.data[7] - m.data[4]*m.data[6]) * invDet
	res.data[7] = -(m.data[0]*m.data[7] - m.data[1]*m.data[6]) * invDet
	res.data[8] = (m.data[0]*m.data[4] - m.data[1]*m.data[3]) * invDet

	return res
}

// Multiply3x2 returns the product of this 3x3 matrix by a 3x2 matrix.
func (m Matrix3x3) Multiply3x2(other Matrix3x2) Matrix3x2 {
	var res Matrix3x2
	for i := 0; i < 3; i++ {
		for j := 0; j < 2; j++ {
			res.Set(i, j, m.At(i, 0)*other.At(0, j)+m.At(i, 1)*other.At(1, j)+m.At(i, 2)*other.At(2, j))
		}
	}
	return res
}

// MultiplyVector multiplies this 3x3 matrix by a 3-element vector and returns the result.
// This is optimized for vector transformation commonly needed in flight control.
func (m Matrix3x3) MultiplyVector(v [3]float32) [3]float32 {
	return [3]float32{
		m.data[0]*v[0] + m.data[1]*v[1] + m.data[2]*v[2],
		m.data[3]*v[0] + m.data[4]*v[1] + m.data[5]*v[2],
		m.data[6]*v[0] + m.data[7]*v[1] + m.data[8]*v[2],
	}
}

// --- Matrix3x2 ---

// Matrix3x2 represents a specialized 3x2 matrix for Kalman filter observation matrices.
type Matrix3x2 struct {
	data [6]float32 // Row-major order: [0,0], [0,1], [1,0], [1,1], [2,0], [2,1]
}

// NewMatrix3x2 creates a new zero-initialized 3x2 matrix.
func NewMatrix3x2() Matrix3x2 {
	return Matrix3x2{}
}

// At returns the value at a specific row and column.
func (m Matrix3x2) At(r, c int) float32 {
	return m.data[r*2+c]
}

// Set sets the value at a specific row and column.
func (m *Matrix3x2) Set(r, c int, val float32) {
	m.data[r*2+c] = val
}

// Multiply2x2 returns the product of this 3x2 matrix by a 2x2 matrix.
func (m Matrix3x2) Multiply2x2(other Matrix2x2) Matrix3x2 {
	var res Matrix3x2
	for i := 0; i < 3; i++ {
		for j := 0; j < 2; j++ {
			res.Set(i, j, m.At(i, 0)*other.At(0, j)+m.At(i, 1)*other.At(1, j))
		}
	}
	return res
}

// Multiply2x1 returns the product of this 3x2 matrix by a 2x1 matrix.
func (m Matrix3x2) Multiply2x1(other Matrix2x1) Matrix3x1 {
	var res Matrix3x1
	res.Set(0, 0, m.At(0, 0)*other.At(0, 0)+m.At(0, 1)*other.At(1, 0))
	res.Set(1, 0, m.At(1, 0)*other.At(0, 0)+m.At(1, 1)*other.At(1, 0))
	res.Set(2, 0, m.At(2, 0)*other.At(0, 0)+m.At(2, 1)*other.At(1, 0))
	return res
}

// Multiply2x3 returns the product of this 3x2 matrix by a 2x3 matrix.
func (m Matrix3x2) Multiply2x3(other Matrix2x3) Matrix3x3 {
	var res Matrix3x3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			res.Set(i, j, m.At(i, 0)*other.At(0, j)+m.At(i, 1)*other.At(1, j))
		}
	}
	return res
}

// --- Matrix3x1 ---

// Matrix3x1 represents a specialized 3x1 matrix (column vector) for Kalman filter state vectors.
type Matrix3x1 struct {
	data [3]float32 // [row0, row1, row2]
}

// NewMatrix3x1 creates a new zero-initialized 3x1 matrix.
func NewMatrix3x1() Matrix3x1 {
	return Matrix3x1{}
}

// At returns the value at a specific row.
func (m Matrix3x1) At(r, c int) float32 {
	if c != 0 {
		panic("Column index for 3x1 matrix must be 0")
	}
	return m.data[r]
}

// Set sets the value at a specific row.
func (m *Matrix3x1) Set(r, c int, val float32) {
	if c != 0 {
		panic("Column index for 3x1 matrix must be 0")
	}
	m.data[r] = val
}

// Add returns the sum of two 3x1 matrices.
func (m Matrix3x1) Add(other Matrix3x1) Matrix3x1 {
	var res Matrix3x1
	res.Set(0, 0, m.At(0, 0)+other.At(0, 0))
	res.Set(1, 0, m.At(1, 0)+other.At(1, 0))
	res.Set(2, 0, m.At(2, 0)+other.At(2, 0))
	return res
}

// --- Matrix2x3 (Observation Matrix) ---

// Matrix2x3 represents a specialized 2x3 matrix for Kalman filter operations.
type Matrix2x3 struct {
	data [6]float32 // Row-major order: [0,0], [0,1], [0,2], [1,0], [1,1], [1,2]
}

// NewMatrix2x3 creates a new zero-initialized 2x3 matrix.
func NewMatrix2x3() Matrix2x3 {
	return Matrix2x3{}
}

// At returns the value at a specific row and column.
func (m Matrix2x3) At(r, c int) float32 {
	return m.data[r*3+c]
}

// Set sets the value at a specific row and column.
func (m *Matrix2x3) Set(r, c int, val float32) {
	m.data[r*3+c] = val
}

// Multiply3x1 returns the product of this 2x3 matrix by a 3x1 matrix.
func (m Matrix2x3) Multiply3x1(other Matrix3x1) Matrix2x1 {
	var res Matrix2x1
	res.Set(0, 0, m.At(0, 0)*other.At(0, 0)+m.At(0, 1)*other.At(1, 0)+m.At(0, 2)*other.At(2, 0))
	res.Set(1, 0, m.At(1, 0)*other.At(0, 0)+m.At(1, 1)*other.At(1, 0)+m.At(1, 2)*other.At(2, 0))
	return res
}

// Transpose returns the transpose of the 2x3 matrix (resulting in a 3x2 matrix).
func (m Matrix2x3) Transpose() Matrix3x2 {
	var res Matrix3x2
	res.Set(0, 0, m.At(0, 0))
	res.Set(0, 1, m.At(1, 0))
	res.Set(1, 0, m.At(0, 1))
	res.Set(1, 1, m.At(1, 1))
	res.Set(2, 0, m.At(0, 2))
	res.Set(2, 1, m.At(1, 2))
	return res
}

// Multiply3x3 returns the product of this 2x3 matrix by a 3x3 matrix.
func (m Matrix2x3) Multiply3x3(other Matrix3x3) Matrix2x3 {
	var res Matrix2x3
	for i := 0; i < 2; i++ {
		for j := 0; j < 3; j++ {
			res.Set(i, j, m.At(i, 0)*other.At(0, j)+m.At(i, 1)*other.At(1, j)+m.At(i, 2)*other.At(2, j))
		}
	}
	return res
}

// Multiply3x2 returns the product of this 2x3 matrix by a 3x2 matrix.
func (m Matrix2x3) Multiply3x2(other Matrix3x2) Matrix2x2 {
	var res Matrix2x2
	for i := 0; i < 2; i++ {
		for j := 0; j < 2; j++ {
			res.Set(i, j, m.At(i, 0)*other.At(0, j)+m.At(i, 1)*other.At(1, j)+m.At(i, 2)*other.At(2, j))
		}
	}
	return res
}
