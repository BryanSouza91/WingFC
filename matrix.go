// --- Simple Matrix Library for this example ---
package main

// Matrix represents a simple 2D float64 matrix.
type Matrix struct {
	rows, cols int
	data       []float64
}

// NewMatrix creates a new matrix of the given size.
func NewMatrix(rows, cols int) *Matrix {
	return &Matrix{
		rows: rows,
		cols: cols,
		data: make([]float64, rows*cols),
	}
}

// At returns the value at a specific row and column.
func (m *Matrix) At(r, c int) float64 {
	return m.data[r*m.cols+c]
}

// Set sets the value at a specific row and column.
func (m *Matrix) Set(r, c int, val float64) {
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
			sum := 0.0
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

// Inverse returns the inverse of a 2x2 matrix.
// This is a simplified function and will panic on non-invertible matrices.
func (m *Matrix) Inverse() *Matrix {
	if m.rows != 2 || m.cols != 2 {
		panic("Inverse function is only for 2x2 matrices in this example")
	}
	a, b := m.At(0, 0), m.At(0, 1)
	c, d := m.At(1, 0), m.At(1, 1)
	det := a*d - b*c
	if det == 0 {
		panic("Matrix is not invertible")
	}
	res := NewMatrix(2, 2)
	res.Set(0, 0, d/det)
	res.Set(0, 1, -b/det)
	res.Set(1, 0, -c/det)
	res.Set(1, 1, a/det)
	return res
}
