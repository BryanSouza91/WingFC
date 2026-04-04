package main

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

// ===== Matrix2x2 Tests =====

func TestNewMatrix2x2(t *testing.T) {
	m := NewMatrix2x2()

	assert.NotNil(t, m)
	assert.Equal(t, 0.0, m.At(0, 0))
	assert.Equal(t, 0.0, m.At(0, 1))
	assert.Equal(t, 0.0, m.At(1, 0))
	assert.Equal(t, 0.0, m.At(1, 1))
}

func TestMatrix2x2Set(t *testing.T) {
	tests := []struct {
		name string
		row  int
		col  int
		val  float64
	}{
		{"Set (0,0)", 0, 0, 1.5},
		{"Set (0,1)", 0, 1, 2.3},
		{"Set (1,0)", 1, 0, -0.5},
		{"Set (1,1)", 1, 1, 3.14},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			m := NewMatrix2x2()
			m.Set(test.row, test.col, test.val)

			assert.Equal(t, test.val, m.At(test.row, test.col))
		})
	}
}

func TestIdentity2x2(t *testing.T) {
	m := Identity2x2()

	assert.Equal(t, 1.0, m.At(0, 0))
	assert.Equal(t, 0.0, m.At(0, 1))
	assert.Equal(t, 0.0, m.At(1, 0))
	assert.Equal(t, 1.0, m.At(1, 1))
}

func TestMatrix2x2Add(t *testing.T) {
	tests := []struct {
		name     string
		a        [4]float64 // [0,0], [0,1], [1,0], [1,1]
		b        [4]float64
		expected [4]float64
	}{
		{
			name:     "Add identity matrices",
			a:        [4]float64{1, 0, 0, 1},
			b:        [4]float64{1, 0, 0, 1},
			expected: [4]float64{2, 0, 0, 2},
		},
		{
			name:     "Add zeros",
			a:        [4]float64{1, 2, 3, 4},
			b:        [4]float64{0, 0, 0, 0},
			expected: [4]float64{1, 2, 3, 4},
		},
		{
			name:     "Add negative values",
			a:        [4]float64{1, 2, 3, 4},
			b:        [4]float64{-1, -2, -3, -4},
			expected: [4]float64{0, 0, 0, 0},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			m1 := NewMatrix2x2()
			m2 := NewMatrix2x2()

			m1.Set(0, 0, test.a[0])
			m1.Set(0, 1, test.a[1])
			m1.Set(1, 0, test.a[2])
			m1.Set(1, 1, test.a[3])

			m2.Set(0, 0, test.b[0])
			m2.Set(0, 1, test.b[1])
			m2.Set(1, 0, test.b[2])
			m2.Set(1, 1, test.b[3])

			result := m1.Add(m2)

			assert.Equal(t, test.expected[0], result.At(0, 0))
			assert.Equal(t, test.expected[1], result.At(0, 1))
			assert.Equal(t, test.expected[2], result.At(1, 0))
			assert.Equal(t, test.expected[3], result.At(1, 1))
		})
	}
}

func TestMatrix2x2Subtract(t *testing.T) {
	tests := []struct {
		name     string
		a        [4]float64
		b        [4]float64
		expected [4]float64
	}{
		{
			name:     "Subtract identity from itself",
			a:        [4]float64{1, 0, 0, 1},
			b:        [4]float64{1, 0, 0, 1},
			expected: [4]float64{0, 0, 0, 0},
		},
		{
			name:     "Subtract zeros",
			a:        [4]float64{1, 2, 3, 4},
			b:        [4]float64{0, 0, 0, 0},
			expected: [4]float64{1, 2, 3, 4},
		},
		{
			name:     "Subtract larger from smaller",
			a:        [4]float64{1, 1, 1, 1},
			b:        [4]float64{2, 2, 2, 2},
			expected: [4]float64{-1, -1, -1, -1},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			m1 := NewMatrix2x2()
			m2 := NewMatrix2x2()

			m1.Set(0, 0, test.a[0])
			m1.Set(0, 1, test.a[1])
			m1.Set(1, 0, test.a[2])
			m1.Set(1, 1, test.a[3])

			m2.Set(0, 0, test.b[0])
			m2.Set(0, 1, test.b[1])
			m2.Set(1, 0, test.b[2])
			m2.Set(1, 1, test.b[3])

			result := m1.Subtract(m2)

			assert.Equal(t, test.expected[0], result.At(0, 0))
			assert.Equal(t, test.expected[1], result.At(0, 1))
			assert.Equal(t, test.expected[2], result.At(1, 0))
			assert.Equal(t, test.expected[3], result.At(1, 1))
		})
	}
}

func TestMatrix2x2Multiply(t *testing.T) {
	tests := []struct {
		name     string
		a        [4]float64
		b        [4]float64
		expected [4]float64
	}{
		{
			name:     "Multiply by identity",
			a:        [4]float64{1, 2, 3, 4},
			b:        [4]float64{1, 0, 0, 1},
			expected: [4]float64{1, 2, 3, 4},
		},
		{
			name:     "Multiply by zero matrix",
			a:        [4]float64{1, 2, 3, 4},
			b:        [4]float64{0, 0, 0, 0},
			expected: [4]float64{0, 0, 0, 0},
		},
		{
			name:     "Simple matrix multiplication",
			a:        [4]float64{2, 0, 0, 2},
			b:        [4]float64{1, 2, 3, 4},
			expected: [4]float64{2, 4, 6, 8},
		},
		{
			name:     "Complex multiplication",
			a:        [4]float64{1, 2, 3, 4},
			b:        [4]float64{2, 0, 1, 2},
			expected: [4]float64{4, 4, 10, 8},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			m1 := NewMatrix2x2()
			m2 := NewMatrix2x2()

			m1.Set(0, 0, test.a[0])
			m1.Set(0, 1, test.a[1])
			m1.Set(1, 0, test.a[2])
			m1.Set(1, 1, test.a[3])

			m2.Set(0, 0, test.b[0])
			m2.Set(0, 1, test.b[1])
			m2.Set(1, 0, test.b[2])
			m2.Set(1, 1, test.b[3])

			result := m1.Multiply(m2)

			assert.Equal(t, test.expected[0], result.At(0, 0))
			assert.Equal(t, test.expected[1], result.At(0, 1))
			assert.Equal(t, test.expected[2], result.At(1, 0))
			assert.Equal(t, test.expected[3], result.At(1, 1))
		})
	}
}

func TestMatrix2x2Transpose(t *testing.T) {
	tests := []struct {
		name     string
		input    [4]float64
		expected [4]float64
	}{
		{
			name:     "Transpose identity",
			input:    [4]float64{1, 0, 0, 1},
			expected: [4]float64{1, 0, 0, 1},
		},
		{
			name:     "Transpose non-symmetric matrix",
			input:    [4]float64{1, 2, 3, 4},
			expected: [4]float64{1, 3, 2, 4},
		},
		{
			name:     "Transpose symmetric matrix",
			input:    [4]float64{1, 2, 2, 1},
			expected: [4]float64{1, 2, 2, 1},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			m := NewMatrix2x2()
			m.Set(0, 0, test.input[0])
			m.Set(0, 1, test.input[1])
			m.Set(1, 0, test.input[2])
			m.Set(1, 1, test.input[3])

			result := m.Transpose()

			assert.Equal(t, test.expected[0], result.At(0, 0))
			assert.Equal(t, test.expected[1], result.At(0, 1))
			assert.Equal(t, test.expected[2], result.At(1, 0))
			assert.Equal(t, test.expected[3], result.At(1, 1))
		})
	}
}

func TestMatrix2x2Inverse(t *testing.T) {
	tests := []struct {
		name     string
		input    [4]float64
		expected [4]float64
	}{
		{
			name:     "Inverse of identity",
			input:    [4]float64{1, 0, 0, 1},
			expected: [4]float64{1, 0, 0, 1},
		},
		{
			name:     "Inverse of diagonal matrix",
			input:    [4]float64{2, 0, 0, 4},
			expected: [4]float64{0.5, 0, 0, 0.25},
		},
		{
			name:     "Inverse of 2x2 matrix",
			input:    [4]float64{1, 2, 3, 4},
			expected: [4]float64{-2, 1, 1.5, -0.5},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			m := NewMatrix2x2()
			m.Set(0, 0, test.input[0])
			m.Set(0, 1, test.input[1])
			m.Set(1, 0, test.input[2])
			m.Set(1, 1, test.input[3])

			result := m.Inverse()

			tolerance := 0.0001
			assert.True(t, FloatEqual(result.At(0, 0), test.expected[0], tolerance))
			assert.True(t, FloatEqual(result.At(0, 1), test.expected[1], tolerance))
			assert.True(t, FloatEqual(result.At(1, 0), test.expected[2], tolerance))
			assert.True(t, FloatEqual(result.At(1, 1), test.expected[3], tolerance))
		})
	}
}

func TestMatrix2x2InverseSingular(t *testing.T) {
	// Singular matrix (determinant = 0)
	m := NewMatrix2x2()
	m.Set(0, 0, 1)
	m.Set(0, 1, 2)
	m.Set(1, 0, 2)
	m.Set(1, 1, 4)

	assert.Panics(t, func() {
		m.Inverse()
	}, "Expected panic for singular matrix")
}

func TestMatrix2x2Multiply2x1(t *testing.T) {
	tests := []struct {
		name     string
		matrix   [4]float64
		vector   [2]float64
		expected [2]float64
	}{
		{
			name:     "Multiply identity by vector",
			matrix:   [4]float64{1, 0, 0, 1},
			vector:   [2]float64{3, 4},
			expected: [2]float64{3, 4},
		},
		{
			name:     "Multiply scaling matrix by vector",
			matrix:   [4]float64{2, 0, 0, 3},
			vector:   [2]float64{1, 1},
			expected: [2]float64{2, 3},
		},
		{
			name:     "Multiply general matrix by vector",
			matrix:   [4]float64{1, 2, 3, 4},
			vector:   [2]float64{1, 2},
			expected: [2]float64{5, 11},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			m := NewMatrix2x2()
			m.Set(0, 0, test.matrix[0])
			m.Set(0, 1, test.matrix[1])
			m.Set(1, 0, test.matrix[2])
			m.Set(1, 1, test.matrix[3])

			v := NewMatrix2x1()
			v.Set(0, 0, test.vector[0])
			v.Set(1, 0, test.vector[1])

			result := m.Multiply2x1(v)

			assert.Equal(t, test.expected[0], result.At(0, 0))
			assert.Equal(t, test.expected[1], result.At(1, 0))
		})
	}
}

// ===== Matrix2x1 Tests =====

func TestNewMatrix2x1(t *testing.T) {
	m := NewMatrix2x1()

	assert.NotNil(t, m)
	assert.Equal(t, 0.0, m.At(0, 0))
	assert.Equal(t, 0.0, m.At(1, 0))
}

func TestMatrix2x1Set(t *testing.T) {
	m := NewMatrix2x1()
	m.Set(0, 0, 1.5)
	m.Set(1, 0, 2.3)

	assert.Equal(t, 1.5, m.At(0, 0))
	assert.Equal(t, 2.3, m.At(1, 0))
}

func TestMatrix2x1SetInvalidColumn(t *testing.T) {
	m := NewMatrix2x1()

	assert.Panics(t, func() {
		m.Set(0, 1, 5.0)
	}, "Expected panic for invalid column index")
}

func TestMatrix2x1Add(t *testing.T) {
	tests := []struct {
		name           string
		a              [2]float64
		b              [2]float64
		expectedResult [2]float64
	}{
		{
			name:           "Add zeros",
			a:              [2]float64{1, 2},
			b:              [2]float64{0, 0},
			expectedResult: [2]float64{1, 2},
		},
		{
			name:           "Add equal vectors",
			a:              [2]float64{1, 2},
			b:              [2]float64{1, 2},
			expectedResult: [2]float64{2, 4},
		},
		{
			name:           "Add positive and negative",
			a:              [2]float64{5, 3},
			b:              [2]float64{-2, -3},
			expectedResult: [2]float64{3, 0},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			m1 := NewMatrix2x1()
			m1.Set(0, 0, test.a[0])
			m1.Set(1, 0, test.a[1])

			m2 := NewMatrix2x1()
			m2.Set(0, 0, test.b[0])
			m2.Set(1, 0, test.b[1])

			result := m1.Add(m2)

			assert.Equal(t, test.expectedResult[0], result.At(0, 0))
			assert.Equal(t, test.expectedResult[1], result.At(1, 0))
		})
	}
}

func TestMatrix2x1Subtract(t *testing.T) {
	tests := []struct {
		name           string
		a              [2]float64
		b              [2]float64
		expectedResult [2]float64
	}{
		{
			name:           "Subtract zeros",
			a:              [2]float64{1, 2},
			b:              [2]float64{0, 0},
			expectedResult: [2]float64{1, 2},
		},
		{
			name:           "Subtract equal vectors",
			a:              [2]float64{1, 2},
			b:              [2]float64{1, 2},
			expectedResult: [2]float64{0, 0},
		},
		{
			name:           "Subtract larger from smaller",
			a:              [2]float64{2, 1},
			b:              [2]float64{5, 3},
			expectedResult: [2]float64{-3, -2},
		},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			m1 := NewMatrix2x1()
			m1.Set(0, 0, test.a[0])
			m1.Set(1, 0, test.a[1])

			m2 := NewMatrix2x1()
			m2.Set(0, 0, test.b[0])
			m2.Set(1, 0, test.b[1])

			result := m1.Subtract(m2)

			assert.Equal(t, test.expectedResult[0], result.At(0, 0))
			assert.Equal(t, test.expectedResult[1], result.At(1, 0))
		})
	}
}
