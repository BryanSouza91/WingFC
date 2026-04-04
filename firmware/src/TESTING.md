# WingFC Testing Guide

## Overview

This guide covers how to run, write, and maintain tests for the WingFC flight controller firmware.

## Test Structure

All tests are located in `src/` alongside their corresponding source files. Tests follow Go's standard naming convention: `*_test.go`.

### Test Files

| Test File | Covers |
|-----------|---------|
| `pid_test.go` | PID controller proportional, integral, and derivative terms |
| `matrix_test.go` | Matrix2x2 and Matrix2x1 operations for Kalman filter |
| `kalman_test.go` | Kalman filter predict and update cycles |
| `imu_test.go` | IMU pitch/roll calculation from accelerometer data |
| `led_test.go` | LED status controller state transitions |
| `ibus_test.go` | iBus receiver protocol packet parsing |
| `helpers_test.go` | Utility functions (constrain, mapRange, low-pass filter) |
| `hardware_adapters_test.go` | Hardware abstraction layer adapters |
| `hardware_test.go` | FC_Hardware struct and component integration |
| `tuning_test.go` | PID parameter tuning via RC channels |
| `test_helpers.go` | Mock implementations (not a test file, but test utilities) |

## Running Tests

### Run all tests
```bash
cd src/
tinygo test -v ./...
```

### Run tests with coverage
```bash
tinygo test -cover ./...
```

### Run tests for a specific module
```bash
tinygo test -v -run TestPIDController ./...
```

### Run with detailed output
```bash
tinygo test -v -run TestKalmanFilter ./...
```

## Test Patterns

### Table-Driven Tests

Most tests use table-driven test cases for comprehensive coverage:

```go
func TestModuleName(t *testing.T) {
    tests := []struct {
        name     string
        input    InputType
        expected OutputType
    }{
        {"case 1", input1, expected1},
        {"case 2", input2, expected2},
    }
    
    for _, test := range tests {
        t.Run(test.name, func(t *testing.T) {
            result := FunctionUnderTest(test.input)
            assert.Equal(t, test.expected, result)
        })
    }
}
```

### Using Assertions

This codebase uses `testify/assert` for readable assertions:

```go
assert.Equal(t, expected, actual)
assert.True(t, condition, "message if false")
assert.False(t, condition)
assert.NoError(t, err)
assert.NotNil(t, value)
```

### Float Comparisons

For floating-point comparisons, use the provided `FloatEqual()` helper:

```go
assert.True(t, FloatEqual(result, expected, tolerance))
```

## Mock Implementations

### Available Mocks

- `mockUART` - Mock UART receiver interface
- `mockPin` - Mock digital pin (High/Low)
- `mockPWM` - Mock PWM for servo/ESC control
- `mockI2C` - Mock I2C for IMU communication
- `mockLEDUpdater` - Mock LED status controller
- `mockIMUDevice` - Mock IMU sensor data

### Using Mocks

```go
// Create a mock PWM
pwm := NewMockPWM()
pwm.Configure(config)
pwm.Set(channel, value)

// Verify behavior
retrieved := pwm.GetChannelValue(channel)
assert.Equal(t, value, retrieved)
```

## Test Coverage Goals

Target coverage metrics:
- **Core algorithms** (PID, Kalman, Matrix): ≥80%
- **Protocols** (CRSF, iBus): ≥85%
- **Hardware abstraction**: ≥70%
- **Overall**: ≥70%

Check current coverage:
```bash
tinygo test -cover ./...
```

Generate coverage report:
```bash
tinygo test -coverprofile=coverage.out ./...
tinygo tool cover -html=coverage.out
```

## Writing New Tests

### Step 1: Create Test File
```bash
touch src/mymodule_test.go
```

### Step 2: Use Table-Driven Pattern
```go
package main

import (
    "testing"
    "github.com/stretchr/testify/assert"
)

func TestMyFunction(t *testing.T) {
    tests := []struct {
        name     string
        input    string
        expected string
    }{
        {"simple case", "input", "output"},
    }
    
    for _, test := range tests {
        t.Run(test.name, func(t *testing.T) {
            result := MyFunction(test.input)
            assert.Equal(t, test.expected, result)
        })
    }
}
```

### Step 3: Run and Verify
```bash
tinygo test -v -run TestMyFunction ./...
```

## Test Dependencies

### Go Standard Library
- `testing` - Built-in testing framework
- `math` - For mathematical functions and constants

### Third-Party
- `github.com/stretchr/testify` v1.8.4 - Assertion library

## Best Practices

1. **Isolation**: Each test should be independent and not rely on others
2. **Clarity**: Use descriptive test names that explain what is being tested
3. **Mocks**: Mock hardware interfaces to keep tests fast and repeatable
4. **Coverage**: Aim for >70% coverage on all modules
5. **Edge Cases**: Test boundary conditions and error scenarios
6. **Documentation**: Document non-obvious test logic with comments

## Common Test Scenarios

### Testing PID Controller
```go
pid := NewPIDController(kp, ki, kd)
output := pid.Update(error, dt)
assert.True(t, FloatEqual(output, expected, tolerance))
```

### Testing Kalman Filter
```go
kf := NewKalmanFilter(dt)
kf.Predict(gyroX, gyroY)
kf.Update(accelPitch, accelRoll)
assert.False(t, math.IsNaN(kf.X.At(0, 0)))
```

### Testing Protocol Parsing
```go
packet := [PACKET_SIZE]byte{...}
channels := processPacket(packet)
assert.Equal(t, expectedValue, channels[0])
```

## Integration Testing Considerations

Current tests focus on unit testing individual components. For future integration testing:

1. **Hardware Simulation**: Expand mocks to simulate flight conditions
2. **Sequence Testing**: Test multiple components interacting
3. **Time-Based Testing**: Test control loops at different time scales
4. **Stress Testing**: Test with extreme input values or rapid changes

## Troubleshooting

### Tests Won't Compile
Ensure `testify` is installed:
```bash
go get github.com/stretchr/testify
```

### Tests Timeout
Check for infinite loops or blocking operations in mocks.

### Float Comparison Issues
Use `FloatEqual()` with appropriate tolerance:
```go
assert.True(t, FloatEqual(a, b, 0.0001))  // 0.01% tolerance
```

### Mock State Leaking Between Tests
Create new mock instances in each test, don't reuse globals.

## CI/CD Integration

To integrate tests into GitHub Actions:

```yaml
name: Tests
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - uses: actions/setup-go@v2
      - run: cd src && go test -v ./...
```

## Performance Benchmarks

To add performance benchmarks (expected in future iterations):

```go
func BenchmarkPIDUpdate(b *testing.B) {
    pid := NewPIDController(1.0, 0.1, 0.05)
    for i := 0; i < b.N; i++ {
        pid.Update(0.5, 0.005)
    }
}
```

Run benchmarks:
```bash
tinygo test -bench=. ./...
```

## Future Improvements

- [ ] Add property-based testing with `go-quick`
- [ ] Implement fuzzing for protocol parsers
- [ ] Add performance benchmarks for control algorithms
- [ ] Expand integration tests with hardware simulation
- [ ] Set up continuous coverage tracking
