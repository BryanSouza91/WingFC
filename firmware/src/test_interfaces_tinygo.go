//go:build tinygo
// +build tinygo

// TinyGo test-specific interface definitions
// This file is only used when compiling with TinyGo

package main

// Note: When compiling with TinyGo, use the real interfaces from interfaces.go
// But we need to ensure test mocks accept machine.* types
// Mock implementations should be updated to match machine.* signatures
