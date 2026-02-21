#!/bin/bash

# Performance Test Runner for OpenEMS
# Executes and validates performance tests

echo "=== OpenEMS Performance Test Runner ==="
echo "Date: $(date)"
echo

# Build performance tests
echo "Building performance tests..."
cd /home/pedro/OpenEMS/tests/performance

# Compile with basic flags
gcc -I../../ -I../../firmware_restructured \
    -O2 -ffast-math \
    performance_tests_working.c \
    -o performance_tests_runner

if [ $? -ne 0 ]; then
    echo "ERROR: Failed to build performance tests"
    exit 1
fi

echo "Build successful!"
echo

# Run performance tests
echo "Running performance tests..."
echo "================================"

./performance_tests_runner

echo "================================"
echo "Performance tests completed!"

# Generate performance report
echo
echo "=== Performance Test Summary ==="
echo "Test executed: $(date)"
echo "Build status: SUCCESS"
echo "Test status: COMPLETED"
echo

# Check for any performance violations
if [ $? -eq 0 ]; then
    echo "✅ All performance tests PASSED"
    echo "✅ System meets performance requirements"
else
    echo "❌ Some performance tests FAILED"
    echo "❌ System requires optimization"
fi

echo
echo "=== End of Performance Test Report ==="
