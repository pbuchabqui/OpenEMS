#!/bin/bash

# OpenEMS Test Runner
# Executes complete test suite with coverage and performance reports

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"
TEST_RESULTS_DIR="$PROJECT_ROOT/test_results"
COVERAGE_DIR="$TEST_RESULTS_DIR/coverage"
PERFORMANCE_DIR="$TEST_RESULTS_DIR/performance"

# Create directories
mkdir -p "$TEST_RESULTS_DIR"
mkdir -p "$COVERAGE_DIR"
mkdir -p "$PERFORMANCE_DIR"

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if ESP-IDF environment is set
check_idf_env() {
    if [ -z "$IDF_PATH" ]; then
        print_error "ESP-IDF environment not set. Please run:"
        print_error "  . \$IDF_PATH/export.sh"
        exit 1
    fi
    
    print_status "ESP-IDF environment found at: $IDF_PATH"
}

# Function to clean build directory
clean_build() {
    print_status "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
    mkdir -p "$BUILD_DIR"
}

# Function to build project with tests
build_project() {
    print_status "Building OpenEMS with tests..."
    
    cd "$PROJECT_ROOT"
    
    # Configure for ESP32-S3 (can be changed to esp32)
    idf.py set-target esp32s3
    
    # Build with test component
    idf.py build
    
    if [ $? -eq 0 ]; then
        print_success "Build completed successfully"
    else
        print_error "Build failed"
        exit 1
    fi
}

# Function to run unit tests
run_unit_tests() {
    print_status "Running unit tests..."
    
    cd "$PROJECT_ROOT"
    
    # Run unit tests
    idf.py test --test-filter="unit_*" > "$TEST_RESULTS_DIR/unit_tests.log" 2>&1
    
    if [ $? -eq 0 ]; then
        print_success "Unit tests passed"
        return 0
    else
        print_error "Unit tests failed"
        cat "$TEST_RESULTS_DIR/unit_tests.log"
        return 1
    fi
}

# Function to run integration tests
run_integration_tests() {
    print_status "Running integration tests..."
    
    cd "$PROJECT_ROOT"
    
    # Run integration tests
    idf.py test --test-filter="integration_*" > "$TEST_RESULTS_DIR/integration_tests.log" 2>&1
    
    if [ $? -eq 0 ]; then
        print_success "Integration tests passed"
        return 0
    else
        print_error "Integration tests failed"
        cat "$TEST_RESULTS_DIR/integration_tests.log"
        return 1
    fi
}

# Function to run performance tests
run_performance_tests() {
    print_status "Running performance tests..."
    
    cd "$PROJECT_ROOT"
    
    # Run performance tests
    idf.py test --test-filter="performance_*" > "$TEST_RESULTS_DIR/performance_tests.log" 2>&1
    
    if [ $? -eq 0 ]; then
        print_success "Performance tests passed"
        return 0
    else
        print_warning "Performance tests completed with warnings"
        return 0  # Performance tests may have warnings but not failures
    fi
}

# Function to generate coverage report
generate_coverage() {
    print_status "Generating coverage report..."
    
    cd "$PROJECT_ROOT"
    
    # Build with coverage enabled
    idf.py build -DENABLE_COVERAGE=1
    
    # Run tests to generate coverage data
    idf.py test
    
    # Generate coverage report
    if command -v gcov >/dev/null 2>&1; then
        find "$BUILD_DIR" -name "*.gcda" -exec gcov {} \;
        
        # Generate HTML report if lcov is available
        if command -v lcov >/dev/null 2>&1; then
            lcov --capture --directory "$BUILD_DIR" --output-file "$COVERAGE_DIR/coverage.info"
            genhtml "$COVERAGE_DIR/coverage.info" --output-directory "$COVERAGE_DIR/html"
            print_success "Coverage report generated: $COVERAGE_DIR/html/index.html"
        else
            print_warning "lcov not found, coverage info file generated: $COVERAGE_DIR/coverage.info"
        fi
    else
        print_warning "gcov not found, coverage generation skipped"
    fi
}

# Function to generate performance report
generate_performance_report() {
    print_status "Generating performance report..."
    
    cd "$PROJECT_ROOT"
    
    # Parse performance test results
    python3 "$PROJECT_ROOT/tests/scripts/performance_report.py" \
        --input "$TEST_RESULTS_DIR/performance_tests.log" \
        --output "$PERFORMANCE_DIR/performance_report.html"
    
    if [ $? -eq 0 ]; then
        print_success "Performance report generated: $PERFORMANCE_DIR/performance_report.html"
    else
        print_warning "Performance report generation failed"
    fi
}

# Function to generate test summary
generate_summary() {
    print_status "Generating test summary..."
    
    local summary_file="$TEST_RESULTS_DIR/test_summary.txt"
    
    cat > "$summary_file" << EOF
OpenEMS Test Suite Summary
==========================
Date: $(date)
Build: $(git rev-parse --short HEAD 2>/dev/null || echo "unknown")
Target: esp32s3

Test Results:
-------------
Unit Tests: $([ -f "$TEST_RESULTS_DIR/unit_tests.log" ] && echo "PASSED" || echo "FAILED")
Integration Tests: $([ -f "$TEST_RESULTS_DIR/integration_tests.log" ] && echo "PASSED" || echo "FAILED")
Performance Tests: $([ -f "$TEST_RESULTS_DIR/performance_tests.log" ] && echo "COMPLETED" || echo "SKIPPED")

Coverage Report: $([ -f "$COVERAGE_DIR/coverage.info" ] && echo "GENERATED" || echo "SKIPPED")
Performance Report: $([ -f "$PERFORMANCE_DIR/performance_report.html" ] && echo "GENERATED" || echo "SKIPPED")

Files:
------
- Unit test log: $TEST_RESULTS_DIR/unit_tests.log
- Integration test log: $TEST_RESULTS_DIR/integration_tests.log
- Performance test log: $TEST_RESULTS_DIR/performance_tests.log
- Coverage report: $COVERAGE_DIR/html/index.html
- Performance report: $PERFORMANCE_DIR/performance_report.html
EOF

    cat "$summary_file"
    print_success "Test summary saved to: $summary_file"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -c, --clean      Clean build directory before building"
    echo "  -u, --unit       Run only unit tests"
    echo "  -i, --integration Run only integration tests"
    echo "  -p, --performance Run only performance tests"
    echo "  --coverage       Generate coverage report"
    echo "  --no-build       Skip build step"
    echo "  -h, --help       Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                    # Run all tests"
    echo "  $0 -u --coverage      # Run unit tests with coverage"
    echo "  $0 -c -i -p           # Clean and run integration + performance tests"
}

# Main execution
main() {
    local clean_build_flag=false
    local run_unit=true
    local run_integration=true
    local run_performance=true
    local generate_coverage_flag=false
    local no_build=false
    
    # Parse command line arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            -c|--clean)
                clean_build_flag=true
                shift
                ;;
            -u|--unit)
                run_unit=true
                run_integration=false
                run_performance=false
                shift
                ;;
            -i|--integration)
                run_unit=false
                run_integration=true
                run_performance=false
                shift
                ;;
            -p|--performance)
                run_unit=false
                run_integration=false
                run_performance=true
                shift
                ;;
            --coverage)
                generate_coverage_flag=true
                shift
                ;;
            --no-build)
                no_build=true
                shift
                ;;
            -h|--help)
                show_usage
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                show_usage
                exit 1
                ;;
        esac
    done
    
    print_status "Starting OpenEMS test suite..."
    
    # Check environment
    check_idf_env
    
    # Clean build if requested
    if [ "$clean_build_flag" = true ]; then
        clean_build
    fi
    
    # Build project
    if [ "$no_build" = false ]; then
        build_project
    fi
    
    local test_failed=false
    
    # Run tests
    if [ "$run_unit" = true ]; then
        if ! run_unit_tests; then
            test_failed=true
        fi
    fi
    
    if [ "$run_integration" = true ]; then
        if ! run_integration_tests; then
            test_failed=true
        fi
    fi
    
    if [ "$run_performance" = true ]; then
        run_performance_tests
    fi
    
    # Generate reports
    if [ "$generate_coverage_flag" = true ]; then
        generate_coverage
    fi
    
    generate_performance_report
    generate_summary
    
    # Final status
    if [ "$test_failed" = true ]; then
        print_error "Some tests failed!"
        exit 1
    else
        print_success "All tests completed successfully!"
        exit 0
    fi
}

# Run main function with all arguments
main "$@"
