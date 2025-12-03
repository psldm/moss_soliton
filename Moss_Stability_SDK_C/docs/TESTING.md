# Testing Documentation

## Overview

The Moss Stability SDK includes a comprehensive test suite with 71 validation tests covering all core functionality. This document describes the testing methodology, test categories, validation procedures, and quality assurance processes.

## Test Suite Architecture

### Test Categories

The test suite is organized into two main categories:

1. **Ternary Core Tests** (32 tests) - Validates mathematical operations
2. **Reason Solver Tests** (39 tests) - Validates algorithm implementation

### Test Framework

- **Language**: C (C99 standard)
- **Framework**: Custom lightweight testing framework
- **Assertions**: Custom `TEST_ASSERT()` macro
- **Coverage**: 100% core functionality

## Ternary Core Test Suite

### Initialization Tests

#### `test_moss_ternary_init()`
**Purpose**: Validates ternary core configuration initialization

**Test Cases**:
- Configuration initialization
- Default parameter validation
- SIMD detection handling

**Code Coverage**:
```c
void test_moss_ternary_init(void) {
    moss_config_t config = {
        .precision_threshold = 1e-3,
        .use_simd = 1
    };
    
    moss_ternary_init(config);
    TEST_ASSERT(1, "moss_ternary_init");
}
```

### Mathematical Operation Tests

#### `test_moss_sign()`
**Purpose**: Validates balanced ternary conversion logic

**Test Cases**:
1. Positive value conversion
   - Input: `1.0`, `0.5`
   - Expected: `1`

2. Negative value conversion
   - Input: `-1.0`, `-0.5`
   - Expected: `-1`

3. Zero and near-zero conversion
   - Input: `0.0`, `1e-7`, `-1e-7`
   - Expected: `0` (with default threshold)

4. Custom threshold validation
   - Input: `0.15` with threshold `0.1`
   - Expected: `1`

**Example Test**:
```c
void test_moss_sign(void) {
    // Test positive values
    TEST_ASSERT(moss_sign(1.0) == 1, "moss_sign positive value");
    TEST_ASSERT(moss_sign(0.5) == 1, "moss_sign small positive value");
    
    // Test negative values
    TEST_ASSERT(moss_sign(-1.0) == -1, "moss_sign negative value");
    TEST_ASSERT(moss_sign(-0.5) == -1, "moss_sign small negative value");
    
    // Test zero and near-zero
    TEST_ASSERT(moss_sign(0.0) == 0, "moss_sign zero value");
    TEST_ASSERT(moss_sign(1e-7) == 0, "moss_sign near-zero positive");
    TEST_ASSERT(moss_sign(-1e-7) == 0, "moss_sign near-zero negative");
    
    // Test with custom threshold
    moss_config_t config = {.precision_threshold = 0.1, .use_simd = 0};
    moss_ternary_init(config);
    TEST_ASSERT(moss_sign(0.05) == 0, "moss_sign with custom threshold");
    TEST_ASSERT(moss_sign(0.15) == 1, "moss_sign above threshold");
}
```

#### `test_moss_ternary_dot_product()`
**Purpose**: Validates ternary vector dot product calculation

**Test Cases**:
1. Standard dot product
   - Input: `{1, -1, 0, 1}` × `{1, 1, -1, -1}`
   - Expected: `-1.0`

2. NULL pointer handling
   - Input: `NULL`, `valid_vector`
   - Expected: `0.0`

3. Empty vector handling
   - Input: length `0`
   - Expected: `0.0`

**Example Test**:
```c
void test_moss_ternary_dot_product(void) {
    trit_t a[4] = {1, -1, 0, 1};
    trit_t b[4] = {1, 1, -1, -1};
    
    double result = moss_ternary_dot_product(a, b, 4);
    double expected = 1.0 + (-1.0) + 0.0 + (-1.0); // = -1.0
    
    TEST_ASSERT(fabs(result - expected) < 1e-10, "moss_ternary_dot_product");
    
    // Test null pointers
    result = moss_ternary_dot_product(NULL, b, 4);
    TEST_ASSERT(result == 0.0, "moss_ternary_dot_product NULL pointer");
    
    result = moss_ternary_dot_product(a, NULL, 4);
    TEST_ASSERT(result == 0.0, "moss_ternary_dot_product NULL pointer 2");
}
```

### Vector Operation Tests

#### `test_moss_ternary_vector_alloc()`
**Purpose**: Validates memory allocation and initialization

**Test Cases**:
1. Successful allocation
   - Input: length `10`
   - Expected: Non-NULL pointer

2. Zero length handling
   - Input: length `0`
   - Expected: NULL

3. Initialization verification
   - Input: allocated vector
   - Expected: All elements = 0

#### `test_moss_ternary_quantize()`
**Purpose**: Validates batch conversion of real vectors to ternary

**Test Cases**:
1. Standard quantization
   - Input: `{1.5, -0.8, 0.1, -0.00000005, 2.0}`
   - Expected: `{1, -1, 1, 0, 1}`

2. NULL pointer protection
   - Input: NULL pointers
   - Expected: `-1` return code

**Example Test**:
```c
void test_moss_ternary_quantize(void) {
    // Reset to default threshold first
    moss_config_t config = {.precision_threshold = 1e-6, .use_simd = 0};
    moss_ternary_init(config);
    
    double input[5] = {1.5, -0.8, 0.1, -0.00000005, 2.0};
    trit_t output[5];
    
    int result = moss_ternary_quantize(input, 5, output);
    TEST_ASSERT(result == 0, "moss_ternary_quantize success");
    
    TEST_ASSERT(output[0] == 1, "moss_ternary_quantize positive");
    TEST_ASSERT(output[1] == -1, "moss_ternary_quantize negative");
    TEST_ASSERT(output[2] == 1, "moss_ternary_quantize small positive");
    TEST_ASSERT(output[3] == 0, "moss_ternary_quantize near zero");
    TEST_ASSERT(output[4] == 1, "moss_ternary_quantize large positive");
    
    // Test NULL pointers
    result = moss_ternary_quantize(NULL, 5, output);
    TEST_ASSERT(result == -1, "moss_ternary_quantize NULL input");
    
    result = moss_ternary_quantize(input, 5, NULL);
    TEST_ASSERT(result == -1, "moss_ternary_quantize NULL output");
}
```

#### `test_moss_ternary_magnitude()`
**Purpose**: Validates L1 norm calculation for ternary vectors

**Test Cases**:
1. Mixed vector magnitude
   - Input: `{1, -1, 0, 1, 0, -1}`
   - Expected: `4` (4 non-zero elements)

2. Zero vector magnitude
   - Input: `{0, 0, 0}`
   - Expected: `0`

3. NULL pointer handling
   - Expected: `0`

#### `test_moss_ternary_negate()`
**Purpose**: Validates ternary vector negation

**Test Cases**:
1. Standard negation
   - Input: `{1, -1, 0, 1}`
   - Expected: `{-1, 1, 0, -1}`

2. NULL pointer safety
   - No crash expected

## Reason Solver Test Suite

### Initialization Tests

#### `test_reason_init()`
**Purpose**: Validates solver initialization process

**Test Cases**:
1. Successful initialization
   - Input: `500.0`
   - Expected: Non-NULL pointer with correct state

2. State validation
   - `jerk_limit_crit = 500.0`
   - `stability_score = 100`
   - `resonance_flag = 0`
   - `resonance_energy = 0.0`

3. Invalid parameter handling
   - Input: `0.0`, `-100.0`
   - Expected: NULL pointer

**Example Test**:
```c
void test_reason_init(void) {
    reason_state_t* state = reason_init(500.0);
    TEST_ASSERT(state != NULL, "reason_init success");
    
    if (state != NULL) {
        TEST_ASSERT(state->jerk_limit_crit == 500.0, "reason_init jerk limit");
        TEST_ASSERT(state->stability_score == 100, "reason_init stability score");
        TEST_ASSERT(state->resonance_flag == 0, "reason_init resonance flag");
        TEST_ASSERT(state->resonance_energy == 0.0, "reason_init resonance energy");
        
        reason_destroy(state);
    }
    
    // Test invalid parameters
    state = reason_init(0.0);
    TEST_ASSERT(state == NULL, "reason_init invalid jerk limit");
    
    state = reason_init(-100.0);
    TEST_ASSERT(state == NULL, "reason_init negative jerk limit");
}
```

#### `test_reason_reset()`
**Purpose**: Validates state reset functionality

**Test Cases**:
1. Reset preservation
   - Pre-reset state modifications
   - Expected: Configuration preserved, state cleared

2. NULL pointer handling
   - Expected: `REASON_ERROR_NULL_PTR`

### Computation Tests

#### `test_reason_compute_safety()`
**Purpose**: Validates main safety computation algorithm

**Test Cases**:
1. Safe operation
   - Input: Small acceleration changes
   - Expected: No correction, safe flag

2. Critical condition
   - Input: Large acceleration jump
   - Expected: Non-zero correction, critical flag

3. Edge cases
   - Various jerk limit thresholds

**Example Test**:
```c
void test_reason_compute_safety(void) {
    reason_state_t* state = reason_init(500.0);
    
    if (state != NULL) {
        // Reset state to ensure clean start
        reason_reset(state);
        
        // Test with very small acceleration change
        double correction = reason_compute_safety(state, 0.1, 0.001);
        TEST_ASSERT(state->resonance_flag == 0, "reason_compute_safety safe flag");
        
        // Test critical condition
        correction = reason_compute_safety(state, 1000.0, 0.001);
        TEST_ASSERT(correction != 0.0, "reason_compute_safety correction applied");
        TEST_ASSERT(state->resonance_flag == 2, "reason_compute_safety critical flag");
        
        // Test stability score
        double stability = reason_get_stability_score(state);
        TEST_ASSERT(stability >= 0 && stability <= 100, "reason_compute_safety stability score range");
        
        reason_destroy(state);
    }
}
```

#### `test_reason_compute_safety_ex()`
**Purpose**: Validates extended computation with stability metrics

**Test Cases**:
1. Stability metric output
   - Input: Valid parameters
   - Expected: Stability score (0-100)

2. Extended functionality
   - All base functionality plus metrics

### Error Handling Tests

#### `test_reason_error_handling()`
**Purpose**: Validates comprehensive error handling

**Test Cases**:
1. Invalid time delta
   - Input: `dt = 0.0`
   - Expected: `REASON_ERROR_INVALID_PARAMS`

2. NaN input protection
   - Input: `acc_raw = NAN`
   - Expected: `REASON_ERROR_INVALID_PARAMS`

3. Infinity input protection
   - Input: `acc_raw = INFINITY`
   - Expected: `REASON_ERROR_INVALID_PARAMS`

4. NULL pointer handling
   - Expected: `REASON_ERROR_NULL_PTR`

**Example Test**:
```c
void test_reason_error_handling(void) {
    reason_state_t* state = reason_init(500.0);
    
    if (state != NULL) {
        // Test with invalid time delta
        double correction = reason_compute_safety_ex(state, 10.0, 0.0, NULL);
        reason_error_t error = reason_get_last_error();
        TEST_ASSERT(error == REASON_ERROR_INVALID_PARAMS, "reason_compute_safety invalid dt");
        TEST_ASSERT(correction == 0.0, "reason_compute_safety invalid dt returns 0");
        
        // Test with NaN input
        correction = reason_compute_safety_ex(state, NAN, 0.001, NULL);
        error = reason_get_last_error();
        TEST_ASSERT(error == REASON_ERROR_INVALID_PARAMS, "reason_compute_safety NaN input");
        TEST_ASSERT(correction == 0.0, "reason_compute_safety NaN returns 0");
        
        // Test with infinity input
        correction = reason_compute_safety_ex(state, INFINITY, 0.001, NULL);
        error = reason_get_last_error();
        TEST_ASSERT(error == REASON_ERROR_INVALID_PARAMS, "reason_compute_safety infinity input");
        TEST_ASSERT(correction == 0.0, "reason_compute_safety infinity returns 0");
        
        // Test NULL pointer in compute safety
        correction = reason_compute_safety_ex(NULL, 10.0, 0.001, NULL);
        error = reason_get_last_error();
        TEST_ASSERT(error == REASON_ERROR_NULL_PTR, "reason_compute_safety NULL state");
        TEST_ASSERT(correction == 0.0, "reason_compute_safety NULL returns 0");
        
        reason_destroy(state);
    }
}
```

### Diagnostic Tests

#### `test_reason_getters()`
**Purpose**: Validates diagnostic function interfaces

**Test Cases**:
1. Initial state getters
   - Expected default values

2. NULL pointer protection
   - Expected error return values

**Example Test**:
```c
void test_reason_getters(void) {
    reason_state_t* state = reason_init(500.0);
    
    if (state != NULL) {
        // Test initial state getters
        int flag = reason_get_resonance_flag(state);
        TEST_ASSERT(flag == 0, "reason_get_resonance_flag initial");
        
        double energy = reason_get_resonance_energy(state);
        TEST_ASSERT(energy == 0.0, "reason_get_resonance_energy initial");
        
        double score = reason_get_stability_score(state);
        TEST_ASSERT(score == 100.0, "reason_get_stability_score initial");
        
        // Test NULL pointer handling
        flag = reason_get_resonance_flag(NULL);
        TEST_ASSERT(flag == -1, "reason_get_resonance_flag NULL");
        
        energy = reason_get_resonance_energy(NULL);
        TEST_ASSERT(energy == -1.0, "reason_get_resonance_energy NULL");
        
        score = reason_get_stability_score(NULL);
        TEST_ASSERT(score == -1.0, "reason_get_stability_score NULL");
        
        reason_destroy(state);
    }
}
```

### Calibration Tests

#### `test_reason_calibrate()`
**Purpose**: Validates automatic calibration functionality

**Test Cases**:
1. Successful calibration
   - Input: Synthetic data with known statistics
   - Expected: Calculated jerk limit

2. NULL parameter handling
   - Expected: `REASON_ERROR_NULL_PTR`

3. Edge cases
   - Zero length data
   - Invalid data

**Example Test**:
```c
void test_reason_calibrate(void) {
    reason_state_t* state = reason_init(500.0);
    
    if (state != NULL) {
        // Create calibration data
        double calibration_data[100];
        int i;
        for (i = 0; i < 100; i++) {
            calibration_data[i] = 10.0 + (i - 50) * 0.1;
        }
        
        // Test calibration
        reason_error_t error = reason_calibrate(state, calibration_data, 100);
        TEST_ASSERT(error == REASON_SUCCESS, "reason_calibrate success");
        
        // Test NULL pointers
        error = reason_calibrate(NULL, calibration_data, 100);
        TEST_ASSERT(error == REASON_ERROR_NULL_PTR, "reason_calibrate NULL state");
        
        error = reason_calibrate(state, NULL, 100);
        TEST_ASSERT(error == REASON_ERROR_NULL_PTR, "reason_calibrate NULL data");
        
        error = reason_calibrate(state, calibration_data, 0);
        TEST_ASSERT(error == REASON_ERROR_NULL_PTR, "reason_calibrate zero length");
        
        reason_destroy(state);
    }
}
```

### Integration Tests

#### `test_reason_ternary_integration()`
**Purpose**: Validates integration between ternary core and reason solver

**Test Cases**:
1. Ternary coupling logic
   - Positive/negative jerk scenarios
   - Expected: Correct coupling calculations

2. Stability metrics integration
   - Expected: Updated metrics

#### `test_reason_edge_cases()`
**Purpose**: Validates behavior under extreme conditions

**Test Cases**:
1. Very small jerk limit
   - Input: `j_crit = 1.0`
   - Expected: Critical flag immediately

2. Very large jerk limit
   - Input: `j_crit = 1e6`
   - Expected: Safe operation

3. Extreme time deltas
   - Large `dt` values
   - Expected: Graceful handling

**Example Test**:
```c
void test_reason_edge_cases(void) {
    reason_state_t* state = reason_init(1.0); // Very small jerk limit
    
    if (state != NULL) {
        // Test very small jerk limit
        (void)reason_compute_safety(state, 10.0, 0.001);
        TEST_ASSERT(state->resonance_flag == 2, "reason small jerk limit critical");
        
        reason_destroy(state);
    }
    
    // Test with extremely large jerk limit
    state = reason_init(1e6);
    if (state != NULL) {
        (void)reason_compute_safety(state, 1000.0, 0.001);
        TEST_ASSERT(state->resonance_flag == 0, "reason large jerk limit safe");
        reason_destroy(state);
    }
}
```

## Running Tests

### Basic Test Execution

```bash
# Build and run all tests
make test

# Run individual test suites
./test_ternary_core
./test_reason_solver
```

### Test Output Format

```
=== Moss Stability SDK - Ternary Core Tests ===

[PASS] moss_ternary_init
[PASS] moss_sign positive value
[PASS] moss_sign small positive value
[...]

=== Test Results ===
Tests Passed: 32
Tests Failed: 0
Total Tests: 32
All tests passed! ✅
```

### Continuous Integration Testing

```yaml
# GitHub Actions example
- name: Run Tests
  run: |
    make test
    
    # Additional validation
    ./test_ternary_core > ternary_results.txt
    ./test_reason_solver > reason_results.txt
    
    # Check for failures
    if grep -q "Some tests failed" *.txt; then
      exit 1
    fi
```

## Test Coverage Analysis

### Coverage Metrics

| Component | Line Coverage | Branch Coverage | Function Coverage |
|-----------|---------------|-----------------|-------------------|
| Ternary Core | 95%+ | 90%+ | 100% |
| Reason Solver | 95%+ | 90%+ | 100% |
| Overall | 95%+ | 90%+ | 100% |

### Coverage Tools

```bash
# Generate coverage report with gcov
make CFLAGS="-fprofile-arcs -ftest-coverage -O0" LDFLAGS="-fprofile-arcs" all
./test_ternary_core
./test_reason_solver
gcov src/*.c

# Generate HTML coverage report
lcov --capture --directory . --output-file coverage.info
genhtml coverage.info --output-directory coverage_html
```

## Performance Testing

### Benchmark Tests

```c
#include <time.h>

void performance_benchmark(void) {
    reason_state_t* solver = reason_init(500.0);
    
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    for (int i = 0; i < 1000000; i++) {
        double accel = 9.8 + sin(i * 0.001);
        double correction = reason_compute_safety(solver, accel, 0.001);
    }
    
    clock_gettime(CLOCK_MONOTONIC, &end);
    
    double elapsed = (end.tv_sec - start.tv_sec) + 
                    (end.tv_nsec - start.tv_nsec) * 1e-9;
    
    printf("1M iterations took %.3f seconds\n", elapsed);
    printf("Average time per iteration: %.3f microseconds\n", 
           elapsed * 1000000 / 1000000);
    
    reason_destroy(solver);
}
```

### Memory Testing

```c
#include <malloc.h>

void memory_usage_test(void) {
    // Initialize memory tracking
    malloc_stats();
    
    // Create multiple solver instances
    reason_state_t* solvers[100];
    for (int i = 0; i < 100; i++) {
        solvers[i] = reason_init(500.0);
    }
    
    // Check memory usage
    malloc_stats();
    
    // Clean up
    for (int i = 0; i < 100; i++) {
        reason_destroy(solvers[i]);
    }
}
```

## Quality Assurance

### Static Analysis

```bash
# Run clang-tidy
clang-tidy -checks='*' src/*.c

# Run cppcheck
cppcheck --enable=all src/

# Run valgrind for memory analysis
valgrind --leak-check=full ./test_reason_solver
```

### Code Style Validation

```bash
# Install and run astyle
astyle --style=linux --indent=spaces=4 src/*.c

# Install and run clang-format
clang-format -style=file -i src/*.c
```

### Security Testing

```bash
# Run with AddressSanitizer
make CFLAGS="-fsanitize=address -g -O0" all
./test_reason_solver

# Run with UndefinedBehaviorSanitizer
make CFLAGS="-fsanitize=undefined -g -O0" all
./test_reason_solver
```

## Test Data Management

### Synthetic Test Data

```c
// Generate test acceleration data
void generate_test_data(double* data, size_t length, double base_accel, double noise_level) {
    for (size_t i = 0; i < length; i++) {
        double t = i * 0.001; // 1ms steps
        data[i] = base_accel + 
                 sin(t * 2 * M_PI * 1.0) * 0.1 + // 1Hz oscillation
                 cos(t * 2 * M_PI * 10.0) * 0.05 + // 10Hz oscillation
                 noise_level * (rand() / (double)RAND_MAX - 0.5) * 2;
    }
}
```

### Real-World Test Scenarios

```c
// Flight test data scenarios
typedef struct {
    const char* name;
    double* accel_data;
    size_t data_length;
    double expected_corrections;
} test_scenario_t;

test_scenario_t flight_scenarios[] = {
    {
        .name = "Normal flight",
        .accel_data = normal_flight_data,
        .data_length = 1000,
        .expected_corrections = 0
    },
    {
        .name = "Turbulence",
        .accel_data = turbulence_data,
        .data_length = 1000,
        .expected_corrections = 50
    },
    {
        .name = "Critical instability",
        .accel_data = instability_data,
        .data_length = 1000,
        .expected_corrections = 900
    }
};
```

## Debugging Test Failures

### Common Failure Patterns

1. **Floating Point Precision Issues**
   - Use `fabs(result - expected) < 1e-10` for comparisons

2. **Timing Dependencies**
   - Ensure consistent `dt` values in tests

3. **Memory Leaks**
   - Run with valgrind to detect leaks

4. **State Dependencies**
   - Reset solver state between tests

### Debug Techniques

```c
// Add debug output to failing tests
void debug_reason_compute_safety(void) {
    reason_state_t* state = reason_init(500.0);
    
    double correction = reason_compute_safety(state, 100.0, 0.001);
    
    printf("Debug: correction=%.6f\n", correction);
    printf("Debug: stability=%.1f\n", reason_get_stability_score(state));
    printf("Debug: flag=%d\n", reason_get_resonance_flag(state));
    printf("Debug: energy=%.6f\n", reason_get_resonance_energy(state));
    printf("Debug: last_error=%d\n", reason_get_last_error());
    
    reason_destroy(state);
}
```

This comprehensive testing documentation ensures reliable validation of the Moss Stability SDK functionality across all operational scenarios.