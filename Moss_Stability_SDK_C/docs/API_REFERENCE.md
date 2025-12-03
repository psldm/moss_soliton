# API Reference

## Overview

The Moss Stability SDK provides two main API interfaces:
- **Reason Solver API**: Core stability control functionality
- **Ternary Core API**: Mathematical operations for ternary logic

All functions are thread-safe and designed for real-time applications.

---

## Reason Solver API

### Data Types

#### `reason_state_t`

Opaque handle to a REASON solver instance. Contains all internal state for stability calculations.

```c
typedef struct reason_state_t reason_state_t;
```

**Memory Requirements**: Approximately 200 bytes per instance

**Lifetime**: Created with `reason_init()`, destroyed with `reason_destroy()`

#### `reason_error_t`

Error codes returned by API functions.

```c
typedef enum {
    REASON_SUCCESS = 0,              // Operation completed successfully
    REASON_ERROR_NULL_PTR = -1,      // NULL pointer passed to function
    REASON_ERROR_INVALID_PARAMS = -2, // Invalid parameters provided
    REASON_ERROR_MALLOC_FAIL = -3,   // Memory allocation failed
    REASON_ERROR_COMPUTATION = -4    // Computation error occurred
} reason_error_t;
```

### Core Functions

#### `reason_init`

```c
reason_state_t* reason_init(double j_crit);
```

**Purpose**: Initialize a new REASON solver instance

**Parameters**:
- `j_crit` (double): Critical jerk limit in m/s³. Must be positive.

**Returns**: 
- Success: Pointer to initialized `reason_state_t`
- Failure: NULL (check `reason_get_last_error()` for details)

**Usage**:
```c
reason_state_t* solver = reason_init(500.0); // 500 m/s³ jerk limit
if (solver == NULL) {
    // Handle initialization failure
}
```

**Error Conditions**:
- `j_crit <= 0.0`: Invalid jerk limit
- Memory allocation failure
- NULL pointer errors

#### `reason_destroy`

```c
void reason_destroy(reason_state_t* state);
```

**Purpose**: Free REASON solver resources

**Parameters**:
- `state` (reason_state_t*): Solver instance to destroy

**Returns**: None

**Usage**:
```c
reason_destroy(solver); // Safe to call with NULL
```

**Notes**:
- Safe to call with NULL pointer
- Should be called when solver is no longer needed

#### `reason_reset`

```c
reason_error_t reason_reset(reason_state_t* state);
```

**Purpose**: Reset solver state while preserving configuration

**Parameters**:
- `state` (reason_state_t*): Solver instance to reset

**Returns**:
- `REASON_SUCCESS` on success
- `REASON_ERROR_NULL_PTR` if state is NULL

**Usage**:
```c
reason_error_t result = reason_reset(solver);
if (result != REASON_SUCCESS) {
    // Handle error
}
```

**Effects**:
- Clears all history buffers
- Resets stability score to 100
- Clears resonance energy
- Resets dampening factor to 1.0
- Preserves `jerk_limit_crit` setting

### Computation Functions

#### `reason_compute_safety`

```c
double reason_compute_safety(reason_state_t* state, double acc_raw, double dt);
```

**Purpose**: Main safety computation with corrective thrust calculation

**Parameters**:
- `state` (reason_state_t*): Solver instance
- `acc_raw` (double): Raw accelerometer data in m/s²
- `dt` (double): Time delta in seconds (must be > 0)

**Returns**: Corrective thrust delta (double)
- Non-zero: Correction needed (positive or negative)
- 0.0: No correction required (safe operation)

**Usage**:
```c
double correction = reason_compute_safety(solver, accelerometer_data, 0.001);
if (correction != 0.0) {
    apply_thrust_correction(correction);
}
```

**Algorithm**:
1. Calculates jerk from acceleration change
2. Compares against critical jerk limit
3. Computes ternary-based correction if needed
4. Updates internal state and metrics

**Error Handling**: Invalid parameters result in 0.0 return and error logging

#### `reason_compute_safety_ex`

```c
double reason_compute_safety_ex(reason_state_t* state, double acc_raw, double dt, double* stability_metric);
```

**Purpose**: Extended safety computation with stability metrics

**Parameters**:
- `state` (reason_state_t*): Solver instance
- `acc_raw` (double): Raw accelerometer data in m/s²
- `dt` (double): Time delta in seconds
- `stability_metric` (double*): Optional output for stability score

**Returns**: Corrective thrust delta (double)

**Usage**:
```c
double stability_score;
double correction = reason_compute_safety_ex(
    solver, 
    accelerometer_data, 
    0.001, 
    &stability_score
);

printf("Stability: %.1f%%\n", stability_score);
```

**Output Parameters**:
- `*stability_metric`: Current stability score (0-100), where 100 = perfect stability

### Diagnostic Functions

#### `reason_get_last_error`

```c
reason_error_t reason_get_last_error(void);
```

**Purpose**: Retrieve last error code from any API function

**Parameters**: None

**Returns**: Last error code

**Usage**:
```c
reason_error_t last_error = reason_get_last_error();
if (last_error != REASON_SUCCESS) {
    printf("Error: %d\n", last_error);
}
```

**Notes**:
- Thread-safe: returns last error for calling thread
- Error codes are reset on successful operations

#### `reason_get_stability_score`

```c
double reason_get_stability_score(const reason_state_t* state);
```

**Purpose**: Get current system stability score

**Parameters**:
- `state` (const reason_state_t*): Solver instance

**Returns**: Stability score (0.0 - 100.0)
- 100.0: Perfect stability
- 0.0: Critical instability
- -1.0: Error (NULL state)

**Usage**:
```c
double score = reason_get_stability_score(solver);
if (score >= 0.0 && score < 50.0) {
    trigger_stability_warning(score);
}
```

#### `reason_get_resonance_flag`

```c
int reason_get_resonance_flag(const reason_state_t* state);
```

**Purpose**: Get current resonance warning flag

**Parameters**:
- `state` (const reason_state_t*): Solver instance

**Returns**: Resonance flag (0, 1, or 2)
- 0: Safe operation
- 1: Warning - approaching limit
- 2: Critical - limit exceeded
- -1: Error (NULL state)

**Usage**:
```c
int flag = reason_get_resonance_flag(solver);
switch (flag) {
    case 0: /* Safe */ break;
    case 1: /* Warning */ log_warning("Approaching jerk limit"); break;
    case 2: /* Critical */ emergency_shutdown(); break;
}
```

#### `reason_get_resonance_energy`

```c
double reason_get_resonance_energy(const reason_state_t* state);
```

**Purpose**: Get accumulated resonance energy

**Parameters**:
- `state` (const reason_state_t*): Solver instance

**Returns**: Resonance energy level (≥ 0.0)
- 0.0: No resonance energy
- Increasing values indicate accumulated violations
- -1.0: Error (NULL state)

**Usage**:
```c
double energy = reason_get_resonance_energy(solver);
if (energy > 5.0) {
    predict_imminent_instability(energy);
}
```

### Calibration Functions

#### `reason_calibrate`

```c
reason_error_t reason_calibrate(reason_state_t* state, const double* calibration_data, size_t data_length);
```

**Purpose**: Calibrate jerk limit using historical data

**Parameters**:
- `state` (reason_state_t*): Solver instance
- `calibration_data` (const double*): Historical acceleration data
- `data_length` (size_t): Number of data points

**Returns**:
- `REASON_SUCCESS` on success
- `REASON_ERROR_NULL_PTR` if parameters are invalid
- Error code on failure

**Usage**:
```c
// Collect sensor data
double historical_accel[1000];
// ... populate with sensor readings ...

reason_error_t result = reason_calibrate(solver, historical_accel, 1000);
if (result == REASON_SUCCESS) {
    printf("Calibration complete. New jerk limit: %.2f\n", 
           solver->jerk_limit_crit);
}
```

**Algorithm**:
- Calculates mean and standard deviation of data
- Sets jerk limit to 3 standard deviations (99.7% confidence)
- Ensures minimum threshold of 1.0 m/s³

**Requirements**:
- `data_length > 0`
- `calibration_data` must contain valid numerical values
- Data should represent normal operating conditions

---

## Ternary Core API

### Data Types

#### `trit_t`

```c
typedef int8_t trit_t;
```

Balanced ternary digit type representing values {-1, 0, +1}.

#### `moss_config_t`

```c
typedef struct {
    double precision_threshold;  // Quantization threshold
    int use_simd;               // SIMD optimization enable
} moss_config_t;
```

Configuration structure for ternary operations.

### Core Functions

#### `moss_ternary_init`

```c
void moss_ternary_init(moss_config_t config);
```

**Purpose**: Initialize ternary core with configuration

**Parameters**:
- `config` (moss_config_t): Configuration parameters

**Usage**:
```c
moss_config_t config = {
    .precision_threshold = 1e-6,
    .use_simd = 1  // Enable SIMD if available
};
moss_ternary_init(config);
```

**Effects**:
- Sets global precision threshold for `moss_sign()`
- Enables SIMD optimizations if supported
- Affects all subsequent ternary operations

#### `moss_sign`

```c
trit_t moss_sign(double value);
```

**Purpose**: Convert real value to balanced ternary

**Parameters**:
- `value` (double): Input value to convert

**Returns**: Ternary digit (-1, 0, or +1)

**Conversion Logic**:
```c
if (value > threshold) return 1;
if (value < -threshold) return -1;
return 0;
```

**Usage**:
```c
trit_t sign = moss_sign(0.5);   // Returns 1
trit_t sign = moss_sign(-0.3);  // Returns -1
trit_t sign = moss_sign(0.0001); // Returns 0 (if threshold = 1e-6)
```

#### `moss_ternary_dot_product`

```c
double moss_ternary_dot_product(const trit_t* a, const trit_t* b, size_t len);
```

**Purpose**: Compute dot product of two ternary vectors

**Parameters**:
- `a` (const trit_t*): First ternary vector
- `b` (const trit_t*): Second ternary vector
- `len` (size_t): Vector length

**Returns**: Dot product result (double)

**Usage**:
```c
trit_t vec1[4] = {1, -1, 0, 1};
trit_t vec2[4] = {1, 1, -1, -1};

double result = moss_ternary_dot_product(vec1, vec2, 4);
// result = 1.0 + (-1.0) + 0.0 + (-1.0) = -1.0
```

**Optimization**:
- Automatic SIMD detection and usage
- Falls back to scalar implementation
- Processes 4 elements at a time when SIMD enabled

### Vector Operations

#### `moss_ternary_vector_alloc`

```c
trit_t* moss_ternary_vector_alloc(size_t length);
```

**Purpose**: Allocate and initialize ternary vector

**Parameters**:
- `length` (size_t): Vector length

**Returns**: 
- Success: Pointer to allocated vector (initialized to zeros)
- Failure: NULL

**Usage**:
```c
trit_t* vector = moss_ternary_vector_alloc(100);
if (vector != NULL) {
    // Use vector
    moss_ternary_vector_free(vector);
}
```

#### `moss_ternary_vector_free`

```c
void moss_ternary_vector_free(trit_t* vector);
```

**Purpose**: Free ternary vector allocated by `moss_ternary_vector_alloc()`

**Parameters**:
- `vector` (trit_t*): Vector to free

**Returns**: None

**Usage**:
```c
moss_ternary_vector_free(vector); // Safe with NULL
```

#### `moss_ternary_quantize`

```c
int moss_ternary_quantize(const double* input, size_t length, trit_t* output);
```

**Purpose**: Convert real-valued vector to ternary representation

**Parameters**:
- `input` (const double*): Real-valued input vector
- `length` (size_t): Vector length
- `output` (trit_t*): Output ternary vector (must be pre-allocated)

**Returns**: 
- 0 on success
- -1 on error (NULL pointers)

**Usage**:
```c
double real_values[5] = {1.5, -0.8, 0.001, -0.05, 2.0};
trit_t ternary_output[5];

int result = moss_ternary_quantize(real_values, 5, ternary_output);
// ternary_output: [1, -1, 0, 0, 1] (depending on threshold)
```

#### `moss_ternary_magnitude`

```c
size_t moss_ternary_magnitude(const trit_t* vector, size_t length);
```

**Purpose**: Compute ternary magnitude (L1 norm)

**Parameters**:
- `vector` (const trit_t*): Input ternary vector
- `length` (size_t): Vector length

**Returns**: Number of non-zero elements

**Usage**:
```c
trit_t vec[6] = {1, -1, 0, 1, 0, -1};
size_t mag = moss_ternary_magnitude(vec, 6); // Returns 4
```

#### `moss_ternary_negate`

```c
void moss_ternary_negate(trit_t* vector, size_t length);
```

**Purpose**: Negate ternary vector elements

**Parameters**:
- `vector` (trit_t*): Vector to negate (modified in place)
- `length` (size_t): Vector length

**Returns**: None

**Usage**:
```c
trit_t vec[4] = {1, -1, 0, 1};
moss_ternary_negate(vec, 4); // vec becomes [-1, 1, 0, -1]
```

---

## Error Handling Best Practices

### Error Checking Pattern

```c
// Initialize
reason_state_t* solver = reason_init(500.0);
if (solver == NULL) {
    fprintf(stderr, "Initialization failed: %d\n", reason_get_last_error());
    return ERROR_INIT_FAILED;
}

// Use in computation
double correction = reason_compute_safety(solver, accel, dt);
if (reason_get_last_error() != REASON_SUCCESS) {
    fprintf(stderr, "Computation error: %d\n", reason_get_last_error());
    // Handle error appropriately
}

// Clean up
reason_destroy(solver);
```

### Null Pointer Safety

All functions handle NULL pointers gracefully:

```c
// Safe operations
reason_destroy(NULL);              // No-op
reason_reset(NULL);                // Returns REASON_ERROR_NULL_PTR
reason_get_stability_score(NULL);  // Returns -1.0
reason_get_resonance_flag(NULL);   // Returns -1
```

### Parameter Validation

```c
// The following parameters are validated automatically:
double correction = reason_compute_safety(solver, NAN, 0.001);
// Returns 0.0, sets REASON_ERROR_INVALID_PARAMS

double correction = reason_compute_safety(solver, 10.0, 0.0);
// Returns 0.0, sets REASON_ERROR_INVALID_PARAMS

double correction = reason_compute_safety(solver, 10.0, -0.001);
// Returns 0.0, sets REASON_ERROR_INVALID_PARAMS
```

## Thread Safety

All API functions are thread-safe:
- Each solver instance maintains its own state
- No shared global mutable data
- Error codes are per-thread
- Safe for concurrent access to different instances

```c
// Thread-safe example
#pragma omp parallel for
for (int i = 0; i < num_solvers; i++) {
    reason_state_t* solver = solvers[i];
    double correction = reason_compute_safety(solver, accel_data[i], dt);
    // Each thread operates on independent solver instance
}
```

## Performance Considerations

### Memory Usage
- Each solver instance: ~200 bytes
- Ternary vectors: `length` bytes each
- No additional memory per function call

### CPU Usage
- Single solver @ 1kHz: < 0.1% CPU
- Linear scaling with number of instances
- SIMD optimization reduces CPU by ~40% when available

### Response Time
- Individual function calls: < 1μs
- Complete safety calculation: < 1μs
- Memory access optimized for cache locality

---

This API reference covers all public interfaces of the Moss Stability SDK. For integration examples and advanced usage patterns, see the [Developer Guide](DEVELOPER_GUIDE.md).