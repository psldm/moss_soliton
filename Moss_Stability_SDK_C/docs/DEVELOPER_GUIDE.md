# Developer Integration Guide

## Overview

This guide provides practical examples and best practices for integrating the Moss Stability SDK into your autonomous systems. It covers common integration patterns, configuration strategies, and real-world usage scenarios.

## Getting Started

### Basic Integration Steps

1. **Include Headers**
```c
#include "include/reason_solver.h"
#include "include/ternary_core.h"
```

2. **Link Against Library**
```bash
gcc your_app.c -L. -lmoss_stability -o your_app -lm
```

3. **Initialize SDK**
```c
// Initialize ternary core with desired precision
moss_config_t config = {
    .precision_threshold = 1e-6,
    .use_simd = 1  // Enable SIMD if available
};
moss_ternary_init(config);

// Create solver instance
reason_state_t* solver = reason_init(500.0); // 500 m/s³ jerk limit
```

## Integration Patterns

### 1. Simple Control Loop Integration

The most common pattern integrates the solver directly into your main control loop:

```c
#include "include/reason_solver.h"

// Global configuration
#define JERK_LIMIT_CRIT 500.0
#define UPDATE_RATE_HZ  1000
#define DT              (1.0 / UPDATE_RATE_HZ)

// Global solver instance (for simple systems)
static reason_state_t* g_solver;

void control_system_init(void) {
    g_solver = reason_init(JERK_LIMIT_CRIT);
    if (g_solver == NULL) {
        // Handle initialization failure
        printf("Failed to initialize stability solver\n");
        exit(1);
    }
}

void main_control_loop(void) {
    while (1) {
        // Read sensor data
        double acc_raw = read_accelerometer();
        
        // Compute stability correction
        double correction = reason_compute_safety(g_solver, acc_raw, DT);
        
        // Apply correction to control system
        if (correction != 0.0) {
            apply_thrust_correction(correction);
        }
        
        // Optional: Log system status
        double stability = reason_get_stability_score(g_solver);
        if (stability < 80.0) {
            log_system_warning("Stability degraded: %.1f%%", stability);
        }
        
        // Sleep until next update
        usleep(1000); // 1ms = 1000 Hz
    }
}

void control_system_shutdown(void) {
    reason_destroy(g_solver);
}
```

### 2. Multi-Axis Control Integration

For systems with multiple control axes (pitch, roll, yaw):

```c
// Multi-axis stability control
typedef struct {
    reason_state_t* pitch_solver;
    reason_state_t* roll_solver;
    reason_state_t* yaw_solver;
    
    double pitch_jerk_limit;
    double roll_jerk_limit;
    double yaw_jerk_limit;
} multi_axis_control_t;

multi_axis_control_t* multi_axis_init(double pitch_jcrit, double roll_jcrit, double yaw_jcrit) {
    multi_axis_control_t* ctrl = malloc(sizeof(multi_axis_control_t));
    if (ctrl == NULL) return NULL;
    
    ctrl->pitch_solver = reason_init(pitch_jcrit);
    ctrl->roll_solver = reason_init(roll_jcrit);
    ctrl->yaw_solver = reason_init(yaw_jcrit);
    
    if (!ctrl->pitch_solver || !ctrl->roll_solver || !ctrl->yaw_solver) {
        // Cleanup on failure
        reason_destroy(ctrl->pitch_solver);
        reason_destroy(ctrl->roll_solver);
        reason_destroy(ctrl->yaw_solver);
        free(ctrl);
        return NULL;
    }
    
    return ctrl;
}

void multi_axis_compute(multi_axis_control_t* ctrl, 
                       double pitch_accel, double roll_accel, double yaw_accel,
                       double dt,
                       double* pitch_corr, double* roll_corr, double* yaw_corr) {
    
    // Compute corrections for each axis
    *pitch_corr = reason_compute_safety(ctrl->pitch_solver, pitch_accel, dt);
    *roll_corr  = reason_compute_safety(ctrl->roll_solver, roll_accel, dt);
    *yaw_corr   = reason_compute_safety(ctrl->yaw_solver, yaw_accel, dt);
    
    // Apply corrections to respective control surfaces
    if (*pitch_corr != 0.0) apply_pitch_correction(*pitch_corr);
    if (*roll_corr != 0.0)  apply_roll_correction(*roll_corr);
    if (*yaw_corr != 0.0)   apply_yaw_correction(*yaw_corr);
}

void multi_axis_shutdown(multi_axis_control_t* ctrl) {
    reason_destroy(ctrl->pitch_solver);
    reason_destroy(ctrl->roll_solver);
    reason_destroy(ctrl->yaw_solver);
    free(ctrl);
}
```

### 3. Callback-Based Integration

For event-driven systems or GUI applications:

```c
typedef void (*stability_callback_t)(double stability, int resonance_flag, void* user_data);

typedef struct {
    reason_state_t* solver;
    stability_callback_t callback;
    void* user_data;
    pthread_t monitor_thread;
    int running;
} stability_monitor_t;

void* stability_monitor_thread(void* arg) {
    stability_monitor_t* monitor = (stability_monitor_t*)arg;
    
    while (monitor->running) {
        double stability = reason_get_stability_score(monitor->solver);
        int flag = reason_get_resonance_flag(monitor->solver);
        
        if (monitor->callback) {
            monitor->callback(stability, flag, monitor->user_data);
        }
        
        usleep(100000); // 100ms = 10Hz monitoring
    }
    
    return NULL;
}

stability_monitor_t* stability_monitor_create(reason_state_t* solver,
                                            stability_callback_t callback,
                                            void* user_data) {
    stability_monitor_t* monitor = malloc(sizeof(stability_monitor_t));
    if (monitor == NULL) return NULL;
    
    monitor->solver = solver;
    monitor->callback = callback;
    monitor->user_data = user_data;
    monitor->running = 1;
    
    if (pthread_create(&monitor->monitor_thread, NULL, stability_monitor_thread, monitor) != 0) {
        free(monitor);
        return NULL;
    }
    
    return monitor;
}

void stability_monitor_stop(stability_monitor_t* monitor) {
    monitor->running = 0;
    pthread_join(monitor->monitor_thread, NULL);
    free(monitor);
}

// Usage example
void stability_alert_callback(double stability, int flag, void* user_data) {
    if (flag >= 1) {
        printf("ALERT: Stability=%.1f%%, Flag=%d\n", stability, flag);
    }
}

void example_callback_integration(void) {
    reason_state_t* solver = reason_init(500.0);
    stability_monitor_t* monitor = stability_monitor_create(
        solver, 
        stability_alert_callback, 
        NULL
    );
    
    // Your main application code here...
    
    stability_monitor_stop(monitor);
    reason_destroy(solver);
}
```

### 4. Real-Time Priority Integration

For hard real-time systems:

```c
#include <sched.h>
#include <pthread.h>

void set_realtime_priority(void) {
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler(0, SCHED_FIFO, &param);
}

void realtime_control_loop(void) {
    // Set high priority for stability control
    set_realtime_priority();
    
    reason_state_t* solver = reason_init(500.0);
    
    // Pin to specific CPU core for cache locality
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset); // Use CPU core 0
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    
    // Real-time control loop
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    
    while (1) {
        // Calculate next wake time (1ms period)
        next_time.tv_nsec += 1000000;
        if (next_time.tv_nsec >= 1000000000) {
            next_time.tv_nsec -= 1000000000;
            next_time.tv_sec++;
        }
        
        // Read sensors with minimal latency
        double acc_raw = read_accelerometer_realtime();
        
        // Compute stability correction
        double correction = reason_compute_safety(solver, acc_raw, 0.001);
        
        // Apply immediate correction
        if (correction != 0.0) {
            apply_immediate_correction(correction);
        }
        
        // Sleep until next period
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }
    
    reason_destroy(solver);
}
```

## Configuration Best Practices

### 1. Jerk Limit Selection

Choose jerk limits based on your system characteristics:

```c
// Conservative approach - for safety-critical systems
reason_state_t* conservative_solver = reason_init(100.0); // 100 m/s³

// Standard approach - for most autonomous systems
reason_state_t* standard_solver = reason_init(500.0); // 500 m/s³

// Aggressive approach - for high-performance systems
reason_state_t* aggressive_solver = reason_init(2000.0); // 2000 m/s³

// Adaptive approach - adjust based on system state
double adaptive_jerk_limit = calculate_adaptive_limit(system_state);
reason_state_t* adaptive_solver = reason_init(adaptive_jerk_limit);
```

### 2. System-Specific Calibration

```c
void calibrate_for_aircraft(void) {
    // Collect flight test data
    double flight_data[10000];
    size_t data_length = collect_accelerometer_data(flight_data, 10000);
    
    reason_state_t* solver = reason_init(500.0);
    
    // Calibrate using actual flight data
    reason_error_t result = reason_calibrate(solver, flight_data, data_length);
    if (result == REASON_SUCCESS) {
        printf("Calibrated jerk limit: %.2f m/s³\n", solver->jerk_limit_crit);
    }
    
    // Save calibrated parameters for production use
    save_calibration_data(solver->jerk_limit_crit, "aircraft_calibration.dat");
}
```

### 3. Error Handling Strategy

```c
typedef enum {
    SYSTEM_OK = 0,
    SYSTEM_WARNING = 1,
    SYSTEM_CRITICAL = 2,
    SYSTEM_EMERGENCY = 3
} system_status_t;

system_status_t evaluate_system_health(reason_state_t* solver) {
    double stability = reason_get_stability_score(solver);
    int flag = reason_get_resonance_flag(solver);
    double energy = reason_get_resonance_energy(solver);
    
    if (flag == 2 || stability < 20.0) return SYSTEM_EMERGENCY;
    if (flag == 1 || stability < 60.0) return SYSTEM_CRITICAL;
    if (stability < 80.0) return SYSTEM_WARNING;
    return SYSTEM_OK;
}

void handle_system_status(system_status_t status) {
    switch (status) {
        case SYSTEM_EMERGENCY:
            emergency_landing_protocol();
            break;
        case SYSTEM_CRITICAL:
            reduce_speed_and_altitude();
            break;
        case SYSTEM_WARNING:
            log_caution("Stability degraded");
            break;
        case SYSTEM_OK:
            // Continue normal operation
            break;
    }
}
```

## Memory Management

### Efficient Memory Usage

```c
// Pre-allocate solver instances during initialization
#define MAX_CONTROL_AXES 6
reason_state_t* axis_solvers[MAX_CONTROL_AXES];

void initialize_all_solvers(void) {
    for (int i = 0; i < MAX_CONTROL_AXES; i++) {
        axis_solvers[i] = reason_init(DEFAULT_JERK_LIMIT);
        if (axis_solvers[i] == NULL) {
            // Handle initialization failure
            fprintf(stderr, "Failed to initialize solver %d\n", i);
            exit(1);
        }
    }
}

// Clean shutdown
void shutdown_all_solvers(void) {
    for (int i = 0; i < MAX_CONTROL_AXES; i++) {
        reason_destroy(axis_solvers[i]);
    }
}
```

### Memory Pool Management

```c
typedef struct {
    reason_state_t* solvers;
    size_t count;
    pthread_mutex_t mutex;
} solver_pool_t;

solver_pool_t* solver_pool_create(size_t count, double jerk_limit) {
    solver_pool_t* pool = malloc(sizeof(solver_pool_t));
    if (pool == NULL) return NULL;
    
    pool->solvers = malloc(count * sizeof(reason_state_t*));
    if (pool->solvers == NULL) {
        free(pool);
        return NULL;
    }
    
    pool->count = count;
    pthread_mutex_init(&pool->mutex, NULL);
    
    // Pre-allocate all solvers
    for (size_t i = 0; i < count; i++) {
        pool->solvers[i] = reason_init(jerk_limit);
        if (pool->solvers[i] == NULL) {
            // Cleanup on failure
            for (size_t j = 0; j < i; j++) {
                reason_destroy(pool->solvers[j]);
            }
            free(pool->solvers);
            free(pool);
            return NULL;
        }
    }
    
    return pool;
}

reason_state_t* solver_pool_acquire(solver_pool_t* pool) {
    pthread_mutex_lock(&pool->mutex);
    
    // Find first available solver (simplified)
    reason_state_t* solver = pool->solvers[0];
    
    pthread_mutex_unlock(&pool->mutex);
    return solver;
}

void solver_pool_release(solver_pool_t* pool, reason_state_t* solver) {
    // In this simple example, solvers are always "available"
    // In practice, you'd implement proper pooling logic
    pthread_mutex_unlock(&pool->mutex);
}

void solver_pool_destroy(solver_pool_t* pool) {
    pthread_mutex_lock(&pool->mutex);
    
    for (size_t i = 0; i < pool->count; i++) {
        reason_destroy(pool->solvers[i]);
    }
    
    free(pool->solvers);
    free(pool);
}
```

## Performance Optimization

### 1. SIMD Optimization

```c
void enable_max_performance(void) {
    // Configure ternary core for maximum performance
    moss_config_t config = {
        .precision_threshold = 1e-6,
        .use_simd = 1  // Enable SIMD on supported platforms
    };
    moss_ternary_init(config);
    
    // Verify SIMD support
    #ifdef __SSE2__
    printf("SSE2 support detected\n");
    #endif
    
    #ifdef __ARM_NEON
    printf("ARM NEON support detected\n");
    #endif
}
```

### 2. Cache-Aware Programming

```c
// Structure layout for optimal cache performance
typedef struct {
    // Hot data (frequently accessed) - keep together
    double current_acc;
    double last_acc;
    double jerk_limit_crit;
    int resonance_flag;
    double stability_score;
    
    // Warm data (less frequently accessed)
    double jerk_history[8];
    double resonance_energy;
    double dampening_factor;
    
    // Cold data (rarely accessed)
    trit_t ternary_buffer[4];
    size_t history_index;
    double avg_jerk;
} __attribute__((packed)) optimized_solver_state_t;
```

### 3. Batch Processing

```c
void batch_stability_update(reason_state_t** solvers, double* accel_data, 
                           size_t count, double dt) {
    #pragma omp parallel for
    for (size_t i = 0; i < count; i++) {
        double correction = reason_compute_safety(solvers[i], accel_data[i], dt);
        // Apply correction for solver i
        apply_correction_to_system(i, correction);
    }
}
```

## Logging and Diagnostics

### Comprehensive Logging

```c
typedef struct {
    FILE* log_file;
    pthread_mutex_t log_mutex;
    int log_level;
} logger_t;

#define LOG_INFO  1
#define LOG_WARN  2
#define LOG_ERROR 3

void log_stability_event(logger_t* logger, const char* format, ...) {
    if (logger->log_level > LOG_INFO) return;
    
    pthread_mutex_lock(&logger->log_mutex);
    
    time_t now = time(NULL);
    char time_str[26];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&now));
    
    fprintf(logger->log_file, "[%s] STABILITY: ", time_str);
    
    va_list args;
    va_start(args, format);
    vfprintf(logger->log_file, format, args);
    va_end(args);
    
    fprintf(logger->log_file, "\n");
    fflush(logger->log_file);
    
    pthread_mutex_unlock(&logger->log_mutex);
}

void log_system_diagnostics(logger_t* logger, reason_state_t* solver) {
    double stability = reason_get_stability_score(solver);
    int flag = reason_get_resonance_flag(solver);
    double energy = reason_get_resonance_energy(solver);
    reason_error_t error = reason_get_last_error();
    
    log_stability_event(logger, "Status: stability=%.1f%%, flag=%d, energy=%.3f, error=%d",
                       stability, flag, energy, error);
}
```

## Testing Integration

### Unit Test Integration

```c
#include <assert.h>

void test_integration(void) {
    reason_state_t* solver = reason_init(500.0);
    assert(solver != NULL);
    
    // Test normal operation
    double correction = reason_compute_safety(solver, 10.0, 0.001);
    assert(reason_get_last_error() == REASON_SUCCESS);
    
    // Test stability metrics
    double stability = reason_get_stability_score(solver);
    assert(stability >= 0.0 && stability <= 100.0);
    
    // Test error conditions
    correction = reason_compute_safety(NULL, 10.0, 0.001);
    assert(reason_get_last_error() == REASON_ERROR_NULL_PTR);
    
    reason_destroy(solver);
    printf("Integration test passed\n");
}
```

### System Test Integration

```c
void system_level_test(void) {
    // Initialize system
    reason_state_t* solver = reason_init(500.0);
    
    // Simulate various operating conditions
    struct test_scenario {
        double accel_data[100];
        double expected_corrections[100];
        const char* description;
    } scenarios[] = {
        // Normal flight conditions
        {{9.8, 9.81, 9.79, 9.82}, {0, 0, 0, 0}, "Normal flight"},
        
        // Turbulence conditions
        {{9.8, 10.5, 11.2, 10.8}, {0, 0, 0, 0}, "Mild turbulence"},
        
        // Critical instability
        {{9.8, 15.0, 25.0, 35.0}, {0, 0, 0, 0}, "Critical instability"}
    };
    
    for (int i = 0; i < 3; i++) {
        printf("Testing scenario: %s\n", scenarios[i].description);
        
        for (int j = 0; j < 4; j++) {
            double correction = reason_compute_safety(solver, 
                                                    scenarios[i].accel_data[j], 
                                                    0.001);
            
            // Validate correction magnitude
            assert(fabs(correction) < 100.0); // Reasonable correction range
        }
    }
    
    reason_destroy(solver);
    printf("System test completed\n");
}
```

## Common Pitfalls and Solutions

### 1. Time Synchronization Issues

**Problem**: Inconsistent time deltas causing unstable behavior

**Solution**: Use consistent time sources
```c
// Bad: System time may jump
double dt = clock() / CLOCKS_PER_SEC;

// Good: Monotonic clock
struct timespec t1, t2;
clock_gettime(CLOCK_MONOTONIC, &t1);
// ... processing ...
clock_gettime(CLOCK_MONOTONIC, &t2);
double dt = (t2.tv_sec - t1.tv_sec) + (t2.tv_nsec - t1.tv_nsec) * 1e-9;
```

### 2. Memory Leaks

**Problem**: Forgetting to destroy solver instances

**Solution**: Use RAII patterns
```c
typedef struct {
    reason_state_t* solver;
    int initialized;
} auto_solver_t;

auto_solver_t* auto_solver_create(double jerk_limit) {
    auto_solver_t* as = malloc(sizeof(auto_solver_t));
    as->solver = reason_init(jerk_limit);
    as->initialized = (as->solver != NULL);
    return as;
}

void auto_solver_destroy(auto_solver_t* as) {
    if (as && as->initialized) {
        reason_destroy(as->solver);
    }
    if (as) free(as);
}

#define WITH_SOLVER(limit, var_name) \
    for(auto_solver_t* var_name = auto_solver_create(limit); \
        var_name && var_name->initialized; \
        auto_solver_destroy(var_name))

// Usage
void example_raii_usage(void) {
    WITH_SOLVER(500.0, solver) {
        double correction = reason_compute_safety(solver->solver, 9.8, 0.001);
        // Automatic cleanup when going out of scope
    }
}
```

### 3. Race Conditions in Multi-Threading

**Problem**: Shared solver instances accessed concurrently

**Solution**: Thread-local instances or proper synchronization
```c
// Thread-local storage
static __thread reason_state_t* thread_local_solver = NULL;

reason_state_t* get_thread_solver(void) {
    if (thread_local_solver == NULL) {
        thread_local_solver = reason_init(500.0);
    }
    return thread_local_solver;
}

// Or use mutex protection
pthread_mutex_t solver_mutex = PTHREAD_MUTEX_INITIALIZER;

double thread_safe_compute(double accel, double dt) {
    pthread_mutex_lock(&solver_mutex);
    double correction = reason_compute_safety(global_solver, accel, dt);
    pthread_mutex_unlock(&solver_mutex);
    return correction;
}
```

## Deployment Considerations

### 1. Production Build Optimization

```makefile
# Makefile optimization flags
CFLAGS_RELEASE = -O3 -DNDEBUG -flto -march=native -mtune=native
CFLAGS_DEBUG = -O0 -g -DDEBUG -fsanitize=address
```

### 2. Configuration Management

```c
typedef struct {
    double jerk_limit_crit;
    double precision_threshold;
    int use_simd;
    int log_level;
} system_config_t;

int load_config(const char* config_file, system_config_t* config) {
    FILE* f = fopen(config_file, "r");
    if (!f) return -1;
    
    // Simple configuration parsing
    if (fscanf(f, "jerk_limit=%lf\n", &config->jerk_limit_crit) != 1) return -1;
    if (fscanf(f, "precision=%lf\n", &config->precision_threshold) != 1) return -1;
    if (fscanf(f, "use_simd=%d\n", &config->use_simd) != 1) return -1;
    if (fscanf(f, "log_level=%d\n", &config->log_level) != 1) return -1;
    
    fclose(f);
    return 0;
}
```

### 3. Health Monitoring

```c
void setup_health_monitoring(void) {
    // Periodic health checks
    signal(SIGALRM, health_check_handler);
    alarm(10); // Check every 10 seconds
}

void health_check_handler(int sig) {
    reason_state_t* solver = get_global_solver();
    
    double stability = reason_get_stability_score(solver);
    int flag = reason_get_resonance_flag(solver);
    
    if (stability < 70.0 || flag >= 1) {
        // Log critical health event
        log_critical_health_event(stability, flag);
        
        // Optionally trigger automated recovery
        if (flag == 2) {
            initiate_safe_mode();
        }
    }
    
    // Schedule next check
    alarm(10);
}
```

This developer guide provides comprehensive integration patterns for the Moss Stability SDK. For specific API details, see the [API Reference](API_REFERENCE.md), and for algorithmic details, see [Algorithms](ALGORITHMS.md).