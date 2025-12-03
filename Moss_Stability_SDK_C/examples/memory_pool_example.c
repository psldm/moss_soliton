#include <stdio.h>
#include <stdlib.h>
#include "../include/reason_solver.h"
#include "../include/ternary_core.h"

/**
 * Memory Pool Example for Embedded Systems
 * This demonstrates pre-allocated solver instances for deterministic timing
 */

#define SOLVER_POOL_SIZE 10
#define MAX_INSTANCES 3

// Pre-allocated solver instances for different control surfaces
static reason_state_t solver_pool[SOLVER_POOL_SIZE];
static int solver_initialized[SOLVER_POOL_SIZE] = {0};
static reason_state_t* control_surface_solvers[MAX_INSTANCES]; // pitch, roll, yaw

/**
 * Initialize the solver memory pool for embedded systems
 * This replaces dynamic allocation with pre-allocated memory
 */
void init_solver_pool(void) {
    printf("[MEMORY_POOL] Initializing solver pool with %d instances...\n", SOLVER_POOL_SIZE);
    
    // Initialize pool
    for (int i = 0; i < SOLVER_POOL_SIZE; i++) {
        solver_initialized[i] = 0;
    }
    
    // Pre-initialize control surface solvers
    control_surface_solvers[0] = reason_init(300.0); // pitch solver
    control_surface_solvers[1] = reason_init(300.0); // roll solver  
    control_surface_solvers[2] = reason_init(400.0); // yaw solver
    
    printf("[MEMORY_POOL] Pool initialized successfully\n");
}

/**
 * Get a solver from the pool
 * @param index Pool index to use
 * @param j_crit Critical jerk limit for this solver
 * @return Pointer to solver instance or NULL if unavailable
 */
reason_state_t* get_solver_from_pool(int index, double j_crit) {
    if (index < 0 || index >= SOLVER_POOL_SIZE) {
        return NULL;
    }
    
    if (!solver_initialized[index]) {
        solver_pool[index] = *reason_init(j_crit);
        solver_initialized[index] = 1;
    }
    
    return &solver_pool[index];
}

/**
 * Release a solver back to the pool (for advanced use cases)
 * @param index Pool index to release
 */
void release_solver_to_pool(int index) {
    if (index < 0 || index >= SOLVER_POOL_SIZE) {
        return;
    }
    
    if (solver_initialized[index]) {
        reason_reset(&solver_pool[index]);
        solver_initialized[index] = 0;
    }
}

/**
 * Example multi-instance stability monitoring
 * Demonstrates using multiple solvers for different control surfaces
 */
void multi_surface_monitoring(void) {
    printf("\n=== Multi-Surface Stability Monitoring ===\n");
    
    double dt = 0.001; // 1kHz update rate
    
    // Simulated sensor data for different axes
    double pitch_accel = 0.0;
    double roll_accel = 0.0;
    double yaw_accel = 0.0;
    
    for (int step = 0; step < 100; step++) {
        // Simulate increasing acceleration (instability)
        if (step > 50) {
            pitch_accel += 2.0;
            roll_accel += 1.5;
            yaw_accel += 1.0;
        } else {
            pitch_accel += 0.1;
            roll_accel += 0.1;
            yaw_accel += 0.1;
        }
        
        // Process each control surface independently
        double pitch_corr = reason_compute_safety(control_surface_solvers[0], pitch_accel, dt);
        double roll_corr = reason_compute_safety(control_surface_solvers[1], roll_accel, dt);
        double yaw_corr = reason_compute_safety(control_surface_solvers[2], yaw_accel, dt);
        
        // Check for critical conditions
        if (control_surface_solvers[0]->resonance_flag == 2) {
            printf("[ALERT] Step %d: PITCH CRITICAL - Correction: %.4f\n", step, pitch_corr);
        }
        if (control_surface_solvers[1]->resonance_flag == 2) {
            printf("[ALERT] Step %d: ROLL CRITICAL - Correction: %.4f\n", step, roll_corr);
        }
        if (control_surface_solvers[2]->resonance_flag == 2) {
            printf("[ALERT] Step %d: YAW CRITICAL - Correction: %.4f\n", step, yaw_corr);
        }
    }
}

/**
 * Performance benchmarking with memory pool
 */
void benchmark_memory_pool(void) {
    printf("\n=== Memory Pool Performance Benchmark ===\n");
    
    const int iterations = 10000;
    double dt = 0.001;
    
    // Get several solvers from pool for testing
    reason_state_t* test_solvers[5];
    for (int i = 0; i < 5; i++) {
        test_solvers[i] = get_solver_from_pool(i, 500.0);
    }
    
    // Benchmark calculations
    for (int iter = 0; iter < iterations; iter++) {
        for (int s = 0; s < 5; s++) {
            double accel = (double)iter * 0.001;
            (void)reason_compute_safety(test_solvers[s], accel, dt);
        }
    }
    
    printf("Completed %d iterations across 5 solvers\n", iterations);
    printf("Total calculations: %d\n", iterations * 5);
}

/**
 * Cleanup function for memory pool
 */
void cleanup_solver_pool(void) {
    printf("\n=== Cleaning Up Memory Pool ===\n");
    
    // Clean up control surface solvers
    for (int i = 0; i < MAX_INSTANCES; i++) {
        if (control_surface_solvers[i] != NULL) {
            reason_destroy(control_surface_solvers[i]);
            control_surface_solvers[i] = NULL;
        }
    }
    
    // Clean up pool solvers
    for (int i = 0; i < SOLVER_POOL_SIZE; i++) {
        if (solver_initialized[i]) {
            // For pre-allocated solvers, we reset rather than destroy
            reason_reset(&solver_pool[i]);
            solver_initialized[i] = 0;
        }
    }
    
    printf("Memory pool cleanup completed\n");
}

int main(void) {
    printf("=== Moss Stability SDK - Memory Pool Example ===\n");
    printf("Demonstrating embedded systems memory management patterns\n\n");
    
    // Initialize the memory pool
    init_solver_pool();
    
    // Demonstrate multi-surface monitoring
    multi_surface_monitoring();
    
    // Run performance benchmark
    benchmark_memory_pool();
    
    // Cleanup
    cleanup_solver_pool();
    
    printf("\nMemory pool example completed successfully!\n");
    return 0;
}