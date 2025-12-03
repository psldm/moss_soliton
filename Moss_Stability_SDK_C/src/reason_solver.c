#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "../include/reason_solver.h"
#include "../include/ternary_core.h"

// Static error state for thread-safe error reporting
static reason_error_t g_last_error = REASON_SUCCESS;

// Global ternary configuration
static moss_config_t g_moss_config = {
    .precision_threshold = 1e-6,
    .use_simd = 0
};

/**
 * Initialize REASON solver with critical jerk limit
 * @param j_crit Critical jerk limit (m/s^3)
 * @return Pointer to initialized state structure, NULL on failure
 */
reason_state_t* reason_init(double j_crit) {
    if (j_crit <= 0.0) {  // J_crit must be positive
        g_last_error = REASON_ERROR_INVALID_PARAMS;
        return NULL;
    }
    
    reason_state_t* state = (reason_state_t*)malloc(sizeof(reason_state_t));
    if (state == NULL) {
        g_last_error = REASON_ERROR_MALLOC_FAIL;
        return NULL;
    }
    
    // Initialize core parameters
    state->jerk_limit_crit = j_crit;
    state->current_acc = 0.0;
    state->last_acc = 0.0;
    state->last_time = 0.0;
    state->resonance_flag = 0;
    
    // Initialize enhanced state tracking
    memset(state->jerk_history, 0, sizeof(state->jerk_history));
    state->history_index = 0;
    state->avg_jerk = 0.0;
    state->resonance_energy = 0.0;
    state->stability_score = 100; // Start with perfect stability
    state->dampening_factor = 1.0;
    
    // Initialize ternary buffer
    memset(state->ternary_buffer, 0, sizeof(state->ternary_buffer));
    
    // Initialize ternary core
    moss_ternary_init(g_moss_config);
    
    g_last_error = REASON_SUCCESS;
    return state;
}

/**
 * Reset REASON solver state
 * @param state Pointer to solver state
 * @return Error code
 */
reason_error_t reason_reset(reason_state_t* state) {
    if (state == NULL) {
        g_last_error = REASON_ERROR_NULL_PTR;
        return REASON_ERROR_NULL_PTR;
    }
    
    // Clear state while preserving configuration (jerk_limit_crit)
    memset(state->jerk_history, 0, sizeof(state->jerk_history));
    state->history_index = 0;
    state->avg_jerk = 0.0;
    state->resonance_energy = 0.0;
    state->stability_score = 100;
    state->dampening_factor = 1.0;
    state->current_acc = 0.0;
    state->last_acc = 0.0;
    state->last_time = 0.0;
    state->resonance_flag = 0;
    
    // Clear ternary buffer
    memset(state->ternary_buffer, 0, sizeof(state->ternary_buffer));
    
    g_last_error = REASON_SUCCESS;
    return REASON_SUCCESS;
}

/**
 * Main safety computation with ternary dampening logic
 * Implements the REASON algorithm with proper ternary mathematics
 * @param state Pointer to solver state
 * @param acc_raw Raw accelerometer data (m/s^2)
 * @param dt Time delta (seconds)
 * @return Corrective thrust delta
 */
double reason_compute_safety(reason_state_t* state, double acc_raw, double dt) {
    return reason_compute_safety_ex(state, acc_raw, dt, NULL);
}

/**
 * Extended safety computation with stability metrics
 * @param state Pointer to solver state
 * @param acc_raw Raw accelerometer data (m/s^2)
 * @param dt Time delta (seconds)
 * @param stability_metric Optional output for stability score (0-100)
 * @return Corrective thrust delta
 */
double reason_compute_safety_ex(reason_state_t* state, double acc_raw, double dt, double* stability_metric) {
    if (state == NULL) {
        g_last_error = REASON_ERROR_NULL_PTR;
        return 0.0;
    }
    
    if (dt <= 0.000001 || dt > 10.0) {
        g_last_error = REASON_ERROR_INVALID_PARAMS;
        return 0.0;
    }
    
    if (isnan(acc_raw) || isinf(acc_raw)) {
        g_last_error = REASON_ERROR_INVALID_PARAMS;
        return 0.0;
    }
    
    // 1. Calculate jerk (da/dt)
    double delta_acc = acc_raw - state->last_acc;
    double jerk = delta_acc / dt;
    
    // Update acceleration history
    state->last_acc = acc_raw;
    
    // 2. Update jerk history for trend analysis
    state->jerk_history[state->history_index] = jerk;
    state->history_index = (state->history_index + 1) % 8; // Circular buffer
    
    // 3. Calculate average jerk over history window
    double jerk_sum = 0.0;
    size_t i;
    for (i = 0; i < 8; i++) {
        jerk_sum += state->jerk_history[i];
    }
    state->avg_jerk = jerk_sum / 8.0;
    
    // 4. REASON Check - Moss Inequality: |da/dt| < J_crit
    double jerk_abs = fabs(jerk);
    double excess_ratio = jerk_abs / state->jerk_limit_crit;
    
    // 5. Ternary analysis for resonance detection
    trit_t jerk_ternary = moss_sign(jerk);
    trit_t acc_ternary = moss_sign(acc_raw);
    
    // Store in ternary buffer for pattern analysis
    state->ternary_buffer[state->history_index % 4] = jerk_ternary;
    
    // 6. Resonance energy accumulation
    if (excess_ratio > 1.0) {
        // Resonance detected
        state->resonance_energy += (excess_ratio - 1.0) * dt;
        state->resonance_flag = (excess_ratio > 2.0) ? 2 : 1; // Critical vs Warning
    } else {
        // Energy decay when safe
        state->resonance_energy *= exp(-dt * 0.5);
        if (state->resonance_energy < 0.01) {
            state->resonance_flag = 0;
        }
    }
    
    // 7. Adaptive dampening factor using ternary logic
    if (excess_ratio > 1.0) {
        // Compute ternary-based dampening
        double ternary_jerk = (double)jerk_ternary;
        double ternary_acc = (double)acc_ternary;
        
        // Ternary dampening calculation
        // Using the property that -1*+1 = -1, +1*-1 = -1, same signs = +1
        double ternary_coupling = ternary_jerk * ternary_acc;
        
        // Adaptive dampening based on ternary coupling and excess
        state->dampening_factor = 1.0 + (excess_ratio * fabs(ternary_coupling) * 2.0);
        
        // 8. Compute corrective thrust using ternary dampening
        double excess = jerk_abs - state->jerk_limit_crit;
        double correction = -ternary_coupling * excess * 0.3 / state->dampening_factor;
        
        // Update stability score
        state->stability_score = (int)(100.0 * (1.0 / (1.0 + excess_ratio)));
        
        if (stability_metric != NULL) {
            *stability_metric = state->stability_score;
        }
        
        g_last_error = REASON_SUCCESS;
        return correction;
    } else {
        // Safe operation
        state->dampening_factor = 1.0;
        state->stability_score = 100;
        
        if (stability_metric != NULL) {
            *stability_metric = state->stability_score;
        }
        
        g_last_error = REASON_SUCCESS;
        return 0.0; // No correction needed
    }
}

/**
 * Destroy REASON solver and free resources
 * @param state Pointer to solver state
 */
void reason_destroy(reason_state_t* state) {
    if (state != NULL) {
        free(state);
    }
}

/**
 * Get last error code
 * @return Last error code
 */
reason_error_t reason_get_last_error(void) {
    return g_last_error;
}

/**
 * Get current stability score
 * @param state Pointer to solver state
 * @return Stability score (0-100), -1 on error
 */
double reason_get_stability_score(const reason_state_t* state) {
    if (state == NULL) {
        return -1.0;
    }
    return state->stability_score;
}

/**
 * Get current resonance flag
 * @param state Pointer to solver state
 * @return Resonance flag (0=Safe, 1=Warning, 2=Critical), -1 on error
 */
int reason_get_resonance_flag(const reason_state_t* state) {
    if (state == NULL) {
        return -1;
    }
    return state->resonance_flag;
}

/**
 * Get current resonance energy
 * @param state Pointer to solver state
 * @return Resonance energy level, -1.0 on error
 */
double reason_get_resonance_energy(const reason_state_t* state) {
    if (state == NULL) {
        return -1.0;
    }
    return state->resonance_energy;
}

/**
 * Calibrate REASON solver with reference data
 * @param state Pointer to solver state
 * @param calibration_data Array of calibration data points
 * @param data_length Length of calibration data
 * @return Error code
 */
reason_error_t reason_calibrate(reason_state_t* state, const double* calibration_data, size_t data_length) {
    if (state == NULL || calibration_data == NULL || data_length == 0) {
        g_last_error = REASON_ERROR_NULL_PTR;
        return REASON_ERROR_NULL_PTR;
    }
    
    // Simple calibration: adjust jerk limit based on data statistics
    double sum = 0.0;
    double sum_sq = 0.0;
    
    size_t i;
    for (i = 0; i < data_length; i++) {
        sum += calibration_data[i];
        sum_sq += calibration_data[i] * calibration_data[i];
    }
    
    double mean = sum / data_length;
    double variance = (sum_sq / data_length) - (mean * mean);
    double std_dev = sqrt(variance);
    
    // Set jerk limit to 3 standard deviations for 99.7% confidence
    state->jerk_limit_crit = 3.0 * std_dev;
    
    if (state->jerk_limit_crit < 1.0) {
        state->jerk_limit_crit = 1.0; // Minimum threshold
    }
    
    g_last_error = REASON_SUCCESS;
    return REASON_SUCCESS;
}
