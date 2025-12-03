#ifndef REASON_SOLVER_H
#define REASON_SOLVER_H

#include "ternary_core.h"

/* The Stability Envelope State */
typedef struct {
    double jerk_limit_crit;   // J_crit
    double current_acc;
    double last_acc;
    double last_time;
    int resonance_flag;       // 0 = Safe, 1 = Warning, 2 = Critical
    
    // Enhanced state tracking
    double jerk_history[8];   // Circular buffer for jerk history
    size_t history_index;     // Current index in history buffer
    double avg_jerk;          // Average jerk over history window
    double resonance_energy;  // Accumulated resonance energy
    int stability_score;      // Overall system stability (0-100)
    
    // Ternary dampening state
    trit_t ternary_buffer[4]; // Ternary representation buffer
    double dampening_factor;  // Current dampening coefficient
    
} reason_state_t;

/* Error codes for reason solver operations */
typedef enum {
    REASON_SUCCESS = 0,
    REASON_ERROR_NULL_PTR = -1,
    REASON_ERROR_INVALID_PARAMS = -2,
    REASON_ERROR_MALLOC_FAIL = -3,
    REASON_ERROR_COMPUTATION = -4
} reason_error_t;

/* API */
reason_state_t* reason_init(double j_crit);
reason_error_t reason_reset(reason_state_t* state);
void reason_destroy(reason_state_t* state);

/* Enhanced API with error handling and diagnostics */
reason_error_t reason_get_last_error(void);
double reason_compute_safety_ex(reason_state_t* state, double acc_raw, double dt, double* stability_metric);
double reason_get_stability_score(const reason_state_t* state);
int reason_get_resonance_flag(const reason_state_t* state);
double reason_get_resonance_energy(const reason_state_t* state);
reason_error_t reason_calibrate(reason_state_t* state, const double* calibration_data, size_t data_length);

/**
 * Main Control Loop Step (O(1) complexity)
 * @param state Pointer to algorithm state
 * @param acc_raw Raw accelerometer data (m/s^2)
 * @param dt Time delta (seconds)
 * @return Corrective thrust delta (if any)
 */
double reason_compute_safety(reason_state_t* state, double acc_raw, double dt);

#endif // REASON_SOLVER_H
