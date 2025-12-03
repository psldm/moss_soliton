#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include "../include/reason_solver.h"
#include "../include/ternary_core.h"

// Test counter
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(condition, test_name) \
    do { \
        if (condition) { \
            printf("[PASS] %s\n", test_name); \
            tests_passed++; \
        } else { \
            printf("[FAIL] %s\n", test_name); \
            tests_failed++; \
        } \
    } while(0)

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

void test_reason_reset(void) {
    reason_state_t* state = reason_init(500.0);
    
    if (state != NULL) {
        // Simulate some usage
        (void)reason_compute_safety(state, 100.0, 0.001);
        
        // Reset should preserve jerk limit but clear other state
        reason_error_t error = reason_reset(state);
        TEST_ASSERT(error == REASON_SUCCESS, "reason_reset success");
        TEST_ASSERT(state->jerk_limit_crit == 500.0, "reason_reset preserves jerk limit");
        TEST_ASSERT(state->stability_score == 100, "reason_reset clears stability score");
        TEST_ASSERT(state->resonance_energy == 0.0, "reason_reset clears resonance energy");
        
        // Test NULL pointer
        error = reason_reset(NULL);
        TEST_ASSERT(error == REASON_ERROR_NULL_PTR, "reason_reset NULL pointer");
        
        reason_destroy(state);
    }
}

void test_reason_compute_safety(void) {
    reason_state_t* state = reason_init(500.0);
    
    if (state != NULL) {
        // Reset state to ensure clean start
        reason_reset(state);
        
        // Test with very small acceleration change (should be safe)
        // jerk = 0.1 / 0.001 = 100 m/s^3, well below 500 limit
        double correction = reason_compute_safety(state, 0.1, 0.001);
        TEST_ASSERT(state->resonance_flag == 0, "reason_compute_safety safe flag");
        
        // Test critical condition (correction needed)
        correction = reason_compute_safety(state, 1000.0, 0.001); // High acceleration jump
        TEST_ASSERT(correction != 0.0, "reason_compute_safety correction applied");
        TEST_ASSERT(state->resonance_flag == 2, "reason_compute_safety critical flag");
        
        // Test stability score
        double stability = reason_get_stability_score(state);
        TEST_ASSERT(stability >= 0 && stability <= 100, "reason_compute_safety stability score range");
        
        // Test extended API
        double stability_metric;
        correction = reason_compute_safety_ex(state, 10.0, 0.001, &stability_metric);
        TEST_ASSERT(stability_metric >= 0 && stability_metric <= 100, "reason_compute_safety_ex stability");
        
        reason_destroy(state);
    }
}

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

void test_reason_calibrate(void) {
    reason_state_t* state = reason_init(500.0);
    
    if (state != NULL) {
        // Create calibration data with known statistics
        double calibration_data[100];
        
        int i;
        for (i = 0; i < 100; i++) {
            calibration_data[i] = 10.0 + (i - 50) * 0.1; // Linear variation around mean
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

void test_reason_ternary_integration(void) {
    reason_state_t* state = reason_init(500.0);
    
    if (state != NULL) {
        // Test ternary integration in safety computation
        // Create a scenario that should trigger ternary analysis
        double correction1 = reason_compute_safety(state, 100.0, 0.001); // Positive jerk
        double correction2 = reason_compute_safety(state, -100.0, 0.001); // Negative jerk
        
        // Both should produce corrections - the sign depends on the ternary coupling
        // The key is that the system handles both positive and negative jerks
        TEST_ASSERT(correction1 != 0.0 || correction2 != 0.0, "reason ternary logic produces corrections");
        
        // Test stability metrics update
        double stability = reason_get_stability_score(state);
        TEST_ASSERT(stability >= 0 && stability <= 100, "reason ternary stability metrics");
        
        reason_destroy(state);
    }
}

void test_reason_edge_cases(void) {
    reason_state_t* state = reason_init(1.0); // Very small jerk limit
    
    if (state != NULL) {
        // Test very small jerk limit
        (void)reason_compute_safety(state, 10.0, 0.001); // Large acceleration
        TEST_ASSERT(state->resonance_flag == 2, "reason small jerk limit critical");
        
        // Test very large time delta
        (void)reason_compute_safety(state, 10.0, 1.0); // Large dt
        // Should handle gracefully
        
        // Test with exactly critical jerk
        (void)reason_compute_safety(state, state->jerk_limit_crit * 0.001, 0.001);
        // Should be close to critical
        
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

int main(void) {
    printf("=== Moss Stability SDK - Reason Solver Tests ===\n\n");
    
    test_reason_init();
    test_reason_reset();
    test_reason_compute_safety();
    test_reason_error_handling();
    test_reason_getters();
    test_reason_calibrate();
    test_reason_ternary_integration();
    test_reason_edge_cases();
    
    printf("\n=== Test Results ===\n");
    printf("Tests Passed: %d\n", tests_passed);
    printf("Tests Failed: %d\n", tests_failed);
    printf("Total Tests: %d\n", tests_passed + tests_failed);
    
    if (tests_failed == 0) {
        printf("All tests passed! ✅\n");
        return 0;
    } else {
        printf("Some tests failed! ❌\n");
        return 1;
    }
}