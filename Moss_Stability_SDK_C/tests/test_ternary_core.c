#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include "../include/ternary_core.h"
#include "../include/reason_solver.h"

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

void test_moss_ternary_init(void) {
    moss_config_t config = {
        .precision_threshold = 1e-3,
        .use_simd = 1
    };
    
    moss_ternary_init(config);
    TEST_ASSERT(1, "moss_ternary_init");
}

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

void test_moss_ternary_vector_alloc(void) {
    trit_t* vec = moss_ternary_vector_alloc(10);
    TEST_ASSERT(vec != NULL, "moss_ternary_vector_alloc success");
    
    if (vec != NULL) {
        // Check if initialized to zero
        int is_zero = 1;
        int i;
        for (i = 0; i < 10; i++) {
            if (vec[i] != 0) {
                is_zero = 0;
                break;
            }
        }
        TEST_ASSERT(is_zero, "moss_ternary_vector_alloc initialization");
        
        moss_ternary_vector_free(vec);
    }
    
    // Test zero length
    vec = moss_ternary_vector_alloc(0);
    TEST_ASSERT(vec == NULL, "moss_ternary_vector_alloc zero length");
}

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

void test_moss_ternary_magnitude(void) {
    trit_t vec[6] = {1, -1, 0, 1, 0, -1};
    size_t mag = moss_ternary_magnitude(vec, 6);
    TEST_ASSERT(mag == 4, "moss_ternary_magnitude");
    
    // Test with all zeros
    trit_t zero_vec[3] = {0, 0, 0};
    mag = moss_ternary_magnitude(zero_vec, 3);
    TEST_ASSERT(mag == 0, "moss_ternary_magnitude all zeros");
    
    // Test NULL pointer
    mag = moss_ternary_magnitude(NULL, 5);
    TEST_ASSERT(mag == 0, "moss_ternary_magnitude NULL");
}

void test_moss_ternary_negate(void) {
    trit_t vec[4] = {1, -1, 0, 1};
    trit_t original[4];
    memcpy(original, vec, sizeof(vec));
    
    moss_ternary_negate(vec, 4);
    
    TEST_ASSERT(vec[0] == -original[0], "moss_ternary_negate element 1");
    TEST_ASSERT(vec[1] == -original[1], "moss_ternary_negate element 2");
    TEST_ASSERT(vec[2] == -original[2], "moss_ternary_negate element 3");
    TEST_ASSERT(vec[3] == -original[3], "moss_ternary_negate element 4");
    
    // Test NULL pointer
    moss_ternary_negate(NULL, 4); // Should not crash
    TEST_ASSERT(1, "moss_ternary_negate NULL pointer");
}

int main(void) {
    printf("=== Moss Stability SDK - Ternary Core Tests ===\n\n");
    
    test_moss_ternary_init();
    test_moss_sign();
    test_moss_ternary_dot_product();
    test_moss_ternary_vector_alloc();
    test_moss_ternary_quantize();
    test_moss_ternary_magnitude();
    test_moss_ternary_negate();
    
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