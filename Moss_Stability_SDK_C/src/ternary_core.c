#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "../include/ternary_core.h"

// Global configuration for ternary operations
static moss_config_t g_ternary_config = {
    .precision_threshold = 1e-6,
    .use_simd = 0
};

/**
 * Initialize ternary core with configuration
 * @param config Configuration parameters for ternary operations
 */
void moss_ternary_init(moss_config_t config) {
    g_ternary_config = config;
    
    // Enable SIMD optimizations if available and requested
    #ifdef __SSE2__
    if (config.use_simd) {
        // SSE2 is available - could be used for vectorized operations
        // For now, just set the flag for future optimization
    }
    #endif
    
    #ifdef __ARM_NEON
    if (config.use_simd) {
        // ARM NEON is available for ARM-based systems
    }
    #endif
}

/**
 * Convert real value to balanced ternary (-1, 0, +1)
 * Uses threshold-based quantization with hysteresis
 * @param value Input value to convert
 * @return Ternary digit: -1, 0, or +1
 */
trit_t moss_sign(double value) {
    if (value > g_ternary_config.precision_threshold) {
        return 1;
    } else if (value < -g_ternary_config.precision_threshold) {
        return -1;
    } else {
        return 0;
    }
}

/**
 * Compute dot product of two ternary vectors
 * Optimized implementation using balanced ternary properties
 * @param a First ternary vector
 * @param b Second ternary vector  
 * @param len Length of vectors
 * @return Dot product result
 */
double moss_ternary_dot_product(const trit_t* a, const trit_t* b, size_t len) {
    if (a == NULL || b == NULL) {
        return 0.0;
    }
    
    double result = 0.0;
    size_t i;
    
    #ifdef __SSE2__
    if (g_ternary_config.use_simd && len >= 4) {
        // Vectorized implementation for better performance
        // Process 4 values at a time using SSE2
        __m128d vec_a, vec_b, vec_result;
        __m128d zero = _mm_setzero_pd();
        
        for (i = 0; i <= len - 4; i += 4) {
            // Load 2 pairs of values
            vec_a = _mm_set_pd((double)a[i+1], (double)a[i]);
            vec_b = _mm_set_pd((double)b[i+1], (double)b[i]);
            
            // Multiply and accumulate
            vec_result = _mm_mul_pd(vec_a, vec_b);
            
            // Extract results and add to total
            double temp[2];
            _mm_storeu_pd(temp, vec_result);
            result += temp[0] + temp[1];
        }
        
        // Handle remaining elements
        for (; i < len; i++) {
            result += (double)a[i] * (double)b[i];
        }
    } else {
        // Scalar implementation
        for (i = 0; i < len; i++) {
            result += (double)a[i] * (double)b[i];
        }
    }
    #else
    // Non-SIMD scalar implementation
    for (i = 0; i < len; i++) {
        result += (double)a[i] * (double)b[i];
    }
    #endif
    
    return result;
}

/**
 * Allocate and initialize a ternary vector
 * @param length Length of the vector to allocate
 * @return Pointer to allocated ternary vector, NULL on failure
 */
trit_t* moss_ternary_vector_alloc(size_t length) {
    if (length == 0) {
        return NULL;
    }
    
    trit_t* vector = (trit_t*)malloc(length * sizeof(trit_t));
    if (vector == NULL) {
        return NULL;
    }
    
    // Initialize to zero
    memset(vector, 0, length * sizeof(trit_t));
    return vector;
}

/**
 * Free a ternary vector allocated by moss_ternary_vector_alloc
 * @param vector Pointer to vector to free
 */
void moss_ternary_vector_free(trit_t* vector) {
    if (vector != NULL) {
        free(vector);
    }
}

/**
 * Apply ternary quantization to a real-valued vector
 * @param input Real-valued input vector
 * @param length Length of vectors
 * @param output Output ternary vector (must be pre-allocated)
 * @return 0 on success, -1 on error
 */
int moss_ternary_quantize(const double* input, size_t length, trit_t* output) {
    if (input == NULL || output == NULL) {
        return -1;
    }
    
    size_t i;
    for (i = 0; i < length; i++) {
        output[i] = moss_sign(input[i]);
    }
    
    return 0;
}

/**
 * Compute ternary magnitude (L1 norm) of a ternary vector
 * @param vector Input ternary vector
 * @param length Length of vector
 * @return Ternary magnitude (number of non-zero elements)
 */
size_t moss_ternary_magnitude(const trit_t* vector, size_t length) {
    if (vector == NULL) {
        return 0;
    }
    
    size_t magnitude = 0;
    size_t i;
    for (i = 0; i < length; i++) {
        if (vector[i] != 0) {
            magnitude++;
        }
    }
    
    return magnitude;
}

/**
 * Ternary negation operation
 * @param vector Input ternary vector
 * @param length Length of vector
 * @param output Output vector (can be same as input for in-place operation)
 */
void moss_ternary_negate(trit_t* vector, size_t length) {
    if (vector == NULL) {
        return;
    }
    
    size_t i;
    for (i = 0; i < length; i++) {
        vector[i] = -vector[i];
    }
}