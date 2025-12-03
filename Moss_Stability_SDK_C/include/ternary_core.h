#ifndef TERNARY_CORE_H
#define TERNARY_CORE_H

#include <stdint.h>
#include <stddef.h>

/* Balanced Ternary Types (-1, 0, +1) */
typedef int8_t trit_t; 

/* Engine Configuration */
typedef struct {
    double precision_threshold;
    int use_simd;
} moss_config_t;

/* Core Operations */
void moss_ternary_init(moss_config_t config);
trit_t moss_sign(double value);
double moss_ternary_dot_product(const trit_t* a, const trit_t* b, size_t len);

/* Vector Operations */
trit_t* moss_ternary_vector_alloc(size_t length);
void moss_ternary_vector_free(trit_t* vector);
int moss_ternary_quantize(const double* input, size_t length, trit_t* output);
size_t moss_ternary_magnitude(const trit_t* vector, size_t length);
void moss_ternary_negate(trit_t* vector, size_t length);

#endif // TERNARY_CORE_H
