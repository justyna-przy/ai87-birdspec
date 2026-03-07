#ifndef SOFTMAX_H
#define SOFTMAX_H

#include <stdint.h>
#include "arm_math.h"

/* Declared in generated softmax.c */
void softmax_q17p14_q15(const q31_t *vec_in, const uint16_t dim_vec, q15_t *p_out);
void softmax_shift_q17p14_q15(q31_t *vec_in, const uint16_t dim_vec,
                               uint8_t in_shift, q15_t *p_out);

#endif /* SOFTMAX_H */
