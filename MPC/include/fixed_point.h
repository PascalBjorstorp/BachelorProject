/*
This module provides Q16.16 fixed-point arithmetic operations.
All operations avoid floating-point hardware.

Format: Q16.16 fixed-point format
- 16 bits for the integer part
- 16 bits for the fractional part
- Total: 32 bits (int32_t)
- Range: -32768.0 to 32767.99998474121
- precision: 1/65536 (approximately 0.0000152587890625)

Qucik note: All operations use intetger arithmetic.

*/

#ifndef FIXED_POINT_H
#define FIXED_POINT_H

#include <stdint.h>

/*=================================================
Type Definitions 
===================================================*/

/* Fixed-point number type */
typedef int32_t fixed_point_t;


/*=================================================
Constants
===================================================*/

#define FIXED_POINT_FRACTIONAL_BITS 16

#define FIXED_POINT_ONE (1 << FIXED_POINT_FRACTIONAL_BITS)

#define FIXED_POINT_HALF (FIXED_POINT_ONE >> 1)

#define FIXED_POINT_PI (205887) // Approximation of pi in Q16.16 format (3.14159265358979323846 * 65536)

#define FIXED_POINT_PI_OVER_2 (102943) 

#define FIXED_POINT_TWO_PI (411775) 

/*=================================================
Basic Arithmetic Operations
===================================================*/
static inline fixed_point_t fixed_point_add(fixed_point_t a, fixed_point_t b) {
    return a + b;
}

static inline fixed_point_t fixed_point_sub(fixed_point_t a, fixed_point_t b) {
    return a - b;
}

static inline fixed_point_t fixed_point_mul(fixed_point_t a, fixed_point_t b) {
    return (int64_t)a * b >> FIXED_POINT_FRACTIONAL_BITS;
}
/*Must be remade at a later time for best FPGA performance*/
static inline fixed_point_t fixed_point_div(fixed_point_t a, fixed_point_t b) {
    if (a == 0) {
        return 0; // Return zero if the numerator is zero
    }
    int64_t scaled_numerator = (int64_t)a << FIXED_POINT_FRACTIONAL_BITS;
    return (fixed_point_t)(scaled_numerator / b);
}

/*=================================================
Unary Operations
===================================================*/

static inline fixed_point_t fixed_point_abs(fixed_point_t a) {
    return (a < 0) ? -a : a;
}

static inline fixed_point_t fixed_point_neg(fixed_point_t a) {
    return -a;
}

/*=================================================
Comparison and Clamping
===================================================*/

static inline fixed_point_t fixed_point_min(fixed_point_t a, fixed_point_t b) {
    return (a < b) ? a : b;
}

static inline fixed_point_t fixed_point_max(fixed_point_t a, fixed_point_t b) {
    return (a > b) ? a : b;
}

static inline fixed_point_t fixed_point_clamp(fixed_point_t value, fixed_point_t min, fixed_point_t max){
    if(value < min) return min;
    if(value > max) return max;
    return value;
}

/*=================================================
Advanced Math Functions
===================================================*/

/*Reciprocal: result = 1 / value)*/
fixed_point_t fixed_point_reciprocal(fixed_point_t a);

/*Square Root: result = sqrt(value)*/
fixed_point_t fixed_point_sqrt(fixed_point_t a);

/*Sine: Result = sin(angle)
* Uses Taylor series approximation for sine function
*/
fixed_point_t fixed_point_sin(fixed_point_t angle);

/*Cosine: Result = cos(angle)
* Uses Taylor series approximation for cosine function
*/
fixed_point_t fixed_point_cos(fixed_point_t angle);

/*Tangente: Result = tan(angle)
* Computed as sin(angle) / cos(angle)
*/
fixed_point_t fixed_point_tan(fixed_point_t angle);

/*Integer power: result = base^exponent*/
fixed_point_t fixed_point_pow(fixed_point_t base, int exponent);


/*==============================================
Bit Shift Operations
===============================================*/

/*Left shift operation*/
#define fixed_point_shift_left(value, shift_amount) \
    ((fixed_point_t)((int64_t)(value) << (shift_amount)))

/*Right shift operation*/
#define fixed_point_shift_right(value, shift_amount) \
    ((fixed_point_t)((int64_t)(value) >> (shift_amount)))


#endif // FIXED_POINT_H