#include "fixed_point.h"

/*=================================================
* Newton-Raphson Iteration Constants
===================================================*/

#define RECIPROCAL_ITERATIONS 4

#define SQRT_ITERATIONS 6

#define SQRT_CONVERGENCE 10

/*=================================================
* Trigonometric Function Constants
===================================================*/

/* 1/3! = 1/6 in fixed-point*/
#define FACTORIAL_3_INV (10923) // Approximation of 1/6 in Q16.16 format (1/6 * 65536)

/* 1/4! = 1/24 in fixed-point*/
#define FACTORIAL_4_INV (2731) // Approximation of 1/24

/* 1/5! = 1/120 in fixed-point*/
#define FACTORIAL_5_INV (546) // Approximation of 1/120

/* 1/6! = 1/720 in fixed-point*/
#define FACTORIAL_6_INV (91) // Approximation of 1/720

/* 1/7! = 1/5040 in fixed-point*/
#define FACTORIAL_7_INV (8) // Approximation of 1/5040 (Real value is 0.000122 (might loose precision))

/* Threshold for tangent overflow protection */
#define TAN_THRESHOLD (FIXED_POINT_PI_OVER_2 - (FIXED_POINT_ONE >> 4)) // Slightly less than pi/2 to avoid overflow

#define TAN_MAXIMUM (fixed_point_t)(1 << 30) // Maximum value for tangent to prevent overflow


/*=================================================
* Helper: Normalize angle to the range [-pi, pi]
* This ensures Taylor series approximations for sine and cosine are more accurate
===================================================*/

static inline fixed_point_t normalize_angle(fixed_point_t angle){

    while (angle > FIXED_POINT_PI){
        angle = fixed_point_sub(angle, FIXED_POINT_TWO_PI);
    }
    while (angle < -FIXED_POINT_PI){
        angle = fixed_point_add(angle, FIXED_POINT_TWO_PI);
    }
    return angle;
}

/* Reciprocal: 1/x using newton-Raphson*/
fixed_point_t fixed_point_reciprocal(fixed_point_t a){
    if(a == 0){
        return 0;
    }

    /*Handle sign separately*/
    int32_t sign = (a < 0) ? -1 : 1;
    fixed_point_t abs_a = fixed_point_abs(a);

    /*Initial guess using bit shifting to estimate 1/a
    * The idea is to find the position of the highest set bit in abs_a, which gives us an estimate of its magnitude.
    * We can then use this to create an initial guess for the reciprocal. */
    int lead_zeros = 0;
    int32_t temp = abs_a;
    while (!(temp & 0x40000000) && lead_zeros < 31) {
        temp <<= 1;
        lead_zeros++;
    }

    /* estimate ≈ 2^(lead_zeros) shifted into Q16.16 range */
    fixed_point_t estimate = (fixed_point_t)(1 << lead_zeros);

    /* Newton-Raphson Iteration */
    for (int iteration = 0; iteration < RECIPROCAL_ITERATIONS; iteration++){
        
        /* Correction = 1 - value * estimate*/
        fixed_point_t product = fixed_point_mul(abs_a, estimate);
        fixed_point_t correction = fixed_point_sub(FIXED_POINT_ONE, product);
        
        /* Update estimate: estimate = estimate + estimate * correction*/
        fixed_point_t adjustment = fixed_point_mul(estimate, correction);
        estimate = fixed_point_add(estimate, adjustment);
    }

    return (sign < 0) ? fixed_point_neg(estimate) : estimate;
}

/*=================================================
* Square Root: sqrt(x) using newton-Raphson 
===================================================*/

fixed_point_t fixed_point_sqrt(fixed_point_t a){
    /*Zero and negative input handling*/
    if(a <= 0){
        return 0;
    }

    /*Initial guess using bit shifting to estimate 1/sqrt(a)*/
    int lead_zeros = 0;
    int32_t temp = a;
    while(!(temp & 0x40000000) && lead_zeros < 31) {
        temp <<= 1;
        lead_zeros++;
    }

    /* Inverse sqrt initial estimate to avoid division*/
    fixed_point_t y = (fixed_point_t)(1 << (9 + (lead_zeros >> 1)));

    /* 3.0 in Q16.16 */
    fixed_point_t three = (fixed_point_t)(3 << 16);

    /* Newton-Raphson for inverse sqrt:
    *   y_{n+1} = (y_n >> 1) * (3 - a * y_n^2)*/
    for (int i = 0; i < SQRT_ITERATIONS; i++){
        fixed_point_t y_sq   = fixed_point_mul(y, y);
        fixed_point_t a_y_sq = fixed_point_mul(a, y_sq);
        fixed_point_t factor = fixed_point_sub(three, a_y_sq);

        fixed_point_t next_y = fixed_point_mul(y >> 1, factor);

        /* Early exit if converged within threshold */
        if(fixed_point_abs(fixed_point_sub(next_y, y)) < SQRT_CONVERGENCE){
            y = next_y;
            break;
        }
        y = next_y;
    }

    /* sqrt(a) = a * (1/sqrt(a)) */
    return fixed_point_mul(a, y);
}

/*=================================================
* Sine: sin(x) using Taylor series
* sin(x) = x - x^3/3! + x^5/5! - x^7/7! + ...
===================================================*/

fixed_point_t fixed_point_sin(fixed_point_t angle) {
    /* Normalize angle to range [-pi, pi] */
    angle = normalize_angle(angle);

    /* Compute power of angle*/
    fixed_point_t angle_sq = fixed_point_mul(angle, angle);

    /* Initialize result with the first term of the series */
    fixed_point_t result = angle;

    /* Compute and add subsequent terms */
    fixed_point_t term = angle;
    term = fixed_point_mul(term, angle_sq); // x^3
    result = fixed_point_sub(result, fixed_point_mul(term, FACTORIAL_3_INV)); // - x^3/3!

    /* Compute x^5 term */
    term = fixed_point_mul(term, angle_sq); // x^5
    result = fixed_point_add(result, fixed_point_mul(term, FACTORIAL_5_INV)); // + x^5/5!

    /* Compute x^7 term */
    term = fixed_point_mul(term, angle_sq); // x^7
    result = fixed_point_sub(result, fixed_point_mul(term, FACTORIAL_7_INV)); // - x^7/7!
    
    return result;
}

/*=================================================
* Cosine: cos(x) using Taylor series
* cos(x) = 1 - x^2/2! + x^4/4! - x^6/6! + ...
===================================================*/

fixed_point_t fixed_point_cos(fixed_point_t angle) {
    /* Normalize angle to range [-pi, pi] */
    angle = normalize_angle(angle);

    /* Compute power of angle*/
    fixed_point_t angle_sq = fixed_point_mul(angle, angle);

    /* Initialize result with the first term of the series */
    fixed_point_t result = FIXED_POINT_ONE;

    /* Compute and add subsequent terms */
    fixed_point_t term = angle_sq; // x^2
    result = fixed_point_sub(result, fixed_point_mul(term, FACTORIAL_4_INV)); // - x^2/2!

    /* Compute x^4 term */
    term = fixed_point_mul(term, angle_sq); // x^4
    result = fixed_point_add(result, fixed_point_mul(term, FACTORIAL_4_INV)); // + x^4/4!

    /* Compute x^6 term */
    term = fixed_point_mul(term, angle_sq); // x^6
    result = fixed_point_sub(result, fixed_point_mul(term, FACTORIAL_6_INV)); // - x^6/6!

    return result;
}

/*=================================================
* Tangent: tan(x) = sin(x) / cos(x)
* Includes overflow protection for angles near pi/2
===================================================*/

fixed_point_t fixed_point_tan(fixed_point_t angle) {
    /* Normalize angle to range [-pi, pi] */
    angle = normalize_angle(angle);

    /* Check for angles near pi/2 or -pi/2 to prevent overflow */
    if (fixed_point_abs(fixed_point_sub(angle, FIXED_POINT_PI_OVER_2)) < TAN_THRESHOLD) {
        return (angle > 0) ? TAN_MAXIMUM : fixed_point_neg(TAN_MAXIMUM);
    } 
    if(fixed_point_abs(fixed_point_add(angle, FIXED_POINT_PI_OVER_2)) < TAN_THRESHOLD) {
        return (angle > 0) ? TAN_MAXIMUM : fixed_point_neg(TAN_MAXIMUM);
    }

    fixed_point_t sin_val = fixed_point_sin(angle);
    fixed_point_t cos_val = fixed_point_cos(angle);
    return fixed_point_div(sin_val, cos_val);
}


/*=================================================
* Integer Power
/*===============================================*/

fixed_point_t fixed_point_pow(fixed_point_t base, int exponent) {
        if(exponent == 0) {
            return FIXED_POINT_ONE;
        }

        if (exponent == 1) {
            return base;
        }

        if(exponent == -1) {
            return fixed_point_reciprocal(base);
        }

        if(exponent == 2) {
            return fixed_point_mul(base, base);
        }

        fixed_point_t result = FIXED_POINT_ONE;
        int abs_exponent = (exponent < 0) ? -exponent : exponent;

        fixed_point_t current_product = base;
        while (abs_exponent > 0) {
            if(abs_exponent & 1) {
                result = fixed_point_mul(result, current_product);
            }
            current_product = fixed_point_mul(current_product, current_product);
            abs_exponent >>= 1;
        }

        /* For negative exponents: base^(-n) = 1 / base^n */
        if(exponent < 0) {
            result = fixed_point_reciprocal(result);
        }

        return result;
}
