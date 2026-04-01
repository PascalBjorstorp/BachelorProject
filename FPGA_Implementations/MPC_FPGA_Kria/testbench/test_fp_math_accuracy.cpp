#include <cmath>
#include <cstdint>
#include <cstdio>

#include "../include/fp_math_hls.h"

static double q16_to_double(fixed_point_t v)
{
    return static_cast<double>(v) / static_cast<double>(FP_ONE);
}

static fixed_point_t double_to_q16(double v)
{
    double scaled = v * static_cast<double>(FP_ONE);
    if (scaled >= 0.0) scaled += 0.5;
    else scaled -= 0.5;
    if (scaled > static_cast<double>(INT32_MAX)) scaled = static_cast<double>(INT32_MAX);
    if (scaled < static_cast<double>(INT32_MIN)) scaled = static_cast<double>(INT32_MIN);
    return static_cast<fixed_point_t>(scaled);
}

int main()
{
    int failures = 0;

    double max_recip_abs_err = 0.0;
    for (double x = -16.0; x <= 16.0; x += 0.25) {
        if (std::fabs(x) < 0.5) {
            continue;
        }
        fixed_point_t x_fp = double_to_q16(x);
        fixed_point_t r_fp = fp_recip(x_fp);
        double r = q16_to_double(r_fp);
        double expected = 1.0 / x;
        double err = std::fabs(r - expected);
        if (err > max_recip_abs_err) max_recip_abs_err = err;
    }

    double max_sin_abs_err = 0.0;
    double max_cos_abs_err = 0.0;
    for (double a = -3.141592653589793; a <= 3.141592653589793; a += 0.01) {
        fixed_point_t a_fp = double_to_q16(a);
        double s = q16_to_double(fp_sin(a_fp));
        double c = q16_to_double(fp_cos(a_fp));
        double se = std::fabs(s - std::sin(a));
        double ce = std::fabs(c - std::cos(a));
        if (se > max_sin_abs_err) max_sin_abs_err = se;
        if (ce > max_cos_abs_err) max_cos_abs_err = ce;
    }

    double max_atan_abs_err = 0.0;
    for (double x = -6.0; x <= 6.0; x += 0.01) {
        fixed_point_t x_fp = double_to_q16(x);
        double a = q16_to_double(fp_atan(x_fp));
        double expected = std::atan(x);
        double err = std::fabs(a - expected);
        if (err > max_atan_abs_err) max_atan_abs_err = err;
    }

    const double recip_limit = 0.13;
    const double sin_limit = 0.05;
    const double cos_limit = 0.05;
    const double atan_limit = 0.12;

    if (max_recip_abs_err > recip_limit) {
        std::printf("FAIL recip max abs err %.6f > %.6f\n", max_recip_abs_err, recip_limit);
        failures++;
    }
    if (max_sin_abs_err > sin_limit) {
        std::printf("FAIL sin max abs err %.6f > %.6f\n", max_sin_abs_err, sin_limit);
        failures++;
    }
    if (max_cos_abs_err > cos_limit) {
        std::printf("FAIL cos max abs err %.6f > %.6f\n", max_cos_abs_err, cos_limit);
        failures++;
    }
    if (max_atan_abs_err > atan_limit) {
        std::printf("FAIL atan max abs err %.6f > %.6f\n", max_atan_abs_err, atan_limit);
        failures++;
    }

    std::printf("recip max abs err: %.6f\n", max_recip_abs_err);
    std::printf("sin   max abs err: %.6f\n", max_sin_abs_err);
    std::printf("cos   max abs err: %.6f\n", max_cos_abs_err);
    std::printf("atan  max abs err: %.6f\n", max_atan_abs_err);

    if (failures == 0) {
        std::printf("PASS test_fp_math_accuracy\n");
        return 0;
    }

    return 1;
}
