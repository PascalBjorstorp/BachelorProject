#include <cstdint>
#include <cstdio>

#include "../include/fp_math_hls.h"

static fixed_point_t legacy_mul(fixed_point_t a, fixed_point_t b)
{
    return static_cast<fixed_point_t>((static_cast<int64_t>(a) * static_cast<int64_t>(b)) >> FP_FRAC_BITS);
}

static fixed_point_t legacy_div(fixed_point_t a, fixed_point_t b)
{
    if (a == 0 || b == 0) return 0;
    return static_cast<fixed_point_t>((static_cast<int64_t>(a) << FP_FRAC_BITS) / static_cast<int64_t>(b));
}

static fixed_point_t q16_from_float(float x)
{
    float scaled = x * static_cast<float>(FP_ONE);
    if (scaled >= 0.0f) scaled += 0.5f;
    else scaled -= 0.5f;
    if (scaled > static_cast<float>(INT32_MAX)) scaled = static_cast<float>(INT32_MAX);
    if (scaled < static_cast<float>(INT32_MIN)) scaled = static_cast<float>(INT32_MIN);
    return static_cast<fixed_point_t>(scaled);
}

int main()
{
    int failures = 0;
    int max_mul_lsb = 0;
    int max_div_lsb = 0;
    float max_mul_a = 0.0f, max_mul_b = 0.0f;
    float max_div_a = 0.0f, max_div_b = 0.0f;

    for (float af = -8.0f; af <= 8.0f; af += 0.125f) {
        for (float bf = -8.0f; bf <= 8.0f; bf += 0.125f) {
            fixed_point_t a = q16_from_float(af);
            fixed_point_t b = q16_from_float(bf);

            fixed_point_t mul_new = fp_mul(a, b);
            fixed_point_t mul_old = legacy_mul(a, b);
            int mul_diff = static_cast<int>(mul_new - mul_old);
            if (mul_diff < 0) mul_diff = -mul_diff;
            if (mul_diff > max_mul_lsb) {
                max_mul_lsb = mul_diff;
                max_mul_a = af;
                max_mul_b = bf;
            }

            if (b != 0) {
                fixed_point_t div_new = fp_div(a, b);
                fixed_point_t div_old = legacy_div(a, b);
                int div_diff = static_cast<int>(div_new - div_old);
                if (div_diff < 0) div_diff = -div_diff;
                if (div_diff > max_div_lsb) {
                    max_div_lsb = div_diff;
                    max_div_a = af;
                    max_div_b = bf;
                }
            }
        }
    }

    const int mul_lsb_limit = 24;
    const int div_lsb_limit = 256;

    if (max_mul_lsb > mul_lsb_limit) {
        std::printf("FAIL mul backend drift: %d LSB > %d LSB\n", max_mul_lsb, mul_lsb_limit);
        failures++;
    }
    if (max_div_lsb > div_lsb_limit) {
        std::printf("FAIL div backend drift: %d LSB > %d LSB\n", max_div_lsb, div_lsb_limit);
        failures++;
    }

    std::printf("max mul drift: %d LSB\n", max_mul_lsb);
    std::printf("max mul at: a=%.3f b=%.3f\n", max_mul_a, max_mul_b);
    std::printf("max div drift: %d LSB\n", max_div_lsb);
    std::printf("max div at: a=%.3f b=%.3f\n", max_div_a, max_div_b);

    if (failures == 0) {
        std::printf("PASS test_fp_backend_consistency\n");
        return 0;
    }

    return 1;
}
