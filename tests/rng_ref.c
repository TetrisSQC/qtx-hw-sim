#include <stdio.h>
#include <stdint.h>
#include <math.h>
static uint64_t rng;
static double noise(void) {
    double sum = 0;
    for (int i = 0; i < 4; i++) {
        rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
        sum += (double)(rng >> 11) / (double)(1ULL << 53);
    }
    return (sum - 2.0) * 1.732 * 0.08;
}
static double threshold(int pin) {
    uint32_t h = (uint32_t)pin * 2654435761u;
    h ^= h >> 15;
    return 1.4 + ((double)(h & 1023) / 1023.0 * 2 - 1) * 0.25;
}
int main(void) {
    rng = 1 * 0x9E3779B97F4A7C15ULL + 1;
    printf("seed %016llx\n", (unsigned long long)rng);
    for (int i = 0; i < 5; i++) printf("noise %.17g\n", noise());
    printf("rng %016llx\n", (unsigned long long)rng);
    for (int p = 0; p < 2000; p += 397) printf("th %d %.17g\n", p, threshold(p));
    printf("drift %.17g\n", 0 + (5.0 - 0) * exp(-123456.0 / 300000.0));
    return 0;
}
