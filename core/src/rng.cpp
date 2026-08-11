#include "interception/rng.hpp"

#include <cmath>
#include <stdexcept>

namespace interception {

int Pcg32::randint(int low, int high) {
    if (high < low) {
        throw std::invalid_argument("randint requires high >= low");
    }
    const auto bound = static_cast<std::uint32_t>(high - low + 1);
    // 2^32 % bound, computed without needing 64-bit intermediates.
    const std::uint32_t threshold = (-bound) % bound;
    for (;;) {
        const std::uint32_t candidate = nextU32();
        if (candidate >= threshold) {
            return low + static_cast<int>(candidate % bound);
        }
    }
}

double Pcg32::gauss(double mu, double sigma) {
    if (cachedGauss_.has_value()) {
        const double spare = *cachedGauss_;
        cachedGauss_.reset();
        return mu + sigma * spare;
    }

    double u = 0.0;
    double v = 0.0;
    double s = 0.0;
    do {
        u = 2.0 * nextDouble() - 1.0;
        v = 2.0 * nextDouble() - 1.0;
        s = u * u + v * v;
    } while (s <= 0.0 || s >= 1.0);

    const double scale = std::sqrt(-2.0 * std::log(s) / s);
    cachedGauss_ = v * scale;
    return mu + sigma * (u * scale);
}

}  // namespace interception
