#pragma once

#include <cstdint>
#include <optional>

namespace interception {

/// PCG32 (O'Neill 2014) with explicitly specified derived distributions.
///
/// This is a line-for-line counterpart of `src/interception/rng.py`. Both
/// must produce identical streams, because "same seed, same trajectory" is
/// the simulator's central guarantee and it has to hold across the language
/// boundary.
///
/// Nothing here delegates to <random>. `std::uniform_real_distribution` and
/// `std::uniform_int_distribution` are explicitly implementation-defined --
/// libstdc++ and libc++ return different values from the same engine and
/// seed, and the standard allows it. Every distribution is therefore spelled
/// out, and the test suite pins the resulting stream.
class Pcg32 {
public:
    static constexpr std::uint64_t kMultiplier = 6364136223846793005ULL;
    static constexpr std::uint64_t kDefaultStream = 1442695040888963407ULL;

    explicit Pcg32(std::uint64_t seed = 0, std::uint64_t stream = kDefaultStream) {
        reseed(seed, stream);
    }

    /// Standard PCG seeding: step, add the seed, step again.
    void reseed(std::uint64_t seed, std::uint64_t stream = kDefaultStream) {
        cachedGauss_.reset();
        state_ = 0;
        inc_ = (stream << 1) | 1ULL;
        nextU32();
        state_ += seed;
        nextU32();
    }

    /// Advance the LCG and return the permuted 32-bit output.
    std::uint32_t nextU32() {
        const std::uint64_t old = state_;
        state_ = old * kMultiplier + inc_;
        const auto xorshifted =
            static_cast<std::uint32_t>(((old >> 18) ^ old) >> 27);
        const auto rot = static_cast<std::uint32_t>(old >> 59);
        return (xorshifted >> rot) | (xorshifted << ((32 - rot) & 31));
    }

    /// A double in [0, 1) with 53 bits of entropy, assembled from two draws
    /// the same way CPython builds its own random().
    double nextDouble() {
        const std::uint32_t a = nextU32() >> 5;  // 27 bits
        const std::uint32_t b = nextU32() >> 6;  // 26 bits
        return (a * 67108864.0 + b) / 9007199254740992.0;
    }

    double uniform(double low, double high) {
        return low + (high - low) * nextDouble();
    }

    /// Uniform integer in [low, high], inclusive at both ends.
    ///
    /// PCG's bounded method: reject the smallest `2^32 % bound` outputs so
    /// the rest divides evenly. Plain modulo would bias toward low values.
    int randint(int low, int high);

    /// Normal variate by the Marsaglia polar method.
    ///
    /// The method yields two variates per accepted pair; the spare is
    /// cached. The cache is part of generator state and is cleared on
    /// reseed, otherwise a reset would not reproduce the stream.
    double gauss(double mu = 0.0, double sigma = 1.0);

    bool hasCachedGauss() const { return cachedGauss_.has_value(); }

private:
    std::uint64_t state_ = 0;
    std::uint64_t inc_ = 0;
    std::optional<double> cachedGauss_;
};

}  // namespace interception
