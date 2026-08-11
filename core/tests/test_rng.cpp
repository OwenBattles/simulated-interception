#include <doctest/doctest.h>

#include <cmath>
#include <set>
#include <vector>

#include "interception/rng.hpp"

using namespace interception;

// These mirror tests/test_rng.py exactly. They are the cross-language
// contract: if either side changes, the two engines have silently forked
// even when every other test still passes.

TEST_CASE("matches the PCG32 reference vector") {
    // From the PCG reference distribution's pcg32-demo.c, seeded with
    // state 42 / sequence 54.
    const std::vector<std::uint32_t> expected = {0xA15C02B7, 0x7B47F409, 0xBA1D3330,
                                                 0x83D2F293, 0xBFA4784B, 0xCBED606E};
    Pcg32 rng(42, 54);
    for (std::uint32_t want : expected) {
        CHECK(rng.nextU32() == want);
    }
}

TEST_CASE("golden u32 stream on the project's default stream") {
    const std::vector<std::uint32_t> expected = {492690617,  1919685028, 3561993920,
                                                 683038915,  1183706632, 413921556};
    Pcg32 rng(42);
    for (std::uint32_t want : expected) {
        CHECK(rng.nextU32() == want);
    }
}

TEST_CASE("golden double stream") {
    const std::vector<double> expected = {0.114713470988, 0.829341338270,
                                          0.275603175881, 0.051818669599};
    Pcg32 rng(42);
    for (double want : expected) {
        CHECK(rng.nextDouble() == doctest::Approx(want).epsilon(1e-11));
    }
}

TEST_CASE("golden gauss stream") {
    const std::vector<double> expected = {0.544614918000, -0.373216297997,
                                          0.241998012534, -0.977950401660};
    Pcg32 rng(42);
    for (double want : expected) {
        CHECK(rng.gauss() == doctest::Approx(want).epsilon(1e-11));
    }
}

TEST_CASE("same seed reproduces the stream") {
    Pcg32 a(1234);
    Pcg32 b(1234);
    for (int i = 0; i < 50; ++i) {
        CHECK(a.nextU32() == b.nextU32());
    }
}

TEST_CASE("reseeding restarts the stream and clears the gaussian cache") {
    Pcg32 rng(3);
    rng.gauss();
    CHECK(rng.hasCachedGauss());
    rng.reseed(3);
    CHECK_FALSE(rng.hasCachedGauss());

    Pcg32 fresh(3);
    CHECK(rng.gauss() == doctest::Approx(fresh.gauss()));
}

TEST_CASE("nextDouble stays in the unit interval") {
    Pcg32 rng(11);
    for (int i = 0; i < 20000; ++i) {
        const double value = rng.nextDouble();
        CHECK(value >= 0.0);
        CHECK(value < 1.0);
    }
}

TEST_CASE("randint is inclusive at both ends") {
    Pcg32 rng(13);
    std::set<int> seen;
    for (int i = 0; i < 2000; ++i) {
        seen.insert(rng.randint(0, 3));
    }
    CHECK(seen == std::set<int>{0, 1, 2, 3});
}

TEST_CASE("randint of a single value is that value") {
    Pcg32 rng(14);
    for (int i = 0; i < 5; ++i) {
        CHECK(rng.randint(7, 7) == 7);
    }
}

TEST_CASE("randint rejects an inverted range") {
    Pcg32 rng(15);
    CHECK_THROWS(rng.randint(5, 4));
}

TEST_CASE("randint is unbiased") {
    // Plain modulo would skew toward the low end of the range.
    Pcg32 rng(16);
    constexpr int kDraws = 120000;
    std::vector<int> buckets(6, 0);
    for (int i = 0; i < kDraws; ++i) {
        ++buckets[static_cast<std::size_t>(rng.randint(0, 5))];
    }
    for (int count : buckets) {
        CHECK(std::abs(count - kDraws / 6.0) < kDraws * 0.01);
    }
}

TEST_CASE("distribution moments are sane") {
    constexpr int kSamples = 100000;

    Pcg32 uniformRng(17);
    double sum = 0.0;
    for (int i = 0; i < kSamples; ++i) {
        sum += uniformRng.nextDouble();
    }
    CHECK(std::abs(sum / kSamples - 0.5) < 0.01);

    Pcg32 normalRng(18);
    std::vector<double> normals;
    normals.reserve(kSamples);
    double total = 0.0;
    for (int i = 0; i < kSamples; ++i) {
        normals.push_back(normalRng.gauss());
        total += normals.back();
    }
    const double mean = total / kSamples;
    double variance = 0.0;
    for (double value : normals) {
        variance += (value - mean) * (value - mean);
    }
    variance /= kSamples;
    CHECK(std::abs(mean) < 0.02);
    CHECK(std::abs(std::sqrt(variance) - 1.0) < 0.02);
}
