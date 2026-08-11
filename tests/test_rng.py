"""
Contract tests for the portable PRNG.

These are the cross-language contract: the C++ port must reproduce every
value pinned here. If a change to rng.py breaks these, it has silently
forked the two engines even if every other test still passes.
"""

import math

import pytest

from interception.rng import Pcg32

# From the PCG reference distribution's pcg32-demo.c, which seeds with
# state 42 / sequence 54. Matching it proves this is really PCG32 and not
# merely a plausible-looking generator.
PCG_REFERENCE_SEED = 42
PCG_REFERENCE_STREAM = 54
PCG_REFERENCE_OUTPUT = [
    0xA15C02B7,
    0x7B47F409,
    0xBA1D3330,
    0x83D2F293,
    0xBFA4784B,
    0xCBED606E,
]

# Golden vectors for this project's own default stream. The C++ engine must
# reproduce these exactly.
GOLDEN_U32 = [492690617, 1919685028, 3561993920, 683038915, 1183706632, 413921556]
GOLDEN_DOUBLE = [0.114713470988, 0.829341338270, 0.275603175881, 0.051818669599]
GOLDEN_GAUSS = [0.544614918000, -0.373216297997, 0.241998012534, -0.977950401660]


def test_matches_the_pcg32_reference_vector():
    rng = Pcg32(PCG_REFERENCE_SEED, stream=PCG_REFERENCE_STREAM)
    assert [rng.next_u32() for _ in range(6)] == PCG_REFERENCE_OUTPUT


def test_golden_u32_stream():
    rng = Pcg32(42)
    assert [rng.next_u32() for _ in range(6)] == GOLDEN_U32


def test_golden_double_stream():
    rng = Pcg32(42)
    got = [rng.next_double() for _ in range(4)]
    assert got == pytest.approx(GOLDEN_DOUBLE, abs=1e-12)


def test_golden_gauss_stream():
    rng = Pcg32(42)
    got = [rng.gauss() for _ in range(4)]
    assert got == pytest.approx(GOLDEN_GAUSS, abs=1e-12)


def test_same_seed_reproduces_the_stream():
    a, b = Pcg32(1234), Pcg32(1234)
    assert [a.next_u32() for _ in range(50)] == [b.next_u32() for _ in range(50)]


def test_different_seeds_diverge():
    a, b = Pcg32(1234), Pcg32(1235)
    assert [a.next_u32() for _ in range(20)] != [b.next_u32() for _ in range(20)]


def test_reseeding_restarts_the_stream():
    rng = Pcg32(9)
    first = [rng.next_u32() for _ in range(10)]
    rng.seed(9)
    assert [rng.next_u32() for _ in range(10)] == first


def test_reseed_clears_the_cached_gaussian():
    """
    The polar method produces two variates per accepted pair and caches the
    spare. If reseeding left the cache populated, the first value after a
    reset would come from the old stream.
    """
    rng = Pcg32(3)
    rng.gauss()  # populates the cache
    assert rng._cached_gauss is not None
    rng.seed(3)
    assert rng._cached_gauss is None
    fresh = Pcg32(3)
    assert rng.gauss() == fresh.gauss()


def test_next_double_is_in_the_unit_interval():
    rng = Pcg32(11)
    for _ in range(20_000):
        value = rng.next_double()
        assert 0.0 <= value < 1.0


def test_uniform_respects_its_bounds():
    rng = Pcg32(12)
    for _ in range(20_000):
        value = rng.uniform(-3.5, 7.25)
        assert -3.5 <= value < 7.25


def test_randint_is_inclusive_at_both_ends():
    rng = Pcg32(13)
    seen = {rng.randint(0, 3) for _ in range(2_000)}
    assert seen == {0, 1, 2, 3}


def test_randint_of_a_single_value_is_that_value():
    rng = Pcg32(14)
    assert [rng.randint(7, 7) for _ in range(5)] == [7] * 5


def test_randint_rejects_an_inverted_range():
    with pytest.raises(ValueError):
        Pcg32(15).randint(5, 4)


def test_randint_is_unbiased():
    """Plain modulo would skew toward the low end of the range."""
    rng = Pcg32(16)
    buckets = [0] * 6
    draws = 120_000
    for _ in range(draws):
        buckets[rng.randint(0, 5)] += 1
    for count in buckets:
        assert abs(count - draws / 6) < draws * 0.01


def test_distribution_moments_are_sane():
    rng = Pcg32(17)
    n = 100_000
    uniforms = [rng.next_double() for _ in range(n)]
    assert abs(sum(uniforms) / n - 0.5) < 0.01

    rng = Pcg32(18)
    normals = [rng.gauss() for _ in range(n)]
    mean = sum(normals) / n
    sd = math.sqrt(sum((x - mean) ** 2 for x in normals) / n)
    assert abs(mean) < 0.02
    assert abs(sd - 1.0) < 0.02


def test_gauss_scales_with_mu_and_sigma():
    plain = Pcg32(19)
    scaled = Pcg32(19)
    for _ in range(100):
        assert scaled.gauss(10.0, 3.0) == pytest.approx(10.0 + 3.0 * plain.gauss())
