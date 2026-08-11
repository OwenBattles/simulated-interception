"""
A portable pseudo-random number generator with an exactly specified stream.

WHY NOT ``random.Random``
-------------------------
The simulator's central guarantee is "same seed, same trajectory". Once the
engine exists in more than one language that guarantee has to survive the
language boundary, and the standard library cannot provide it:

- CPython's ``random`` is Mersenne Twister; C++'s ``std::mt19937`` matches it
  in raw output but not in how the distributions consume it.
- ``std::uniform_real_distribution`` and ``std::uniform_int_distribution`` are
  explicitly *implementation-defined*. libstdc++ and libc++ return different
  numbers from the same engine and the same seed. Nothing in the standard
  forbids that.
- ``random.gauss`` uses a specific polar method with a cached second variate.
  ``std::normal_distribution`` also caches, but not necessarily the same way.

So the generator and every distribution derived from it are specified here and
reimplemented identically in C++, rather than delegated to either standard
library. The algorithm is PCG32 (O'Neill 2014): 64 bits of state, a 32-bit
permuted output, small enough to port without ambiguity.

Every operation below is defined in terms of exact integer arithmetic, and the
test suite pins a golden output vector that both implementations must match.
"""

import math

MASK32 = 0xFFFFFFFF
MASK64 = 0xFFFFFFFFFFFFFFFF
MULTIPLIER = 6364136223846793005
DEFAULT_STREAM = 1442695040888963407

# 2**26 and 2**53, used to assemble a double from two 32-bit draws.
_TWO_26 = 67108864.0
_TWO_53 = 9007199254740992.0


class Pcg32:
    """PCG32 with explicitly specified derived distributions."""

    __slots__ = ("_state", "_inc", "_cached_gauss")

    def __init__(self, seed=0, stream=DEFAULT_STREAM):
        self.seed(seed, stream)

    def seed(self, seed, stream=DEFAULT_STREAM):
        """Standard PCG seeding: step, add the seed, step again."""
        self._cached_gauss = None
        self._state = 0
        self._inc = ((stream << 1) | 1) & MASK64
        self.next_u32()
        self._state = (self._state + (seed & MASK64)) & MASK64
        self.next_u32()

    # --- core -----------------------------------------------------------
    def next_u32(self):
        """Advance the LCG and return the permuted 32-bit output."""
        old = self._state
        self._state = (old * MULTIPLIER + self._inc) & MASK64
        xorshifted = (((old >> 18) ^ old) >> 27) & MASK32
        rot = (old >> 59) & 31
        # 32-bit rotate right.
        return ((xorshifted >> rot) | (xorshifted << ((32 - rot) & 31))) & MASK32

    def next_double(self):
        """
        A double in [0, 1) with 53 bits of entropy.

        Assembled from two 32-bit draws the same way CPython builds its own
        random(), so the bit budget is unambiguous across languages.
        """
        a = self.next_u32() >> 5  # 27 bits
        b = self.next_u32() >> 6  # 26 bits
        return (a * _TWO_26 + b) / _TWO_53

    # --- distributions --------------------------------------------------
    def uniform(self, low, high):
        return low + (high - low) * self.next_double()

    def randint(self, low, high):
        """
        Uniform integer in [low, high], inclusive at both ends.

        Uses PCG's bounded method: reject the smallest ``2**32 % bound``
        outputs so the remaining range divides evenly. Plain modulo would
        bias toward small values.
        """
        bound = high - low + 1
        if bound <= 0:
            raise ValueError("randint requires high >= low")
        threshold = ((1 << 32) - bound) % bound
        while True:
            candidate = self.next_u32()
            if candidate >= threshold:
                return low + (candidate % bound)

    def gauss(self, mu=0.0, sigma=1.0):
        """
        Normal variate by the Marsaglia polar method.

        The method produces two variates per pair of accepted uniforms; the
        spare is cached. The cache is part of the generator state and is
        cleared on reseed, otherwise a reset would not reproduce the stream.
        """
        if self._cached_gauss is not None:
            spare = self._cached_gauss
            self._cached_gauss = None
            return mu + sigma * spare

        while True:
            u = 2.0 * self.next_double() - 1.0
            v = 2.0 * self.next_double() - 1.0
            s = u * u + v * v
            if 0.0 < s < 1.0:
                break

        scale = math.sqrt(-2.0 * math.log(s) / s)
        self._cached_gauss = v * scale
        return mu + sigma * (u * scale)
