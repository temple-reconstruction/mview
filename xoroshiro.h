/*  Written in 2018 by David Blackman and Sebastiano Vigna (vigna@acm.org)

To the extent possible under law, the author has dedicated all copyright
and related and neighboring rights to this software to the public domain
worldwide. This software is distributed without any warranty.

See <http://creativecommons.org/publicdomain/zero/1.0/>. 

Adapted to C++ by Andreas Molzer, under the same conditions.
*/

#include <stdint.h>
#include <chrono>
#include <limits>
#include <random>

/* This is xoshiro256** 1.0, our all-purpose, rock-solid generator. It has
   excellent (sub-ns) speed, a state (256 bits) that is large enough for
   any parallel application, and it passes all tests we are aware of.

   For generating just floating-point numbers, xoshiro256+ is even faster.

   The state must be seeded so that it is not everywhere zero. If you have
   a 64-bit seed, we suggest to seed a splitmix64 generator and use its
   output to fill s. */

static inline uint64_t rotl(const uint64_t x, int k) {
	return (x << k) | (x >> (64 - k));
}

struct Xoroshiro {
	using result_type = uint64_t;

	uint64_t s[4];

	Xoroshiro() : Xoroshiro(Now()) { }

	Xoroshiro(uint64_t init) {
		s[0] = init;
		s[1] = 0;
		s[2] = 0;
		s[3] = 0;
	}

	uint64_t next() {
		const uint64_t result_starstar = rotl(s[1] * 5, 7) * 9;

		const uint64_t t = s[1] << 17;

		s[2] ^= s[0];
		s[3] ^= s[1];
		s[1] ^= s[2];
		s[0] ^= s[3];

		s[2] ^= t;

		s[3] = rotl(s[3], 45);

		return result_starstar;
	}

	uint64_t operator()() {
		return next();
	}

	static uint64_t max() {
		return std::numeric_limits<uint64_t>::max();
	}

	static uint64_t min() {
		return std::numeric_limits<uint64_t>::min();
	}

	static uint64_t Now() {
		auto now = std::chrono::system_clock::now();
		auto nanos = (std::chrono::nanoseconds) now.time_since_epoch();
		return nanos.count();
	}
};

