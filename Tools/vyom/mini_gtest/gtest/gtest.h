// Minimal stand-in for <gtest/gtest.h> used by Tools/vyom/run_host_tests.sh, so Vyom unit tests run
// on a host without PX4's full test build (which downloads dependencies). CI runs the real
// googletest via `make tests`. Supports TEST, EXPECT_/ASSERT_ {TRUE,FALSE,EQ,NE,GT,GE,LT,LE,FLOAT_EQ}.
#pragma once
#include <cmath>
#include <cstdio>
#include <functional>
#include <vector>

namespace mini_gtest
{
struct Case { const char *suite; const char *name; std::function<void(bool &)> fn; };
inline std::vector<Case> &cases() { static std::vector<Case> c; return c; }
struct Reg { Reg(const char *s, const char *n, std::function<void(bool &)> f) { cases().push_back({s, n, f}); } };
inline bool &failed() { static bool f = false; return f; }
}

#define TEST(suite, name) \
	static void suite##_##name##_body(bool &mg_ok); \
	static mini_gtest::Reg suite##_##name##_reg(#suite, #name, suite##_##name##_body); \
	static void suite##_##name##_body(bool &mg_ok)

#define MG_CHECK(cond, fatal) do { if (!(cond)) { std::printf("  %s:%d: check failed: %s\n", __FILE__, __LINE__, #cond); \
	mg_ok = false; if (fatal) { return; } } } while (0)

#define EXPECT_TRUE(a) MG_CHECK((a), false)
#define EXPECT_FALSE(a) MG_CHECK(!(a), false)
#define EXPECT_EQ(a, b) MG_CHECK((a) == (b), false)
#define EXPECT_NE(a, b) MG_CHECK((a) != (b), false)
#define EXPECT_GT(a, b) MG_CHECK((a) > (b), false)
#define EXPECT_GE(a, b) MG_CHECK((a) >= (b), false)
#define EXPECT_LT(a, b) MG_CHECK((a) < (b), false)
#define EXPECT_LE(a, b) MG_CHECK((a) <= (b), false)
#define EXPECT_FLOAT_EQ(a, b) MG_CHECK(std::fabs((double)(a) - (double)(b)) <= 1e-6 * (1.0 + std::fabs((double)(b))), false)
#define ASSERT_TRUE(a) MG_CHECK((a), true)
#define ASSERT_FALSE(a) MG_CHECK(!(a), true)
#define ASSERT_EQ(a, b) MG_CHECK((a) == (b), true)
#define ASSERT_GT(a, b) MG_CHECK((a) > (b), true)

int main()
{
	int fails = 0;

	for (auto &c : mini_gtest::cases()) {
		bool ok = true;
		c.fn(ok);
		std::printf("[%s] %s.%s\n", ok ? "  OK  " : " FAIL ", c.suite, c.name);
		fails += ok ? 0 : 1;
	}

	std::printf("%zu tests, %d failed\n", mini_gtest::cases().size(), fails);
	return fails == 0 ? 0 : 1;
}
