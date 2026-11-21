/** Unit tests for vyom::GnssGuard. Run: make tests TESTFILTER=GnssGuard (or Tools/vyom/run_host_tests.sh) */
#include <gtest/gtest.h>

#include <math.h>

#include "GnssGuard.hpp"

using namespace vyom;

namespace
{

constexpr uint64_t DT_US = 100000u;   // 10 Hz GNSS

struct Sim {
	GnssGuard guard;
	uint64_t t_us{1000000u};
	float n_m{0.f};            // true north position
	float vn_m_s{2.f};         // true north velocity
	float spoof_n_m{0.f};      // spoofer offset added to GNSS
	bool vio{true};
	bool landed{false};
	Cn0Stats cn0{12u, 38.f, 5.f};
	bool rx_spoof{false};
	bool rx_jam{false};

	bool step()
	{
		t_us += DT_US;
		n_m += vn_m_s * 0.1f;
		GnssSample g{t_us, n_m + spoof_n_m, 0.f, vn_m_s, 0.f, rx_spoof, rx_jam};
		EkfState e{true, vn_m_s, 0.f, true, n_m, 0.f, landed};
		VioSample v{vio ? t_us : 0u, n_m, 0.f, vn_m_s, 0.f, 0u};
		return guard.update(g, e, v, cn0);
	}

	int steps_until_reject(int max_steps)
	{
		for (int i = 1; i <= max_steps; i++) { if (step()) { return i; } }

		return -1;
	}
};

} // namespace

TEST(GnssGuard, CleanFlightNeverRejects)
{
	Sim s;

	for (int i = 0; i < 3000; i++) { EXPECT_FALSE(s.step()); }   // 5 minutes

	EXPECT_EQ(s.guard.reject_count(), 0u);
}

TEST(GnssGuard, StepSpoofIsRejectedOnTheNextSample)
{
	Sim s;

	for (int i = 0; i < 50; i++) { s.step(); }

	s.spoof_n_m = 50.f;
	EXPECT_EQ(s.steps_until_reject(5), 1);
	EXPECT_TRUE(s.guard.reasons_latched() & GnssGuard::REASON_JUMP);
}

TEST(GnssGuard, StepSpoofStaysRejectedWhileOffsetPersists)
{
	Sim s;

	for (int i = 0; i < 50; i++) { s.step(); }

	s.spoof_n_m = 50.f;

	for (int i = 0; i < 600; i++) { s.step(); }   // 60 s with the offset still there

	EXPECT_TRUE(s.guard.reject());
	EXPECT_EQ(s.guard.reject_count(), 1u);
}

TEST(GnssGuard, RecoversAfterSpoofEndsWithVio)
{
	Sim s;

	for (int i = 0; i < 50; i++) { s.step(); }

	s.spoof_n_m = 50.f;

	for (int i = 0; i < 100; i++) { s.step(); }

	s.spoof_n_m = 0.f;     // spoofer off: GNSS jumps back (another JUMP), then agrees with EKF
	int clean = 0;

	for (int i = 0; i < 300 && s.guard.reject(); i++) { s.step(); clean++; }

	EXPECT_FALSE(s.guard.reject());
	EXPECT_GE(clean, 100);   // at least recover_s (10 s) of clean data
}

TEST(GnssGuard, SlowDriftSpoofCaughtByVio)
{
	Sim s;

	for (int i = 0; i < 150; i++) { s.step(); }

	// 0.5 m/s drag-off: too slow for the jump test (0.05 m per sample)
	int n = 0;

	while (n < 200) {
		s.spoof_n_m += 0.05f;
		n++;

		if (s.step()) { break; }
	}

	EXPECT_TRUE(s.guard.reject());
	EXPECT_FALSE(s.guard.reasons_latched() & GnssGuard::REASON_JUMP);
	EXPECT_LE(s.spoof_n_m, 3.6f);   // caught before 3.6 m of drift (VIO_POS limit 3 m)
}

TEST(GnssGuard, SlowDriftWithoutVioIsNotDetectedHere)
{
	// Documents the limit: without VIO a slow drift is only caught by receiver flags or C/N0.
	Sim s;
	s.vio = false;

	for (int i = 0; i < 300; i++) { s.spoof_n_m += 0.05f; s.step(); }

	EXPECT_FALSE(s.guard.reject());
}

TEST(GnssGuard, ReceiverFlagsReject)
{
	Sim a;
	a.rx_spoof = true;
	EXPECT_EQ(a.steps_until_reject(3), 1);
	EXPECT_TRUE(a.guard.reasons() & GnssGuard::REASON_RX_SPOOF);

	Sim b;
	b.rx_jam = true;
	EXPECT_EQ(b.steps_until_reject(3), 1);
	EXPECT_TRUE(b.guard.reasons() & GnssGuard::REASON_RX_JAM);
}

TEST(GnssGuard, UniformStrongCn0Rejects)
{
	Sim s;
	s.cn0 = Cn0Stats{10u, 46.f, 0.8f};
	EXPECT_EQ(s.steps_until_reject(3), 1);
	EXPECT_TRUE(s.guard.reasons() & GnssGuard::REASON_CN0);

	Sim few;
	few.cn0 = Cn0Stats{4u, 46.f, 0.8f};   // too few satellites to judge
	EXPECT_EQ(few.steps_until_reject(10), -1);
}

TEST(GnssGuard, WithoutVioRecoveryOnlyOnGround)
{
	Sim s;
	s.vio = false;
	s.rx_spoof = true;
	s.step();
	s.rx_spoof = false;

	for (int i = 0; i < 300; i++) { s.step(); }

	EXPECT_TRUE(s.guard.reject());   // in the air: nothing to verify against
	s.landed = true;
	s.step();
	EXPECT_FALSE(s.guard.reject());
}

TEST(GnssGuard, VioResetRestartsTheWindow)
{
	GnssGuard g;
	uint64_t t = 1000000u;

	for (int i = 0; i < 100; i++) {
		t += DT_US;
		// VIO frame jumps by 20 m at i == 50 together with a reset_counter increment
		const float vio_n = (i < 50) ? 0.f : 20.f;
		const uint8_t rc = (i < 50) ? 0u : 1u;
		EXPECT_FALSE(g.update(GnssSample{t, 0.f, 0.f, 0.f, 0.f, false, false}, EkfState{true, 0.f, 0.f, true, 0.f, 0.f, false},
				      VioSample{t, vio_n, 0.f, 0.f, 0.f, rc}, Cn0Stats{0u, NAN, NAN}));
	}
}
