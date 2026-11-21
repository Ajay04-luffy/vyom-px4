/** Unit tests for the external-watchdog kick policy. Run: make tests TESTFILTER=WdtPolicy */
#include <gtest/gtest.h>

#include "WdtPolicy.hpp"

using namespace vyom;

static WdtInputs in(uint64_t now, uint64_t gyro, uint64_t torque)
{
	WdtInputs i{};
	i.now_us = now;
	i.start_us = 1000000u;
	i.last_gyro_us = gyro;
	i.last_torque_us = torque;
	return i;
}

TEST(WdtPolicy, KicksDuringBootGrace)
{
	const WdtLimits lim{};
	EXPECT_EQ(wdt_decide(in(1000000u, 0u, 0u), lim), WdtDecision::KickBoot);
	EXPECT_EQ(wdt_decide(in(21000000u, 0u, 0u), lim), WdtDecision::KickBoot);
	EXPECT_EQ(wdt_decide(in(21000001u, 0u, 0u), lim), WdtDecision::HoldBootTimeout);
	// Gyro up but rate controller never ran: still boot, then hold.
	EXPECT_EQ(wdt_decide(in(5000000u, 4999000u, 0u), lim), WdtDecision::KickBoot);
	EXPECT_FALSE(wdt_is_kick(wdt_decide(in(30000000u, 29999000u, 0u), lim)));
}

TEST(WdtPolicy, KicksWhileControlPathIsFresh)
{
	const WdtLimits lim{};
	EXPECT_EQ(wdt_decide(in(10000000u, 9999000u, 9998000u), lim), WdtDecision::Kick);
	EXPECT_EQ(wdt_decide(in(10000000u, 9800000u, 9800000u), lim), WdtDecision::Kick); // exactly at the limit
}

TEST(WdtPolicy, StopsKickingWhenGyroOrRateLoopStalls)
{
	const WdtLimits lim{};
	EXPECT_EQ(wdt_decide(in(10000000u, 9799999u, 9999000u), lim), WdtDecision::HoldGyroStale);
	EXPECT_EQ(wdt_decide(in(10000000u, 9999000u, 9799999u), lim), WdtDecision::HoldRateStale);
	EXPECT_FALSE(wdt_is_kick(WdtDecision::HoldGyroStale));
	EXPECT_FALSE(wdt_is_kick(WdtDecision::HoldRateStale));
}

TEST(WdtPolicy, TimestampFromTheFutureIsNotTrusted)
{
	const WdtLimits lim{};
	EXPECT_EQ(wdt_decide(in(10000000u, 10000001u, 9999000u), lim), WdtDecision::HoldGyroStale);
}
