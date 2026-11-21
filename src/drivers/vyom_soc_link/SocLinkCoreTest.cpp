/**
 * Unit tests for vyom::SocLinkCore (FC side of the /proto v0 link). Run: make tests TESTFILTER=SocLinkCore
 */
#include <gtest/gtest.h>

#include "SocLinkCore.hpp"

using vyom::SocLinkCore;

namespace
{

struct Wire {
	uint8_t buf[VL_MAX_ENCODED];
	size_t len;
};

Wire soc_heartbeat(uint8_t role = SocLinkCore::ROLE_SOC, uint8_t health = 1u)
{
	vl_heartbeat_t hb{};
	hb.uptime_ms = 42u;
	hb.role = role;
	hb.health = health;
	uint8_t p[VL_HEARTBEAT_LEN];
	Wire w{};
	w.len = vl_frame_encode(VL_MSG_HEARTBEAT, 1u, p, vl_heartbeat_pack(&hb, p), w.buf, sizeof(w.buf));
	return w;
}

Wire raw_frame(uint8_t id, const uint8_t *payload, size_t len)
{
	Wire w{};
	w.len = vl_frame_encode(id, 0u, payload, len, w.buf, sizeof(w.buf));
	return w;
}

} // namespace

TEST(SocLinkCore, HeartbeatMakesSocAliveUntilTimeout)
{
	SocLinkCore core;
	core.set_heartbeat_timeout_us(500000u);
	EXPECT_FALSE(core.soc_alive(1000u));

	const Wire w = soc_heartbeat();
	core.push(w.buf, w.len, 1000000u);
	EXPECT_TRUE(core.soc_alive(1000000u));
	EXPECT_TRUE(core.soc_alive(1500000u));
	EXPECT_FALSE(core.soc_alive(1500001u));
	EXPECT_EQ(core.last_heartbeat().uptime_ms, 42u);
}

TEST(SocLinkCore, HeartbeatFromWrongRoleIsRejected)
{
	SocLinkCore core;
	const Wire w = soc_heartbeat(SocLinkCore::ROLE_FC);
	core.push(w.buf, w.len, 1000u);
	EXPECT_FALSE(core.soc_alive(1000u));
	EXPECT_EQ(core.counters().rejected_msgs, 1u);
}

TEST(SocLinkCore, OwnEncodedHeartbeatIsNotMistakenForSoc)
{
	// A UART looped back to itself must not look like a live SoC.
	SocLinkCore core;
	vl_heartbeat_t hb{};
	hb.role = SocLinkCore::ROLE_FC;
	uint8_t out[VL_MAX_ENCODED];
	const size_t n = core.encode_heartbeat(hb, out, sizeof(out));
	ASSERT_GT(n, 0u);
	core.push(out, n, 1000u);
	EXPECT_FALSE(core.soc_alive(1000u));
}

TEST(SocLinkCore, VisionOdometryIsQueuedOnce)
{
	SocLinkCore core;
	vl_vision_odometry_t od{};
	od.timestamp_us = 123456u;
	od.pos_ned_m[2] = -10.5f;
	od.q[0] = 1.f;
	od.quality_pct = 80u;
	uint8_t p[VL_VISION_ODOMETRY_LEN];
	const Wire w = raw_frame(VL_MSG_VISION_ODOMETRY, p, vl_vision_odometry_pack(&od, p));
	core.push(w.buf, w.len, 2000u);

	vl_vision_odometry_t got{};
	uint64_t rx_us = 0;
	ASSERT_TRUE(core.take_odometry(got, rx_us));
	EXPECT_EQ(rx_us, 2000u);
	EXPECT_EQ(got.timestamp_us, 123456u);
	EXPECT_FLOAT_EQ(got.pos_ned_m[2], -10.5f);
	EXPECT_EQ(got.quality_pct, 80u);
	EXPECT_FALSE(core.take_odometry(got, rx_us));
}

TEST(SocLinkCore, TimesyncRequestAndReplyAreSeparated)
{
	SocLinkCore core;
	vl_timesync_t ts{};
	uint8_t p[VL_TIMESYNC_LEN];

	ts.tc1_us = 0;
	ts.ts1_us = 777;
	Wire w = raw_frame(VL_MSG_TIMESYNC, p, vl_timesync_pack(&ts, p));
	core.push(w.buf, w.len, 10u);

	ts.tc1_us = 5000;
	ts.ts1_us = 100;
	w = raw_frame(VL_MSG_TIMESYNC, p, vl_timesync_pack(&ts, p));
	core.push(w.buf, w.len, 20u);

	vl_timesync_t got{};
	uint64_t rx_us = 0;
	ASSERT_TRUE(core.take_timesync_reply(got));
	EXPECT_EQ(got.ts1_us, 777);
	ASSERT_TRUE(core.take_timesync_sample(got, rx_us));
	EXPECT_EQ(got.tc1_us, 5000);
	EXPECT_EQ(rx_us, 20u);
	EXPECT_FALSE(core.take_timesync_reply(got));
	EXPECT_FALSE(core.take_timesync_sample(got, rx_us));
}

TEST(SocLinkCore, SetpointsAreCountedNotUsed)
{
	SocLinkCore core;
	vl_trajectory_setpoint_t sp{};
	uint8_t p[VL_TRAJECTORY_SETPOINT_LEN];
	const Wire w = raw_frame(VL_MSG_TRAJECTORY_SETPOINT, p, vl_trajectory_setpoint_pack(&sp, p));
	core.push(w.buf, w.len, 1u);
	EXPECT_EQ(core.counters().setpoints_ignored, 1u);
}

TEST(SocLinkCore, WrongDirectionUnknownAndShortMessagesAreRejected)
{
	SocLinkCore core;
	uint8_t p[VL_MAX_PAYLOAD] {};
	Wire w = raw_frame(VL_MSG_FC_STATE, p, VL_FC_STATE_LEN);   // FC->SoC only
	core.push(w.buf, w.len, 1u);
	w = raw_frame(200u, p, 4u);                                 // unknown id
	core.push(w.buf, w.len, 1u);
	w = raw_frame(VL_MSG_VISION_ODOMETRY, p, VL_VISION_ODOMETRY_LEN - 1u);  // too short
	core.push(w.buf, w.len, 1u);
	EXPECT_EQ(core.counters().rejected_msgs, 3u);
	EXPECT_EQ(core.counters().frames_ok, 3u);
	vl_vision_odometry_t got{};
	uint64_t rx_us = 0;
	EXPECT_FALSE(core.take_odometry(got, rx_us));
}

TEST(SocLinkCore, CorruptedFrameIsDroppedAndLinkRecovers)
{
	SocLinkCore core;
	Wire w = soc_heartbeat();
	w.buf[w.len / 2] ^= 0x5Au;   // corrupt the middle (stays non-zero or becomes a delimiter: both are errors)
	core.push(w.buf, w.len, 1u);
	EXPECT_FALSE(core.soc_alive(1u));
	const SocLinkCore::Counters c = core.counters();
	EXPECT_GE(c.crc_errors + c.framing_errors, 1u);

	const Wire good = soc_heartbeat();
	core.push(good.buf, good.len, 2u);
	EXPECT_TRUE(core.soc_alive(2u));
}

TEST(SocLinkCore, NoiseNeverProducesAHeartbeat)
{
	SocLinkCore core;
	uint32_t x = 12345u;
	uint8_t noise[4096];

	for (size_t i = 0; i < sizeof(noise); i++) {
		x = x * 1103515245u + 12345u;
		noise[i] = static_cast<uint8_t>(x >> 16);
	}

	core.push(noise, sizeof(noise), 1u);
	EXPECT_FALSE(core.soc_alive(1u));
}
