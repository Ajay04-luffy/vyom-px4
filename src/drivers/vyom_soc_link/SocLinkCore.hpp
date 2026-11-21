/**
 * @file SocLinkCore.hpp
 * Vyom FC<->SoC link: protocol logic with no PX4 dependencies (unit-testable on the host).
 *
 * Feeds received bytes through the /proto v0 decoder, checks direction and length of every
 * message, tracks SoC liveness, answers time-sync requests and queues decoded SoC data for the
 * module to publish. Fixed-size state only: no allocation, no recursion.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "proto/vyom_link_v0.h"

namespace vyom
{

class SocLinkCore
{
public:
	struct Counters {
		uint32_t frames_ok{0};
		uint32_t crc_errors{0};
		uint32_t framing_errors{0};
		uint32_t version_errors{0};
		uint32_t rejected_msgs{0};
		uint32_t setpoints_ignored{0};
	};

	static constexpr uint8_t ROLE_FC = 1u;
	static constexpr uint8_t ROLE_SOC = 2u;

	SocLinkCore();

	/** @param timeout_us SoC is declared dead when no heartbeat arrived for this long. */
	void set_heartbeat_timeout_us(uint64_t timeout_us) { _hb_timeout_us = timeout_us; }

	/** Process received bytes. now_us is the FC clock at reception. */
	void push(const uint8_t *data, size_t len, uint64_t now_us);

	bool soc_alive(uint64_t now_us) const;
	uint64_t last_heartbeat_us() const { return _last_hb_us; }
	const vl_heartbeat_t &last_heartbeat() const { return _hb; }

	/** Newest vision odometry since the last call. Returns false if none. */
	bool take_odometry(vl_vision_odometry_t &out, uint64_t &rx_time_us);

	/** A time-sync request from the SoC that needs a reply (tc1 filled with our clock). */
	bool take_timesync_reply(vl_timesync_t &out);

	/** A reply to our own time-sync request: feed it to the offset filter. */
	bool take_timesync_sample(vl_timesync_t &out, uint64_t &rx_time_us);

	Counters counters() const;

	/** Encode a message into out (capacity >= VL_MAX_ENCODED). Returns bytes, 0 on error. */
	size_t encode_heartbeat(const vl_heartbeat_t &m, uint8_t *out, size_t cap);
	size_t encode_fc_state(const vl_fc_state_t &m, uint8_t *out, size_t cap);
	size_t encode_timesync(const vl_timesync_t &m, uint8_t *out, size_t cap);

private:
	void handle(const vl_frame_t &f, uint64_t now_us);

	vl_decoder_t _dec{};
	vl_frame_t _frame{};
	uint8_t _tx_seq{0};
	uint64_t _hb_timeout_us{500000};

	vl_heartbeat_t _hb{};
	uint64_t _last_hb_us{0};

	vl_vision_odometry_t _odom{};
	uint64_t _odom_rx_us{0};
	bool _odom_new{false};

	vl_timesync_t _ts_req{};
	bool _ts_req_new{false};
	vl_timesync_t _ts_reply{};
	uint64_t _ts_reply_rx_us{0};
	bool _ts_reply_new{false};

	uint32_t _rejected{0};
	uint32_t _setpoints_ignored{0};
};

} // namespace vyom
