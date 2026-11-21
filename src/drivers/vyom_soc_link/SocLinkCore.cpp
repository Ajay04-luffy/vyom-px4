#include "SocLinkCore.hpp"

namespace vyom
{

SocLinkCore::SocLinkCore()
{
	vl_decoder_init(&_dec);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): length and time have distinct names and units
void SocLinkCore::push(const uint8_t *data, size_t len, uint64_t now_us)
{
	for (size_t i = 0; i < len; i++) {
		if (vl_decoder_push(&_dec, data[i], &_frame)) {
			handle(_frame, now_us);
		}
	}
}

void SocLinkCore::handle(const vl_frame_t &f, uint64_t now_us)
{
	switch (f.msg_id) {
	case VL_MSG_HEARTBEAT: {
			vl_heartbeat_t hb{};

			if (vl_heartbeat_unpack(f.payload, f.payload_len, &hb) && (hb.role == ROLE_SOC)) {
				_hb = hb;
				_last_hb_us = now_us;

			} else {
				_rejected++;
			}

			break;
		}

	case VL_MSG_TIMESYNC: {
			vl_timesync_t ts{};

			if (!vl_timesync_unpack(f.payload, f.payload_len, &ts)) {
				_rejected++;

			} else if (ts.tc1_us == 0) {
				// Request from the SoC: reply with our clock (filled by the caller when sending).
				_ts_req = ts;
				_ts_req_new = true;

			} else {
				// Reply to our request.
				_ts_reply = ts;
				_ts_reply_rx_us = now_us;
				_ts_reply_new = true;
			}

			break;
		}

	case VL_MSG_VISION_ODOMETRY: {
			vl_vision_odometry_t od{};

			if (vl_vision_odometry_unpack(f.payload, f.payload_len, &od)) {
				_odom = od;
				_odom_rx_us = now_us;
				_odom_new = true;

			} else {
				_rejected++;
			}

			break;
		}

	case VL_MSG_TRAJECTORY_SETPOINT:
		// Accepted on the wire, not acted on: SoC-driven modes arrive in WP5, and only those
		// modes may use it. Counting it makes an unexpected sender visible.
		_setpoints_ignored++;
		break;

	default:
		// fc_state is FC->SoC only; anything else is unknown to this schema version.
		_rejected++;
		break;
	}
}

bool SocLinkCore::soc_alive(uint64_t now_us) const
{
	return (_last_hb_us != 0u) && (now_us >= _last_hb_us) && ((now_us - _last_hb_us) <= _hb_timeout_us);
}

bool SocLinkCore::take_odometry(vl_vision_odometry_t &out, uint64_t &rx_time_us)
{
	if (!_odom_new) {
		return false;
	}

	out = _odom;
	rx_time_us = _odom_rx_us;
	_odom_new = false;
	return true;
}

bool SocLinkCore::take_timesync_reply(vl_timesync_t &out)
{
	if (!_ts_req_new) {
		return false;
	}

	out = _ts_req;
	_ts_req_new = false;
	return true;
}

bool SocLinkCore::take_timesync_sample(vl_timesync_t &out, uint64_t &rx_time_us)
{
	if (!_ts_reply_new) {
		return false;
	}

	out = _ts_reply;
	rx_time_us = _ts_reply_rx_us;
	_ts_reply_new = false;
	return true;
}

SocLinkCore::Counters SocLinkCore::counters() const
{
	Counters c{};
	c.frames_ok = _dec.frames_ok;
	c.crc_errors = _dec.crc_errors;
	c.framing_errors = _dec.framing_errors;
	c.version_errors = _dec.version_errors;
	c.rejected_msgs = _rejected;
	c.setpoints_ignored = _setpoints_ignored;
	return c;
}

size_t SocLinkCore::encode_heartbeat(const vl_heartbeat_t &m, uint8_t *out, size_t cap)
{
	uint8_t p[VL_HEARTBEAT_LEN];
	const size_t n = vl_heartbeat_pack(&m, p);
	return vl_frame_encode(VL_MSG_HEARTBEAT, _tx_seq++, p, n, out, cap);
}

size_t SocLinkCore::encode_fc_state(const vl_fc_state_t &m, uint8_t *out, size_t cap)
{
	uint8_t p[VL_FC_STATE_LEN];
	const size_t n = vl_fc_state_pack(&m, p);
	return vl_frame_encode(VL_MSG_FC_STATE, _tx_seq++, p, n, out, cap);
}

size_t SocLinkCore::encode_timesync(const vl_timesync_t &m, uint8_t *out, size_t cap)
{
	uint8_t p[VL_TIMESYNC_LEN];
	const size_t n = vl_timesync_pack(&m, p);
	return vl_frame_encode(VL_MSG_TIMESYNC, _tx_seq++, p, n, out, cap);
}

} // namespace vyom
