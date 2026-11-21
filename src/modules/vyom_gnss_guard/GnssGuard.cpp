#include "GnssGuard.hpp"

#include <math.h>

namespace vyom
{

namespace
{
float hypot2(float a, float b)
{
	return sqrtf(a * a + b * b);
}
} // namespace

bool GnssGuard::vio_fresh(const VioSample &vio, uint64_t now_us) const
{
	if (vio.time_us == 0u || !isfinite(vio.pos_n_m) || !isfinite(vio.pos_e_m)) {
		return false;
	}

	const uint64_t age_us = (now_us > vio.time_us) ? (now_us - vio.time_us) : 0u;
	return static_cast<float>(age_us) * 1e-6f <= _p.vio_timeout_s;
}

void GnssGuard::ring_push(const Entry &e)
{
	_ring[_ring_head] = e;
	_ring_head = (_ring_head + 1u) % RING;

	if (_ring_len < RING) {
		_ring_len++;
	}
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): time and window have distinct units
const GnssGuard::Entry *GnssGuard::ring_oldest_within(uint64_t now_us, float window_s) const
{
	const uint64_t window_us = static_cast<uint64_t>(window_s * 1e6f);
	const Entry *best = nullptr;

	// Walk from oldest to newest; take the first entry inside the window.
	for (size_t i = 0u; i < _ring_len; i++) {
		const size_t idx = (_ring_head + RING - _ring_len + i) % RING;
		const Entry &e = _ring[idx];

		if ((now_us >= e.time_us) && ((now_us - e.time_us) <= window_us)) {
			best = &e;
			break;
		}
	}

	return best;
}

uint16_t GnssGuard::check_jump(const GnssSample &gnss, const EkfState &ekf)
{
	// Compare the step with what the EKF velocity explains (EKF velocity is IMU-driven and
	// smooth; a spoofer can fake GNSS velocity but not the IMU).
	uint16_t reasons = 0u;
	_jump_m = 0.f;

	if (_have_prev && (gnss.time_us > _prev.time_us)) {
		const float dt_s = static_cast<float>(gnss.time_us - _prev.time_us) * 1e-6f;

		if (dt_s <= 1.0f) {
			const float vn = ekf.vel_valid ? ekf.vel_n_m_s : 0.5f * (gnss.vel_n_m_s + _prev.vel_n_m_s);
			const float ve = ekf.vel_valid ? ekf.vel_e_m_s : 0.5f * (gnss.vel_e_m_s + _prev.vel_e_m_s);
			_jump_m = hypot2(gnss.pos_n_m - (_prev.pos_n_m + vn * dt_s), gnss.pos_e_m - (_prev.pos_e_m + ve * dt_s));

			if (_jump_m > _p.jump_m) { reasons = REASON_JUMP; }
		}
	}

	_prev = gnss;
	_have_prev = true;
	return reasons;
}

uint16_t GnssGuard::check_vio_pos(const GnssSample &gnss, const VioSample &vio)
{
	// GNSS displacement over the window must match VIO displacement (VIO drifts slowly, a
	// spoofer that walks the position away does not match it).
	Entry e{};
	e.time_us = gnss.time_us;
	e.gnss_n = gnss.pos_n_m;
	e.gnss_e = gnss.pos_e_m;
	e.vio_n = vio.pos_n_m;
	e.vio_e = vio.pos_e_m;
	ring_push(e);

	_vio_pos_diff_m = NAN;
	const Entry *old = ring_oldest_within(gnss.time_us, _p.vio_window_s);

	if ((old == nullptr) || (static_cast<float>(gnss.time_us - old->time_us) * 1e-6f < _p.vio_min_window_s)) {
		return 0u;
	}

	_vio_pos_diff_m = hypot2((gnss.pos_n_m - old->gnss_n) - (vio.pos_n_m - old->vio_n),
				 (gnss.pos_e_m - old->gnss_e) - (vio.pos_e_m - old->vio_e));
	return (_vio_pos_diff_m > _p.vio_pos_m) ? REASON_VIO_POS : 0u;
}

uint16_t GnssGuard::check_vio_vel(const GnssSample &gnss, const VioSample &vio)
{
	_vio_vel_diff_m_s = NAN;

	if (!isfinite(vio.vel_n_m_s) || !isfinite(vio.vel_e_m_s)) {
		return 0u;
	}

	const float d = hypot2(gnss.vel_n_m_s - vio.vel_n_m_s, gnss.vel_e_m_s - vio.vel_e_m_s);
	const float dt_s = 0.1f;   // nominal GNSS period; the filter only needs to be roughly right
	const float alpha = dt_s / (_p.vio_vel_tau_s + dt_s);
	_vel_diff_lpf = _vel_lpf_init ? (_vel_diff_lpf + alpha * (d - _vel_diff_lpf)) : d;
	_vel_lpf_init = true;
	_vio_vel_diff_m_s = _vel_diff_lpf;
	return (_vel_diff_lpf > _p.vio_vel_m_s) ? REASON_VIO_VEL : 0u;
}

uint16_t GnssGuard::check_vio(const GnssSample &gnss, const VioSample &vio, bool have_vio)
{
	if (!have_vio || (_vio_reset_known && (vio.reset_counter != _vio_reset))) {
		ring_reset();
		_vel_lpf_init = false;
	}

	if (!have_vio) {
		_vio_pos_diff_m = NAN;
		_vio_vel_diff_m_s = NAN;
		return 0u;
	}

	_vio_reset_known = true;
	_vio_reset = vio.reset_counter;
	return check_vio_pos(gnss, vio) | check_vio_vel(gnss, vio);
}

uint16_t GnssGuard::check_cn0(const Cn0Stats &cn0) const
{
	// Every satellite at the same, strong level: one transmitter on the ground, not the sky.
	const bool enough = static_cast<float>(cn0.used_sats) >= _p.cn0_min_sats;
	const bool valid = isfinite(cn0.mean_dbhz) && isfinite(cn0.std_dbhz);
	const bool uniform = valid && (cn0.std_dbhz < _p.cn0_uniform_std_dbhz) && (cn0.mean_dbhz > _p.cn0_uniform_mean_dbhz);
	return (enough && uniform) ? REASON_CN0 : 0u;
}

void GnssGuard::latch(uint16_t reasons, const GnssSample &gnss, const EkfState &ekf, bool have_vio)
{
	const uint64_t now_us = gnss.time_us;
	_reasons = reasons;

	if (reasons != 0u) {
		_last_reason_us = now_us;

		if (!_reject) {
			_reject = true;
			_reject_count++;
			_reasons_latched = 0u;
		}

		_reasons_latched |= reasons;
		return;
	}

	if (!_reject) {
		return;
	}

	const bool clean_long_enough = static_cast<float>(now_us - _last_reason_us) * 1e-6f >= _p.recover_s;
	// With VIO, GNSS must agree with the (VIO-held) EKF position again. Without VIO there is
	// nothing independent to compare with in the air, so recover only on the ground.
	const bool position_agrees = have_vio
				     ? (ekf.pos_valid && (hypot2(gnss.pos_n_m - ekf.pos_n_m, gnss.pos_e_m - ekf.pos_e_m) <= _p.recover_ekf_dist_m))
				     : ekf.landed;

	if (clean_long_enough && position_agrees) {
		_reject = false;
		_reasons_latched = 0u;
	}
}

bool GnssGuard::update(const GnssSample &gnss, const EkfState &ekf, const VioSample &vio, const Cn0Stats &cn0)
{
	uint16_t reasons = 0u;

	if (gnss.rx_spoofing) { reasons |= REASON_RX_SPOOF; }

	if (gnss.rx_jamming) { reasons |= REASON_RX_JAM; }

	const bool have_vio = vio_fresh(vio, gnss.time_us);
	reasons |= check_jump(gnss, ekf);
	reasons |= check_vio(gnss, vio, have_vio);
	reasons |= check_cn0(cn0);
	latch(reasons, gnss, ekf, have_vio);
	return _reject;
}

} // namespace vyom
