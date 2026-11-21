/**
 * @file GnssGuard.hpp
 * GNSS integrity checks (no PX4 dependencies; unit-tested on the host).
 *
 * Decides whether GNSS may be fused. Checks, each giving a reason bit:
 *  - RX_SPOOF / RX_JAM: the receiver's own spoofing / jamming detection (u-blox M10 reports both).
 *  - JUMP:     GNSS position moved further than the EKF velocity explains (a step spoof).
 *  - VIO_POS:  over a sliding window, GNSS displacement differs from visual-odometry displacement
 *              (catches slow "drag-off" spoofing that the EKF would otherwise follow).
 *  - VIO_VEL:  low-pass |GNSS - VIO| horizontal velocity too large.
 *  - CN0:      used satellites all have nearly the same, strong C/N0 (a simulator transmitter).
 * Any reason rejects GNSS. Rejection is latched; it clears only after recover_s without any
 * reason AND (with VIO) GNSS agrees with the EKF position again, or (without VIO) on the ground.
 * Fixed-size state only: no allocation, no recursion.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace vyom
{

struct GnssGuardParams {
	float jump_m{5.0f};            // unexplained step per sample
	float vio_pos_m{3.0f};         // displacement disagreement over the window
	float vio_window_s{10.0f};     // comparison window length
	float vio_min_window_s{2.0f};  // shortest window that is evaluated
	float vio_vel_m_s{0.5f};       // filtered velocity disagreement
	float vio_vel_tau_s{1.0f};     // low-pass time constant
	float cn0_min_sats{6.0f};      // satellites needed for the C/N0 test
	float cn0_uniform_std_dbhz{1.5f};
	float cn0_uniform_mean_dbhz{42.0f};
	float recover_s{10.0f};        // clean time before GNSS is accepted again
	float recover_ekf_dist_m{3.0f};// with VIO: GNSS must agree with the EKF position this well
	float vio_timeout_s{0.3f};     // VIO older than this counts as unavailable
};

struct GnssSample {
	uint64_t time_us;
	float pos_n_m, pos_e_m;        // GNSS position in the EKF local frame
	float vel_n_m_s, vel_e_m_s;
	bool rx_spoofing;              // receiver says spoofing detected
	bool rx_jamming;               // receiver says jamming detected
};

struct EkfState {
	bool vel_valid;
	float vel_n_m_s, vel_e_m_s;
	bool pos_valid;
	float pos_n_m, pos_e_m;
	bool landed;
};

struct VioSample {
	uint64_t time_us;              // 0 = none received yet
	float pos_n_m, pos_e_m;
	float vel_n_m_s, vel_e_m_s;    // NaN if not provided
	uint8_t reset_counter;
};

struct Cn0Stats {
	uint8_t used_sats;             // 0 = unknown
	float mean_dbhz;
	float std_dbhz;
};

class GnssGuard
{
public:
	static constexpr uint16_t REASON_RX_SPOOF = 1u;
	static constexpr uint16_t REASON_RX_JAM = 2u;
	static constexpr uint16_t REASON_JUMP = 4u;
	static constexpr uint16_t REASON_VIO_POS = 8u;
	static constexpr uint16_t REASON_VIO_VEL = 16u;
	static constexpr uint16_t REASON_CN0 = 32u;

	void set_params(const GnssGuardParams &p) { _p = p; }

	/** Process one GNSS sample (5-10 Hz). Returns true if GNSS is rejected after this sample. */
	bool update(const GnssSample &gnss, const EkfState &ekf, const VioSample &vio, const Cn0Stats &cn0);

	bool reject() const { return _reject; }
	uint16_t reasons() const { return _reasons; }
	uint16_t reasons_latched() const { return _reasons_latched; }
	uint32_t reject_count() const { return _reject_count; }
	float jump_m() const { return _jump_m; }
	float vio_pos_diff_m() const { return _vio_pos_diff_m; }
	float vio_vel_diff_m_s() const { return _vio_vel_diff_m_s; }

private:
	struct Entry {
		uint64_t time_us;
		float gnss_n, gnss_e, vio_n, vio_e;
	};
	static constexpr size_t RING = 128;   // 12.8 s at 10 Hz

	bool vio_fresh(const VioSample &vio, uint64_t now_us) const;
	void ring_reset() { _ring_len = 0u; _ring_head = 0u; }
	void ring_push(const Entry &e);
	const Entry *ring_oldest_within(uint64_t now_us, float window_s) const;
	uint16_t check_jump(const GnssSample &gnss, const EkfState &ekf);
	uint16_t check_vio(const GnssSample &gnss, const VioSample &vio, bool have_vio);
	uint16_t check_vio_pos(const GnssSample &gnss, const VioSample &vio);
	uint16_t check_vio_vel(const GnssSample &gnss, const VioSample &vio);
	uint16_t check_cn0(const Cn0Stats &cn0) const;
	void latch(uint16_t reasons, const GnssSample &gnss, const EkfState &ekf, bool have_vio);

	GnssGuardParams _p{};
	bool _have_prev{false};
	GnssSample _prev{};

	Entry _ring[RING] {};
	size_t _ring_head{0};   // next write position
	size_t _ring_len{0};
	bool _vio_reset_known{false};
	uint8_t _vio_reset{0};
	float _vel_diff_lpf{0.f};
	bool _vel_lpf_init{false};

	bool _reject{false};
	uint16_t _reasons{0};
	uint16_t _reasons_latched{0};
	uint32_t _reject_count{0};
	uint64_t _last_reason_us{0};
	float _jump_m{0.f};
	float _vio_pos_diff_m{0.f};
	float _vio_vel_diff_m_s{0.f};
};

} // namespace vyom
