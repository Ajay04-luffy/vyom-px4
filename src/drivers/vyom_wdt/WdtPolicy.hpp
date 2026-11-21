/**
 * @file WdtPolicy.hpp
 * Decides whether the FC may kick its external watchdog (no PX4 dependencies; unit-tested).
 *
 * The external watchdog resets the FC if it stops being kicked. We kick only while the flight
 * control path is alive: gyro data arriving (sensors) and the rate controller producing torque
 * setpoints. During boot (before the first gyro sample) we kick for a grace period so the FC can
 * finish starting; after that a dead control path stops the kicks and the watchdog resets the FC.
 */
#pragma once

#include <stdint.h>

namespace vyom
{

enum class WdtDecision : uint8_t {
	KickBoot = 0,      // boot grace period, control path not up yet
	Kick = 1,          // control path healthy
	HoldGyroStale = 2, // no gyro sample within the limit
	HoldRateStale = 3, // no rate-controller output within the limit
	HoldBootTimeout = 4, // control path never came up within the grace period
};

struct WdtInputs {
	uint64_t now_us;
	uint64_t start_us;        // when the watchdog module started
	uint64_t last_gyro_us;    // 0 = never
	uint64_t last_torque_us;  // 0 = never
};

struct WdtLimits {
	uint64_t stale_us{200000};       // 200 ms: the gyro runs at >= 1 kHz, so this is a dead loop
	uint64_t boot_grace_us{20000000}; // 20 s for sensors, estimator and controllers to start
};

inline WdtDecision wdt_decide(const WdtInputs &in, const WdtLimits &lim)
{
	const bool control_path_seen = (in.last_gyro_us != 0u) && (in.last_torque_us != 0u);

	if (!control_path_seen) {
		return ((in.now_us - in.start_us) <= lim.boot_grace_us) ? WdtDecision::KickBoot : WdtDecision::HoldBootTimeout;
	}

	if ((in.now_us < in.last_gyro_us) || ((in.now_us - in.last_gyro_us) > lim.stale_us)) {
		return WdtDecision::HoldGyroStale;
	}

	if ((in.now_us < in.last_torque_us) || ((in.now_us - in.last_torque_us) > lim.stale_us)) {
		return WdtDecision::HoldRateStale;
	}

	return WdtDecision::Kick;
}

inline bool wdt_is_kick(WdtDecision d)
{
	return (d == WdtDecision::Kick) || (d == WdtDecision::KickBoot);
}

} // namespace vyom
