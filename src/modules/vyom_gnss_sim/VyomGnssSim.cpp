#include "VyomGnssSim.hpp"

#include <lib/geo/geo.h>
#include <math.h>
#include <px4_platform_common/log.h>

VyomGnssSim::VyomGnssSim() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

float VyomGnssSim::gauss()
{
	// Box-Muller on a small LCG: deterministic runs, no libc RNG state shared with other modules.
	_rng = _rng * 1664525u + 1013904223u;
	const float u1 = (static_cast<float>(_rng >> 8) + 1.f) / 16777217.f;
	_rng = _rng * 1664525u + 1013904223u;
	const float u2 = static_cast<float>(_rng >> 8) / 16777216.f;
	return sqrtf(-2.f * logf(u1)) * cosf(2.f * M_PI_F * u2);
}

void VyomGnssSim::publish_sat_info(hrt_abstime now, bool uniform)
{
	satellite_info_s s{};
	s.count = 12;

	for (uint8_t i = 0; i < s.count; i++) {
		s.svid[i] = static_cast<uint8_t>(i + 1u);
		s.used[i] = 1u;
		s.elevation[i] = static_cast<uint8_t>(15u + 6u * i);
		s.azimuth[i] = static_cast<uint8_t>(20u * i);
		// Real sky: C/N0 depends on elevation (~30-48 dB-Hz). A spoofer's single antenna: all ~46.
		s.snr[i] = uniform ? static_cast<uint8_t>(46u + (i % 2u)) : static_cast<uint8_t>(30u + (i * 37u) % 19u);
	}

	s.timestamp = now;
	_sat_pub.publish(s);
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters): north/east outputs, named at the call
float VyomGnssSim::update_fault(hrt_abstime now, int32_t mode, float &spoof_vn, float &spoof_ve)
{
	if (mode != _mode) {
		_mode = mode;
		_fault_start = now;
		_offset_n_m = 0.f;
		_offset_e_m = 0.f;
		PX4_INFO("GNSS fault mode %" PRId32, mode);
	}

	const float dir_rad = math::radians(_param_dir.get());
	const float t_fault_s = static_cast<float>(now - _fault_start) * 1e-6f;

	if (mode == 1) {
		_offset_n_m = _param_mag.get() * cosf(dir_rad);
		_offset_e_m = _param_mag.get() * sinf(dir_rad);

	} else if ((mode == 2) || (mode == 4) || (mode == 5)) {
		spoof_vn = _param_mag.get() * cosf(dir_rad);   // a consistent spoofer moves velocity too
		spoof_ve = _param_mag.get() * sinf(dir_rad);
		_offset_n_m = spoof_vn * t_fault_s;
		_offset_e_m = spoof_ve * t_fault_s;
	}

	return t_fault_s;
}

void VyomGnssSim::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pu{};
		_parameter_update_sub.copy(&pu);
		updateParams();
	}

	vehicle_global_position_s gpos{};
	vehicle_local_position_s lpos{};

	if (!_gpos_sub.copy(&gpos) || !_lpos_sub.copy(&lpos) || (gpos.timestamp == 0u)) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const int32_t mode = _param_mode.get();
	float spoof_vn = 0.f;
	float spoof_ve = 0.f;
	const float t_fault_s = update_fault(now, mode, spoof_vn, spoof_ve);

	const bool jammed = (mode == 3);
	const float noise_m = jammed ? 3.0f : 0.3f;
	const bool fix = !jammed || (t_fault_s <= 10.f);

	const MapProjection ref(gpos.lat, gpos.lon);
	double lat = 0.0;
	double lon = 0.0;
	ref.reproject(_offset_n_m + gauss() * noise_m, _offset_e_m + gauss() * noise_m, lat, lon);

	sensor_gps_s g{};
	g.timestamp_sample = gpos.timestamp_sample;
	g.device_id = 0x00ad0001;   // arbitrary simulation device id
	g.latitude_deg = lat;
	g.longitude_deg = lon;
	g.altitude_msl_m = static_cast<double>(gpos.alt + gauss() * (jammed ? 5.f : 0.5f));
	g.altitude_ellipsoid_m = g.altitude_msl_m;
	g.vel_n_m_s = lpos.vx + spoof_vn + gauss() * (jammed ? 0.6f : 0.06f);
	g.vel_e_m_s = lpos.vy + spoof_ve + gauss() * (jammed ? 0.6f : 0.06f);
	g.vel_d_m_s = lpos.vz + gauss() * (jammed ? 0.8f : 0.08f);
	g.vel_m_s = sqrtf(g.vel_n_m_s * g.vel_n_m_s + g.vel_e_m_s * g.vel_e_m_s);
	g.cog_rad = atan2f(g.vel_e_m_s, g.vel_n_m_s);
	g.vel_ned_valid = true;
	g.fix_type = fix ? 3 : 0;
	g.eph = jammed ? 8.f : 0.9f;
	g.epv = jammed ? 12.f : 1.8f;
	g.hdop = jammed ? 4.f : 0.7f;
	g.vdop = jammed ? 6.f : 1.1f;
	g.s_variance_m_s = jammed ? 2.f : 0.4f;
	g.c_variance_rad = 0.1f;
	g.satellites_used = fix ? 12 : 0;
	g.heading = NAN;
	g.heading_offset = NAN;
	g.noise_per_ms = jammed ? 200 : 80;
	g.jamming_indicator = jammed ? 200 : 10;
	g.jamming_state = jammed ? sensor_gps_s::JAMMING_STATE_DETECTED : sensor_gps_s::JAMMING_STATE_OK;
	g.spoofing_state = (mode == 4) ? sensor_gps_s::SPOOFING_STATE_DETECTED : sensor_gps_s::SPOOFING_STATE_OK;
	g.timestamp = now;
	_gps_pub.publish(g);

	if (now - _last_sat >= 1_s) {
		_last_sat = now;
		publish_sat_info(now, mode == 5);
	}
}

int VyomGnssSim::print_status()
{
	PX4_INFO("fault mode %" PRId32 ", offset N %.1f E %.1f m", _mode, (double)_offset_n_m, (double)_offset_e_m);
	return 0;
}

int VyomGnssSim::task_spawn(int argc, char *argv[])
{
	VyomGnssSim *instance = new VyomGnssSim(); // vyom-alloc-ok: simulation-only module start

	if (instance == nullptr) {
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleOnInterval(100_ms);   // 10 Hz, like a u-blox M10 in navigation mode
	return PX4_OK;
}

int VyomGnssSim::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VyomGnssSim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
SIMULATION ONLY. GNSS receiver model with injectable jamming/spoofing faults (VYM_SIM_GF_*).
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("vyom_gnss_sim", "simulation");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int vyom_gnss_sim_main(int argc, char *argv[])
{
	return VyomGnssSim::main(argc, argv);
}
