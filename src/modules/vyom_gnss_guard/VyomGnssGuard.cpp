#include "VyomGnssGuard.hpp"

#include <math.h>
#include <px4_platform_common/log.h>

VyomGnssGuard::VyomGnssGuard() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	update_params();
}

VyomGnssGuard::~VyomGnssGuard()
{
	perf_free(_loop_perf);
}

bool VyomGnssGuard::init()
{
	return _gps_sub.registerCallback();
}

void VyomGnssGuard::update_params()
{
	vyom::GnssGuardParams p{};
	p.jump_m = _param_jump.get();
	p.vio_pos_m = _param_vio_pos.get();
	p.vio_vel_m_s = _param_vio_vel.get();
	p.recover_s = _param_recov.get();
	_guard.set_params(p);
}

vyom::Cn0Stats VyomGnssGuard::cn0_stats()
{
	vyom::Cn0Stats s{0u, NAN, NAN};

	if (_sat_sub.update(&_sat)) {
		_sat_us = _sat.timestamp;
	}

	if ((_sat_us == 0u) || (hrt_elapsed_time(&_sat_us) > 3_s)) {
		return s;
	}

	float sum = 0.f;
	float sum_sq = 0.f;
	uint8_t n = 0u;
	const uint8_t count = (_sat.count < satellite_info_s::SAT_INFO_MAX_SATELLITES) ? _sat.count :
			      satellite_info_s::SAT_INFO_MAX_SATELLITES;

	for (uint8_t i = 0u; i < count; i++) {
		if ((_sat.used[i] != 0u) && (_sat.snr[i] > 0u)) {
			const float c = static_cast<float>(_sat.snr[i]);
			sum += c;
			sum_sq += c * c;
			n++;
		}
	}

	if (n > 0u) {
		const float mean = sum / static_cast<float>(n);
		s.used_sats = n;
		s.mean_dbhz = mean;
		s.std_dbhz = sqrtf(fmaxf(sum_sq / static_cast<float>(n) - mean * mean, 0.f));
	}

	return s;
}

void VyomGnssGuard::project_to_ekf_frame(const sensor_gps_s &gps, vyom::EkfState &ekf, float &gn, float &ge)
{
	// GNSS position in the EKF's local frame (or our own frame until the EKF has a reference).
	if (_lpos.xy_global && (_lpos.timestamp != 0u)) {
		const MapProjection ref(_lpos.ref_lat, _lpos.ref_lon);
		ref.project(gps.latitude_deg, gps.longitude_deg, gn, ge);
		ekf.pos_valid = _lpos.xy_valid;
		ekf.pos_n_m = _lpos.x;
		ekf.pos_e_m = _lpos.y;

	} else {
		if (!_own_ref.isInitialized()) {
			_own_ref.initReference(gps.latitude_deg, gps.longitude_deg, gps.timestamp);
		}

		_own_ref.project(gps.latitude_deg, gps.longitude_deg, gn, ge);
		ekf.pos_valid = false;
	}
}

void VyomGnssGuard::Run()
{
	if (should_exit()) {
		_gps_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pu{};
		_parameter_update_sub.copy(&pu);
		updateParams();
		update_params();
	}

	sensor_gps_s gps{};

	if (!_gps_sub.update(&gps)) {
		return;
	}

	perf_begin(_loop_perf);
	(void)_lpos_sub.update(&_lpos);
	(void)_vio_sub.update(&_vio);
	vehicle_land_detected_s land{};

	if (_land_sub.update(&land)) {
		_landed = land.landed;
	}

	const vyom::Cn0Stats cn0 = cn0_stats();

	// Only judge samples with a 3D fix; without one the EKF does not use GNSS anyway.
	if (gps.fix_type < 3) {
		perf_end(_loop_perf);
		return;
	}

	vyom::EkfState ekf{};
	float gn = 0.f;
	float ge = 0.f;
	project_to_ekf_frame(gps, ekf, gn, ge);
	ekf.vel_valid = _lpos.v_xy_valid;
	ekf.vel_n_m_s = _lpos.vx;
	ekf.vel_e_m_s = _lpos.vy;
	ekf.landed = _landed;

	vyom::GnssSample g{};
	g.time_us = gps.timestamp;
	g.pos_n_m = gn;
	g.pos_e_m = ge;
	g.vel_n_m_s = gps.vel_ned_valid ? gps.vel_n_m_s : NAN;
	g.vel_e_m_s = gps.vel_ned_valid ? gps.vel_e_m_s : NAN;
	g.rx_spoofing = (gps.spoofing_state == sensor_gps_s::SPOOFING_STATE_DETECTED);
	g.rx_jamming = (gps.jamming_state == sensor_gps_s::JAMMING_STATE_DETECTED);

	vyom::VioSample v{};
	const bool vio_ned = (_vio.pose_frame == vehicle_odometry_s::POSE_FRAME_NED);
	v.time_us = (vio_ned && (_vio.timestamp_sample != 0u)) ? _vio.timestamp_sample : 0u;
	v.pos_n_m = _vio.position[0];
	v.pos_e_m = _vio.position[1];
	const bool vel_ned = (_vio.velocity_frame == vehicle_odometry_s::VELOCITY_FRAME_NED);
	v.vel_n_m_s = vel_ned ? _vio.velocity[0] : NAN;
	v.vel_e_m_s = vel_ned ? _vio.velocity[1] : NAN;
	v.reset_counter = _vio.reset_counter;

	const bool reject = _guard.update(g, ekf, v, cn0);

	vyom_gnss_status_s st{};
	st.reject = reject;
	st.reasons = _guard.reasons();
	st.reasons_latched = _guard.reasons_latched();
	st.jump_m = _guard.jump_m();
	st.vio_pos_diff_m = _guard.vio_pos_diff_m();
	st.vio_vel_diff_m_s = _guard.vio_vel_diff_m_s();
	st.cn0_mean_dbhz = cn0.mean_dbhz;
	st.cn0_std_dbhz = cn0.std_dbhz;
	st.reject_count = _guard.reject_count();
	st.timestamp = hrt_absolute_time();
	_status_pub.publish(st);

	if (reject != _was_rejected) {
		if (reject) {
			PX4_WARN("GNSS rejected (reasons 0x%02x)", static_cast<unsigned>(st.reasons_latched));

		} else {
			PX4_INFO("GNSS accepted again");
		}

		_was_rejected = reject;
	}

	perf_end(_loop_perf);
}

int VyomGnssGuard::print_status()
{
	PX4_INFO("reject %d, reasons 0x%02x (latched 0x%02x), rejections %" PRIu32,
		 _guard.reject(), _guard.reasons(), _guard.reasons_latched(), _guard.reject_count());
	PX4_INFO("jump %.2f m, vio pos diff %.2f m, vio vel diff %.2f m/s", (double)_guard.jump_m(),
		 (double)_guard.vio_pos_diff_m(), (double)_guard.vio_vel_diff_m_s());
	perf_print_counter(_loop_perf);
	return 0;
}

int VyomGnssGuard::task_spawn(int argc, char *argv[])
{
	VyomGnssGuard *instance = new VyomGnssGuard(); // vyom-alloc-ok: module start (init), once

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (!instance->init()) {
		PX4_ERR("init failed");
		delete instance; // vyom-alloc-ok: start failed, nothing is running yet
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int VyomGnssGuard::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VyomGnssGuard::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
GNSS integrity monitor: receiver spoofing/jamming flags, position jumps, disagreement with
visual odometry and C/N0 uniformity. Publishes vyom_gnss_status; EKF2 stops GNSS fusion while
it rejects.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("vyom_gnss_guard", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int vyom_gnss_guard_main(int argc, char *argv[])
{
	return VyomGnssGuard::main(argc, argv);
}
