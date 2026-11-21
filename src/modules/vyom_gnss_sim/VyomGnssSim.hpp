/**
 * @file VyomGnssSim.hpp
 * SIMULATION ONLY (POSIX builds). A GNSS receiver model with injectable faults, used instead of
 * PX4's sensor_gps_sim (set SENS_EN_GPSSIM 0) in the WP3 fault scenarios.
 *
 * VYM_SIM_GF_MODE: 0 none, 1 step jump of VYM_SIM_GF_MAG m, 2 drift spoof at VYM_SIM_GF_MAG m/s,
 * 3 jamming (receiver flag, noise x10, fix lost after 10 s), 4 drift spoof the receiver detects
 * (spoofing flag), 5 drift spoof with uniform strong C/N0 (no receiver flag).
 * Direction of the spoof offset: VYM_SIM_GF_DIR (deg from north).
 */
#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

class VyomGnssSim : public ModuleBase<VyomGnssSim>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VyomGnssSim();
	~VyomGnssSim() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

private:
	void Run() override;
	float gauss();
	float update_fault(hrt_abstime now, int32_t mode, float &spoof_vn, float &spoof_ve);
	void publish_sat_info(hrt_abstime now, bool uniform);

	uORB::Subscription _gpos_sub{ORB_ID(vehicle_global_position_groundtruth)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position_groundtruth)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::PublicationMulti<sensor_gps_s> _gps_pub{ORB_ID(sensor_gps)};
	uORB::PublicationMulti<satellite_info_s> _sat_pub{ORB_ID(satellite_info)};

	uint32_t _rng{0x12345678u};
	int32_t _mode{0};
	hrt_abstime _fault_start{0};
	hrt_abstime _last_sat{0};
	float _offset_n_m{0.f};
	float _offset_e_m{0.f};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VYM_SIM_GF_MODE>) _param_mode,
		(ParamFloat<px4::params::VYM_SIM_GF_MAG>) _param_mag,
		(ParamFloat<px4::params::VYM_SIM_GF_DIR>) _param_dir
	)
};
