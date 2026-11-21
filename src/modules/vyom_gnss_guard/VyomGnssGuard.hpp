/**
 * @file VyomGnssGuard.hpp
 * Runs vyom::GnssGuard on every GNSS sample and publishes vyom_gnss_status, which EKF2 uses to
 * stop GNSS fusion (upstream patch #3).
 */
#pragma once

#include "GnssGuard.hpp"

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vyom_gnss_status.h>

using namespace time_literals;

class VyomGnssGuard : public ModuleBase<VyomGnssGuard>, public ModuleParams, public px4::WorkItem
{
public:
	VyomGnssGuard();
	~VyomGnssGuard() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	bool init();

private:
	void Run() override;
	void project_to_ekf_frame(const sensor_gps_s &gps, vyom::EkfState &ekf, float &gn, float &ge);
	void update_params();
	vyom::Cn0Stats cn0_stats();

	vyom::GnssGuard _guard;

	MapProjection _own_ref{};   // used only until the EKF has a global reference

	uORB::SubscriptionCallbackWorkItem _gps_sub{this, ORB_ID(vehicle_gps_position)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vio_sub{ORB_ID(vehicle_visual_odometry)};
	uORB::Subscription _land_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _sat_sub{ORB_ID(satellite_info)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Publication<vyom_gnss_status_s> _status_pub{ORB_ID(vyom_gnss_status)};

	vehicle_local_position_s _lpos{};
	vehicle_odometry_s _vio{};
	bool _landed{true};
	satellite_info_s _sat{};
	hrt_abstime _sat_us{0};
	bool _was_rejected{false};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::VYM_GG_JUMP>) _param_jump,
		(ParamFloat<px4::params::VYM_GG_VIO_POS>) _param_vio_pos,
		(ParamFloat<px4::params::VYM_GG_VIO_VEL>) _param_vio_vel,
		(ParamFloat<px4::params::VYM_GG_RECOV>) _param_recov
	)
};
