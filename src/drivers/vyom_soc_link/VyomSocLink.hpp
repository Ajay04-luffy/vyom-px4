/**
 * @file VyomSocLink.hpp
 * FC side of the Vyom FC<->Vision SoC link (/proto v0 over UART).
 *
 * Sends: heartbeat (10 Hz), fc_state (50 Hz), time-sync requests (10 Hz) and replies.
 * Receives: SoC heartbeat (-> vyom_soc_status.soc_alive), vision odometry
 * (-> vehicle_visual_odometry, FC clock), time sync. Trajectory setpoints are counted, not used (WP5).
 */
#pragma once

#include "SocLinkCore.hpp"

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/timesync/Timesync.hpp>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vyom_soc_status.h>

class VyomSocLink : public ModuleBase<VyomSocLink>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VyomSocLink(const char *device, uint32_t baudrate_bps);
	~VyomSocLink() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

private:
	static constexpr uint32_t RUN_INTERVAL_US{2000};      // 500 Hz: < 1 kB per run at 3 Mbaud
	static constexpr hrt_abstime HEARTBEAT_PERIOD_US{100000};
	static constexpr hrt_abstime FC_STATE_PERIOD_US{20000};
	static constexpr hrt_abstime TIMESYNC_PERIOD_US{100000};
	static constexpr hrt_abstime STATUS_PERIOD_US{100000};

	void Run() override;
	bool open_port();
	void receive(hrt_abstime now_us);
	void send_periodic(hrt_abstime now_us);
	void publish_odometry();
	void publish_status(hrt_abstime now_us);
	void send(const uint8_t *buf, size_t len);
	void fill_fc_state(vl_fc_state_t &m, hrt_abstime now_us);

	device::Serial _uart;
	char _device[32] {};
	uint32_t _baudrate_bps{0};

	vyom::SocLinkCore _core;
	Timesync _timesync{timesync_status_s::SOURCE_PROTOCOL_UNKNOWN};

	uint8_t _rx_buf[1024] {};
	uint8_t _tx_buf[VL_MAX_ENCODED] {};

	hrt_abstime _next_heartbeat_us{0};
	hrt_abstime _next_fc_state_us{0};
	hrt_abstime _next_timesync_us{0};
	hrt_abstime _next_status_us{0};
	bool _was_alive{false};
	bool _open_error_reported{false};
	uint32_t _odometry_dropped{0};
	uint32_t _tx_overruns{0};
	uint64_t _rx_bytes{0};

	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _battery_sub{ORB_ID(battery_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Publication<vehicle_odometry_s> _odometry_pub{ORB_ID(vehicle_visual_odometry)};
	uORB::Publication<vyom_soc_status_s> _status_pub{ORB_ID(vyom_soc_status)};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _rx_bytes_perf{perf_alloc(PC_COUNT, MODULE_NAME": rx bytes")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::VYM_SOC_TOUT>) _param_soc_tout
	)
};
