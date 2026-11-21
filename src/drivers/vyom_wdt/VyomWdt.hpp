/**
 * @file VyomWdt.hpp
 * Kicks the core-v1 external watchdog (GPIO_WDT_WDI) while the flight control path is alive.
 * See WdtPolicy.hpp for the rule. On boards without GPIO_WDT_WDI (SITL) it only counts kicks.
 */
#pragma once

#include "WdtPolicy.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

class VyomWdt : public ModuleBase<VyomWdt>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VyomWdt();
	~VyomWdt() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

private:
	void Run() override;
	static void set_wdi(bool level);

	uORB::Subscription _gyro_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _torque_sub{ORB_ID(vehicle_torque_setpoint)};

	vyom::WdtLimits _limits{};
	hrt_abstime _start_us{0};
	hrt_abstime _last_gyro_us{0};
	hrt_abstime _last_torque_us{0};
	bool _wdi_level{false};
	uint32_t _kicks{0};
	uint32_t _holds{0};
	vyom::WdtDecision _last{vyom::WdtDecision::KickBoot};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VYM_WDT_KICK_MS>) _param_kick_ms
	)
};
