#include "VyomWdt.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>  // board_config.h: GPIO_WDT_WDI on core-v1

VyomWdt::VyomWdt() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

void VyomWdt::set_wdi(bool level)
{
#if defined(GPIO_WDT_WDI)
	px4_arch_gpiowrite(GPIO_WDT_WDI, level);
#else
	(void)level;
#endif
}

void VyomWdt::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	vehicle_angular_velocity_s gyro{};

	if (_gyro_sub.update(&gyro)) {
		_last_gyro_us = gyro.timestamp;
	}

	vehicle_torque_setpoint_s torque{};

	if (_torque_sub.update(&torque)) {
		_last_torque_us = torque.timestamp;
	}

	vyom::WdtInputs in{};
	in.now_us = hrt_absolute_time();
	in.start_us = _start_us;
	in.last_gyro_us = _last_gyro_us;
	in.last_torque_us = _last_torque_us;
	const vyom::WdtDecision d = vyom::wdt_decide(in, _limits);

	if (vyom::wdt_is_kick(d)) {
		_wdi_level = !_wdi_level;   // the watchdog counts edges
		set_wdi(_wdi_level);
		_kicks++;

	} else {
		_holds++;

		if (d != _last) {
			// Last words before the external watchdog resets the FC.
			PX4_ERR("watchdog kicks stopped (%s)", d == vyom::WdtDecision::HoldGyroStale ? "gyro stale" :
				d == vyom::WdtDecision::HoldRateStale ? "rate loop stale" : "control path never started");
		}
	}

	_last = d;
}

int VyomWdt::print_status()
{
	PX4_INFO("kick period %" PRId32 " ms, kicks %" PRIu32 ", holds %" PRIu32 ", last decision %u",
		 _param_kick_ms.get(), _kicks, _holds, static_cast<unsigned>(_last));
#if !defined(GPIO_WDT_WDI)
	PX4_INFO("no GPIO_WDT_WDI on this board: kicks are counted only");
#endif
	return 0;
}

int VyomWdt::task_spawn(int argc, char *argv[])
{
	VyomWdt *instance = new VyomWdt(); // vyom-alloc-ok: module start (init), once

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->_start_us = hrt_absolute_time();
	const int32_t kick_ms = instance->_param_kick_ms.get();
	instance->ScheduleOnInterval(static_cast<uint32_t>(kick_ms > 0 ? kick_ms : 100) * 1000u);
	return PX4_OK;
}

int VyomWdt::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VyomWdt::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Kicks the external watchdog while gyro data and rate-controller output are fresh.
If the flight control path stalls, kicks stop and the watchdog resets the FC.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("vyom_wdt", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int vyom_wdt_main(int argc, char *argv[])
{
	return VyomWdt::main(argc, argv);
}
