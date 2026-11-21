#include "VyomSocLink.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <math.h>
#include <string.h>

VyomSocLink::VyomSocLink(const char *device, uint32_t baudrate_bps) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device)),
	_baudrate_bps(baudrate_bps)
{
	strncpy(_device, device, sizeof(_device) - 1);
	_device[sizeof(_device) - 1] = '\0';
	_core.set_heartbeat_timeout_us(static_cast<uint64_t>(_param_soc_tout.get() * 1e6f));
}

VyomSocLink::~VyomSocLink()
{
	perf_free(_loop_perf);
	perf_free(_rx_bytes_perf);
}

bool VyomSocLink::open_port()
{
	if (_uart.isOpen()) {
		return true;
	}

	if (!_uart.setPort(_device) || !_uart.setBaudrate(_baudrate_bps) || !_uart.open()) {
		if (!_open_error_reported) {
			PX4_ERR("cannot open %s at %" PRIu32 " baud, retrying every second", _device, _baudrate_bps);
			_open_error_reported = true;
		}

		return false;
	}

	_uart.flush();
	PX4_INFO("SoC link on %s at %" PRIu32 " baud", _device, _baudrate_bps);
	return true;
}

void VyomSocLink::Run()
{
	if (should_exit()) {
		ScheduleClear();
		(void)_uart.close();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pu{};
		_parameter_update_sub.copy(&pu);
		updateParams();
		_core.set_heartbeat_timeout_us(static_cast<uint64_t>(_param_soc_tout.get() * 1e6f));
	}

	if (!open_port()) {
		// Retry once a second; soc_alive stays false so failsafes see a dead SoC.
		publish_status(hrt_absolute_time());
		ScheduleDelayed(1_s);
		return;
	}

	perf_begin(_loop_perf);
	const hrt_abstime now_us = hrt_absolute_time();
	receive(now_us);
	publish_odometry();
	send_periodic(now_us);

	if (now_us >= _next_status_us || (_core.soc_alive(now_us) != _was_alive)) {
		publish_status(now_us);
	}

	perf_end(_loop_perf);
	ScheduleDelayed(RUN_INTERVAL_US);
}

void VyomSocLink::receive(hrt_abstime now_us)
{
	for (int i = 0; i < 8; i++) {   // bounded: at most 8 kB per run
		const ssize_t n = _uart.read(_rx_buf, sizeof(_rx_buf));

		if (n <= 0) {
			break;
		}

		_rx_bytes += static_cast<uint64_t>(n);
		perf_set_count(_rx_bytes_perf, _rx_bytes);
		_core.push(_rx_buf, static_cast<size_t>(n), now_us);
	}

	vl_timesync_t ts{};
	uint64_t rx_us = 0;

	if (_core.take_timesync_sample(ts, rx_us)) {
		_timesync.update(rx_us, ts.tc1_us * 1000, ts.ts1_us * 1000);
	}

	if (_core.take_timesync_reply(ts)) {
		ts.tc1_us = static_cast<int64_t>(hrt_absolute_time());
		send(_tx_buf, _core.encode_timesync(ts, _tx_buf, sizeof(_tx_buf)));
	}
}

void VyomSocLink::publish_odometry()
{
	vl_vision_odometry_t od{};
	uint64_t rx_us = 0;

	if (!_core.take_odometry(od, rx_us)) {
		return;
	}

	if (!_timesync.sync_converged() || (od.quality_pct == 0u)) {
		// Without a converged clock offset the sample time is unknown; fusing it would be wrong.
		_odometry_dropped++;
		return;
	}

	vehicle_odometry_s out{};
	out.timestamp_sample = _timesync.sync_stamp(od.timestamp_us);

	if (out.timestamp_sample > rx_us) {
		out.timestamp_sample = rx_us;   // never in the future
	}

	out.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
	out.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;

	for (int i = 0; i < 3; i++) {
		out.position[i] = od.pos_ned_m[i];
		out.velocity[i] = od.vel_ned_m_s[i];
		out.angular_velocity[i] = NAN;
		out.position_variance[i] = od.pos_var_m2;
		out.orientation_variance[i] = od.att_var_rad2;
		out.velocity_variance[i] = od.vel_var_m2_s2;
	}

	for (int i = 0; i < 4; i++) {
		out.q[i] = od.q[i];
	}

	out.reset_counter = od.reset_counter;
	out.quality = static_cast<int8_t>(od.quality_pct > 100u ? 100u : od.quality_pct);
	out.timestamp = hrt_absolute_time();
	_odometry_pub.publish(out);
}

void VyomSocLink::fill_fc_state(vl_fc_state_t &m, hrt_abstime now_us)
{
	m.timestamp_us = now_us;

	vehicle_attitude_s att{};

	if (_attitude_sub.copy(&att)) {
		for (int i = 0; i < 4; i++) { m.q[i] = att.q[i]; }

	} else {
		for (int i = 0; i < 4; i++) { m.q[i] = NAN; }
	}

	vehicle_local_position_s lp{};
	const bool have_lp = _local_pos_sub.copy(&lp);
	m.pos_ned_m[0] = (have_lp && lp.xy_valid) ? lp.x : NAN;
	m.pos_ned_m[1] = (have_lp && lp.xy_valid) ? lp.y : NAN;
	m.pos_ned_m[2] = (have_lp && lp.z_valid) ? lp.z : NAN;
	m.vel_ned_m_s[0] = (have_lp && lp.v_xy_valid) ? lp.vx : NAN;
	m.vel_ned_m_s[1] = (have_lp && lp.v_xy_valid) ? lp.vy : NAN;
	m.vel_ned_m_s[2] = (have_lp && lp.v_z_valid) ? lp.vz : NAN;

	battery_status_s bat{};
	m.battery_remaining_pct = 255u;

	if (_battery_sub.copy(&bat) && bat.connected && (bat.remaining >= 0.f)) {
		m.battery_remaining_pct = static_cast<uint8_t>(lroundf(fminf(bat.remaining, 1.f) * 100.f));
	}

	vehicle_status_s st{};

	if (_status_sub.copy(&st)) {
		m.armed = (st.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 1u : 0u;
		m.nav_state = st.nav_state;
		m.failsafe = st.failsafe ? 1u : 0u;
	}
}

void VyomSocLink::send_periodic(hrt_abstime now_us)
{
	if (now_us >= _next_heartbeat_us) {
		_next_heartbeat_us = now_us + HEARTBEAT_PERIOD_US;
		vl_heartbeat_t hb{};
		hb.uptime_ms = static_cast<uint32_t>(now_us / 1000u);
		hb.schema_minor = VL_SCHEMA_MINOR;
		hb.role = vyom::SocLinkCore::ROLE_FC;
		hb.health = 1u;
		send(_tx_buf, _core.encode_heartbeat(hb, _tx_buf, sizeof(_tx_buf)));
	}

	if (now_us >= _next_fc_state_us) {
		_next_fc_state_us = now_us + FC_STATE_PERIOD_US;
		vl_fc_state_t st{};
		fill_fc_state(st, now_us);
		send(_tx_buf, _core.encode_fc_state(st, _tx_buf, sizeof(_tx_buf)));
	}

	if (now_us >= _next_timesync_us) {
		_next_timesync_us = now_us + TIMESYNC_PERIOD_US;
		vl_timesync_t ts{};
		ts.tc1_us = 0;
		ts.ts1_us = static_cast<int64_t>(now_us);
		send(_tx_buf, _core.encode_timesync(ts, _tx_buf, sizeof(_tx_buf)));
	}
}

void VyomSocLink::send(const uint8_t *buf, size_t len)
{
	if (len == 0u) {
		return;
	}

	// Never block the work queue: a frame that does not fit in the TX buffer is dropped whole.
	const ssize_t n = _uart.write(buf, len);

	if (n != static_cast<ssize_t>(len)) {
		_tx_overruns++;
	}
}

void VyomSocLink::publish_status(hrt_abstime now_us)
{
	const vyom::SocLinkCore::Counters c = _core.counters();
	vyom_soc_status_s s{};
	s.soc_alive = _core.soc_alive(now_us);
	s.last_heartbeat = _core.last_heartbeat_us();
	s.soc_health = _core.last_heartbeat().health;
	s.soc_schema_minor = _core.last_heartbeat().schema_minor;
	s.soc_uptime_ms = _core.last_heartbeat().uptime_ms;
	s.timesync_converged = _timesync.sync_converged();
	s.timesync_offset_us = _timesync.offset();
	s.frames_ok = c.frames_ok;
	s.crc_errors = c.crc_errors;
	s.framing_errors = c.framing_errors;
	s.version_errors = c.version_errors;
	s.rejected_msgs = c.rejected_msgs;
	s.odometry_dropped = _odometry_dropped;
	s.setpoints_ignored = c.setpoints_ignored;
	s.tx_overruns = _tx_overruns;
	s.timestamp = hrt_absolute_time();
	_status_pub.publish(s);

	if (s.soc_alive != _was_alive) {
		if (s.soc_alive) {
			PX4_INFO("SoC link up");

		} else {
			PX4_WARN("SoC link lost");
		}

		_was_alive = s.soc_alive;
	}

	_next_status_us = now_us + STATUS_PERIOD_US;
}

int VyomSocLink::print_status()
{
	const hrt_abstime now_us = hrt_absolute_time();
	const vyom::SocLinkCore::Counters c = _core.counters();
	PX4_INFO("device %s, %" PRIu32 " baud, %s", _device, _baudrate_bps, _uart.isOpen() ? "open" : "NOT open");
	PX4_INFO("soc_alive %d, soc_health %u, timesync %s (offset %" PRId64 " us)", _core.soc_alive(now_us),
		 _core.last_heartbeat().health, _timesync.sync_converged() ? "converged" : "not converged", _timesync.offset());
	PX4_INFO("frames ok %" PRIu32 ", crc %" PRIu32 ", framing %" PRIu32 ", version %" PRIu32 ", rejected %" PRIu32,
		 c.frames_ok, c.crc_errors, c.framing_errors, c.version_errors, c.rejected_msgs);
	PX4_INFO("odometry dropped %" PRIu32 ", setpoints ignored %" PRIu32 ", tx overruns %" PRIu32,
		 _odometry_dropped, c.setpoints_ignored, _tx_overruns);
	perf_print_counter(_loop_perf);
	perf_print_counter(_rx_bytes_perf);
	return 0;
}

int VyomSocLink::task_spawn(int argc, char *argv[])
{
	const char *device = nullptr;
	uint32_t baudrate_bps = 3000000;
	int myoptind = 1;
	int ch = 0;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case 'b':
			baudrate_bps = static_cast<uint32_t>(strtoul(myoptarg, nullptr, 10));
			break;

		default:
			print_usage("unrecognized flag");
			return PX4_ERROR;
		}
	}

	if ((device == nullptr) || (baudrate_bps == 0u)) {
		print_usage("device and baud rate required");
		return PX4_ERROR;
	}

	VyomSocLink *instance = new VyomSocLink(device, baudrate_bps); // vyom-alloc-ok: module start (init), once

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return PX4_OK;
}

int VyomSocLink::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VyomSocLink::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
FC side of the Vyom FC<->Vision SoC link (/proto v0: COBS frames with CRC-16 over UART).
Publishes vyom_soc_status (SoC liveness for failsafes) and vehicle_visual_odometry.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vyom_soc_link", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('b', 3000000, 9600, 12000000, "Baud rate", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int vyom_soc_link_main(int argc, char *argv[])
{
	return VyomSocLink::main(argc, argv);
}
