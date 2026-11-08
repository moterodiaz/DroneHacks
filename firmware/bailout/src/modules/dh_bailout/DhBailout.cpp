/****************************************************************************
 * DroneHacks "DH Bailout" flight mode, running on the flight controller.
 * See DhBailout.hpp and firmware/bailout/README.md.
 ****************************************************************************/

#include "DhBailout.hpp"

#include <string.h>

using dh_bailout::Phase;

namespace
{
DhBailout *g_instance{nullptr};
}

DhBailout::DhBailout() :
	ScheduledWorkItem("dh_bailout", px4::wq_configurations::nav_and_controllers)
{
}

bool DhBailout::init()
{
	if (!_local_pos_sub.registerCallback() || !_arming_check_request_sub.registerCallback()) {
		return false;
	}

	// Registration retries and housekeeping when no estimate arrives.
	ScheduleOnInterval(100_ms);
	return true;
}

void DhBailout::Run()
{
	const hrt_abstime now = hrt_absolute_time();

	vehicle_status_s status;

	if (_vehicle_status_sub.update(&status)) {
		_nav_state = status.nav_state;
		_armed = status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	}

	vehicle_land_detected_s land_detected;

	if (_land_detected_sub.update(&land_detected)) {
		_landed = land_detected.landed;
	}

	if (_mode_id < 0) {
		handleRegistration(now);
	}

	arming_check_request_s request;

	if (_arming_check_request_sub.update(&request) && _arming_check_id >= 0) {
		replyToArmingCheck(request);
	}

	const bool new_local_pos = _local_pos_sub.update(&_local_pos);

	if (new_local_pos) {
		_local_pos_received = now;
	}

	// Another publisher's trajectory_setpoint is the latest sample: in this kit the
	// Jetson's flight_interface streams /fmu/in/trajectory_setpoint in every mode.
	bool foreign_setpoint = false;
	trajectory_setpoint_s latest;

	if (_trajectory_setpoint_sub.update(&latest)) {
		foreign_setpoint = latest.timestamp != _last_published_timestamp;
	}

	const bool want_active = _mode_id >= 0 && _nav_state == static_cast<uint8_t>(_mode_id) && _armed;

	if (want_active && !_logic.active()) {
		_t_origin = now;
		_logic.activate(vehicleState(now));
		_trajectory_setpoint_sub.registerCallback();
		++_activations;

	} else if (!want_active && _logic.active()) {
		_logic.deactivate();
		_trajectory_setpoint_sub.unregisterCallback();
	}

	if (!_logic.active()) {
		return;
	}

	if (new_local_pos || now - _last_publish >= kMinPublishInterval) {
		_last_sp = _logic.update(vehicleState(now));
		publishSetpoint(_last_sp);

	} else if (foreign_setpoint) {
		// Put ours back on top straight away, so the position controller and the land
		// detector keep reading the bail-out setpoint.
		++_foreign_setpoints;
		publishSetpoint(_last_sp);
	}
}

void DhBailout::handleRegistration(hrt_abstime now)
{
	register_ext_component_reply_s reply;

	while (_register_reply_sub.update(&reply)) {
		reply.name[sizeof(reply.name) - 1] = '\0';

		if (reply.request_id != _request_id || strncmp(reply.name, kModeName, sizeof(reply.name)) != 0) {
			continue;  // someone else's (a ROS 2 mode)
		}

		if (reply.success && reply.mode_id >= 0 && reply.arming_check_id >= 0) {
			_mode_id = reply.mode_id;
			_arming_check_id = reply.arming_check_id;
			PX4_INFO("External Mode %d", _mode_id - vehicle_status_s::NAVIGATION_STATE_EXTERNAL1 + 1);
			publishControlConfig();
			return;
		}
	}

	// Commander refuses registrations while armed; ask only on the ground.
	if (!_armed && (_request_sent == 0 || now - _request_sent >= kRegistrationRetry)) {
		register_ext_component_request_s request{};
		request.timestamp = now;
		_request_id = now;
		request.request_id = _request_id;
		strncpy(request.name, kModeName, sizeof(request.name) - 1);
		request.px4_ros2_api_version = register_ext_component_request_s::LATEST_PX4_ROS2_API_VERSION;
		request.register_arming_check = true;
		request.register_mode = true;
		_register_request_pub.publish(request);
		_request_sent = now;
		++_registration_attempts;
	}
}

void DhBailout::replyToArmingCheck(const arming_check_request_s &request)
{
	arming_check_reply_s reply{};
	reply.timestamp = hrt_absolute_time();
	reply.request_id = request.request_id;
	reply.registration_id = static_cast<uint8_t>(_arming_check_id);
	reply.health_component_index = arming_check_reply_s::HEALTH_COMPONENT_INDEX_NONE;
	reply.can_arm_and_run = true;
	// What the mode needs to be selectable: attitude control only. Without a
	// height or horizontal estimate it still flies level and descends (see
	// BailoutLogic), as PX4's own Descend does. It never arms. Every other
	// requirement flag stays false (zero-initialised).
	reply.mode_req_angular_velocity = true;
	reply.mode_req_attitude = true;
	reply.mode_req_prevent_arming = true;
	_arming_check_reply_pub.publish(reply);
	++_arming_check_replies;
}

void DhBailout::publishControlConfig()
{
	// Position, velocity and acceleration control in all axes, like Land, but not
	// an auto mode: commander copies these flags as-is for our nav_state. Manual,
	// auto, offboard and termination stay false (zero-initialised).
	vehicle_control_mode_s config{};
	config.timestamp = hrt_absolute_time();
	config.flag_control_position_enabled = true;
	config.flag_control_velocity_enabled = true;
	config.flag_control_altitude_enabled = true;
	config.flag_control_climb_rate_enabled = true;
	config.flag_control_acceleration_enabled = true;
	config.flag_control_attitude_enabled = true;
	config.flag_control_rates_enabled = true;
	config.flag_control_allocation_enabled = true;
	config.source_id = static_cast<uint8_t>(_mode_id);
	_config_control_setpoints_pub.publish(config);
}

dh_bailout::VehicleState DhBailout::vehicleState(hrt_abstime now) const
{
	dh_bailout::VehicleState s;
	s.time_s = static_cast<float>(now - _t_origin) * 1e-6f;
	s.fresh = _local_pos_received != 0 && now - _local_pos_received < kLocalPositionStale;
	s.x = _local_pos.x;
	s.y = _local_pos.y;
	s.z = _local_pos.z;
	s.xy_valid = _local_pos.xy_valid;
	s.z_valid = _local_pos.z_valid;
	s.vx = _local_pos.vx;
	s.vy = _local_pos.vy;
	s.vz = _local_pos.vz;
	s.v_xy_valid = _local_pos.v_xy_valid;
	s.v_z_valid = _local_pos.v_z_valid;
	s.heading_rad = _local_pos.heading;
	s.dist_bottom_m = _local_pos.dist_bottom;
	s.dist_bottom_valid = _local_pos.dist_bottom_valid;
	s.xy_reset_counter = _local_pos.xy_reset_counter;
	s.delta_x = _local_pos.delta_xy[0];
	s.delta_y = _local_pos.delta_xy[1];
	s.z_reset_counter = _local_pos.z_reset_counter;
	s.delta_z = _local_pos.delta_z;
	s.heading_reset_counter = _local_pos.heading_reset_counter;
	s.delta_heading = _local_pos.delta_heading;
	s.landed = _landed;
	return s;
}

void DhBailout::publishSetpoint(const dh_bailout::Setpoint &sp)
{
	trajectory_setpoint_s out{};
	const hrt_abstime now = hrt_absolute_time();
	// Strictly increasing, so a foreign sample can always be told apart from ours.
	out.timestamp = now > _last_published_timestamp ? now : _last_published_timestamp + 1;
	out.position[0] = sp.x.v;
	out.position[1] = sp.y.v;
	out.position[2] = sp.z.v;
	out.velocity[0] = sp.vx.v;
	out.velocity[1] = sp.vy.v;
	out.velocity[2] = sp.vz.v;
	out.acceleration[0] = sp.ax.v;
	out.acceleration[1] = sp.ay.v;
	out.acceleration[2] = sp.az.v;
	out.jerk[0] = out.jerk[1] = out.jerk[2] = NAN;
	out.yaw = sp.yaw.v;
	out.yawspeed = sp.yaw_rate.v;
	_trajectory_setpoint_pub.publish(out);
	_last_published_timestamp = out.timestamp;
	_last_publish = now;
}

void DhBailout::printStatus() const
{
	// One line, terse on purpose (flash): what `dh_bailout status` and the SITL test read.
	PX4_INFO("mode %d (nav %d) arm_chk %d req %u | %s act %u foreign %lu replies %lu",
		 _mode_id >= 0 ? _mode_id - vehicle_status_s::NAVIGATION_STATE_EXTERNAL1 + 1 : -1, _mode_id,
		 _arming_check_id, static_cast<unsigned>(_registration_attempts), dh_bailout::phaseName(_logic.phase()),
		 static_cast<unsigned>(_activations), static_cast<unsigned long>(_foreign_setpoints),
		 static_cast<unsigned long>(_arming_check_replies));
}

extern "C" __EXPORT int dh_bailout_main(int argc, char *argv[])
{
	if (argc >= 2 && strcmp(argv[1], "start") == 0) {
		if (g_instance) {
			return 0;
		}

		DhBailout *instance = new DhBailout();

		if (!instance || !instance->init()) {
			delete instance;
			PX4_ERR("start failed");
			return 1;
		}

		g_instance = instance;
		return 0;
	}

	if (argc >= 2 && strcmp(argv[1], "status") == 0 && g_instance) {
		g_instance->printStatus();
		return 0;
	}

	PX4_INFO("usage: dh_bailout start|status");
	return 1;
}
