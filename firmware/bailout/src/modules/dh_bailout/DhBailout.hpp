/****************************************************************************
 * DroneHacks "DH Bailout" flight mode, running on the flight controller.
 *
 * Registers itself with commander through the same interface a ROS 2
 * external mode uses (register_ext_component_request), so commander lists it
 * as an external mode ("DH Bailout", External Mode 1 on a fresh FC) and a
 * COM_FLTMODEx value of 100 + index puts it on the RC mode switch. Nothing runs
 * on the companion computer: it is part of the firmware.
 *
 * While selected and armed it streams trajectory_setpoint from BailoutLogic:
 * brake to rest, settle, descend; PX4's land detector and COM_DISARM_LAND then
 * disarm. It never arms and never takes off (mode_req_prevent_arming).
 *
 * Kept small on purpose: the organisers' Pixhawk 6C image leaves ~6 KB of flash.
 * So no ModuleBase (only "start" and "status"), no parameters, short strings.
 ****************************************************************************/
#pragma once

#include "bailout_logic.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class DhBailout : public px4::ScheduledWorkItem
{
public:
	DhBailout();

	bool init();
	void printStatus() const;

	static constexpr const char *kModeName = "DH Bailout";

private:
	void Run() override;

	void handleRegistration(hrt_abstime now);
	void replyToArmingCheck(const arming_check_request_s &request);
	void publishControlConfig();
	dh_bailout::VehicleState vehicleState(hrt_abstime now) const;
	void publishSetpoint(const dh_bailout::Setpoint &sp);

	/// No estimate update for this long: the logic treats the estimate as stale.
	static constexpr hrt_abstime kLocalPositionStale = 300_ms;
	/// Retry a registration request that got no reply.
	static constexpr hrt_abstime kRegistrationRetry = 1_s;
	/// Keep publishing at least this often while active, even without estimates.
	static constexpr hrt_abstime kMinPublishInterval = 20_ms;

	// Run() is triggered by any of these.
	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};
	uORB::SubscriptionCallbackWorkItem _arming_check_request_sub{this, ORB_ID(arming_check_request)};
	uORB::SubscriptionCallbackWorkItem _trajectory_setpoint_sub{this, ORB_ID(trajectory_setpoint)};

	uORB::Subscription _register_reply_sub{ORB_ID(register_ext_component_reply)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};

	uORB::Publication<register_ext_component_request_s> _register_request_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<arming_check_reply_s> _arming_check_reply_pub{ORB_ID(arming_check_reply)};
	uORB::Publication<vehicle_control_mode_s> _config_control_setpoints_pub{ORB_ID(config_control_setpoints)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};

	vehicle_local_position_s _local_pos{};
	hrt_abstime _local_pos_received{0};
	bool _landed{true};
	uint8_t _nav_state{0};
	bool _armed{false};

	// Registration with commander.
	uint64_t _request_id{0};
	hrt_abstime _request_sent{0};
	int8_t _arming_check_id{-1};
	int8_t _mode_id{-1};  ///< our nav_state once registered
	uint16_t _registration_attempts{0};

	dh_bailout::BailoutLogic _logic{};
	hrt_abstime _t_origin{0};  ///< activation time: the logic's clock starts at 0 there (float precision)
	dh_bailout::Setpoint _last_sp{};
	uint64_t _last_published_timestamp{0};
	hrt_abstime _last_publish{0};

	// Status counters.
	uint16_t _activations{0};
	uint32_t _foreign_setpoints{0};  ///< another publisher's trajectory_setpoint seen while active
	uint32_t _arming_check_replies{0};
};
