/****************************************************************************
 * SITL TEST ONLY. Emulates the skybeta kit's flight_interface (Jetson), which
 * publishes /fmu/in/trajectory_setpoint at 10 Hz in every PX4 mode, through
 * the uXRCE-DDS client, into uORB trajectory_setpoint. While autonomy is still
 * flying, that sample is its forward velocity: the worst case for a bail-out.
 *
 *   dh_fi_emulator start [-x vx] [-y vy]    (m/s NED, default 1 0)
 ****************************************************************************/

#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <stdlib.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/trajectory_setpoint.h>

using namespace time_literals;

class DhFiEmulator : public ModuleBase<DhFiEmulator>, public px4::ScheduledWorkItem
{
public:
	DhFiEmulator(float vx, float vy) :
		ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default), _vx(vx), _vy(vy) {}

	static int task_spawn(int argc, char *argv[])
	{
		float vx = 1.f;
		float vy = 0.f;
		int myoptind = 1;
		int ch;
		const char *myoptarg = nullptr;

		while ((ch = px4_getopt(argc, argv, "x:y:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'x': vx = strtof(myoptarg, nullptr); break;

			case 'y': vy = strtof(myoptarg, nullptr); break;

			default: return print_usage("unknown option");
			}
		}

		DhFiEmulator *instance = new DhFiEmulator(vx, vy);

		if (!instance) {
			return PX4_ERROR;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;
		instance->ScheduleOnInterval(100_ms);  // flight_interface setpoint_rate_hz 10
		return PX4_OK;
	}

	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }

	static int print_usage(const char *reason = nullptr)
	{
		if (reason) {
			PX4_WARN("%s", reason);
		}

		PRINT_MODULE_DESCRIPTION("SITL test only: flight_interface trajectory_setpoint stream (10 Hz).");
		PRINT_MODULE_USAGE_NAME("dh_fi_emulator", "test");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_PARAM_FLOAT('x', 1.f, -5.f, 5.f, "north velocity m/s", true);
		PRINT_MODULE_USAGE_PARAM_FLOAT('y', 0.f, -5.f, 5.f, "east velocity m/s", true);
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}

	int print_status() override
	{
		PX4_INFO("published %u setpoints (%.2f, %.2f) m/s", static_cast<unsigned>(_count),
			 static_cast<double>(_vx), static_cast<double>(_vy));
		return 0;
	}

private:
	void Run() override
	{
		if (should_exit()) {
			ScheduleClear();
			exit_and_cleanup();
			return;
		}

		// flight_interface build_setpoint(), mode 'velocity'.
		trajectory_setpoint_s sp{};
		sp.timestamp = hrt_absolute_time();
		sp.position[0] = sp.position[1] = sp.position[2] = NAN;
		sp.velocity[0] = _vx;
		sp.velocity[1] = _vy;
		sp.velocity[2] = 0.f;
		sp.acceleration[0] = sp.acceleration[1] = sp.acceleration[2] = NAN;
		sp.jerk[0] = sp.jerk[1] = sp.jerk[2] = NAN;
		sp.yaw = NAN;
		sp.yawspeed = 0.f;
		_pub.publish(sp);
		++_count;
	}

	uORB::Publication<trajectory_setpoint_s> _pub{ORB_ID(trajectory_setpoint)};
	float _vx;
	float _vy;
	uint32_t _count{0};
};

extern "C" __EXPORT int dh_fi_emulator_main(int argc, char *argv[])
{
	return DhFiEmulator::main(argc, argv);
}
