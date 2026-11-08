/****************************************************************************
 * DroneHacks bail-out mode: the state machine.
 *
 * Brake hard to a stop, settle, then descend until PX4's land detector and
 * auto-disarm (COM_DISARM_LAND) finish the job.
 *
 * Header-only, no STL and no PX4 headers: DhBailout.cpp feeds it
 * vehicle_local_position and publishes what it returns as trajectory_setpoint;
 * the host unit test (firmware/bailout/test) flies it against a simulated
 * vehicle. Frames and units: PX4 local NED, metres, seconds. +z is DOWN.
 *
 * Checked against drone 11's FC (contracts/px4-baselines/skybeta-11-2026-11-07.params):
 *   MPC_TILTMAX_AIR 45 deg   -> the 3 m/s^2 brake (17 deg) has plenty of margin
 *   LNDMC_Z_VEL_MAX 0.25     -> PX4 only sees a "commanded descent" (and so ground
 *                               contact) when the setpoint is >= 1.1 * 0.25 m/s
 *   COM_DISARM_LAND 5 s      -> PX4 disarms 5 s after it detects the landing
 ****************************************************************************/
#pragma once

#include <math.h>
#include <stdint.h>

#include "brake_profile.hpp"

namespace dh_bailout
{

/// A setpoint field. Unset is NaN, which is exactly what trajectory_setpoint
/// means by "not controlled", so a field is copied to PX4 as is.
struct OptFloat {
	float v{NAN};
	bool has_value() const { return !__builtin_isnan(v); }
	explicit operator bool() const { return has_value(); }
	float operator*() const { return v; }
	float value_or(float d) const { return has_value() ? v : d; }
	OptFloat &operator=(float x) { v = x; return *this; }
};

/// Tunables. Compiled in: the firmware adds no parameters.
struct BailoutParams {
	float brake_decel_xy_m_s2{3.0f};  ///< ~17 deg of tilt
	float brake_jerk_xy_m_s3{20.0f};  ///< how fast the tilt builds up
	float brake_decel_z_m_s2{2.0f};
	float brake_jerk_z_m_s3{10.0f};
	float settle_speed_m_s{0.15f};    ///< "stopped" when every axis is slower than this
	float settle_time_s{0.3f};        ///< ... for this long
	float settle_timeout_s{1.5f};     ///< descend anyway if it has not settled by then
	float descent_fast_m_s{0.7f};     ///< above slow_below_m (needs the rangefinder)
	float descent_slow_m_s{0.35f};    ///< near the ground, or with no rangefinder
	float slow_below_m{0.8f};         ///< rangefinder height where the descent slows
	float descent_accel_m_s2{1.0f};   ///< ramp between descent speeds
};

#ifndef __PX4_NUTTX
// Hard limits on BailoutParams, for the host and SITL builds. The firmware has no
// tunable parameters (the values above are compiled in), and the host test checks
// that those defaults are inside these limits. Leaving the table and sanitize() out
// of the NuttX image saves flash the Pixhawk 6C does not have to spare.

/// Hard limits on BailoutParams. Values outside are clamped; NaN takes the default.
struct ParamLimit {
	float BailoutParams::*field;
	const char *name;
	float min;
	float max;
};

static constexpr ParamLimit kParamLimits[] = {
	// 4.0 m/s^2 is 22 deg: room for the position controller's own corrections
	// even under the fleet's tightest tilt cap seen (25 deg, drone 1).
	{&BailoutParams::brake_decel_xy_m_s2, "brake_decel_xy_m_s2", 0.5f, 4.0f},
	{&BailoutParams::brake_jerk_xy_m_s3, "brake_jerk_xy_m_s3", 2.0f, 60.0f},
	{&BailoutParams::brake_decel_z_m_s2, "brake_decel_z_m_s2", 0.5f, 3.0f},
	{&BailoutParams::brake_jerk_z_m_s3, "brake_jerk_z_m_s3", 2.0f, 40.0f},
	{&BailoutParams::settle_speed_m_s, "settle_speed_m_s", 0.05f, 0.5f},
	{&BailoutParams::settle_time_s, "settle_time_s", 0.0f, 2.0f},
	{&BailoutParams::settle_timeout_s, "settle_timeout_s", 0.2f, 5.0f},
	// 0.3 m/s is the floor because PX4's land detector ignores slower commanded
	// descents (1.1 * LNDMC_Z_VEL_MAX = 0.275 m/s), and then never disarms.
	{&BailoutParams::descent_slow_m_s, "descent_slow_m_s", 0.3f, 1.0f},
	{&BailoutParams::descent_fast_m_s, "descent_fast_m_s", 0.3f, 1.0f},
	{&BailoutParams::slow_below_m, "slow_below_m", 0.2f, 3.0f},
	{&BailoutParams::descent_accel_m_s2, "descent_accel_m_s2", 0.2f, 3.0f},
};

/// Clamp every field into its limit. Returns how many fields changed.
inline int sanitize(BailoutParams &p)
{
	const BailoutParams defaults{};
	int changed = 0;

	for (const ParamLimit &limit : kParamLimits) {
		float &value = p.*(limit.field);
		const float before = value;

		if (!finitef(value)) {
			value = defaults.*(limit.field);
		}

		value = minf(maxf(value, limit.min), limit.max);

		const bool same = finitef(before) && !(value < before) && !(value > before);

		if (!same) {
			++changed;
		}
	}

	if (p.descent_fast_m_s < p.descent_slow_m_s) {
		p.descent_fast_m_s = p.descent_slow_m_s;
		++changed;
	}

	return changed;
}
#endif // __PX4_NUTTX

enum class Phase : uint8_t {
	Inactive,
	Brake,    ///< following the jerk-limited stop
	Settle,   ///< holding the stop point until the vehicle is actually still
	Descend,  ///< holding x/y, descending until PX4 detects the landing and disarms
};

inline const char *phaseName(Phase phase)
{
	switch (phase) {
	case Phase::Inactive: return "inactive";

	case Phase::Brake: return "brake";

	case Phase::Settle: return "settle";

	case Phase::Descend: return "descend";
	}

	return "?";
}

/// What the logic needs from vehicle_local_position, plus the land detector.
struct VehicleState {
	float time_s{0.f};      ///< monotonic clock
	bool fresh{false};      ///< a local-position sample arrived recently
	float x{0.f}, y{0.f}, z{0.f};
	bool xy_valid{false}, z_valid{false};
	float vx{0.f}, vy{0.f}, vz{0.f};
	bool v_xy_valid{false}, v_z_valid{false};
	float heading_rad{NAN};
	float dist_bottom_m{NAN};
	bool dist_bottom_valid{false};
	uint8_t xy_reset_counter{0};
	float delta_x{0.f}, delta_y{0.f};  ///< last xy reset: new = old + delta
	uint8_t z_reset_counter{0};
	float delta_z{0.f};
	uint8_t heading_reset_counter{0};
	float delta_heading{0.f};          ///< last heading reset: new = old + delta
	bool landed{false};
};

inline float wrapPi(float angle_rad)
{
	constexpr float kPi = 3.14159265358979f;
	constexpr float kTwoPi = 2.f * kPi;
	float a = fmodf(angle_rad + kPi, kTwoPi);

	if (a < 0.f) {
		a += kTwoPi;
	}

	return a - kPi;
}

/// Per-axis setpoint; an unset field is NaN ("not controlled") in PX4.
struct Setpoint {
	OptFloat x, y, z;
	OptFloat vx, vy, vz;
	OptFloat ax, ay, az;
	OptFloat yaw, yaw_rate;
};

class BailoutLogic
{
public:
	/// The stop reference runs ahead of the vehicle by about v^2 / (2 a). A glitched
	/// velocity estimate (likely exactly when vision is struggling) must not turn
	/// that into metres of commanded flight, so the braked speed is capped (worst
	/// case about 0.8 m ahead horizontally, 0.35 m vertically, with the defaults).
	/// A faster real vehicle is still braked, by the velocity loop.
	static constexpr float kMaxBrakeSpeedXY = 2.0f;
	static constexpr float kMaxBrakeSpeedZ = 1.0f;
	/// After a gap this long in the position estimate, re-anchor where the vehicle
	/// is instead of flying back to where it was.
	static constexpr float kReanchorAfterGapS = 0.5f;

#ifndef __PX4_NUTTX
	explicit BailoutLogic(const BailoutParams &params = {}) : _p(params) { sanitize(_p); }
#else
	BailoutLogic() = default;  // the compiled-in defaults (inside the limits: host test)
#endif

	const BailoutParams &params() const { return _p; }
	Phase phase() const { return _phase; }
	bool active() const { return _phase != Phase::Inactive; }

	/// Start a bail-out from this state (the mode was just selected).
	void activate(const VehicleState &s)
	{
		_t0 = s.time_s;
		_last_time = s.time_s;
		_phase = Phase::Brake;
		_settle_start = 0.f;
		_have_settled_since = false;
		_settle_timed_out = false;
		_descent_cmd = 0.f;
		_yaw_hold = finitef(s.heading_rad) ? s.heading_rad : NAN;
		_xy_reset_counter = s.xy_reset_counter;
		_z_reset_counter = s.z_reset_counter;
		_heading_reset_counter = s.heading_reset_counter;

		// Horizontal: brake along the current direction of travel.
		_dir_xy_x = _dir_xy_y = 0.f;
		_brake_xy = BrakeProfile();

		if (s.fresh && s.v_xy_valid) {
			const float speed = sqrtf(s.vx * s.vx + s.vy * s.vy);

			if (speed > BrakeProfile::kMinSpeed) {
				_dir_xy_x = s.vx / speed;
				_dir_xy_y = s.vy / speed;
				_brake_xy = BrakeProfile(minf(speed, kMaxBrakeSpeedXY), _p.brake_decel_xy_m_s2, _p.brake_jerk_xy_m_s3);
			}
		}

		_have_xy_anchor = s.fresh && s.xy_valid;
		_p0_x = s.x;
		_p0_y = s.y;
		_last_xy_pos_time = s.time_s;

		// Vertical: stop any climb or sink before the controlled descent starts.
		_dir_z = 0.f;
		_brake_z = BrakeProfile();

		if (s.fresh && s.v_z_valid && fabsf(s.vz) > BrakeProfile::kMinSpeed) {
			_dir_z = s.vz > 0.f ? 1.f : -1.f;
			_brake_z = BrakeProfile(minf(fabsf(s.vz), kMaxBrakeSpeedZ), _p.brake_decel_z_m_s2, _p.brake_jerk_z_m_s3);
		}

		_have_z_anchor = s.fresh && s.z_valid;
		_p0_z = s.z;
		_last_z_pos_time = s.time_s;

		if (s.landed) {
			// Already down (armed on the ground): just keep it down until PX4 disarms.
			enterDescend(s);
		}
	}

	void deactivate() { _phase = Phase::Inactive; }

	/// One control step. Inactive: an empty setpoint.
	Setpoint update(const VehicleState &s)
	{
		Setpoint sp;

		if (_phase == Phase::Inactive) {
			return sp;
		}

		const float dt = minf(maxf(s.time_s - _last_time, 0.f), 0.1f);
		_last_time = s.time_s;
		const float tau = maxf(0.f, s.time_s - _t0);

		if (s.fresh) {
			followEstimatorResets(s);

			// Anchor late if the estimate became valid after activation, or re-anchor
			// after a gap (the vehicle drifted meanwhile): continue the same reference
			// from where the vehicle is now, rather than flying back.
			if (s.xy_valid) {
				if (!_have_xy_anchor || s.time_s - _last_xy_pos_time > kReanchorAfterGapS) {
					const BrakeSample b = _brake_xy.at(tau);
					_p0_x = s.x - _dir_xy_x * b.distance_m;
					_p0_y = s.y - _dir_xy_y * b.distance_m;
					_have_xy_anchor = true;
				}

				_last_xy_pos_time = s.time_s;
			}

			if (s.z_valid) {
				if (!_have_z_anchor || s.time_s - _last_z_pos_time > kReanchorAfterGapS) {
					_p0_z = s.z - _dir_z * _brake_z.at(tau).distance_m;
					_have_z_anchor = true;
				}

				_last_z_pos_time = s.time_s;
			}
		}

		stepPhase(s, tau);

		if (finitef(_yaw_hold)) {
			sp.yaw = _yaw_hold;

		} else {
			sp.yaw_rate = 0.f;
		}

		const bool use_xy_pos = s.fresh && s.xy_valid && _have_xy_anchor;
		const bool use_xy_vel = s.fresh && s.v_xy_valid;
		const bool use_z_pos = s.fresh && s.z_valid && _have_z_anchor;
		const bool use_z_vel = s.fresh && s.v_z_valid;

		// --- horizontal ---------------------------------------------------------
		// After the stop, at(tau) stays at the stop point with zero speed, so Brake,
		// Settle and Descend all follow the same reference. Each field is only sent
		// with its estimate: PX4 rejects a position or velocity setpoint without one.
		const BrakeSample bxy = _brake_xy.at(tau);

		if (!s.fresh) {
			// No estimate update lately: ask for zero velocity. If PX4 does have an
			// estimate it stops; if it has none it rejects this and runs its own
			// failsafe setpoint.
			sp.vx = 0.f;
			sp.vy = 0.f;

		} else if (use_xy_pos || use_xy_vel) {
			if (use_xy_pos) {
				sp.x = _p0_x + _dir_xy_x * bxy.distance_m;
				sp.y = _p0_y + _dir_xy_y * bxy.distance_m;
			}

			if (use_xy_vel) {
				sp.vx = _dir_xy_x * bxy.speed_m_s;
				sp.vy = _dir_xy_y * bxy.speed_m_s;
			}

			sp.ax = -_dir_xy_x * bxy.decel_m_s2;
			sp.ay = -_dir_xy_y * bxy.decel_m_s2;

		} else {
			// PX4 reports no horizontal estimate at all: fly level and let it drift, as
			// PX4's own Descend does. There is nothing to brake against.
			sp.ax = 0.f;
			sp.ay = 0.f;
		}

		// --- vertical -----------------------------------------------------------
		if (_phase == Phase::Descend) {
			const bool high = s.fresh && s.dist_bottom_valid && finitef(s.dist_bottom_m) &&
					  s.dist_bottom_m > _p.slow_below_m;
			const float target = high ? _p.descent_fast_m_s : _p.descent_slow_m_s;
			const float step = _p.descent_accel_m_s2 * dt;
			_descent_cmd = _descent_cmd < target ? minf(target, _descent_cmd + step) : maxf(target, _descent_cmd - step);
			sp.vz = _descent_cmd;

		} else if (!s.fresh) {
			sp.vz = 0.f;

		} else if (use_z_pos || use_z_vel) {
			const BrakeSample bz = _brake_z.at(tau);

			if (use_z_pos) {
				sp.z = _p0_z + _dir_z * bz.distance_m;
			}

			if (use_z_vel) {
				sp.vz = _dir_z * bz.speed_m_s;
			}

			sp.az = -_dir_z * bz.decel_m_s2;

		} else {
			// No height estimate: PX4 rejects a velocity setpoint without one and falls
			// back to its own failsafe descent, which is what we want anyway.
			sp.vz = _p.descent_slow_m_s;
		}

		return sp;
	}

	// Introspection, for status output and tests.
	float brakeDuration() const { return maxf(_brake_xy.duration(), _brake_z.duration()); }
	float stopDistanceXY() const { return _brake_xy.stopDistance(); }
	float initialSpeedXY() const { return _brake_xy.initialSpeed(); }
	bool settleTimedOut() const { return _settle_timed_out; }
	float descentCommand() const { return _descent_cmd; }

private:
	void followEstimatorResets(const VehicleState &s)
	{
		// On an EKF reset the same physical point gets new coordinates (new = old +
		// delta). Move the reference with it, or the vehicle would jump to the old ones.
		if (s.xy_reset_counter != _xy_reset_counter) {
			_xy_reset_counter = s.xy_reset_counter;

			if (finitef(s.delta_x) && finitef(s.delta_y)) {
				_p0_x += s.delta_x;
				_p0_y += s.delta_y;
			}
		}

		if (s.z_reset_counter != _z_reset_counter) {
			_z_reset_counter = s.z_reset_counter;

			if (finitef(s.delta_z)) {
				_p0_z += s.delta_z;
			}
		}

		// Same for heading (a vision yaw reset), or the vehicle would turn mid-bail-out.
		if (s.heading_reset_counter != _heading_reset_counter) {
			_heading_reset_counter = s.heading_reset_counter;

			if (finitef(_yaw_hold) && finitef(s.delta_heading)) {
				_yaw_hold = wrapPi(_yaw_hold + s.delta_heading);
			}
		}
	}

	void stepPhase(const VehicleState &s, float tau)
	{
		if (s.landed && _phase != Phase::Descend) {
			enterDescend(s);
			return;
		}

		if (_phase == Phase::Brake && tau >= brakeDuration()) {
			_phase = Phase::Settle;
			_settle_start = tau;
			_have_settled_since = false;
		}

		if (_phase != Phase::Settle) {
			return;
		}

		// An axis with no estimate cannot be confirmed still; do not wait on it.
		const bool still = s.fresh &&
				   (!s.v_xy_valid || sqrtf(s.vx * s.vx + s.vy * s.vy) < _p.settle_speed_m_s) &&
				   (!s.v_z_valid || fabsf(s.vz) < _p.settle_speed_m_s);

		if (still) {
			if (!_have_settled_since) {
				_have_settled_since = true;
				_settled_since = tau;
			}

			if (tau - _settled_since >= _p.settle_time_s) {
				enterDescend(s);
				return;
			}

		} else {
			_have_settled_since = false;
		}

		if (tau - _settle_start >= _p.settle_timeout_s) {
			_settle_timed_out = true;
			enterDescend(s);
		}
	}

	void enterDescend(const VehicleState &s)
	{
		_phase = Phase::Descend;
		// Start the descent ramp from the current sink rate, so it is continuous.
		_descent_cmd = (s.fresh && s.v_z_valid) ? minf(maxf(s.vz, 0.f), _p.descent_fast_m_s) : 0.f;

		if (s.landed) {
			_descent_cmd = _p.descent_slow_m_s;
		}
	}

	BailoutParams _p;
	Phase _phase{Phase::Inactive};
	float _t0{0.f};
	float _last_time{0.f};

	BrakeProfile _brake_xy;
	float _dir_xy_x{0.f}, _dir_xy_y{0.f};
	bool _have_xy_anchor{false};
	float _p0_x{0.f}, _p0_y{0.f};

	BrakeProfile _brake_z;
	float _dir_z{0.f};
	bool _have_z_anchor{false};
	float _p0_z{0.f};

	float _last_xy_pos_time{0.f};
	float _last_z_pos_time{0.f};

	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _heading_reset_counter{0};
	float _yaw_hold{NAN};

	float _settle_start{0.f};
	bool _have_settled_since{false};
	float _settled_since{0.f};
	bool _settle_timed_out{false};
	float _descent_cmd{0.f};
};

} // namespace dh_bailout
