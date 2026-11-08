/****************************************************************************
 * DroneHacks bail-out mode: jerk-limited stop profile.
 *
 * Header-only, no STL and no PX4 headers, so the same file builds into the
 * PX4 module and into the host unit test (firmware/bailout/test).
 ****************************************************************************/
#pragma once

#include <math.h>

namespace dh_bailout
{

inline float minf(float a, float b) { return (b < a) ? b : a; }  // std::min semantics
inline float maxf(float a, float b) { return (a < b) ? b : a; }  // std::max semantics
inline bool finitef(float x) { return __builtin_isfinite(x); }

/// One instant of a stop, as positive magnitudes along the direction of travel.
struct BrakeSample {
	float distance_m{0.f};  ///< travelled since the stop began
	float speed_m_s{0.f};   ///< still moving at
	float decel_m_s2{0.f};  ///< deceleration being applied
};

/// Stop from `initial_speed` to rest. Deceleration ramps up at `jerk`, holds at
/// `max_decel` (or less, if the speed is too low to reach it), then ramps back
/// down so that speed and deceleration reach zero at the same instant. Speed
/// never goes negative, so the reference never overshoots and comes back.
///
/// The speed curve is point-symmetric about the middle of the stop, so the
/// stopping distance is exactly initial_speed * duration / 2.
class BrakeProfile
{
public:
	/// Speeds below this are already "stopped": the profile is empty.
	static constexpr float kMinSpeed = 1e-3f;

	BrakeProfile() = default;

	BrakeProfile(float initial_speed_m_s, float max_decel_m_s2, float jerk_m_s3)
	{
		const float s0 = fabsf(initial_speed_m_s);

		if (!(s0 > kMinSpeed) || !(max_decel_m_s2 > 0.f) || !(jerk_m_s3 > 0.f) ||
		    !finitef(s0) || !finitef(max_decel_m_s2) || !finitef(jerk_m_s3)) {
			return;
		}

		_s0 = s0;
		_jerk = jerk_m_s3;
		// Reaching max_decel and ramping back costs max_decel^2 / jerk of speed. With
		// less speed than that the profile is a triangle peaking at sqrt(s0 * jerk).
		_peak = minf(max_decel_m_s2, sqrtf(_s0 * _jerk));
		_t_ramp = _peak / _jerk;
		_t_hold = maxf(0.f, (_s0 - _peak * _peak / _jerk) / _peak);
	}

	bool empty() const { return _s0 <= 0.f; }
	float initialSpeed() const { return _s0; }
	float peakDecel() const { return _peak; }
	float duration() const { return empty() ? 0.f : 2.f * _t_ramp + _t_hold; }
	float stopDistance() const { return 0.5f * _s0 * duration(); }

	/// The stop `t_s` seconds after it began. Before 0 it has not started; after
	/// duration() it is over and holds at the stop point.
	BrakeSample at(float t_s) const
	{
		if (empty()) {
			return {};
		}

		if (!(t_s > 0.f)) {
			return {0.f, _s0, 0.f};
		}

		if (t_s >= duration()) {
			return {stopDistance(), 0.f, 0.f};
		}

		const float j = _jerk;
		const float a = _peak;
		const float t1 = _t_ramp;
		// End of the ramp-up.
		const float v1 = _s0 - 0.5f * j * t1 * t1;
		const float x1 = _s0 * t1 - j * t1 * t1 * t1 / 6.f;

		BrakeSample out{};

		if (t_s < t1) {
			out.decel_m_s2 = j * t_s;
			out.speed_m_s = _s0 - 0.5f * j * t_s * t_s;
			out.distance_m = _s0 * t_s - j * t_s * t_s * t_s / 6.f;

		} else if (t_s < t1 + _t_hold) {
			const float tau = t_s - t1;
			out.decel_m_s2 = a;
			out.speed_m_s = v1 - a * tau;
			out.distance_m = x1 + v1 * tau - 0.5f * a * tau * tau;

		} else {
			// End of the hold.
			const float v2 = v1 - a * _t_hold;
			const float x2 = x1 + v1 * _t_hold - 0.5f * a * _t_hold * _t_hold;
			const float tau = t_s - t1 - _t_hold;
			out.decel_m_s2 = a - j * tau;
			out.speed_m_s = v2 - a * tau + 0.5f * j * tau * tau;
			out.distance_m = x2 + v2 * tau - 0.5f * a * tau * tau + j * tau * tau * tau / 6.f;
		}

		out.decel_m_s2 = maxf(0.f, out.decel_m_s2);
		out.speed_m_s = maxf(0.f, out.speed_m_s);
		out.distance_m = minf(maxf(0.f, out.distance_m), stopDistance());
		return out;
	}

private:
	float _s0{0.f};
	float _jerk{0.f};
	float _peak{0.f};
	float _t_ramp{0.f};
	float _t_hold{0.f};
};

} // namespace dh_bailout
