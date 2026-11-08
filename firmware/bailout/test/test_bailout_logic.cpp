// Unit and closed-loop tests for the bail-out logic that runs inside the PX4
// module (src/modules/dh_bailout). No PX4, no test framework:
//
//   g++ -std=c++17 -Wall -Wextra -Werror -Isrc/modules/dh_bailout test/test_bailout_logic.cpp && ./a.out
//
// (from firmware/bailout). The closed-loop cases fly the logic against a
// point-mass vehicle with PX4 v1.16's position-control structure (P on position
// -> velocity, P on velocity -> acceleration, tilt limit, attitude lag) and drone
// 11's FC values, so they check the behaviour, not just the arithmetic. They are
// not a substitute for PX4 SITL or a props-off bench test.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "bailout_logic.hpp"
#include "brake_profile.hpp"

using dh_bailout::BailoutLogic;
using dh_bailout::OptFloat;
using dh_bailout::BailoutParams;
using dh_bailout::BrakeProfile;
using dh_bailout::BrakeSample;
using dh_bailout::Phase;
using dh_bailout::Setpoint;
using dh_bailout::VehicleState;

namespace
{

int g_failures = 0;
int g_checks = 0;

#define CHECK(cond) check((cond), #cond, __FILE__, __LINE__, "")
#define CHECK_MSG(cond, msg) check((cond), #cond, __FILE__, __LINE__, (msg))

void check(bool ok, const char * expr, const char * file, int line, const std::string & msg)
{
  ++g_checks;
  if (!ok) {
    ++g_failures;
    std::fprintf(stderr, "FAIL %s:%d: %s %s\n", file, line, expr, msg.c_str());
  }
}

bool near(float a, float b, float tol) {return std::fabs(a - b) <= tol;}

constexpr float kG = 9.80665f;
// Drone 11 (skybeta-11-2026-11-07.params): MPC_TILTMAX_AIR 45, COM_DISARM_LAND 5 s,
// LNDMC_Z_VEL_MAX 0.25, LNDMC_TRIG_TIME 1 s, MPC_XY_P 0.95, MPC_XY_VEL_P_ACC 1.8,
// MPC_Z_P 1, MPC_Z_VEL_P_ACC 4.
const float kTiltCapRad = 45.f * static_cast<float>(M_PI) / 180.f;     // the sim's cap (FC)
const float kTiltBudgetRad = 25.f * static_cast<float>(M_PI) / 180.f;  // what braking may use
constexpr float kDisarmLandS = 5.0f;                                   // COM_DISARM_LAND

// ---------------------------------------------------------------------------
// BrakeProfile
// ---------------------------------------------------------------------------

void testBrakeProfileTrapezoid()
{
  const BrakeProfile p(1.0f, 3.0f, 20.0f);  // 1 m/s, enough speed to reach 3 m/s^2
  CHECK(!p.empty());
  CHECK(near(p.peakDecel(), 3.0f, 1e-6f));
  // t_ramp = 0.15 s, t_hold = (1 - 0.45) / 3 = 0.1833 s
  CHECK(near(p.duration(), 0.4833f, 1e-3f));
  CHECK(near(p.stopDistance(), 0.5f * 1.0f * p.duration(), 1e-6f));

  const BrakeSample start = p.at(0.f);
  CHECK(near(start.speed_m_s, 1.0f, 1e-6f) && start.distance_m == 0.f && start.decel_m_s2 == 0.f);
  const BrakeSample end = p.at(p.duration());
  CHECK(end.speed_m_s == 0.f && end.decel_m_s2 == 0.f);
  CHECK(near(end.distance_m, p.stopDistance(), 1e-6f));
  CHECK(near(p.at(10.f).distance_m, p.stopDistance(), 1e-6f));

  // Continuous, monotonic, within limits, and consistent: integrating the speed
  // gives the distance; differentiating it gives the deceleration.
  float prev_speed = 1.0f;
  float prev_dist = 0.f;
  float integrated = 0.f;
  const float dt = 1e-4f;
  for (float t = dt; t <= p.duration() + 0.05f; t += dt) {
    const BrakeSample s = p.at(t);
    CHECK_MSG(s.speed_m_s <= prev_speed + 1e-6f, "speed must not increase");
    CHECK_MSG(s.distance_m >= prev_dist - 1e-6f, "distance must not decrease");
    CHECK(s.decel_m_s2 <= 3.0f + 1e-5f);
    CHECK(std::fabs(s.speed_m_s - prev_speed) < 3.0f * dt + 1e-4f);
    integrated += 0.5f * (s.speed_m_s + prev_speed) * dt;
    if (t > dt && t < p.duration() - dt) {
      const float finite_diff = (prev_speed - s.speed_m_s) / dt;
      CHECK(near(finite_diff, s.decel_m_s2, 0.05f));
    }
    prev_speed = s.speed_m_s;
    prev_dist = s.distance_m;
  }
  CHECK(near(integrated, p.stopDistance(), 2e-3f));
}

void testBrakeProfileTriangle()
{
  // 0.2 m/s cannot reach 3 m/s^2 at 20 m/s^3: peak sqrt(0.2 * 20) = 2 m/s^2.
  const BrakeProfile p(0.2f, 3.0f, 20.0f);
  CHECK(near(p.peakDecel(), 2.0f, 1e-5f));
  CHECK(near(p.duration(), 0.2f, 1e-5f));
  CHECK(near(p.at(p.duration() * 0.5f).decel_m_s2, 2.0f, 1e-4f));
  CHECK(near(p.at(p.duration() * 0.5f).speed_m_s, 0.1f, 1e-4f));
  CHECK(near(p.stopDistance(), 0.02f, 1e-5f));
}

void testBrakeProfileDegenerate()
{
  CHECK(BrakeProfile().empty());
  CHECK(BrakeProfile(0.f, 3.f, 20.f).empty());
  CHECK(BrakeProfile(1.f, 0.f, 20.f).empty());
  CHECK(BrakeProfile(1.f, 3.f, NAN).empty());
  CHECK(BrakeProfile(NAN, 3.f, 20.f).empty());
  CHECK(BrakeProfile().duration() == 0.f && BrakeProfile().stopDistance() == 0.f);
  CHECK(BrakeProfile().at(0.5f).speed_m_s == 0.f);
  // Direction is the caller's: the profile is symmetric in the sign of the speed.
  CHECK(near(BrakeProfile(-1.f, 3.f, 20.f).stopDistance(), BrakeProfile(1.f, 3.f, 20.f).stopDistance(), 1e-7f));
}

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------

void testSanitize()
{
  BailoutParams p;
  p.brake_decel_xy_m_s2 = 10.f;  // would ask for 45 deg of tilt
  p.descent_slow_m_s = 0.2f;     // below the land detector's 0.275 m/s
  p.descent_fast_m_s = 0.25f;
  p.settle_timeout_s = NAN;
  BailoutParams s = p;
  const int changed = dh_bailout::sanitize(s);
  CHECK(s.brake_decel_xy_m_s2 == 4.0f);
  CHECK(s.descent_slow_m_s == 0.3f);
  CHECK(s.descent_fast_m_s >= s.descent_slow_m_s);
  CHECK(s.settle_timeout_s == BailoutParams{}.settle_timeout_s);
  CHECK(changed >= 4);

  // Defaults are inside the limits and respect the FC's envelope.
  BailoutParams d{};
  CHECK(dh_bailout::sanitize(d) == 0);
  CHECK(std::atan(d.brake_decel_xy_m_s2 / kG) < kTiltBudgetRad);
  CHECK(d.descent_slow_m_s >= 1.1f * 0.25f);  // LNDMC_Z_VEL_MAX on drone 11
}

// ---------------------------------------------------------------------------
// Setpoint well-formedness: what PX4's PositionControl::_inputValid() demands
// ---------------------------------------------------------------------------

void checkWellFormed(const Setpoint & sp, const VehicleState & s, const char * where)
{
  const std::string ctx = where;
  // x and y always come in pairs.
  CHECK_MSG(sp.x.has_value() == sp.y.has_value(), ctx);
  CHECK_MSG(sp.vx.has_value() == sp.vy.has_value(), ctx);
  CHECK_MSG(sp.ax.has_value() == sp.ay.has_value(), ctx);
  // Every axis has something to control.
  CHECK_MSG(sp.x || sp.vx || sp.ax, ctx);
  CHECK_MSG(sp.z || sp.vz || sp.az, ctx);
  // Never a position or velocity setpoint on an axis PX4 reports no estimate
  // for. Exceptions, both deliberate: a stale sample asks for zero velocity (PX4
  // probably still has an estimate), and with no height estimate the logic sends
  // a descent rate so that PX4's own failsafe descent takes over.
  if (sp.x) {CHECK_MSG(s.fresh && s.xy_valid, ctx);}
  if (sp.vx) {CHECK_MSG(!s.fresh || s.v_xy_valid, ctx);}
  if (sp.z) {CHECK_MSG(s.fresh && s.z_valid, ctx);}
  // A finite yaw, or a yaw rate, never neither.
  CHECK_MSG(sp.yaw.has_value() != sp.yaw_rate.has_value(), ctx);
  for (const auto * f : {&sp.x, &sp.y, &sp.z, &sp.vx, &sp.vy, &sp.vz, &sp.ax, &sp.ay, &sp.az,
      &sp.yaw, &sp.yaw_rate})
  {
    if (*f) {CHECK_MSG(std::isfinite(**f), ctx);}
  }
}

// ---------------------------------------------------------------------------
// Closed loop
// ---------------------------------------------------------------------------

struct Gains
{
  float xy_p{0.95f};      // MPC_XY_P
  float xy_vel_p{1.8f};   // MPC_XY_VEL_P_ACC
  float z_p{1.0f};        // MPC_Z_P (default)
  float z_vel_p{4.0f};    // MPC_Z_VEL_P_ACC (default)
  float att_tau_s{0.08f}; // attitude/thrust response lag
};

struct Sim
{
  // Truth, NED. Ground at z = 0.
  float x{0.f}, y{0.f}, z{-1.0f};
  float vx{0.f}, vy{0.f}, vz{0.f};
  float ax{0.f}, ay{0.f}, az{0.f};  // achieved acceleration (lags the command)
  float heading{0.3f};
  bool on_ground{false};
  float ground_contact_s{0.f};
  bool landed{false};
  bool disarmed{false};
  float landed_at_s{NAN};
  float disarmed_at_s{NAN};
  float max_cmd_tilt_rad{0.f};
  float max_cmd_acc_xy{0.f};
  // What the estimator reports.
  bool xy_valid{true}, z_valid{true}, v_xy_valid{true}, v_z_valid{true};
  float vel_noise{0.f};
  uint8_t xy_reset_counter{0};
  float delta_x{0.f}, delta_y{0.f};
  float offset_x{0.f}, offset_y{0.f};  // estimate = truth + offset (EKF frame)
  unsigned noise_seed{12345u};

  float noise()
  {
    noise_seed = noise_seed * 1664525u + 1013904223u;
    return (static_cast<float>(noise_seed >> 8) / 16777216.f - 0.5f) * 2.f * vel_noise;
  }

  VehicleState estimate(float t)
  {
    VehicleState s;
    s.time_s = t;
    s.fresh = true;
    s.x = x + offset_x;
    s.y = y + offset_y;
    s.z = z;
    s.xy_valid = xy_valid;
    s.z_valid = z_valid;
    s.vx = vx + noise();
    s.vy = vy + noise();
    s.vz = vz + noise();
    s.v_xy_valid = v_xy_valid;
    s.v_z_valid = v_z_valid;
    s.heading_rad = heading;
    s.dist_bottom_m = -z;
    s.dist_bottom_valid = -z < 4.0f;  // rangefinder limit
    s.xy_reset_counter = xy_reset_counter;
    s.delta_x = delta_x;
    s.delta_y = delta_y;
    s.landed = landed;
    return s;
  }

  /// PX4's position controller, simplified: one axis pair at a time.
  void control(const Setpoint & sp, const Gains & g, float dt, float t)
  {
    if (disarmed) {
      return;
    }
    const VehicleState est = estimate(t);
    auto axis = [](OptFloat p_sp, OptFloat v_sp, OptFloat a_sp,
        float p, float v, float kp, float kv) {
        float vel = v_sp.value_or(0.f);
        bool have_vel = v_sp.has_value();
        if (p_sp) {
          vel += kp * (*p_sp - p);
          have_vel = true;
        }
        float acc = a_sp.value_or(0.f);
        if (have_vel) {
          acc += kv * (vel - v);
        }
        return acc;
      };
    float cax = axis(sp.x, sp.vx, sp.ax, est.x, est.vx, g.xy_p, g.xy_vel_p);
    float cay = axis(sp.y, sp.vy, sp.ay, est.y, est.vy, g.xy_p, g.xy_vel_p);
    float caz = axis(sp.z, sp.vz, sp.az, est.z, est.vz, g.z_p, g.z_vel_p);
    // Tilt limit.
    const float acc_xy = std::hypot(cax, cay);
    max_cmd_acc_xy = std::max(max_cmd_acc_xy, acc_xy);
    const float lift = std::max(0.1f, kG - caz);
    const float max_xy = lift * std::tan(kTiltCapRad);
    if (acc_xy > max_xy) {
      cax *= max_xy / acc_xy;
      cay *= max_xy / acc_xy;
    }
    max_cmd_tilt_rad = std::max(max_cmd_tilt_rad, std::atan2(std::hypot(cax, cay), lift));
    // Ground contact with a commanded descent: mc_pos_control cuts thrust.
    const bool commanded_descent = sp.vz && *sp.vz >= 1.1f * 0.25f;
    if (on_ground && commanded_descent) {
      ground_contact_s += dt;
    } else if (!on_ground) {
      ground_contact_s = 0.f;
    }
    // Land detector (LNDMC_TRIG_TIME 1 s, simplified) and COM_DISARM_LAND.
    if (!landed && ground_contact_s > 1.0f) {
      landed = true;
      landed_at_s = t;
    }
    if (landed && !disarmed && t - landed_at_s >= kDisarmLandS) {
      disarmed = true;
      disarmed_at_s = t;
    }
    const float k = dt / (g.att_tau_s + dt);
    ax += k * (cax - ax);
    ay += k * (cay - ay);
    az += k * (caz - az);
  }

  void integrate(float dt)
  {
    if (disarmed) {
      vx = vy = vz = 0.f;
      return;
    }
    vx += ax * dt;
    vy += ay * dt;
    vz += az * dt;
    x += vx * dt;
    y += vy * dt;
    z += vz * dt;
    if (z >= 0.f) {
      // Hit the floor: friction stops it.
      z = 0.f;
      vz = std::min(vz, 0.f);
      vx *= 0.5f;
      vy *= 0.5f;
      on_ground = true;
    } else {
      on_ground = false;
    }
  }
};

struct RunResult
{
  float stop_time_s{NAN};       // first time horizontal speed < 0.05 m/s
  float travel_after_activation_m{0.f};
  float max_reversal_m{0.f};    // how far it came back after the furthest point
  float descend_at_s{NAN};
  float max_alt_gain_m{0.f};
  bool settle_timed_out{false};
  Sim sim;
};

RunResult fly(Sim sim, const BailoutParams & params = {}, const Gains & gains = {},
  float duration_s = 12.f, const char * label = "")
{
  BailoutLogic logic(params);
  RunResult r;
  const float dt = 0.004f;          // 250 Hz vehicle
  const int steps_per_setpoint = 2; // 125 Hz setpoints, close to the mode's 100 Hz
  const float x0 = sim.x;
  const float y0 = sim.y;
  const float z0 = sim.z;
  const float dir_x = sim.vx;
  const float dir_y = sim.vy;
  const float dir_norm = std::max(1e-6f, std::hypot(dir_x, dir_y));
  float max_along = 0.f;
  Setpoint sp;
  logic.activate(sim.estimate(0.f));
  int step = 0;
  for (float t = 0.f; t < duration_s; t += dt, ++step) {
    if (step % steps_per_setpoint == 0) {
      const VehicleState est = sim.estimate(t);
      sp = logic.update(est);
      checkWellFormed(sp, est, label);
      if (logic.phase() == Phase::Descend && std::isnan(r.descend_at_s)) {
        r.descend_at_s = t;
      }
    }
    sim.control(sp, gains, dt, t);
    sim.integrate(dt);
    const float along = ((sim.x - x0) * dir_x + (sim.y - y0) * dir_y) / dir_norm;
    max_along = std::max(max_along, along);
    r.max_reversal_m = std::max(r.max_reversal_m, max_along - along);
    r.max_alt_gain_m = std::max(r.max_alt_gain_m, z0 - sim.z);
    if (std::isnan(r.stop_time_s) && std::hypot(sim.vx, sim.vy) < 0.05f) {
      r.stop_time_s = t;
    }
    if (sim.disarmed && t > sim.disarmed_at_s + 0.5f) {
      break;
    }
  }
  r.travel_after_activation_m = max_along;
  r.settle_timed_out = logic.settleTimedOut();
  r.sim = sim;
  return r;
}

void report(const char * name, const RunResult & r)
{
  std::printf(
    "  %-34s stop %.2f s, travel %.3f m, back %.3f m, tilt %.1f deg, descend at %.2f s, "
    "landed %.2f s, disarmed %.2f s\n",
    name, r.stop_time_s, r.travel_after_activation_m, r.max_reversal_m,
    r.sim.max_cmd_tilt_rad * 180.f / static_cast<float>(M_PI), r.descend_at_s,
    r.sim.landed_at_s, r.sim.disarmed_at_s);
}

void testClosedLoopNominal()
{
  // Autonomy flying 1 m/s at 1 m up, diagonally (the team's planner cap; drone
  // 11's FC itself allows 12 m/s).
  Sim sim;
  sim.vx = 0.8f;
  sim.vy = 0.6f;
  const RunResult r = fly(sim, {}, {}, 12.f, "nominal");
  report("1 m/s, 1 m up", r);
  const BrakeProfile ref(1.0f, 3.0f, 20.0f);
  // Stops close to the reference profile: about 0.5 s and 0.25 m. Forward travel
  // (towards whatever it was about to hit) is the number that matters.
  CHECK(r.stop_time_s < ref.duration() + 0.35f);
  CHECK(r.travel_after_activation_m < ref.stopDistance() + 0.1f);
  // Attitude lag carries it a few cm past the reference stop point, and the
  // position loop then pulls it back along the path it just flew, which is known
  // free space. That pull-back is also what shortens the forward travel.
  CHECK(r.max_reversal_m < 0.12f);
  CHECK(r.sim.max_cmd_tilt_rad < kTiltBudgetRad);
  CHECK(r.descend_at_s < 1.3f);
  CHECK(!r.settle_timed_out);
  CHECK(r.max_alt_gain_m < 0.05f);
  CHECK(r.sim.landed);
  CHECK(r.sim.disarmed);
  // Lands where it stopped.
  CHECK(std::hypot(r.sim.x, r.sim.y) < ref.stopDistance() + 0.15f);
  // From 1 m: fast above 0.8 m, slow below; touchdown + 1 s detect + 5 s disarm.
  CHECK(r.sim.disarmed_at_s < 6.f + kDisarmLandS);
}

void testClosedLoopFasterThanCap()
{
  // A setpoint glitch or a fall can exceed the cap; the brake still has to hold.
  Sim sim;
  sim.vx = 2.0f;
  const RunResult r = fly(sim, {}, {}, 12.f, "2 m/s");
  report("2 m/s", r);
  const BrakeProfile ref(2.0f, 3.0f, 20.0f);
  CHECK(r.travel_after_activation_m < ref.stopDistance() + 0.15f);
  CHECK(r.max_reversal_m < 0.18f);
  CHECK(r.sim.max_cmd_tilt_rad < kTiltCapRad + 1e-3f);
  CHECK(r.sim.disarmed);
}

void testClosedLoopThreeMetresPerSecond()
{
  // Drone 11's FC does not cap speed indoors (MPC_XY_VEL_MAX 12), so autonomy at
  // 3 m/s is possible. The reference starts at the 2 m/s cap; the velocity loop
  // brakes the rest.
  Sim sim;
  sim.vx = 3.0f;
  const RunResult r = fly(sim, {}, {}, 14.f, "3 m/s");
  report("3 m/s", r);
  CHECK(r.travel_after_activation_m < 2.0f);
  CHECK(r.max_reversal_m < 0.5f);
  CHECK(r.sim.max_cmd_tilt_rad < kTiltCapRad + 1e-3f);
  CHECK(r.sim.disarmed);
}

void testClosedLoopClimbing()
{
  // Bail out during a climb: stop the climb first, then descend.
  Sim sim;
  sim.z = -0.6f;
  sim.vx = 0.5f;
  sim.vz = -0.8f;
  const RunResult r = fly(sim, {}, {}, 12.f, "climbing");
  report("0.5 m/s fwd, 0.8 m/s climb", r);
  const BrakeProfile ref_z(0.8f, 2.0f, 10.0f);
  CHECK(r.max_alt_gain_m < ref_z.stopDistance() + 0.12f);
  CHECK(r.sim.disarmed);
}

void testClosedLoopHover()
{
  // Already hovering: no braking needed, straight to settle and descend.
  Sim sim;
  const RunResult r = fly(sim, {}, {}, 12.f, "hover");
  report("hover", r);
  CHECK(r.travel_after_activation_m < 0.02f);
  CHECK(r.descend_at_s < 0.5f);
  CHECK(r.sim.disarmed);
}

void testClosedLoopNoisyVelocitySettleTimeout()
{
  // Velocity noise above settle_speed: it must still descend, after the timeout.
  Sim sim;
  sim.vx = 1.0f;
  sim.vel_noise = 0.4f;
  const RunResult r = fly(sim, {}, {}, 14.f, "noisy");
  report("noisy velocity (timeout)", r);
  CHECK(r.settle_timed_out);
  CHECK(r.descend_at_s < BrakeProfile(1.0f, 3.0f, 20.0f).duration() + 1.5f + 0.05f);
  CHECK(r.sim.disarmed);
}

void testClosedLoopNoHorizontalEstimate()
{
  // Tunnel case after EKF2_NOAID_TOUT: no horizontal position or velocity. It
  // cannot brake; it must fly level, still descend and land.
  Sim sim;
  sim.vx = 0.6f;
  sim.xy_valid = false;
  sim.v_xy_valid = false;
  const RunResult r = fly(sim, {}, {}, 12.f, "no-xy");
  report("no horizontal estimate", r);
  CHECK(r.sim.max_cmd_tilt_rad < 1e-3f);
  CHECK(r.sim.disarmed);
}

void testClosedLoopVelocityOnly()
{
  // Position invalid, velocity valid (dead reckoning): brakes on velocity.
  Sim sim;
  sim.vx = 1.0f;
  sim.xy_valid = false;
  const RunResult r = fly(sim, {}, {}, 12.f, "vel-only");
  report("velocity only", r);
  CHECK(r.travel_after_activation_m < BrakeProfile(1.0f, 3.0f, 20.0f).stopDistance() + 0.15f);
  CHECK(r.sim.disarmed);
}

void testClosedLoopSlowGainsStillStop()
{
  // A softer vehicle (slower attitude response, lower velocity gain).
  Sim sim;
  sim.vx = 1.0f;
  Gains g;
  g.att_tau_s = 0.15f;
  g.xy_vel_p = 1.2f;
  const RunResult r = fly(sim, {}, g, 12.f, "soft");
  report("soft vehicle", r);
  CHECK(r.travel_after_activation_m < BrakeProfile(1.0f, 3.0f, 20.0f).stopDistance() + 0.2f);
  CHECK(r.max_reversal_m < 0.22f);
  CHECK(r.sim.disarmed);
}

// ---------------------------------------------------------------------------
// Logic edge cases (open loop)
// ---------------------------------------------------------------------------

VehicleState flying(float t, float vx = 1.f)
{
  VehicleState s;
  s.time_s = t;
  s.fresh = true;
  s.x = 2.f;
  s.y = -1.f;
  s.z = -1.f;
  s.xy_valid = s.z_valid = s.v_xy_valid = s.v_z_valid = true;
  s.vx = vx;
  s.heading_rad = 1.0f;
  s.dist_bottom_m = 1.f;
  s.dist_bottom_valid = true;
  return s;
}

void testInactiveIsEmpty()
{
  BailoutLogic logic;
  const Setpoint sp = logic.update(flying(0.f));
  CHECK(!sp.x && !sp.vx && !sp.ax && !sp.z && !sp.vz && !sp.yaw && !sp.yaw_rate);
  CHECK(logic.phase() == Phase::Inactive);
}

void testFirstSetpointMatchesState()
{
  // The first setpoint must agree with where the vehicle is and how fast it
  // moves, so the handover from Offboard has no step.
  BailoutLogic logic;
  const VehicleState s = flying(5.f);
  logic.activate(s);
  const Setpoint sp = logic.update(s);
  CHECK(logic.phase() == Phase::Brake);
  CHECK(near(*sp.x, s.x, 1e-6f) && near(*sp.y, s.y, 1e-6f) && near(*sp.z, s.z, 1e-6f));
  CHECK(near(*sp.vx, 1.f, 1e-6f) && near(*sp.vy, 0.f, 1e-6f));
  CHECK(near(*sp.ax, 0.f, 1e-6f));  // the deceleration ramps in at the jerk limit
  CHECK(near(*sp.yaw, 1.0f, 1e-6f));
  checkWellFormed(sp, s, "first");
  // Shortly after, it is decelerating against the direction of travel.
  const Setpoint later = logic.update(flying(5.1f));
  CHECK(*later.ax < -1.5f && *later.vx < 1.f && *later.x > s.x);
}

void testLandedAtActivation()
{
  BailoutLogic logic;
  VehicleState s = flying(0.f, 0.f);
  s.landed = true;
  s.z = 0.f;
  logic.activate(s);
  CHECK(logic.phase() == Phase::Descend);
  const Setpoint sp = logic.update(s);
  CHECK(sp.vz && *sp.vz >= 0.3f && !sp.z);
}

void testLandedDuringBrakeSkipsToDescend()
{
  BailoutLogic logic;
  logic.activate(flying(0.f));
  VehicleState s = flying(0.05f);
  s.landed = true;
  logic.update(s);
  CHECK(logic.phase() == Phase::Descend);
}

void testEstimatorResetMovesReference()
{
  BailoutLogic logic;
  logic.activate(flying(0.f, 0.f));  // hovering at (2, -1)
  VehicleState s = flying(0.3f, 0.f);
  const Setpoint before = logic.update(s);
  // The EKF re-anchors: same physical point, coordinates shifted by (+3, +4).
  s.time_s = 0.32f;
  s.x += 3.f;
  s.y += 4.f;
  s.xy_reset_counter = 1;
  s.delta_x = 3.f;
  s.delta_y = 4.f;
  const Setpoint after = logic.update(s);
  CHECK(near(*after.x - *before.x, 3.f, 1e-5f));
  CHECK(near(*after.y - *before.y, 4.f, 1e-5f));
}

void testLateAnchor()
{
  // Position invalid at activation, valid later: anchor to where it is then.
  BailoutLogic logic;
  VehicleState s = flying(0.f);
  s.xy_valid = false;
  logic.activate(s);
  const Setpoint sp0 = logic.update(s);
  CHECK(!sp0.x && sp0.vx);
  VehicleState later = flying(0.1f);
  later.x = 7.f;
  const Setpoint sp1 = logic.update(later);
  CHECK(sp1.x && near(*sp1.x, 7.f, 1e-5f));
}

void testStaleEstimate()
{
  // No sample for a while (the DDS link, usually): ask for zero velocity, so
  // PX4 still stops the vehicle if its own estimate is fine.
  BailoutLogic logic;
  logic.activate(flying(0.f));
  VehicleState s = flying(0.1f);
  s.fresh = false;
  const Setpoint sp = logic.update(s);
  checkWellFormed(sp, s, "stale");
  CHECK(!sp.x && sp.vx && *sp.vx == 0.f && *sp.vy == 0.f && !sp.ax);
  CHECK(!sp.z && sp.vz && *sp.vz == 0.f);
}

void testPositionWithoutVelocity()
{
  // PX4 rejects a velocity setpoint without a velocity estimate, even if the
  // position is fine: send position and acceleration only.
  BailoutLogic logic;
  VehicleState s = flying(0.f);
  logic.activate(s);
  s.time_s = 0.05f;
  s.v_xy_valid = false;
  s.v_z_valid = false;
  const Setpoint sp = logic.update(s);
  checkWellFormed(sp, s, "pos-only");
  CHECK(sp.x && !sp.vx && sp.ax);
  CHECK(sp.z && !sp.vz && sp.az);
}

void testHeadingResetMovesYaw()
{
  BailoutLogic logic;
  VehicleState s = flying(0.f);
  s.heading_rad = 3.0f;
  logic.activate(s);
  s.time_s = 0.02f;
  s.heading_reset_counter = 1;
  s.delta_heading = 0.5f;  // wraps past +pi
  const Setpoint sp = logic.update(s);
  CHECK(near(*sp.yaw, 3.5f - 2.f * static_cast<float>(M_PI), 1e-5f));
  // The same counter again is not a new reset.
  s.time_s = 0.04f;
  CHECK(near(*logic.update(s).yaw, *sp.yaw, 1e-6f));
}

void testGlitchedSpeedIsCapped()
{
  // A 10 m/s (and 4 m/s down) velocity glitch must not become 17 m (and 4 m) of
  // commanded flight: the reference runs ahead by the capped stop distance only.
  BailoutLogic logic;
  VehicleState s = flying(0.f, 10.f);
  s.vz = 4.f;
  logic.activate(s);
  const BailoutParams & p = logic.params();
  const float max_ahead_xy = BrakeProfile(
    BailoutLogic::kMaxBrakeSpeedXY, p.brake_decel_xy_m_s2, p.brake_jerk_xy_m_s3).stopDistance();
  const float max_ahead_z = BrakeProfile(
    BailoutLogic::kMaxBrakeSpeedZ, p.brake_decel_z_m_s2, p.brake_jerk_z_m_s3).stopDistance();
  CHECK(near(logic.initialSpeedXY(), BailoutLogic::kMaxBrakeSpeedXY, 1e-6f));
  CHECK(max_ahead_xy < 0.85f && max_ahead_z < 0.4f);
  float max_x = -1e9f;
  float max_z = -1e9f;
  for (float t = 0.f; t < 3.f; t += 0.01f) {
    s.time_s = t;
    const Setpoint sp = logic.update(s);
    if (sp.x) {max_x = std::max(max_x, *sp.x);}
    if (sp.z) {max_z = std::max(max_z, *sp.z);}
  }
  CHECK(max_x - 2.f <= max_ahead_xy + 1e-4f);   // x0 = 2
  CHECK(max_z - -1.f <= max_ahead_z + 1e-4f);   // z0 = -1
}

void testReanchorAfterPositionGap()
{
  // Position lost for 1 s while it drifted 2 m, then back: hold where it is now,
  // do not fly back to the old stop point.
  BailoutLogic logic;
  logic.activate(flying(0.f, 0.f));  // hovering at x = 2
  VehicleState s = flying(0.1f, 0.f);
  logic.update(s);
  s.xy_valid = false;
  for (float t = 0.2f; t < 1.2f; t += 0.05f) {
    s.time_s = t;
    logic.update(s);
  }
  s.time_s = 1.25f;
  s.xy_valid = true;
  s.x = 4.f;
  const Setpoint sp = logic.update(s);
  CHECK(sp.x && near(*sp.x, 4.f, 1e-5f));
  // A short gap (one sample) keeps the anchor.
  s.time_s = 1.3f;
  s.x = 4.1f;
  CHECK(near(*logic.update(s).x, 4.f, 1e-5f));
}

void testDescentSpeedSchedule()
{
  BailoutLogic logic;
  VehicleState s = flying(0.f, 0.f);
  s.dist_bottom_m = 2.0f;
  logic.activate(s);
  float t = 0.f;
  for (; t < 3.f && logic.phase() != Phase::Descend; t += 0.01f) {
    s.time_s = t;
    logic.update(s);
  }
  CHECK(logic.phase() == Phase::Descend);
  // High: ramps to the fast rate.
  for (int i = 0; i < 200; ++i) {
    t += 0.01f;
    s.time_s = t;
    logic.update(s);
  }
  CHECK(near(logic.descentCommand(), 0.7f, 1e-4f));
  // Low: ramps down to the slow rate, never below what the land detector needs.
  s.dist_bottom_m = 0.5f;
  float min_cmd = 1.f;
  for (int i = 0; i < 200; ++i) {
    t += 0.01f;
    s.time_s = t;
    const Setpoint sp = logic.update(s);
    min_cmd = std::min(min_cmd, *sp.vz);
  }
  CHECK(near(logic.descentCommand(), 0.35f, 1e-4f));
  CHECK(min_cmd >= 0.35f - 1e-4f);
  // No rangefinder: slow.
  s.dist_bottom_valid = false;
  s.time_s = t + 0.01f;
  CHECK(near(*logic.update(s).vz, 0.35f, 1e-4f));
}

void testDeactivateAndReactivate()
{
  BailoutLogic logic;
  logic.activate(flying(0.f));
  logic.update(flying(0.2f));
  logic.deactivate();
  CHECK(!logic.update(flying(0.3f)).vx);
  // A second bail-out starts from scratch, from the new state.
  VehicleState s = flying(10.f, -0.5f);
  s.x = 9.f;
  logic.activate(s);
  const Setpoint sp = logic.update(s);
  CHECK(logic.phase() == Phase::Brake);
  CHECK(near(*sp.x, 9.f, 1e-6f) && near(*sp.vx, -0.5f, 1e-6f));
}

}  // namespace

int main()
{
  testBrakeProfileTrapezoid();
  testBrakeProfileTriangle();
  testBrakeProfileDegenerate();
  testSanitize();
  testInactiveIsEmpty();
  testFirstSetpointMatchesState();
  testLandedAtActivation();
  testLandedDuringBrakeSkipsToDescend();
  testEstimatorResetMovesReference();
  testLateAnchor();
  testStaleEstimate();
  testPositionWithoutVelocity();
  testHeadingResetMovesYaw();
  testGlitchedSpeedIsCapped();
  testReanchorAfterPositionGap();
  testDescentSpeedSchedule();
  testDeactivateAndReactivate();
  std::printf("closed loop (PX4-like controller, drone 11 gains, 45 deg tilt cap, 5 s disarm):\n");
  testClosedLoopNominal();
  testClosedLoopFasterThanCap();
  testClosedLoopThreeMetresPerSecond();
  testClosedLoopClimbing();
  testClosedLoopHover();
  testClosedLoopNoisyVelocitySettleTimeout();
  testClosedLoopNoHorizontalEstimate();
  testClosedLoopVelocityOnly();
  testClosedLoopSlowGainsStillStop();
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
