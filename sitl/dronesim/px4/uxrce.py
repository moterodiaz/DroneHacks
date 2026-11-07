"""PX4's uxrce_dds_client + the Jetson's MicroXRCEAgent, as one graph participant.

* ``/fmu/out/*``: BEST_EFFORT + TRANSIENT_LOCAL writers at the v1.16
  ``dds_topics.yaml`` rate limits; versioned names (``vehicle_status_v1``) from
  the vendored px4_msgs, as the agent advertises them.
* ``/fmu/in/*``: BEST_EFFORT + VOLATILE readers; samples reach PX4 after the
  UART hop.  Incoming ``timestamp`` / ``timestamp_sample`` are converted from
  the agent's (ROS) time base with the time-sync offset (``UXRCE_DDS_SYNCT``).
* Any ``/fmu/in`` publisher other than the kit's ``flight_interface`` and
  ``px4_odom_bridge`` is recorded: the kit README forbids team code doing it.
* The session comes up ``session_start_s`` after boot, like the real agent.
"""
from __future__ import annotations

import math
import os

from ..mathutil import q_mul, q_rotate
from ..rosfake.internal import InternalNode
from ..rosfake.runtime import NS, QoSInfo
from . import commander as C

PX4_OUT_QOS = QoSInfo(reliable=False, transient_local=True, depth=1)
PX4_IN_QOS = QoSInfo(reliable=False, transient_local=False, depth=10)

KIT_PUBLISHERS = {'/flight_interface', '/px4_odom_bridge'}

# topic -> rate limit (Hz) per PX4 v1.16 dds_topics.yaml (None = every update)
OUT_RATES = {
    'vehicle_status': 5.0, 'vehicle_local_position': 50.0, 'vehicle_land_detected': 5.0,
    'vehicle_attitude': 100.0, 'vehicle_odometry': 100.0, 'vehicle_control_mode': 5.0,
    'failsafe_flags': 5.0, 'estimator_status_flags': 5.0, 'battery_status': 1.0,
    'timesync_status': 10.0, 'manual_control_setpoint': 25.0, 'home_position': 5.0,
    'sensor_combined': 50.0,
}
HIGH_RATE = {'vehicle_attitude', 'vehicle_odometry', 'vehicle_local_position', 'sensor_combined',
             'manual_control_setpoint'}


def _camel(name):
    return ''.join(part.capitalize() for part in name.split('_'))


def _version(msg_dir, msg_name):
    path = os.path.join(msg_dir, msg_name + '.msg')
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            for line in fh:
                parts = line.split('#')[0].split()
                if len(parts) >= 3 and parts[0] == 'uint32' and parts[1].startswith('MESSAGE_VERSION'):
                    return int(line.split('=')[1].split('#')[0])
    except OSError:
        return 0
    return 0


class UxrceBridge:
    def __init__(self, rt, px4, msg_dir, session_start_s=1.5, uart_latency_s=0.003, sync_error_us=60):
        self.rt = rt
        self.px4 = px4
        self.msg_dir = msg_dir
        self.session_start_s = session_start_s
        self.uart_latency_s = uart_latency_s
        self.sync_error_us = int(rt.rng.gauss(0, sync_error_us))
        self.node = InternalNode(rt, 'uxrce_dds_client', instant_discovery=False)
        self.pubs = {}
        self.subs = []
        self.violations = []
        self._seen_publishers = set()
        self.started = False
        self.in_counts = {}
        import px4_msgs.msg as M
        self.M = M
        rt.at(int(session_start_s * NS), self._start)

    # -- names / time -----------------------------------------------------------------------
    def topic(self, direction, base, msg_name=None):
        msg_name = msg_name or _camel(base)
        v = _version(self.msg_dir, msg_name)
        return f'/fmu/{direction}/{base}' + (f'_v{v}' if v > 0 else '')

    def px4_us(self):
        return int(self.rt.now_ns // 1000)

    def ros_us_to_px4_s(self, ros_us):
        return (int(ros_us) - self.rt.epoch0_ns // 1000 - self.sync_error_us) / 1e6

    # -- session ------------------------------------------------------------------------------
    def _start(self):
        self.started = True
        M = self.M
        out = {
            'vehicle_status': M.VehicleStatus, 'vehicle_local_position': M.VehicleLocalPosition,
            'vehicle_land_detected': M.VehicleLandDetected, 'vehicle_attitude': M.VehicleAttitude,
            'vehicle_odometry': M.VehicleOdometry, 'vehicle_control_mode': M.VehicleControlMode,
            'failsafe_flags': M.FailsafeFlags, 'estimator_status_flags': M.EstimatorStatusFlags,
            'battery_status': M.BatteryStatus, 'timesync_status': M.TimesyncStatus,
            'manual_control_setpoint': M.ManualControlSetpoint, 'home_position': M.HomePosition,
            'sensor_combined': M.SensorCombined, 'vehicle_command_ack': M.VehicleCommandAck,
        }
        for base, mtype in out.items():
            self.pubs[base] = self.node.publisher(mtype, self.topic('out', base, mtype.__name__), PX4_OUT_QOS)
        ins = {
            'offboard_control_mode': (M.OffboardControlMode, self._on_offboard_mode),
            'trajectory_setpoint': (M.TrajectorySetpoint, self._on_trajectory),
            'vehicle_command': (M.VehicleCommand, self._on_command),
            'vehicle_visual_odometry': (M.VehicleOdometry, self._on_visual_odometry),
            'vehicle_mocap_odometry': (M.VehicleOdometry, self._on_unsupported),
            'vehicle_attitude_setpoint': (M.VehicleAttitudeSetpoint, self._on_unsupported),
            'vehicle_rates_setpoint': (M.VehicleRatesSetpoint, self._on_unsupported),
            'vehicle_thrust_setpoint': (M.VehicleThrustSetpoint, self._on_unsupported),
            'vehicle_torque_setpoint': (M.VehicleTorqueSetpoint, self._on_unsupported),
            'actuator_motors': (M.ActuatorMotors, self._on_unsupported),
            'goto_setpoint': (M.GotoSetpoint, self._on_unsupported),
        }
        for base, (mtype, cb) in ins.items():
            topic = self.topic('in', base, mtype.__name__)
            self.subs.append(self.node.subscribe(mtype, topic, self._delayed(cb, topic), PX4_IN_QOS))
        for base, rate in OUT_RATES.items():
            self.rt.after(0.01, self._publish_loop, base, 1.0 / rate)
        self.node.log('INFO', 'session established (simulated MicroXRCEAgent v2.4.3, serial 921600)')

    def _delayed(self, cb, topic):
        def handler(msg):
            self.in_counts[topic] = self.in_counts.get(topic, 0) + 1
            self._check_publishers(topic)
            self.rt.after(self.uart_latency_s, cb, msg)
        return handler

    def _check_publishers(self, topic):
        for ep in self.rt.graph.pubs.get(topic, ()):
            who = getattr(ep.owner, 'sim_fqn', '?')
            key = (topic, who)
            if key in self._seen_publishers:
                continue
            self._seen_publishers.add(key)
            if who not in KIT_PUBLISHERS:
                text = (f'{who} publishes to {topic}: team code must not publish to /fmu/in/* '
                        '(kit README; contracts/px4.md "Prohibition")')
                self.violations.append((self.rt.now_s, text))
                self.node.log('ERROR', 'SIM RULE CHECK: ' + text)

    # -- inbound ---------------------------------------------------------------------------------
    def _on_offboard_mode(self, msg):
        self.px4.offboard_mode = {
            't': self.px4.t, 'position': bool(msg.position), 'velocity': bool(msg.velocity),
            'acceleration': bool(msg.acceleration), 'attitude': bool(msg.attitude),
            'body_rate': bool(msg.body_rate), 'thrust_and_torque': bool(msg.thrust_and_torque),
            'direct_actuator': bool(msg.direct_actuator)}

    def _on_trajectory(self, msg):
        self.px4.traj_sp = {
            't': self.px4.t, 'position': tuple(float(v) for v in msg.position),
            'velocity': tuple(float(v) for v in msg.velocity),
            'acceleration': tuple(float(v) for v in msg.acceleration),
            'yaw': float(msg.yaw), 'yawspeed': float(msg.yawspeed)}

    def _on_command(self, msg):
        self.px4.push_command({
            'command': int(msg.command), 'param1': float(msg.param1), 'param2': float(msg.param2),
            'param3': float(msg.param3), 'param4': float(msg.param4), 'param7': float(msg.param7),
            'target_system': int(msg.target_system), 'target_component': int(msg.target_component),
            'from_external': bool(msg.from_external)})

    def _on_visual_odometry(self, msg):
        from ..mathutil import q_yaw
        q = tuple(float(v) for v in msg.q)
        yaw = q_yaw(q) if all(math.isfinite(v) for v in q) and any(q) else float('nan')
        pv = [float(v) for v in msg.position_variance]
        ov = [float(v) for v in msg.orientation_variance]
        self.px4.ev_queue.append({
            'sample_t': self.ros_us_to_px4_s(msg.timestamp_sample),
            'pos': tuple(float(v) for v in msg.position), 'yaw': yaw,
            'pos_var': max(pv[0], pv[1]) if all(math.isfinite(v) for v in pv[:2]) else 0.0,
            'yaw_var': ov[2] if math.isfinite(ov[2]) else 0.0,
            'reset_counter': int(msg.reset_counter), 'quality': int(msg.quality),
            'pose_frame': int(msg.pose_frame)})

    def _on_unsupported(self, msg):
        pass

    # -- outbound ---------------------------------------------------------------------------------
    def _publish_loop(self, base, period):
        pub = self.pubs[base]
        rate_period = period
        if base in HIGH_RATE and pub.subscription_count() == 0:
            rate_period = 1.0            # keep the TRANSIENT_LOCAL history fresh for late joiners
        try:
            msg = getattr(self, '_msg_' + base)()
        except Exception as exc:  # noqa: BLE001 - a modelling bug must not stop the sim silently
            self.node.log('ERROR', f'building {base}: {exc!r}')
            msg = None
        if msg is not None:
            self.rt.after(self.uart_latency_s, pub.publish, msg)
        self.rt.after(rate_period, self._publish_loop, base, period)

    def publish_acks(self):
        while self.px4.acks:
            cmd, result = self.px4.acks.popleft()
            m = self.M.VehicleCommandAck()
            m.timestamp = self.px4_us()
            m.command = int(cmd['command'])
            m.result = int(result)
            m.target_system = int(cmd.get('target_system', 1)) & 0xFF
            m.target_component = int(cmd.get('target_component', 1)) & 0xFFFF
            m.from_external = False
            if 'vehicle_command_ack' in self.pubs:
                self.rt.after(self.uart_latency_s, self.pubs['vehicle_command_ack'].publish, m)

    def _msg_vehicle_status(self):
        px4, cm = self.px4, self.px4.commander
        m = self.M.VehicleStatus()
        m.timestamp = self.px4_us()
        m.armed_time = int(cm.armed_time * 1e6) if cm.armed_time is not None else 0
        m.takeoff_time = int(cm.takeoff_time * 1e6) if cm.takeoff_time is not None else 0
        m.arming_state = C.ARMING_STATE_ARMED if cm.armed else C.ARMING_STATE_DISARMED
        m.latest_arming_reason = int(cm.latest_arming_reason)
        m.latest_disarming_reason = int(cm.latest_disarming_reason)
        m.nav_state_timestamp = int(cm.nav_state_timestamp * 1e6)
        m.nav_state_user_intention = int(cm.user_intended)
        m.nav_state = int(cm.nav_state)
        mask = 0
        for nav in C.NAV_NAMES:
            if nav != C.ACRO and cm.mode_can_run(nav):
                mask |= 1 << nav
        m.can_set_nav_states_mask = mask
        m.valid_nav_states_mask = mask
        m.vehicle_type = 1
        m.failsafe = bool(cm.failsafe)
        m.failsafe_and_user_took_over = bool(cm.failsafe_and_user_took_over)
        m.system_type = 2
        m.system_id = cm.p.i('MAV_SYS_ID') & 0xFF
        m.component_id = 1
        m.safety_off = True
        m.power_input_valid = True
        m.pre_flight_checks_pass = bool(cm.pre_flight_checks_pass)
        self.publish_acks()
        return m

    def _msg_vehicle_local_position(self):
        lp = self.px4.lpos_out
        if lp is None:
            return None
        e = self.px4.ekf
        m = self.M.VehicleLocalPosition()
        m.timestamp = m.timestamp_sample = self.px4_us()
        m.xy_valid, m.z_valid = bool(lp['xy_valid']), bool(lp['z_valid'])
        m.v_xy_valid, m.v_z_valid = bool(lp['v_xy_valid']), bool(lp['v_z_valid'])
        m.x, m.y, m.z = float(lp['x']), float(lp['y']), float(lp['z'])
        m.vx, m.vy, m.vz = float(lp['vx']), float(lp['vy']), float(lp['vz'])
        m.z_deriv = float(lp['vz'])
        m.ax, m.ay, m.az = float(lp['ax']), float(lp['ay']), float(lp['az'])
        m.heading = float(lp['heading'])
        m.heading_var = float(e.yaw_var)
        m.unaided_heading = float('nan')
        m.delta_heading = float(e.delta_heading)
        m.heading_reset_counter = e.heading_reset_counter & 0xFF
        m.heading_good_for_control = bool(e.ev_yaw_active or e.mag_heading)
        m.delta_xy = [float(e.delta_xy[0]), float(e.delta_xy[1])]
        m.xy_reset_counter = e.xy_reset_counter & 0xFF
        m.delta_z = float(e.delta_z)
        m.z_reset_counter = e.z_reset_counter & 0xFF
        m.xy_global = False
        m.z_global = False
        m.ref_lat = float('nan')
        m.ref_lon = float('nan')
        m.ref_alt = float('nan')
        m.dist_bottom_valid = bool(lp['dist_bottom_valid'])
        m.dist_bottom = float(lp['dist_bottom']) if math.isfinite(lp['dist_bottom']) else 0.0
        m.dist_bottom_sensor_bitfield = 1 if lp['dist_bottom_valid'] else 0
        m.eph, m.epv = float(min(lp['eph'], 1e6)), float(min(lp['epv'], 1e6))
        m.evh, m.evv = float(min(lp['evh'], 1e6)), float(min(lp['evv'], 1e6))
        m.dead_reckoning = bool(lp['dead_reckoning'])
        m.vxy_max = float('inf')
        m.vz_max = float('inf')
        m.hagl_min = float('inf')
        m.hagl_max_z = float('inf')
        m.hagl_max_xy = float('inf')
        return m

    def _msg_vehicle_land_detected(self):
        ld = self.px4.land_detector
        m = self.M.VehicleLandDetected()
        m.timestamp = self.px4_us()
        m.freefall, m.ground_contact, m.maybe_landed, m.landed = (bool(ld.freefall), bool(ld.ground_contact),
                                                                  bool(ld.maybe_landed), bool(ld.landed))
        m.in_descend, m.has_low_throttle = bool(ld.in_descend), bool(ld.has_low_throttle)
        m.vertical_movement, m.horizontal_movement = bool(ld.vertical_movement), bool(ld.horizontal_movement)
        m.rotational_movement, m.at_rest = bool(ld.rotational_movement), bool(ld.at_rest)
        m.close_to_ground_or_skipped_check = True
        return m

    def _msg_vehicle_attitude(self):
        m = self.M.VehicleAttitude()
        m.timestamp = m.timestamp_sample = self.px4_us()
        m.q = [float(v) for v in self.px4.att_q]
        m.quat_reset_counter = self.px4.ekf.heading_reset_counter & 0xFF
        return m

    def _msg_vehicle_odometry(self):
        lp = self.px4.lpos_out
        if lp is None:
            return None
        m = self.M.VehicleOdometry()
        m.timestamp = m.timestamp_sample = self.px4_us()
        m.pose_frame = m.POSE_FRAME_NED
        nan = float('nan')
        m.position = [float(lp['x']), float(lp['y']), float(lp['z'])] if lp['xy_valid'] else [nan, nan, float(lp['z'])]
        m.q = [float(v) for v in self.px4.att_q]
        m.velocity_frame = m.VELOCITY_FRAME_NED
        m.velocity = [float(lp['vx']), float(lp['vy']), float(lp['vz'])]
        m.angular_velocity = [float(v) for v in self.px4.rates]
        m.position_variance = [float(lp['eph']) ** 2, float(lp['eph']) ** 2, float(lp['epv']) ** 2]
        m.orientation_variance = [1e-4, 1e-4, float(self.px4.ekf.yaw_var)]
        m.velocity_variance = [float(lp['evh']) ** 2, float(lp['evh']) ** 2, float(lp['evv']) ** 2]
        m.reset_counter = self.px4.ekf.xy_reset_counter & 0xFF
        m.quality = 0
        return m

    def _msg_vehicle_control_mode(self):
        cm = self.px4.commander
        nav = cm.nav_state
        m = self.M.VehicleControlMode()
        m.timestamp = self.px4_us()
        m.flag_armed = bool(cm.armed)
        ocm = self.px4.offboard_mode or {}
        manual = nav in (C.MANUAL, C.STAB, C.ALTCTL, C.POSCTL, C.ACRO)
        auto = nav in (C.AUTO_LOITER, C.AUTO_LAND, C.AUTO_TAKEOFF, C.AUTO_RTL, C.AUTO_MISSION, C.DESCEND)
        offb = nav == C.OFFBOARD
        m.flag_control_manual_enabled = manual
        m.flag_control_auto_enabled = auto
        m.flag_control_offboard_enabled = offb
        pos = nav in (C.POSCTL, C.AUTO_LOITER, C.AUTO_LAND, C.AUTO_TAKEOFF) or (offb and ocm.get('position', False))
        vel = pos or (offb and ocm.get('velocity', False))
        alt = vel or nav in (C.ALTCTL, C.DESCEND)
        m.flag_control_position_enabled = pos
        m.flag_control_velocity_enabled = vel
        m.flag_control_altitude_enabled = alt
        m.flag_control_climb_rate_enabled = alt
        m.flag_control_acceleration_enabled = vel or (offb and ocm.get('acceleration', False))
        m.flag_control_attitude_enabled = nav != C.ACRO and nav != C.TERMINATION
        m.flag_control_rates_enabled = nav != C.TERMINATION
        m.flag_control_allocation_enabled = nav != C.TERMINATION
        m.flag_control_termination_enabled = nav == C.TERMINATION
        m.flag_multicopter_position_control_enabled = alt
        return m

    def _msg_failsafe_flags(self):
        f = self.px4.commander.flags
        m = self.M.FailsafeFlags()
        m.timestamp = self.px4_us()
        for k, v in f.items():
            if hasattr(m, k):
                if k == 'battery_warning':
                    setattr(m, k, int(v))
                else:
                    setattr(m, k, bool(v))
        return m

    def _msg_estimator_status_flags(self):
        e = self.px4.ekf
        m = self.M.EstimatorStatusFlags()
        m.timestamp = self.px4_us()
        vals = {'cs_tilt_align': e.tilt_aligned, 'cs_yaw_align': True, 'cs_ev_pos': e.ev_active and e.xy_valid(),
                'cs_ev_yaw': e.ev_yaw_active, 'cs_rng_hgt': e.rng_is_ref, 'cs_baro_hgt': e.baro_is_ref,
                'cs_mag_hdg': e.mag_heading, 'cs_in_air': e.in_air,
                'cs_inertial_dead_reckoning': e.ev_active and e.dead_reckoning_s > 0.5,
                'cs_rng_kin_consistent': e.rng_good_since is not None, 'cs_vehicle_at_rest': e.on_ground,
                'reject_ver_pos': False}
        for k, v in vals.items():
            if hasattr(m, k):
                setattr(m, k, bool(v))
        return m

    def _msg_battery_status(self):
        b = self.px4.physics.battery
        m = self.M.BatteryStatus()
        m.timestamp = self.px4_us()
        m.connected = True
        m.voltage_v = float(b.voltage())
        m.current_a = float(b.current)
        m.current_average_a = float(b.current)
        m.remaining = float(b.soc)
        m.cell_count = int(b.cells)
        m.warning = int(self.px4.battery_warning)
        return m

    def _msg_timesync_status(self):
        m = self.M.TimesyncStatus()
        m.timestamp = self.px4_us()
        m.source_protocol = 2
        m.remote_timestamp = int(self.rt.ros_time_ns() // 1000)
        m.observed_offset = int(self.rt.epoch0_ns // 1000 + self.sync_error_us)
        m.estimated_offset = int(self.rt.epoch0_ns // 1000 + self.sync_error_us)
        m.round_trip_time = 6000
        return m

    def _msg_manual_control_setpoint(self):
        rc = self.px4.rc
        m = self.M.ManualControlSetpoint()
        m.timestamp = m.timestamp_sample = self.px4_us()
        m.valid = bool(rc.link and not self.px4.commander.rc_lost)
        m.data_source = 1
        m.roll, m.pitch, m.yaw, m.throttle = float(rc.roll), float(rc.pitch), float(rc.yaw), float(rc.throttle)
        return m

    def _msg_home_position(self):
        cm = self.px4.commander
        if cm.home is None:
            return None
        m = self.M.HomePosition()
        m.timestamp = self.px4_us()
        m.lat = m.lon = float('nan')
        m.alt = float('nan')
        m.z = float(cm.home)
        m.valid_alt = True
        m.valid_lpos = bool(self.px4.lpos_out and self.px4.lpos_out['xy_valid'])
        return m

    def _msg_sensor_combined(self):
        m = self.M.SensorCombined()
        m.timestamp = self.px4_us()
        m.gyro_rad = [float(v) for v in self.px4.imu_gyro]
        m.accelerometer_m_s2 = [float(v) for v in self.px4.imu_accel]
        return m

    def _msg_vehicle_command_ack(self):
        return None


_unused = (q_mul, q_rotate)
