"""DroneHacks SITL -- the graphical test environment.

    cd sitl && py -m dronesim gui          (or double-click sitl/run_gui.cmd)

Pick an autonomy version and a follower, set the scenario, press Start.  The
simulation runs in a separate process (``gui/worker.py``); this window only
draws and sends pilot inputs and faults.
"""
from __future__ import annotations

import json
import math
import multiprocessing as mp
import os
import sys
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
SITL_ROOT = os.path.dirname(os.path.dirname(HERE))
REPO_ROOT = os.path.dirname(SITL_ROOT)
if SITL_ROOT not in sys.path:
    sys.path.insert(0, SITL_ROOT)

from dronesim.autonomy.catalog import load_catalog  # noqa: E402
from dronesim.gui.mapview import MapView, StripChart, course_shapes  # noqa: E402
from dronesim.px4.params import DEFAULT_PROFILE, PROFILES  # noqa: E402

NAV_COLORS = {'OFFBOARD': '#2e7d32', 'POSCTL': '#1565c0', 'ALTCTL': '#ef6c00', 'MANUAL': '#c62828',
              'STAB': '#c62828', 'AUTO_LOITER': '#6a1b9a', 'AUTO_LAND': '#6a1b9a', 'DESCEND': '#6a1b9a',
              'TERMINATION': '#000000'}
LEVEL_TAGS = {'WARN': 'warn', 'ERROR': 'error', 'CRITICAL': 'error', 'FATAL': 'error', 'DEBUG': 'debug'}
SPEEDS = [('0.5x', 0.5), ('1x real time', 1.0), ('2x', 2.0), ('4x', 4.0), ('max', None)]


class App:
    def __init__(self, root):
        self.root = root
        root.title('DroneHacks SITL - drone 1 (skybeta-01), PX4 v1.16 + skybeta 0.3.2 kit, STEP course')
        root.geometry('1600x980')
        try:
            root.tk.call('tk', 'scaling', 1.25)
        except tk.TclError:
            pass
        style = ttk.Style()
        if 'vista' in style.theme_names():
            style.theme_use('vista')
        self.catalog = load_catalog(os.path.join(SITL_ROOT, 'versions.yaml'))
        self.proc = None
        self.conn = None
        self.info = None
        self.result = None
        self.last_snap = None
        self.log_lines = 0
        self._build_layout()
        self._load_course_async()
        self.root.after(50, self._poll)
        root.protocol('WM_DELETE_WINDOW', self._on_close)

    # ------------------------------------------------------------------------------------------
    # layout
    # ------------------------------------------------------------------------------------------
    def _build_layout(self):
        outer = ttk.PanedWindow(self.root, orient='horizontal')
        outer.pack(fill='both', expand=True)
        left = ttk.Frame(outer, width=380)
        center = ttk.PanedWindow(outer, orient='vertical')
        right = ttk.Frame(outer, padding=6, width=360)
        outer.add(left, weight=0)
        outer.add(center, weight=1)
        outer.add(right, weight=0)
        # the run controls stay at the bottom of the left column; the setup above them scrolls
        footer = ttk.Frame(left, padding=(6, 4, 6, 6))
        footer.pack(side='bottom', fill='x')
        ttk.Separator(left, orient='horizontal').pack(side='bottom', fill='x')
        setup = self._scrollable(left)
        self._build_setup(setup, footer)
        self._build_center(center)
        self._build_status(right)

    def _scrollable(self, parent):
        """A vertically scrolling frame filling *parent*: the mouse wheel scrolls it while over it."""
        holder = ttk.Frame(parent)
        holder.pack(side='top', fill='both', expand=True)
        canvas = tk.Canvas(holder, highlightthickness=0, borderwidth=0,
                           background=self.root.cget('background'))
        bar = ttk.Scrollbar(holder, orient='vertical', command=canvas.yview)
        canvas.configure(yscrollcommand=bar.set)
        bar.pack(side='right', fill='y')
        canvas.pack(side='left', fill='both', expand=True)
        inner = ttk.Frame(canvas, padding=6)
        window = canvas.create_window((0, 0), window=inner, anchor='nw')
        inner.bind('<Configure>', lambda e: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<Configure>', lambda e: canvas.itemconfigure(window, width=e.width))
        # A wheel turn over a dropdown would otherwise change its value while scrolling past it
        # (ttk's TCombobox class binding), silently changing the version or an option.
        for seq in ('<MouseWheel>', '<Button-4>', '<Button-5>'):
            self.root.unbind_class('TCombobox', seq)

        def wheel(event):
            # one handler for the whole window: scroll only when the pointer is over this column
            # (the map keeps its own wheel zoom)
            try:
                under = self.root.winfo_containing(event.x_root, event.y_root)
            except (KeyError, tk.TclError):            # a dropdown's popup list is not a tkinter widget
                return
            if under is None or not str(under).startswith(str(holder)):
                return
            if inner.winfo_height() <= canvas.winfo_height():
                return
            num = getattr(event, 'num', 0)
            step = -1 if num == 4 else 1 if num == 5 else (-1 if event.delta > 0 else 1) * max(1, abs(event.delta) // 120)
            canvas.yview_scroll(step * 2, 'units')

        for seq in ('<MouseWheel>', '<Button-4>', '<Button-5>'):
            self.root.bind_all(seq, wheel, add='+')
        self.setup_canvas = canvas
        return inner

    def _build_setup(self, f, footer):
        ttk.Label(f, text='Autonomy version', font=('Segoe UI', 10, 'bold')).pack(anchor='w')
        self.version_ids = list(self.catalog)
        self.version_var = tk.StringVar(value=self.version_ids[0])
        labels = [self.catalog[v]['label'] for v in self.version_ids]
        self.version_box = ttk.Combobox(f, values=labels, state='readonly', width=52)
        self.version_box.current(0)
        self.version_box.pack(fill='x')
        self.version_box.bind('<<ComboboxSelected>>', lambda e: self._on_version())
        self.version_desc = tk.Text(f, height=4, wrap='word', font=('Segoe UI', 8), relief='flat',
                                    background=self.root.cget('background'))
        self.version_desc.pack(fill='x', pady=(2, 6))
        ttk.Label(f, text='Follower algorithm', font=('Segoe UI', 10, 'bold')).pack(anchor='w')
        self.follower_box = ttk.Combobox(f, state='readonly', width=52)
        self.follower_box.pack(fill='x')
        self.follower_note = ttk.Label(f, text='', foreground='#ad1457', wraplength=340, font=('Segoe UI', 8))
        self.follower_note.pack(anchor='w')
        self.follower_box.bind('<<ComboboxSelected>>', lambda e: self._on_follower())
        self.options_frame = ttk.LabelFrame(f, text='Version options', padding=4)
        self.options_frame.pack(fill='x', pady=4)
        self.option_vars = {}
        ttk.Label(f, text='PX4 parameter profile', font=('Segoe UI', 10, 'bold')).pack(anchor='w', pady=(6, 0))
        self.profile_names = list(PROFILES)
        self.profile_box = ttk.Combobox(f, values=self.profile_names, state='readonly', width=52)
        self.profile_box.set(DEFAULT_PROFILE)
        self.profile_box.pack(fill='x')
        self.profile_desc = ttk.Label(f, text=PROFILES[DEFAULT_PROFILE][0], wraplength=340, font=('Segoe UI', 8))
        self.profile_desc.pack(anchor='w')
        self.profile_box.bind('<<ComboboxSelected>>', lambda e: self._on_profile())
        sc = ttk.LabelFrame(f, text='Scenario', padding=4)
        sc.pack(fill='x', pady=6)
        self.fields = {}

        def field(row, label, key, default, width=10, choices=None):
            ttk.Label(sc, text=label).grid(row=row, column=0, sticky='w')
            var = tk.StringVar(value=str(default))
            if choices:
                w = ttk.Combobox(sc, textvariable=var, values=choices, state='readonly', width=width)
            else:
                w = ttk.Entry(sc, textvariable=var, width=width)
            w.grid(row=row, column=1, sticky='we', padx=4, pady=1)
            self.fields[key] = var
        self._cap_default = self._profile_speed_cap(DEFAULT_PROFILE)
        field(0, 'Speed cap, MPC_XY_VEL_MAX (m/s)', 'speed_cap', self._cap_default)
        field(1, 'Track speed (m/s, blank: version default)', 'track_speed', '')
        field(2, 'Landing descent (m/s, blank: version default)', 'descent_speed', '')
        field(3, 'Seed', 'seed', 1)
        field(4, 'Duration (s)', 'duration_s', 200)
        field(5, 'Start heading (deg cw from N)', 'start_heading_deg', 0)
        field(6, 'Start offset east, north (m)', 'start_offset_m', '0, 0')
        field(7, 'Calibration error pos (m, 1-sigma)', 'calib_pos_error_m', 0.0)
        field(8, 'Calibration error yaw (deg)', 'calib_yaw_error_deg', 0.0)
        field(9, 'Tunnel lighting', 'tunnel_lighting', 'dark', choices=['dark', 'dim', 'lit'])
        field(10, 'cuVSLAM drift x (1: a few cm; 3+: touches)', 'vo_drift', 1.0)
        field(11, 'cuVSLAM latency mean (s; PX4 drops vision past ~0.18)', 'vo_latency', 0.062)
        field(12, 'EKF2_EV_DELAY override (ms, blank)', 'ev_delay', '')
        field(13, 'Other PX4 overrides (N=v, ...)', 'param_overrides', '')
        field(14, 'Wind east, north (m/s)', 'wind', '0, 0')
        field(15, 'Pilot arms at (s)', 'pilot_arm_at_s', 15.0)
        field(16, 'Pilot -> Position after (s)', 'pilot_position_after_s', 2.0)
        self.script_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(sc, text='Scripted pilot (organizers\' arm-then-Position procedure)',
                        variable=self.script_var).grid(row=17, column=0, columnspan=2, sticky='w')
        sc.columnconfigure(1, weight=1)
        f = footer                                       # the run controls: always visible
        sp = ttk.Frame(f)
        sp.pack(fill='x', pady=4)
        ttk.Label(sp, text='Speed').pack(side='left')
        self.speed_box = ttk.Combobox(sp, values=[s[0] for s in SPEEDS], state='readonly', width=14)
        self.speed_box.current(1)
        self.speed_box.pack(side='left', padx=4)
        self.speed_box.bind('<<ComboboxSelected>>', lambda e: self._send(('speed', self._speed())))
        btns = ttk.Frame(f)
        btns.pack(fill='x', pady=4)
        self.start_btn = ttk.Button(btns, text='Start', command=self.start)
        self.start_btn.pack(side='left', expand=True, fill='x')
        self.pause_btn = ttk.Button(btns, text='Pause', command=self.toggle_pause, state='disabled')
        self.pause_btn.pack(side='left', expand=True, fill='x')
        self.stop_btn = ttk.Button(btns, text='Stop', command=self.stop, state='disabled')
        self.stop_btn.pack(side='left', expand=True, fill='x')
        self.save_btn = ttk.Button(f, text='Save run (result.json, log.txt, trajectory.csv)...', command=self.save,
                                   state='disabled')
        self.save_btn.pack(fill='x')
        self.status_line = ttk.Label(f, text='Loading the STEP course...', foreground='#455a64', wraplength=340)
        self.status_line.pack(anchor='w', pady=6)
        self._on_version()

    def _build_center(self, pane):
        top = ttk.Frame(pane)
        pane.add(top, weight=4)
        self.map_holder = ttk.Frame(top)
        self.map_holder.pack(fill='both', expand=True)
        bar = ttk.Frame(top)
        bar.pack(fill='x')
        self.map = None
        self._map_bar = bar
        charts = ttk.Frame(pane)
        pane.add(charts, weight=1)
        self.chart_z = StripChart(charts, 'Height above the floor (m)', [
            ('true', '#212121', 'true', None), ('ekf', '#1565c0', 'EKF2', None),
            ('vo', '#2e7d32', 'cuVSLAM+rest', None), ('sp', '#c62828', 'setpoint', (4, 2))], height=140)
        self.chart_z.canvas.pack(side='left', fill='both', expand=True)
        self.chart_v = StripChart(charts, 'Speed (m/s) / EKF horizontal error (m)', [
            ('speed', '#212121', 'speed', None), ('err', '#ef6c00', 'EKF2 error', None),
            ('voerr', '#2e7d32', 'cuVSLAM error', None)], height=140)
        self.chart_v.canvas.pack(side='left', fill='both', expand=True)
        nb = ttk.Notebook(pane)
        pane.add(nb, weight=2)
        self.nb = nb
        # logs
        lf = ttk.Frame(nb)
        nb.add(lf, text='Logs')
        fb = ttk.Frame(lf)
        fb.pack(fill='x')
        ttk.Label(fb, text='filter').pack(side='left')
        self.log_filter = tk.StringVar()
        ttk.Entry(fb, textvariable=self.log_filter, width=30).pack(side='left', padx=4)
        self.hide_info = tk.BooleanVar(value=False)
        ttk.Checkbutton(fb, text='warnings and errors only', variable=self.hide_info).pack(side='left')
        self.hide_clamp = tk.BooleanVar(value=True)
        ttk.Checkbutton(fb, text='hide repeated clamp warnings', variable=self.hide_clamp).pack(side='left')
        self.log_text = tk.Text(lf, wrap='none', font=('Consolas', 8), height=10)
        sy = ttk.Scrollbar(lf, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=sy.set)
        sy.pack(side='right', fill='y')
        self.log_text.pack(fill='both', expand=True)
        self.log_text.tag_configure('warn', foreground='#e65100')
        self.log_text.tag_configure('error', foreground='#c62828')
        self.log_text.tag_configure('debug', foreground='#9e9e9e')
        self.log_text.tag_configure('px4', foreground='#4a148c')
        # events
        ef = ttk.Frame(nb)
        nb.add(ef, text='Referee events')
        self.events_list = tk.Listbox(ef, font=('Consolas', 9))
        self.events_list.pack(fill='both', expand=True)
        # topics
        tf = ttk.Frame(nb)
        nb.add(tf, text='Topics')
        cols = ('type', 'pubs', 'subs', 'hz')
        self.topic_tree = ttk.Treeview(tf, columns=cols, height=8)
        self.topic_tree.heading('#0', text='topic')
        for c, w in zip(cols, (300, 50, 50, 60)):
            self.topic_tree.heading(c, text=c)
            self.topic_tree.column(c, width=w, anchor='w')
        self.topic_tree.column('#0', width=330)
        self.topic_tree.pack(side='left', fill='both', expand=True)
        self.topic_tree.bind('<<TreeviewSelect>>', self._on_topic)
        self.echo_text = tk.Text(tf, width=60, font=('Consolas', 8))
        self.echo_text.pack(side='left', fill='both', expand=True)
        # px4
        pf = ttk.Frame(nb)
        nb.add(pf, text='PX4 messages')
        self.px4_list = tk.Listbox(pf, font=('Consolas', 9))
        self.px4_list.pack(fill='both', expand=True)
        # run info
        rf = ttk.Frame(nb)
        nb.add(rf, text='Run info / result')
        self.info_text = tk.Text(rf, wrap='word', font=('Consolas', 8))
        self.info_text.pack(fill='both', expand=True)

    def _build_status(self, f):
        self.nav_label = tk.Label(f, text='-', font=('Segoe UI', 18, 'bold'), fg='white', bg='#607d8b')
        self.nav_label.pack(fill='x')
        self.svars = {}
        grid = ttk.Frame(f)
        grid.pack(fill='x', pady=4)
        rows = [('t', 'sim time'), ('rtf', 'real-time factor'), ('armed', 'armed / landed'),
                ('intended', 'user intended mode'), ('failsafe', 'failsafe'), ('takeoff', 'takeoff state'),
                ('ekf', 'EKF2 position'), ('ev', 'vision fusion'), ('height', 'height src / dist_bottom'),
                ('vo', 'cuVSLAM'), ('sp', 'setpoint / offboard age'), ('pos', 'true position (course)'),
                ('bat', 'battery'), ('lidar', 'MS200K')]
        for i, (k, label) in enumerate(rows):
            ttk.Label(grid, text=label, foreground='#546e7a').grid(row=i, column=0, sticky='w')
            v = tk.StringVar(value='-')
            ttk.Label(grid, textvariable=v, font=('Consolas', 9)).grid(row=i, column=1, sticky='w', padx=4)
            self.svars[k] = v
        ref = ttk.LabelFrame(f, text='Referee (rulebook section 4, ground truth)', padding=4)
        ref.pack(fill='x', pady=4)
        self.ref_tree = ttk.Treeview(ref, columns=('pts', 'centre', 'whole', 'route'), height=5)
        self.ref_tree.heading('#0', text='checkpoint')
        for c, t, w in (('pts', 'pts', 40), ('centre', 'centre s', 60), ('whole', 'whole s', 60), ('route', 'route', 60)):
            self.ref_tree.heading(c, text=t)
            self.ref_tree.column(c, width=w, anchor='center')
        self.ref_tree.column('#0', width=170)
        self.ref_tree.pack(fill='x')
        self.ref_summary = ttk.Label(ref, text='', wraplength=330, font=('Segoe UI', 8))
        self.ref_summary.pack(anchor='w')
        pil = ttk.LabelFrame(f, text='Safety pilot (RC)', padding=4)
        pil.pack(fill='x', pady=4)
        self.arm_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(pil, text='arm switch (ch 7)', variable=self.arm_var,
                        command=lambda: self._pilot('arm' if self.arm_var.get() else 'disarm')).grid(row=0, column=0, sticky='w')
        self.kill_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(pil, text='KILL (ch 5)', variable=self.kill_var,
                        command=lambda: self._pilot('kill', self.kill_var.get())).grid(row=0, column=1, sticky='w')
        self.link_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(pil, text='RC link', variable=self.link_var,
                        command=lambda: self._pilot('rc_link', self.link_var.get())).grid(row=0, column=2, sticky='w')
        self.mode_var = tk.StringVar(value='altitude')
        for i, m in enumerate(('manual', 'altitude', 'position')):
            ttk.Radiobutton(pil, text=m.capitalize(), value=m, variable=self.mode_var,
                            command=lambda: self._pilot('mode', self.mode_var.get())).grid(row=1, column=i, sticky='w')
        ttk.Label(pil, text='throttle').grid(row=2, column=0, sticky='w')
        self.thr_scale = tk.Scale(pil, from_=-1.0, to=1.0, resolution=0.05, orient='horizontal', length=170,
                                  command=lambda v: None)
        self.thr_scale.set(-1.0)
        self.thr_scale.grid(row=2, column=1, columnspan=2, sticky='we')
        self.thr_scale.bind('<ButtonRelease-1>', lambda e: self._pilot('throttle', self.thr_scale.get()))
        ttk.Button(pil, text='Take over (throttle to centre)', command=lambda: self._pilot('takeover')).grid(
            row=3, column=0, columnspan=2, sticky='we')
        ttk.Button(pil, text='Release sticks', command=lambda: self._pilot('release')).grid(row=3, column=2, sticky='we')
        flt = ttk.LabelFrame(f, text='Faults', padding=4)
        flt.pack(fill='x', pady=4)
        ttk.Button(flt, text='cuVSLAM lost 2 s', command=lambda: self._cmd({'action': 'vo_lost', 'duration': 2.0})).grid(
            row=0, column=0, sticky='we')
        ttk.Button(flt, text='cuVSLAM lost', command=lambda: self._cmd({'action': 'vo_lost'})).grid(row=0, column=1, sticky='we')
        ttk.Button(flt, text='restore', command=lambda: self._cmd({'action': 'vo_restore'})).grid(row=0, column=2, sticky='we')
        ttk.Button(flt, text='lidar off', command=lambda: self._cmd({'action': 'lidar', 'value': False})).grid(
            row=1, column=0, sticky='we')
        ttk.Button(flt, text='lidar on', command=lambda: self._cmd({'action': 'lidar', 'value': True})).grid(
            row=1, column=1, sticky='we')
        ttk.Button(flt, text='gust 4 N east', command=lambda: self._cmd({'action': 'push', 'force_n': (4.0, 0.0, 0.0),
                                                                          'duration': 0.6})).grid(row=1, column=2, sticky='we')
        self.kill_node_box = ttk.Combobox(flt, values=[], width=22)
        self.kill_node_box.grid(row=2, column=0, columnspan=2, sticky='we')
        ttk.Button(flt, text='kill process', command=lambda: self._cmd(
            {'action': 'kill_node', 'name': self.kill_node_box.get()})).grid(row=2, column=2, sticky='we')
        legend = ttk.Label(f, text='Map: black/green = true drone (props green when armed), blue o = EKF2, '
                                   'green o = cuVSLAM, red x = setpoint, purple = lidar returns, magenta dashes = '
                                   'checkpoint gates. Wheel = zoom, right-drag = pan, double-click = fit.',
                           wraplength=340, font=('Segoe UI', 8), foreground='#546e7a')
        legend.pack(anchor='w', pady=4)

    # ------------------------------------------------------------------------------------------
    # course
    # ------------------------------------------------------------------------------------------
    def _load_course_async(self):
        import threading

        def work():
            try:
                from dronesim.px4.params import load_profile
                from dronesim.scoring.referee import Referee
                from dronesim.vehicle.physics import AirframeConfig
                from dronesim.world.course import Course
                cache = os.path.join(os.environ.get('LOCALAPPDATA') or os.path.expanduser('~'), 'dronesim', 'cache')
                course = Course(os.path.join(REPO_ROOT, 'dronehacks-course-v3.step'), cache)
                params = load_profile(REPO_ROOT, DEFAULT_PROFILE)
                af = AirframeConfig.from_params(params)
                ref = Referee(course, os.path.join(SITL_ROOT, 'course', 'course_v3_rules.json'), af)
                self._course_ready = (course_shapes(course), ref.gates_for_display(), af.rotors, af.prop_radius_m,
                                      course.describe())
            except Exception as exc:  # noqa: BLE001
                self._course_ready = exc
        self._course_ready = None
        threading.Thread(target=work, daemon=True).start()

    def _install_map(self, data):
        shapes, gates, rotors, prop_r, desc = data
        self.map = MapView(self.map_holder, shapes, gates, rotors, prop_r)
        self.map.canvas.pack(fill='both', expand=True)
        ttk.Checkbutton(self._map_bar, text='lidar', variable=self.map.show_lidar).pack(side='left')
        ttk.Checkbutton(self._map_bar, text='estimates', variable=self.map.show_est).pack(side='left')
        ttk.Button(self._map_bar, text='Fit', command=self.map.fit).pack(side='left')
        ttk.Label(self._map_bar, text=f"  {desc['step']}  ({desc['solids']} solids, {desc['triangles']} triangles, "
                                      f"sha256 {desc['sha256'][:12]})", foreground='#607d8b').pack(side='left')
        self.root.after(100, self.map.fit)
        self.status_line.configure(text='Ready. Choose a version and press Start.')

    # ------------------------------------------------------------------------------------------
    # setup callbacks
    # ------------------------------------------------------------------------------------------
    def _current_version(self):
        return self.version_ids[self.version_box.current()]

    def _on_version(self):
        v = self.catalog[self._current_version()]
        self.version_desc.delete('1.0', 'end')
        src = v.get('source') or {}
        src_txt = ('git ' + src['git']) if 'git' in src else ('dir ' + src['dir']) if 'dir' in src else 'organizer kit'
        self.version_desc.insert('1.0', f"{v.get('description', '')}\nsource: {src_txt}\n"
                                        f"COMPETITOR_PKG={v['launch']['package']}  COMPETITOR_LAUNCH={v['launch']['file']}")
        fol = v['followers']
        self.follower_ids = list(fol['choices'])
        self.follower_box.configure(values=[fol['choices'][k].get('label', k) for k in self.follower_ids])
        self.follower_box.current(self.follower_ids.index(fol.get('default', self.follower_ids[0])))
        self._on_follower()
        for w in self.options_frame.winfo_children():
            w.destroy()
        self.option_vars = {}
        for i, (name, opt) in enumerate(v.get('options', {}).items()):
            ttk.Label(self.options_frame, text=opt.get('label', name)).grid(row=i, column=0, sticky='w')
            keys = list(opt['choices'])
            box = ttk.Combobox(self.options_frame, values=[opt['choices'][k].get('label', k) for k in keys],
                               state='readonly', width=34)
            box.current(keys.index(str(opt.get('default', keys[0]))))
            box.grid(row=i, column=1, sticky='we', padx=4)
            self.option_vars[name] = (box, keys)
        if not v.get('options'):
            ttk.Label(self.options_frame, text='(none)').grid(row=0, column=0)

    def _on_follower(self):
        v = self.catalog[self._current_version()]
        fid = self.follower_ids[self.follower_box.current()]
        ch = v['followers']['choices'][fid]
        if ch.get('hook'):
            self.follower_note.configure(text='SIMULATOR INJECTION: the drone would run the shipped follower; this '
                                              'patches the mission module in the simulated process only.')
        elif ch.get('params'):
            self.follower_note.configure(text=f"Real parameter of this version, written to its installed config: "
                                              f"{json.dumps(ch['params'])}")
        else:
            self.follower_note.configure(text='Shipped default.')

    def _speed(self):
        return SPEEDS[self.speed_box.current()][1]

    @staticmethod
    def _profile_speed_cap(name):
        """The profile's MPC_XY_VEL_MAX, as the field shows it ('' if the files can't be read)."""
        try:
            from dronesim.px4.params import load_profile
            return f"{load_profile(REPO_ROOT, name).f('MPC_XY_VEL_MAX'):g}"
        except Exception:  # noqa: BLE001 - the field then starts blank: the profile's value
            return ''

    def _on_profile(self):
        name = self.profile_box.get()
        self.profile_desc.configure(text=PROFILES[name][0])
        new = self._profile_speed_cap(name)
        var = self.fields['speed_cap']
        if var.get().strip() in ('', self._cap_default):     # not edited: follow the profile
            var.set(new)
        self._cap_default = new

    def _pair(self, text):
        parts = [p.strip() for p in str(text).replace(';', ',').split(',') if p.strip()]
        return (float(parts[0]), float(parts[1])) if len(parts) == 2 else (0.0, 0.0)

    def _scenario(self):
        v = self.catalog[self._current_version()]
        f = self.fields
        sc = {
            'version': v['id'], 'follower': self.follower_ids[self.follower_box.current()],
            'options': {name: keys[box.current()] for name, (box, keys) in self.option_vars.items()},
            'param_profile': self.profile_box.get(), 'seed': int(float(f['seed'].get())),
            'duration_s': float(f['duration_s'].get()), 'start_heading_deg': float(f['start_heading_deg'].get()),
            'start_offset_m': self._pair(f['start_offset_m'].get()),
            'calib_pos_error_m': float(f['calib_pos_error_m'].get()),
            'calib_yaw_error_deg': float(f['calib_yaw_error_deg'].get()),
            'pilot_arm_at_s': float(f['pilot_arm_at_s'].get()),
            'pilot_position_after_s': float(f['pilot_position_after_s'].get()),
            'pilot_script': bool(self.script_var.get()), 'wind_mps': self._pair(f['wind'].get()),
        }
        drift = float(f['vo_drift'].get())
        sc['vo'] = {'tunnel_lighting': f['tunnel_lighting'].get(), 'latency_mean_s': float(f['vo_latency'].get()),
                    'scale_error': 0.006 * drift, 'yaw_drift_deg_per_m': 0.25 * drift,
                    'yaw_drift_deg_per_rad': 0.4 * drift, 'pos_rw_per_sqrt_m': 0.004 * drift,
                    'z_rw_per_sqrt_m': 0.012 * drift}
        over = {}
        if f['ev_delay'].get().strip():
            over['EKF2_EV_DELAY'] = float(f['ev_delay'].get())
        for item in f['param_overrides'].get().split(','):
            if '=' in item:
                k, _, val = item.partition('=')
                try:
                    over[k.strip()] = float(val) if '.' in val else int(val)
                except ValueError:
                    pass
        sc['param_overrides'] = over
        for key, name in (('track_speed', 'track_speed_mps'), ('descent_speed', 'descent_mps')):
            text = f[key].get().strip()
            if text:
                sc[name] = float(text)
                if not (math.isfinite(sc[name]) and sc[name] > 0.0):
                    raise ValueError(f'the {key.replace("_", " ")} must be a positive number of m/s')
        cap = f['speed_cap'].get().strip()
        if cap:
            sc['speed_cap_mps'] = float(cap)
            if not (math.isfinite(sc['speed_cap_mps']) and sc['speed_cap_mps'] > 0.0):
                raise ValueError('the speed cap must be a positive number of m/s')
        return sc

    # ------------------------------------------------------------------------------------------
    # run control
    # ------------------------------------------------------------------------------------------
    def start(self):
        if self.proc is not None:
            self._send(('quit',))           # the last run, kept after Stop for Save
            self._finish_proc()
        if self.map is None:
            messagebox.showinfo('SITL', 'The course is still loading (first start builds the STEP cache, ~75 s).')
            return
        try:
            sc = self._scenario()
        except ValueError as exc:
            messagebox.showerror('SITL', f'Scenario field error: {exc}')
            return
        from dronesim.gui.worker import worker_main
        parent, child = mp.Pipe()
        ctx = mp.get_context('spawn')
        self.proc = ctx.Process(target=worker_main, args=(child, sc, self._speed()), daemon=True)
        self.proc.start()
        self.conn = parent
        self.result = None
        self.info = None
        self.map.reset()
        self.chart_z.clear()
        self.chart_v.clear()
        self.log_text.delete('1.0', 'end')
        self.events_list.delete(0, 'end')
        self.px4_list.delete(0, 'end')
        self.info_text.delete('1.0', 'end')
        self._rtf_ref = None
        self.start_btn.configure(state='disabled')
        self.pause_btn.configure(state='normal', text='Pause')
        self.stop_btn.configure(state='normal')
        self.save_btn.configure(state='disabled')
        self.arm_var.set(False)
        self.kill_var.set(False)
        self.link_var.set(True)
        self.mode_var.set('altitude')
        self.thr_scale.set(-1.0)
        self.status_line.configure(text=f"Preparing {sc['version']}: archiving the source, building the workspace...")
        self._paused = False

    def toggle_pause(self):
        self._paused = not self._paused
        self._send(('pause',) if self._paused else ('resume',))
        self.pause_btn.configure(text='Resume' if self._paused else 'Pause')

    def stop(self):
        """Stop flying. The worker keeps the run, so it can still be saved; Start or closing
        the window ends it."""
        self._send(('stop',))
        self.pause_btn.configure(state='disabled')
        self.stop_btn.configure(state='disabled')
        self.start_btn.configure(state='normal')

    def save(self):
        d = filedialog.askdirectory(title='Save the run into a folder', initialdir=os.path.join(SITL_ROOT, 'runs'))
        if d:
            self._send(('save', d))

    def _finish_proc(self):
        if self.proc is not None:
            self.proc.join(timeout=3)
            if self.proc.is_alive():
                self.proc.terminate()
        self.proc = None
        self.conn = None
        self.start_btn.configure(state='normal')
        self.pause_btn.configure(state='disabled')
        self.stop_btn.configure(state='disabled')
        self.save_btn.configure(state='disabled')     # the run went with the worker

    def _send(self, msg):
        if self.conn is not None:
            try:
                self.conn.send(msg)
            except (BrokenPipeError, OSError):
                pass

    def _pilot(self, what, value=None):
        self._cmd({'action': 'pilot', 'what': what, 'value': value})

    def _cmd(self, cmd):
        self._send(('command', cmd))

    def _on_topic(self, _e):
        sel = self.topic_tree.selection()
        if sel:
            self._send(('echo', sel[0]))

    def _on_close(self):
        self._send(('quit',))
        self._finish_proc()
        self.root.destroy()

    # ------------------------------------------------------------------------------------------
    # polling + drawing
    # ------------------------------------------------------------------------------------------
    def _poll(self):
        if self._course_ready is not None and self.map is None:
            data, self._course_ready = self._course_ready, None
            if isinstance(data, Exception):
                self.status_line.configure(text=f'Could not load the course: {data}')
            else:
                self._install_map(data)
        snap = None
        try:
            while self.conn is not None and self.conn.poll():
                msg = self.conn.recv()
                kind = msg[0]
                if kind == 'tel':
                    snap = msg[1]
                    self._append_logs(msg[2])
                elif kind == 'ready':
                    self.info = msg[1]
                    self._show_info()
                    self.status_line.configure(text=f"Running {self.info['label']}")
                elif kind == 'topics':
                    self._show_topics(msg[1])
                elif kind == 'echo':
                    self.echo_text.delete('1.0', 'end')
                    self.echo_text.insert('1.0', f'{msg[1]}\n\n{msg[2]}')
                elif kind == 'done':
                    self.result = msg[1]
                    self._show_result()
                elif kind == 'saved':
                    self.status_line.configure(text=f'Saved to {msg[1]}')
                elif kind == 'error':
                    self.status_line.configure(text='The simulation stopped with an error (see Run info).')
                    self.info_text.insert('end', '\n\nERROR\n' + msg[1])
                    self.nb.select(4)
                    self._finish_proc()
                    break
        except (EOFError, OSError):
            self._finish_proc()
        if snap is not None:
            self._update_status(snap)
        self.root.after(60, self._poll)

    def _append_logs(self, records):
        if not records:
            return
        from dronesim.sim import format_log
        flt = self.log_filter.get().strip().lower()
        only_warn = self.hide_info.get()
        hide_clamp = self.hide_clamp.get()
        for r in records:
            level = r.get('level', 'INFO')
            if only_warn and level in ('INFO', 'DEBUG'):
                continue
            if level == 'DEBUG':
                continue
            line = format_log(r)
            if hide_clamp and ('setpoint clamped' in line or 'CLAMPING' in line):
                continue
            if flt and flt not in line.lower():
                continue
            tag = 'px4' if r.get('source') == 'px4' else LEVEL_TAGS.get(level, '')
            self.log_text.insert('end', line + '\n', tag)
            if r.get('source') == 'px4':
                self.px4_list.insert('end', f"{r['t']:8.2f}  {level:8s} {r['msg']}")
            self.log_lines += 1
        if self.log_lines > 8000:
            self.log_text.delete('1.0', '2000.0')
            self.log_lines -= 2000
        self.log_text.see('end')

    def _update_status(self, s):
        px4 = s['px4']
        nav = px4['nav_name']
        self.nav_label.configure(text=nav + ('  (paused)' if s.get('paused') else ''),
                                 bg=NAV_COLORS.get(nav, '#607d8b'))
        v = self.svars
        v['t'].set(f"{s['t']:.2f} s")
        now = time.perf_counter()
        if self._rtf_ref is None:
            self._rtf_ref = (now, s['t'])
        elif now - self._rtf_ref[0] > 1.0:
            rtf = (s['t'] - self._rtf_ref[1]) / (now - self._rtf_ref[0])
            v['rtf'].set(f'{rtf:.2f}x')
            self._rtf_ref = (now, s['t'])
        v['armed'].set(f"{'ARMED' if px4['armed'] else 'disarmed'} / {'landed' if px4['landed'] else 'in air'}"
                       + ('  KILL' if px4['kill'] else '') + ('  RC LOST' if px4['rc_lost'] else ''))
        v['intended'].set(px4['intended'])
        v['failsafe'].set(('YES: ' + ', '.join(px4['failsafe_reasons'])) if px4['failsafe'] else 'no')
        v['takeoff'].set(['disarmed', 'spoolup', 'ready', 'rampup', 'flight'][px4['takeoff_state']] +
                         f"   thrust {px4['thrust']:.2f}  hover {px4['hover_thrust']:.2f}")
        e = px4['ekf']
        v['ekf'].set(f"{'valid' if e['xy_valid'] else 'INVALID'}  eph {e['eph'] or 0:.2f} m"
                     + ('  dead-reckoning' if e['dead_reckoning'] else ''))
        counts = e.get('ev_counts') or {}
        v['ev'].set(f"{'active' if e['ev'] else 'off'}  last: {e['ev_status']}  "
                    + "  ".join(f"{k} {n}" for k, n in sorted(counts.items())))
        db = e['dist_bottom']
        v['height'].set(f"{'range' if e['rng_ref'] else 'baro'}  {db if db is None else round(db, 2)} m")
        vo = s['vo']
        v['vo'].set(f"{['unknown', 'TRACKING', 'LOST'][vo['state']]}  features {vo['score']:.2f}  losses {vo['lost_events']}")
        sp = px4['setpoint']
        v['sp'].set((f"z {-sp['pos'][2]:.2f}  age {sp['age']:.1f} s" if sp and sp['pos'][2] == sp['pos'][2] else '-')
                    + (f"   ocm {px4['offboard_age']:.1f} s" if px4['offboard_age'] is not None else ''))
        tr = s['true']
        v['pos'].set(f"E {tr['x']:+.2f} N {tr['y']:+.2f} up {tr['z']:.2f}  {tr['speed']:.2f} m/s")
        v['bat'].set(f"{s['battery_v']:.2f} V  {100 * s['battery_soc']:.0f} %")
        lid = s.get('lidar')
        v['lidar'].set(f"{'open' if lid['open'] else 'closed'}  {lid['frames']} frames" if lid else 'not fitted')
        # referee
        ref = s['referee']
        self.ref_tree.delete(*self.ref_tree.get_children())
        for row in ref['checkpoints']:
            self.ref_tree.insert('', 'end', text=row['name'], values=(
                f"{row['earned']:g}/{row['points']}", row['centre_crossed_s'] if row['centre_crossed_s'] is not None else '',
                row['whole_drone_s'] if row['whole_drone_s'] is not None else '', row['route'] or ''))
        txt = f"points {ref['checkpoint_points']:g}   touches {ref['touches']} (+{ref['touch_penalty_s']:g} s)"
        if ref['time_s'] is not None:
            txt += f"   time {ref['time_s']} s"
        if ref.get('chain_broken'):
            txt += f"\nskipped: {ref['chain_broken']['at']}"
        if ref['ended']:
            txt += f"\nENDED at {ref['ended']['t']} s: {ref['ended']['reason']}"
        if ref['violations']:
            txt += '\nVIOLATIONS: ' + '; '.join(x['text'] for x in ref['violations'])
        self.ref_summary.configure(text=txt)
        if self.events_list.size() != len(s['events']) or True:
            self.events_list.delete(0, 'end')
            for t, e_ in s['events']:
                self.events_list.insert('end', f'{t:8.2f}  {e_}')
        procs = [p['name'] for p in s['processes'] if p['alive']]
        self.kill_node_box.configure(values=procs)
        if self.map is not None:
            self.map.update(s)
        # charts
        est = s.get('est') or {}
        rest = 0.07
        vo_z = (vo['odom'][3] + rest) if vo.get('odom') else None
        self.chart_z.add(s['t'], {'true': tr['z'], 'ekf': -e['z'] if e['z'] is not None else None, 'vo': vo_z,
                                  'sp': est.get('setpoint_z', None) and est['setpoint_z'] + 0.0})
        err = math.dist(est['ekf'], (tr['x'], tr['y'])) if est.get('ekf') else None
        voerr = math.dist(est['vo'], (tr['x'], tr['y'])) if est.get('vo') else None
        self.chart_v.add(s['t'], {'speed': tr['speed'], 'err': err, 'voerr': voerr})
        self.chart_z.draw(s['t'])
        self.chart_v.draw(s['t'])
        if s.get('done') and self.result is None:
            self.status_line.configure(text='Run finished.')

    def _show_topics(self, topics):
        existing = set(self.topic_tree.get_children())
        for t in topics:
            vals = (t['type'], t['pubs'], t['subs'], t['hz'])
            if t['topic'] in existing:
                self.topic_tree.item(t['topic'], values=vals)
            else:
                self.topic_tree.insert('', 'end', iid=t['topic'], text=t['topic'], values=vals)

    def _show_info(self):
        i = self.info
        lines = [f"VERSION   {i['label']}", f"SOURCE    {i['provenance']}", f"PACKAGES  {', '.join(i['team_packages'])}",
                 '', 'SELECTION', *('  ' + n for n in i['notes']), '',
                 'INSTALLED CONFIG (what an operator / install script writes on the drone)']
        for c in i['install_config']:
            lines.append(f"  {c['file']} [{c['node']}]: {json.dumps(c['values'])}")
        lines += ['', 'MEASURED VALUES used (truth + scenario calibration error)', '  ' + json.dumps(i['measured']),
                  '', f"PX4 PROFILE {i['params_profile']}"]
        for k, (val, src) in i['param_sources'].items():
            lines.append(f'  {k:18s} {val!s:>10}   <- {src}')
        self.info_text.delete('1.0', 'end')
        self.info_text.insert('1.0', '\n'.join(lines))

    def _show_result(self):
        r = self.result
        ref = r['referee']
        lines = ['', '=' * 70, 'RESULT', f"  sim time {r['sim_time_s']} s", f"  checkpoint points {ref['checkpoint_points']}",
                 f"  finished {ref['finished']}   time {ref['time_s']}   penalised {ref['penalised_time_s']}",
                 f"  touches {ref['touches']}   net contacts {ref['net_contacts']}", f"  ended {ref['ended']}",
                 f"  violations {ref['violations']}"]
        vis = r.get('vision') or {}
        if vis:
            lines.append(f"  cuVSLAM error vs truth: median {vis.get('cuvslam_error_median_m')} m, max "
                         f"{vis.get('cuvslam_error_max_m')} m; tracking losses {vis.get('cuvslam_losses')}; "
                         f"PX4 vision samples {vis.get('fusion')}")
        ret = ref.get('return') or {}
        if ret.get('state'):
            lines.append(f"  return {ret['state']}: {ret.get('reason') or ''}  (run points x {ret['run_points_multiplier']})")
        lines += ['', '  ' + ref['note'], '', 'EVENTS']
        lines += [f'  {t:8.2f}  {e}' for t, e in r['events']]
        lines += ['', 'PROCESSES'] + [f"  {p['name']:24s} alive={p['alive']} exit={p['exit_code']}" for p in r['processes']]
        self.info_text.insert('end', '\n'.join(lines))
        self.save_btn.configure(state='normal')
        self.status_line.configure(text='Run finished: see Run info / result. Save it, or Stop to set up another run.')


def main():
    mp.freeze_support()
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == '__main__':
    main()
