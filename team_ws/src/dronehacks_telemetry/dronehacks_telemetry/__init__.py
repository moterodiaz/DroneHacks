"""Flight telemetry: a per-flight recording folder on the drone, and the report read from it.

`bag` reads a rosbag2 mcap folder into time series, `metrics` measures them, `course` places
them on the course map, `ulog_triage` reads PX4's own log, and `analyze` is the command line.
"""
