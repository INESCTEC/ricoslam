#!/usr/bin/env python3
"""
clf2bag.py - Convert CARMEN log files (Radish / Freiburg SLAM datasets) into a
ROS 1 (.bag) or ROS 2 bag, with robust, monotonic timestamps.

Why this exists
---------------
The classic CARMEN logs (Intel, ACES, MIT CSAIL/Killian, Freiburg 079/101, ...)
carry two on-disk timestamps per line (`ipc_timestamp`, `logger_timestamp`).
They are asynchronous and, in several distributed logs, non-monotonic. Feeding
them straight into a bag is what produces the classic "time jumps / irregular
frequency" and TF-extrapolation failures.

Robust-fix design (what this script does):
  * Ignore the CARMEN clock entirely. Assign every emitted scan a synthetic,
    strictly monotonic stamp  t = t0 + i / rate.
  * Publish odom->base_link TF from the pose embedded in each laser line, with
    the SAME stamp as the scan -> no interpolation/extrapolation errors.
  * Publish a static base_link->laser TF from the laser offset (ROBOTLASER1),
    or identity (old FLASER, override with --laser-x/y/theta).
  * Take scan geometry from the message header (ROBOTLASER1). For the old
    FLASER format, infer it from the reading count (override with --angle-*).

Supported messages: FLASER, RLASER (old, pose embedded) and
ROBOTLASER1 (new, pose + geometry embedded). RAWLASER1 (no pose) is skipped
with a warning, since it needs an ODOM-sync path that is out of scope here.

Requires:  pip install rosbags        (pure Python, no ROS install needed)
Tested with rosbags 0.11.x.
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import sys
from pathlib import Path

import numpy as np

from rosbags.typesys import Stores, get_typestore, get_types_from_msg


# --------------------------------------------------------------------------- #
# Small helpers
# --------------------------------------------------------------------------- #
def yaw_to_quat(yaw: float):
    """Return (x, y, z, w) for a rotation about +Z."""
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# --------------------------------------------------------------------------- #
# Parsed record
# --------------------------------------------------------------------------- #
@dataclasses.dataclass
class Scan:
    ranges: np.ndarray          # float32
    intensities: np.ndarray     # float32 (may be empty)
    angle_min: float
    angle_max: float
    angle_increment: float
    range_max: float
    # odom (robot) pose used for odom->base_link
    ox: float
    oy: float
    otheta: float
    # laser offset in the *base_link* frame (for tf_static); None -> identity
    laser_dx: float | None = None
    laser_dy: float | None = None
    laser_dtheta: float | None = None
    # optional body velocities (ROBOTLASER1)
    tv: float = 0.0
    rv: float = 0.0


def parse_robotlaser1(tok: list[str]) -> Scan:
    """tok = tokens AFTER the 'ROBOTLASER1' keyword."""
    start_angle = float(tok[1])
    ang_res = float(tok[3])
    max_range = float(tok[4])
    n = int(tok[7])
    base = 8
    ranges = np.asarray(tok[base:base + n], dtype=np.float32)
    idx = base + n
    m = int(tok[idx]); idx += 1
    remissions = np.asarray(tok[idx:idx + m], dtype=np.float32); idx += m
    lx, ly, lth = float(tok[idx]), float(tok[idx + 1]), float(tok[idx + 2])
    rx, ry, rth = float(tok[idx + 3]), float(tok[idx + 4]), float(tok[idx + 5])
    tv, rv = float(tok[idx + 6]), float(tok[idx + 7])

    # laser offset expressed in the base_link frame
    c, s = math.cos(-rth), math.sin(-rth)
    dx = c * (lx - rx) - s * (ly - ry)
    dy = s * (lx - rx) + c * (ly - ry)
    dth = wrap_angle(lth - rth)

    angle_min = start_angle
    angle_max = start_angle + (n - 1) * ang_res
    inten = remissions if remissions.size == ranges.size else np.empty(0, np.float32)
    return Scan(ranges, inten, angle_min, angle_max, ang_res, max_range,
                rx, ry, rth, dx, dy, dth, tv, rv)


def parse_flaser(tok: list[str], fov: float, use_corrected: bool,
                 lx: float, ly: float, lth: float) -> Scan:
    """tok = tokens AFTER the 'FLASER'/'RLASER' keyword."""
    n = int(tok[0])
    ranges = np.asarray(tok[1:1 + n], dtype=np.float32)
    idx = 1 + n
    ex, ey, eth = float(tok[idx]), float(tok[idx + 1]), float(tok[idx + 2])       # estimate
    odx, ody, odth = float(tok[idx + 3]), float(tok[idx + 4]), float(tok[idx + 5])  # odometry
    ox, oy, oth = (ex, ey, eth) if use_corrected else (odx, ody, odth)

    angle_min = -fov / 2.0
    angle_max = fov / 2.0
    ang_res = fov / (n - 1) if n > 1 else fov
    # no range_max in the old format -> caller supplies a default
    return Scan(ranges, np.empty(0, np.float32), angle_min, angle_max, ang_res,
                float("nan"), ox, oy, oth, lx, ly, lth, 0.0, 0.0)


# --------------------------------------------------------------------------- #
# Message construction (version-aware)
# --------------------------------------------------------------------------- #
class Builder:
    def __init__(self, ts, ros_version: int):
        self.ts = ts
        self.v = ros_version
        self.Header = ts.types['std_msgs/msg/Header']
        self.Time = ts.types['builtin_interfaces/msg/Time']
        self.LaserScan = ts.types['sensor_msgs/msg/LaserScan']
        self.TFMessage = ts.types['tf2_msgs/msg/TFMessage']
        self.TransformStamped = ts.types['geometry_msgs/msg/TransformStamped']
        self.Transform = ts.types['geometry_msgs/msg/Transform']
        self.Vector3 = ts.types['geometry_msgs/msg/Vector3']
        self.Quaternion = ts.types['geometry_msgs/msg/Quaternion']
        self.Odometry = ts.types['nav_msgs/msg/Odometry']
        self.Pose = ts.types['geometry_msgs/msg/Pose']
        self.Point = ts.types['geometry_msgs/msg/Point']
        self.PWC = ts.types['geometry_msgs/msg/PoseWithCovariance']
        self.TWC = ts.types['geometry_msgs/msg/TwistWithCovariance']
        self.Twist = ts.types['geometry_msgs/msg/Twist']
        self._has_seq = any(f.name == 'seq' for f in dataclasses.fields(self.Header))

    def header(self, stamp_ns: int, frame_id: str, seq: int = 0):
        sec, nsec = divmod(stamp_ns, 1_000_000_000)
        kw = dict(stamp=self.Time(sec=int(sec), nanosec=int(nsec)), frame_id=frame_id)
        if self._has_seq:
            kw['seq'] = seq
        return self.Header(**kw)

    def laserscan(self, sc: Scan, stamp_ns: int, frame: str, range_min: float,
                  range_max: float, scan_dt: float, seq: int = 0):
        return self.LaserScan(
            header=self.header(stamp_ns, frame, seq),
            angle_min=sc.angle_min, angle_max=sc.angle_max,
            angle_increment=sc.angle_increment,
            time_increment=0.0, scan_time=scan_dt,
            range_min=range_min, range_max=range_max,
            ranges=sc.ranges, intensities=sc.intensities)

    def _tf(self, stamp_ns, parent, child, x, y, theta):
        qx, qy, qz, qw = yaw_to_quat(theta)
        return self.TransformStamped(
            header=self.header(stamp_ns, parent), child_frame_id=child,
            transform=self.Transform(
                translation=self.Vector3(x=x, y=y, z=0.0),
                rotation=self.Quaternion(x=qx, y=qy, z=qz, w=qw)))

    def tf_odom(self, sc: Scan, stamp_ns, odom_frame, base_frame):
        return self.TFMessage(transforms=[
            self._tf(stamp_ns, odom_frame, base_frame, sc.ox, sc.oy, sc.otheta)])

    def tf_static(self, sc: Scan, stamp_ns, base_frame, laser_frame):
        dx = sc.laser_dx or 0.0
        dy = sc.laser_dy or 0.0
        dth = sc.laser_dtheta or 0.0
        return self.TFMessage(transforms=[
            self._tf(stamp_ns, base_frame, laser_frame, dx, dy, dth)])

    def odom(self, sc: Scan, stamp_ns, odom_frame, base_frame, seq: int = 0):
        qx, qy, qz, qw = yaw_to_quat(sc.otheta)
        cov = np.zeros(36, dtype=np.float64)
        return self.Odometry(
            header=self.header(stamp_ns, odom_frame, seq),
            child_frame_id=base_frame,
            pose=self.PWC(pose=self.Pose(
                position=self.Point(x=sc.ox, y=sc.oy, z=0.0),
                orientation=self.Quaternion(x=qx, y=qy, z=qz, w=qw)),
                covariance=cov.copy()),
            twist=self.TWC(twist=self.Twist(
                linear=self.Vector3(x=sc.tv, y=0.0, z=0.0),
                angular=self.Vector3(x=0.0, y=0.0, z=sc.rv)),
                covariance=cov.copy()))


# --------------------------------------------------------------------------- #
# Main conversion
# --------------------------------------------------------------------------- #
def build_typestore(ros_version: int):
    store = Stores.ROS1_NOETIC if ros_version == 1 else Stores.ROS2_HUMBLE
    ts = get_typestore(store)
    if 'tf2_msgs/msg/TFMessage' not in ts.types:  # missing in the ROS1 store
        ts.register(get_types_from_msg(
            'geometry_msgs/TransformStamped[] transforms', 'tf2_msgs/msg/TFMessage'))
    return ts


def open_writer(path: Path, ros_version: int, compression: str):
    if ros_version == 1:
        from rosbags.rosbag1 import Writer
        w = Writer(path)
        if compression != 'none':
            w.set_compression({'bz2': Writer.CompressionFormat.BZ2,
                               'lz4': Writer.CompressionFormat.LZ4}[compression])
        return w
    from rosbags.rosbag2 import Writer
    # rosbag2 uses file/message zstd, not bz2/lz4 - left uncompressed here
    return Writer(path, version=Writer.VERSION_LATEST)


def serialize(ts, msg, ros_version: int):
    if ros_version == 1:
        return ts.serialize_ros1(msg, msg.__msgtype__)
    return ts.serialize_cdr(msg, msg.__msgtype__)


def convert(args) -> None:
    ros_version = args.ros_version
    ts = build_typestore(ros_version)
    b = Builder(ts, ros_version)

    t0_ns = int(round(args.start_time * 1e9))
    dt_ns = int(round(1e9 / args.rate))
    scan_dt = 1.0 / args.rate
    fov = math.radians(args.flaser_fov_deg)

    writer = open_writer(Path(args.output), ros_version, args.compression)
    writer.open()
    try:
        # /tf_static must be latched (ROS 1) so late subscribers and rosbag
        # play still see the base_link->laser transform. ROS 2 has no latching
        # kwarg here (uses QoS); left as-is since this package targets ROS 1.
        static_kw = {'latching': 1} if ros_version == 1 else {}
        conns = {
            'scan': writer.add_connection(args.scan_topic,
                                          b.LaserScan.__msgtype__, typestore=ts),
            'tf': writer.add_connection('/tf', b.TFMessage.__msgtype__, typestore=ts),
            'tf_static': writer.add_connection('/tf_static',
                                               b.TFMessage.__msgtype__, typestore=ts,
                                               **static_kw),
        }
        if args.odom_topic:
            conns['odom'] = writer.add_connection(args.odom_topic,
                                                  b.Odometry.__msgtype__, typestore=ts)

        i = 0
        seq_scan = 0
        seq_odom = 0
        static_written = False
        n_raw_skipped = 0
        want = {'FLASER', 'ROBOTLASER1'}
        if args.include_rlaser:
            want.add('RLASER')

        with open(args.input, 'r', errors='replace') as fh:
            for line in fh:
                line = line.strip()
                if not line or line[0] == '#':
                    continue
                tok = line.split()
                key = tok[0]
                if key == 'RAWLASER1':
                    n_raw_skipped += 1
                    continue
                if key not in want:
                    continue

                rest = tok[1:]
                try:
                    if key == 'ROBOTLASER1':
                        sc = parse_robotlaser1(rest)
                    else:  # FLASER / RLASER
                        sc = parse_flaser(rest, fov, args.use_corrected_pose,
                                          args.laser_x, args.laser_y,
                                          math.radians(args.laser_theta_deg))
                except (ValueError, IndexError) as exc:
                    print(f"  skip malformed {key} line: {exc}", file=sys.stderr)
                    continue

                # resolve range_max (old format has none)
                rmax = sc.range_max
                if not math.isfinite(rmax):
                    rmax = args.range_max
                if args.max_range_to_inf:
                    sc.ranges = np.where(sc.ranges >= rmax, np.inf, sc.ranges)

                stamp = max(t0_ns + i * dt_ns, 1)  # ROS 1 time 0 is invalid

                if not static_written:
                    writer.write(conns['tf_static'], stamp,
                                 serialize(ts, b.tf_static(sc, stamp, args.base_frame,
                                                           args.laser_frame), ros_version))
                    static_written = True

                writer.write(conns['scan'], stamp,
                             serialize(ts, b.laserscan(sc, stamp, args.laser_frame,
                                                       args.range_min, rmax, scan_dt,
                                                       seq_scan),
                                       ros_version))
                seq_scan += 1
                writer.write(conns['tf'], stamp,
                             serialize(ts, b.tf_odom(sc, stamp, args.odom_frame,
                                                     args.base_frame), ros_version))
                if args.odom_topic:
                    writer.write(conns['odom'], stamp,
                                 serialize(ts, b.odom(sc, stamp, args.odom_frame,
                                                      args.base_frame, seq_odom),
                                           ros_version))
                    seq_odom += 1
                i += 1

        # write tf_static timestamp (bag write-time) - already done above at first stamp
        print(f"Done: {i} scans -> {args.output}")
        if n_raw_skipped:
            print(f"Note: skipped {n_raw_skipped} RAWLASER1 lines (no pose; "
                  f"needs ODOM-sync path, not handled).")
    finally:
        writer.close()


def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('input', help='CARMEN .clf/.log file')
    p.add_argument('output', help='output bag: file.bag (ROS1) or a directory (ROS2)')
    p.add_argument('--ros-version', type=int, choices=(1, 2), default=1,
                   help='1 -> .bag file, 2 -> rosbag2 directory (default: 1)')
    p.add_argument('--compression', choices=('none', 'bz2', 'lz4'), default='bz2',
                   help='ROS 1 bag compression (default: bz2; ignored for ROS 2)')

    p.add_argument('--rate', type=float, default=10.0,
                   help='synthetic scan rate in Hz for monotonic stamps (default: 10)')
    p.add_argument('--start-time', type=float, default=1_000_000_000.0,
                   help='epoch seconds for the first stamp; MUST be > 0, since '
                        'ROS 1 rejects time 0 as invalid (default: 1e9)')

    p.add_argument('--scan-topic', default='/scan')
    p.add_argument('--odom-topic', default='/odom',
                   help="also publish nav_msgs/Odometry ('' to disable)")
    p.add_argument('--odom-frame', default='odom')
    p.add_argument('--base-frame', default='base_link')
    p.add_argument('--laser-frame', default='laser')

    p.add_argument('--range-min', type=float, default=0.0)
    p.add_argument('--range-max', type=float, default=50.0,
                   help='range_max for old FLASER (no header range); default 50 m')
    p.add_argument('--max-range-to-inf', action='store_true',
                   help='set readings >= range_max to +inf (mark as no-return)')

    # old-FLASER geometry (ROBOTLASER1 uses its own header)
    p.add_argument('--flaser-fov-deg', type=float, default=180.0,
                   help='field of view for old FLASER in degrees (default: 180)')
    p.add_argument('--use-corrected-pose', action='store_true',
                   help='FLASER: use the CARMEN estimate (x y theta) instead of raw odometry')
    p.add_argument('--laser-x', type=float, default=0.0,
                   help='FLASER: laser x offset in base_link (m)')
    p.add_argument('--laser-y', type=float, default=0.0)
    p.add_argument('--laser-theta-deg', type=float, default=0.0)

    p.add_argument('--include-rlaser', action='store_true',
                   help='also convert rear RLASER lines (default: front only)')
    return p


if __name__ == '__main__':
    convert(build_argparser().parse_args())
