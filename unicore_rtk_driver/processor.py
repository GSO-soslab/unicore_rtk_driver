"""Convert NMEA and Unicore sentences from the receiver into ROS messages.

The node calls `handle_nmea()` / `handle_unicore()` with each line and its
arrival stamp; GnssProcessor validates the line, parses it and passes the
resulting messages to the `publish(name, msg)` callback.  The tests call the
same methods with recorded lines.

Output names (the node maps them to topics):
    fix, gpsfix, time_ref, global_velocity, heading_odometry, heading_enu_pose,
    heading_enu_pose_imu, heading_enu_pose_imu_manual, simple_heading,
    unicore_velocity

Frame conventions are described in the README.
"""
import math
from dataclasses import dataclass

import tf_transformations
from builtin_interfaces.msg import Time
from gps_msgs.msg import GPSFix, GPSStatus
from geometry_msgs.msg import (PoseWithCovarianceStamped, Quaternion,
                               TwistStamped, TwistWithCovarianceStamped)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from std_msgs.msg import Float32

from unicore_rtk_driver.protocol import nmea_parser, unicore_parser
from unicore_rtk_driver.protocol.checksums import nmea_is_valid, unicore_is_valid

FIX_COVARIANCE_MODES = ('hdop_scaled', 'gst')

# GGA quality indicator -> gps_msgs/GPSStatus status (gps_msgs 3.1, ROS 2 Jazzy).
# Dead reckoning (6) and simulation (8) are not GNSS solutions and map to
# STATUS_NO_FIX; protocol.solution.GGA_SOLUTION_NAMES keeps the exact type for diagnostics.
GGA_TO_GPS_STATUS = {
    0: GPSStatus.STATUS_NO_FIX,
    1: GPSStatus.STATUS_FIX,
    2: GPSStatus.STATUS_DGPS_FIX,
    3: GPSStatus.STATUS_FIX,          # PPS fix
    4: GPSStatus.STATUS_RTK_FIX,
    5: GPSStatus.STATUS_RTK_FLOAT,
    6: GPSStatus.STATUS_NO_FIX,       # dead reckoning
    7: GPSStatus.STATUS_FIX,          # manually entered position, e.g. a fixed base
    8: GPSStatus.STATUS_NO_FIX,       # simulation
    9: GPSStatus.STATUS_WAAS_FIX,
}


@dataclass
class Config:
    gps_frame_id: str = 'unicore_gps'
    heading_frame_id: str = 'world_ned'
    heading_enu_frame_id: str = 'world_enu'
    # Frame for global_velocity and unicore_velocity.  Their data is
    # East/North/Up, so this should be an ENU world frame (<robot>/world).
    velocity_frame_id: str = 'world'
    use_gnss_time: bool = False
    # degrees
    heading_correction_angle_gps_to_imu: float = -3.0
    epe_quality0: float = 1000000.0
    epe_quality1: float = 4.0
    epe_quality2: float = 0.1
    epe_quality4: float = 0.02
    epe_quality5: float = 4.0
    epe_quality9: float = 3.0
    # ---- optional behaviour ----
    # 'hdop_scaled': var = (hdop * gst_std)^2, alt uses 2*hdop.
    # 'gst': var = gst_std^2 (GST std devs are already 1-sigma metres).
    fix_covariance_mode: str = 'hdop_scaled'
    # False: reject UNIHEADINGA only when there is no solution AND the status
    #        is not SOL_COMPUTED.
    # True:  reject it when either is the case, and only publish a fix when a
    #        valid GGA arrived since the last published fix.
    strict_validation: bool = False
    # True: stamp heading outputs with the GPS time in the UNIHEADINGA header.
    # False: arrival time, independent of use_gnss_time.
    heading_use_gnss_time: bool = False


def _time_to_float(t: Time) -> float:
    return t.sec + t.nanosec * 1e-9


def _quat(q) -> Quaternion:
    return Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))


def rpy_to_quaternion(r, p, y) -> Quaternion:
    return _quat(tf_transformations.quaternion_from_euler(r, p, y))


class GnssProcessor:

    def __init__(self, cfg: Config, publish, logger, lookup_rotation=None):
        """
        Args:
            cfg: Config
            publish: callable(name: str, msg) -> None
            logger: rclpy-style logger (debug/info/warn/error)
            lookup_rotation: callable(target_frame, source_frame) -> [x,y,z,w]
                rotation of source in target (static TF).  Raises on failure.
                Only needed for the IMU-corrected heading.
        """
        if cfg.fix_covariance_mode not in FIX_COVARIANCE_MODES:
            raise ValueError(f'fix_covariance_mode must be one of {FIX_COVARIANCE_MODES}')
        self.cfg = cfg
        self.publish = publish
        self.log = logger
        self.lookup_rotation = lookup_rotation

        self.imu_frame = None
        self.imu_rotation = None

        self.hdop = float('nan')
        self.gga_quality = None
        self.num_satellites = 0
        self.valid_fix = False
        self._gga_fresh = False
        self.last_gga = None  # raw text, used e.g. by NTRIP casters (VRS)

        # fix type from GGA -> (default EPE, NavSatStatus, covariance type)
        self.gps_qualities = {
            -1: [cfg.epe_quality0, NavSatStatus.STATUS_NO_FIX, NavSatFix.COVARIANCE_TYPE_UNKNOWN],
            0: [cfg.epe_quality0, NavSatStatus.STATUS_NO_FIX, NavSatFix.COVARIANCE_TYPE_UNKNOWN],
            1: [cfg.epe_quality1, NavSatStatus.STATUS_FIX, NavSatFix.COVARIANCE_TYPE_APPROXIMATED],
            2: [cfg.epe_quality2, NavSatStatus.STATUS_SBAS_FIX, NavSatFix.COVARIANCE_TYPE_APPROXIMATED],
            4: [cfg.epe_quality4, NavSatStatus.STATUS_GBAS_FIX, NavSatFix.COVARIANCE_TYPE_APPROXIMATED],
            5: [cfg.epe_quality5, NavSatStatus.STATUS_GBAS_FIX, NavSatFix.COVARIANCE_TYPE_APPROXIMATED],
            9: [cfg.epe_quality9, NavSatStatus.STATUS_GBAS_FIX, NavSatFix.COVARIANCE_TYPE_APPROXIMATED],
        }

        self.fix_msg = NavSatFix()

    # ------------------------------------------------------------------ IMU
    def set_imu(self, frame_id: str, orientation: Quaternion):
        # one tuple assignment: safe to call from another thread
        self.imu_frame, self.imu_rotation = frame_id, orientation

    # ----------------------------------------------------------------- NMEA
    def handle_nmea(self, line: str, stamp: Time) -> bool:
        if not nmea_is_valid(line):
            self.log.warn(
                f"Unicore Driver - Received a sentence with an invalid checksum. "
                f"nmea string was: {line!r}")
            return False

        parsed = nmea_parser.parse_nmea_sentence(line, now=_time_to_float(stamp))
        if not parsed:
            self.log.debug(f"Unicore Driver - Failed to parse NMEA sentence. Sentence was: {line}")
            return False
        self.log.debug(f"Unicore Driver - NMEA: {line}")
        self.log.debug(str(parsed))

        if 'GGA' in parsed:
            return self._gga(parsed['GGA'], stamp, line)
        if 'RMC' in parsed:
            return self._rmc(parsed['RMC'], stamp)
        if 'GST' in parsed:
            return self._gst(parsed['GST'])
        if 'HPR' in parsed:
            data = parsed['HPR']
            if math.isnan(data['utc_time'][0]):
                self.log.warn("Unicore Driver - Time in the NMEA HPR sentence is NOT valid")
                return False
            if data['fix_type'] == 0:
                self.log.warn("Unicore Driver - NMEA HPR sentence is NOT valid")
                return False
        return True

    def _gga(self, data, stamp, line):
        if math.isnan(data['utc_time'][0]):
            self.log.warn("Unicore Driver - Time in the NMEA GGA sentence is NOT valid")
            return False

        fix = self.fix_msg
        if self.cfg.use_gnss_time:
            fix.header.stamp = Time(sec=data['utc_time'][0], nanosec=data['utc_time'][1])
        else:
            fix.header.stamp = stamp
        fix.header.frame_id = self.cfg.gps_frame_id

        fix_type = data['fix_type']
        self.gga_quality = fix_type
        self.num_satellites = data['num_satellites']
        if fix_type not in self.gps_qualities:
            fix_type = -1
        gps_qual = self.gps_qualities[fix_type]
        self.valid_fix = fix_type > 0

        latitude = data['latitude']
        if data['latitude_direction'] == 'S':
            latitude = -latitude
        longitude = data['longitude']
        if data['longitude_direction'] == 'W':
            longitude = -longitude
        # GGA field 9 is height above mean sea level and field 11 the geoid
        # separation, so their sum is the height above the WGS84 ellipsoid,
        # which is what NavSatFix.altitude is defined as.
        altitude = data['altitude'] + data['mean_sea_level']
        self.hdop = data['hdop']

        fix.status.status = gps_qual[1]
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.position_covariance_type = gps_qual[2]
        fix.latitude = latitude
        fix.longitude = longitude
        fix.altitude = altitude

        self._gga_fresh = True
        self.last_gga = line
        return True

    def _rmc(self, data, stamp):
        if math.isnan(data['utc_time'][0]):
            self.log.warn("Time in the NMEA RMC sentence is NOT valid")
            return False

        if abs(data['utc_time'][0] - data['utc_time_pc'][0]) > 2.0:
            self.log.warn(
                f"Unicore Driver - System may not time sync, UTC and System time are different: "
                f"UTC:{data['utc_time'][0]}, System:{data['utc_time_pc'][0]}")

        gnss_stamp = Time(sec=data['utc_time'][0], nanosec=data['utc_time'][1])

        time_ref = TimeReference()
        time_ref.header.stamp = gnss_stamp if self.cfg.use_gnss_time else stamp
        time_ref.header.frame_id = self.cfg.gps_frame_id
        time_ref.source = self.cfg.gps_frame_id
        time_ref.time_ref = time_ref.header.stamp
        self.publish('time_ref', time_ref)

        if self.valid_fix:
            vel = TwistStamped()
            vel.header.stamp = gnss_stamp if self.cfg.use_gnss_time else stamp
            vel.header.frame_id = self.cfg.velocity_frame_id
            # East / North from speed over ground and course (from true north)
            vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
            vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
            self.publish('global_velocity', vel)
        return True

    def _gst(self, data):
        fix = self.fix_msg
        if self.cfg.strict_validation and not (self._gga_fresh and not math.isnan(self.hdop)):
            self.log.debug("Unicore Driver - GST without a fresh valid GGA, fix not published")
            return False

        lon_std, lat_std, alt_std = data['lon_std_dev'], data['lat_std_dev'], data['alt_std_dev']
        if self.cfg.fix_covariance_mode == 'gst':
            fix.position_covariance[0] = lon_std ** 2
            fix.position_covariance[4] = lat_std ** 2
            fix.position_covariance[8] = alt_std ** 2
        else:  # 'hdop_scaled'
            fix.position_covariance[0] = (self.hdop * lon_std) ** 2
            fix.position_covariance[4] = (self.hdop * lat_std) ** 2
            fix.position_covariance[8] = (2 * self.hdop * alt_std) ** 2

        self.publish('fix', fix)
        self.publish('gpsfix', self._gpsfix(fix))
        self._gga_fresh = False
        return True

    def _gpsfix(self, fix: NavSatFix) -> GPSFix:
        """GPSFix with the same position and covariance as `fix`, and the
        receiver's solution type (RTK fixed / float) in status.status.
        Fields the driver has no source for are NaN."""
        msg = GPSFix()
        for name in ('track', 'speed', 'climb', 'pitch', 'roll', 'dip', 'time', 'gdop',
                     'pdop', 'vdop', 'tdop', 'err', 'err_horz', 'err_vert', 'err_track',
                     'err_speed', 'err_climb', 'err_time', 'err_pitch', 'err_roll', 'err_dip'):
            setattr(msg, name, float('nan'))
        msg.header = fix.header
        msg.status.header = fix.header
        msg.status.status = GGA_TO_GPS_STATUS.get(self.gga_quality, GPSStatus.STATUS_NO_FIX)
        msg.status.satellites_used = self.num_satellites
        msg.status.position_source = GPSStatus.SOURCE_GPS
        msg.latitude = fix.latitude
        msg.longitude = fix.longitude
        msg.altitude = fix.altitude
        msg.hdop = self.hdop
        msg.position_covariance = fix.position_covariance
        msg.position_covariance_type = fix.position_covariance_type
        return msg

    # -------------------------------------------------------------- Unicore
    def handle_unicore(self, line: str, stamp: Time) -> bool:
        if not unicore_is_valid(line):
            self.log.warn(
                f"Unicore Driver - Received a sentence with an invalid checksum. "
                f"unicore string was: {line!r}")
            return False

        parsed = unicore_parser.parse_unicore_sentence(line)
        if not parsed:
            self.log.debug(f"Unicore Driver - Failed to parse UNICORE sentence. Sentence was: {line}")
            return False
        self.log.debug(f"Unicore Driver - UNICORE: {line}")
        self.log.debug(str(parsed))

        if 'UNIHEADINGA' in parsed:
            return self._uniheading(parsed['HEAD'], parsed['UNIHEADINGA'], stamp)
        if 'AGRICA' in parsed:
            return self._agrica(parsed['AGRICA'], stamp)
        return True

    def _heading_stamp(self, head, stamp):
        if self.cfg.heading_use_gnss_time and head.get('Time_Ref') == 'GPS':
            t = unicore_parser.gps_week_ms_to_unix(
                head['Week_Num'], head['Week_MS'], head['Leap_Seccond'])
            if t is not None:
                return Time(sec=t[0], nanosec=t[1])
        return stamp

    def _uniheading(self, head, data, stamp):
        no_solution = data['Pose_Type'] == 0
        bad_status = data['Sol_Stat'] != 0
        if self.cfg.strict_validation:
            invalid = no_solution or data['Pose_Type'] is None or bad_status
        else:
            invalid = no_solution and bad_status
        if invalid:
            self.log.warn("Unicore Driver - UNICORE UNIHEADINGA sentence is not vaild")
            return False

        stamp = self._heading_stamp(head, stamp)
        heading = data['Heading']            # rad, clockwise from true north
        heading_var = data['Heading_Std'] ** 2

        # --- heading as Odometry in NED (yaw about down = compass heading)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.cfg.heading_frame_id
        odom.child_frame_id = self.cfg.gps_frame_id
        odom.pose.pose.orientation = rpy_to_quaternion(0, 0, heading)
        odom.pose.covariance[35] = heading_var
        self.publish('heading_odometry', odom)

        # --- heading in ENU (counter-clockwise from east)
        heading_enu = -(heading - math.pi / 2)
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.cfg.heading_enu_frame_id
        pose.pose.pose.orientation = rpy_to_quaternion(0, 0, heading_enu)
        pose.pose.covariance[35] = heading_var
        self.publish('heading_enu_pose', pose)

        # --- heading expressed for the IMU (consumed as MicroStrain ext heading)
        imu_frame, imu_rotation = self.imu_frame, self.imu_rotation
        if imu_frame is not None and imu_rotation is not None:
            try:
                self._heading_for_imu(imu_frame, imu_rotation, heading_enu, heading_var, stamp)
            except Exception as e:  # TF not available yet, etc.
                self.log.warn(f"IMU-corrected heading TF failed: {e}")

        simple = Float32()
        simple.data = heading
        self.publish('simple_heading', simple)
        return True

    def _heading_for_imu(self, imu_frame, imu_rotation, heading_enu, heading_var, stamp):
        # (a) fixed mounting offset from a parameter (degrees)
        manual = PoseWithCovarianceStamped()
        manual.header.stamp = stamp
        manual.header.frame_id = imu_frame
        corrected = heading_enu + math.radians(self.cfg.heading_correction_angle_gps_to_imu)
        manual.pose.pose.orientation = rpy_to_quaternion(0, 0, corrected)
        manual.pose.covariance[35] = heading_var
        self.publish('heading_enu_pose_imu_manual', manual)

        # (b) from TF: keep the IMU's roll/pitch, replace the GPS-frame yaw
        # with the GNSS heading, then rotate back into the IMU frame.
        if self.lookup_rotation is None:
            return
        q_imu = [imu_rotation.x, imu_rotation.y, imu_rotation.z, imu_rotation.w]
        R_IMU_WORLD = tf_transformations.quaternion_matrix(q_imu)
        R_GPS_IMU = tf_transformations.quaternion_matrix(
            self.lookup_rotation(imu_frame, self.cfg.gps_frame_id))
        R_GPS_WORLD = R_IMU_WORLD @ R_GPS_IMU
        roll, pitch, _ = tf_transformations.euler_from_matrix(R_GPS_WORLD)
        R_GPS_WORLD_CORRECTED = tf_transformations.quaternion_matrix(
            tf_transformations.quaternion_from_euler(roll, pitch, heading_enu))
        R_IMU_WORLD = R_GPS_WORLD_CORRECTED @ R_GPS_IMU.T

        final = PoseWithCovarianceStamped()
        final.header.stamp = stamp
        final.header.frame_id = imu_frame
        final.pose.pose.orientation = _quat(tf_transformations.quaternion_from_matrix(R_IMU_WORLD))
        final.pose.covariance[35] = heading_var
        self.publish('heading_enu_pose_imu', final)

    def _agrica(self, data, stamp):
        if data['Pose_Type'] == 0 or not data['Speed_Valid']:
            self.log.warn("Unicore Driver - UNICORE AGRICA sentence is not vaild")
            return False

        twist = TwistWithCovarianceStamped()
        if self.cfg.use_gnss_time:
            unix_secs = unicore_parser.convert_time_argica(
                data['Year'], data['Month'], data['Day'],
                data['Hour'], data['Minute'], data['Second'])
            twist.header.stamp = Time(sec=unix_secs, nanosec=0)
        else:
            twist.header.stamp = stamp
        twist.header.frame_id = self.cfg.velocity_frame_id

        # AGRICA velocities are geographic East / North / Up (m/s)
        twist.twist.twist.linear.x = data['Vel_East']
        twist.twist.twist.linear.y = data['Vel_North']
        twist.twist.twist.linear.z = data['Vel_Up']
        twist.twist.covariance[0] = data['Vel_Std_East'] ** 2
        twist.twist.covariance[7] = data['Vel_Std_North'] ** 2
        twist.twist.covariance[14] = data['Vel_Std_Up'] ** 2
        self.publish('unicore_velocity', twist)
        return True
