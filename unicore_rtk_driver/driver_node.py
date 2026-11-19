"""ROS 2 node for Unicore UM98x (and other NMEA/RTCM) GNSS receivers.

Data flow
---------
                 +-------------------- receiver (serial) -------------------+
                 |                                                          |
   SerialLink thread --bytes--> StreamFramer --+-- NMEA / Unicore text --> GnssProcessor --> ROS topics
                                               |                        +-> ~/nmea, gpsd UDP
                                               +-- RTCM frames --------> dropped (base stations: rtk_base)
   RTCM sources (UDP/TCP/NTRIP/serial) --bytes--> RtcmRouter.inject --> SerialLink.write

The receiver class (``receiver``) turns the rover settings into module
commands: mode and logs when ``configure_receiver`` is true, the ``rover.*``
timeouts and heading baseline whenever they are set.  After the commands the
driver asks the receiver for its configuration and logs the report.  See
README for parameters.
"""
import logging
import math
import signal
import socket

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from gps_msgs.msg import GPSFix
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from nmea_msgs.msg import Sentence
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, TimeReference
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from unicore_rtk_driver.processor import Config, GnssProcessor
from unicore_rtk_driver.protocol.framer import FrameType, StreamFramer
from unicore_rtk_driver.protocol.solution import GGA_SOLUTION_NAMES
from unicore_rtk_driver.receivers import RoverSettings, create_receiver, parse_log_list
from unicore_rtk_driver.receivers.unicore import DEFAULT_LOGS, DEFAULT_ROVER_MODE_COMMAND
from unicore_rtk_driver.rtcm.router import RtcmRouter
from unicore_rtk_driver.rtcm.transport import create_source
from unicore_rtk_driver.serial_link import SerialLink


def _non_empty(values):
    return [v for v in values if v and v.strip()]


def _seconds(value):
    return '-' if math.isnan(value) else f'{value:.1f}'


class UnicoreDriver(Node):

    def __init__(self):
        super().__init__('unicore_rtk_driver_node')
        self._declare_parameters()
        self._setup_logging()
        self._setup_publishers()

        self.framer = StreamFramer()
        self.processor = GnssProcessor(self._processor_config(), self._publish,
                                       self.get_logger(), self._lookup_rotation)
        self.router = RtcmRouter(self.get_logger(), self.p('rtcm.failover_timeout_s'))
        self.receiver = create_receiver(self.p('receiver'), self.p('receiver_port'))
        self._configuration_commands(True)  # invalid settings stop the node here
        if self.receiver.name == 'generic' and self._rover_settings().has_receiver_settings:
            self.get_logger().warn('Unicore Driver - rover timeouts and heading baseline '
                                   'are not sent to a generic receiver')

        self.link = SerialLink(self.p('device'), self.p('baudrate'), self.get_logger(),
                               on_data=self._on_serial_data,
                               on_connect=self._on_receiver_connected,
                               reconnect_s=self.p('serial_reconnect_s'))

        self.udp_socket = None
        if self.p('use_gpsd_udp'):
            self.server_address = (self.p('gpsd_udp_host'), self.p('virtual_port'))
            self.gpsd_patterns = tuple(_non_empty(self.p('gpsd_udp_patterns')))
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(String, self.p('tx_cmd_topic'), self._on_tx_cmd, 10)
        self.create_subscription(Imu, self.p('imu_topic'), self._on_imu, 10)
        self.create_service(Trigger, '~/configure_receiver', self._on_configure_request)

        self._setup_rtcm()
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(self.p('diagnostics_period'), self._publish_diagnostics)

        self.router.start()
        self.link.start()
        self.get_logger().info(
            f'Unicore Driver - receiver={self.receiver.name}, '
            f'device={self.p("device")}, RTCM in={len(self.router.sources)}')

    # ================================================================ params
    def _declare_parameters(self):
        d = self.declare_parameter
        # --- serial port, gpsd and topics
        d('device', '/dev/ttyUSB0')
        d('baudrate', 115200)
        d('use_gnss_time', False)
        d('use_gpsd_udp', False)
        d('virtual_port', 5005)
        d('gpsd_udp_host', '127.0.0.1')
        d('gpsd_udp_patterns', ['$GN'])  # lines containing any of these go to gpsd
        d('serial_reconnect_s', 1.0)
        d('debug', False)
        d('heading_enu_pose_topic', '~/heading_enu_pose')
        d('fix_topic', '~/fix')
        d('gpsfix_topic', '~/gpsfix')
        d('time_ref_topic', '~/time_ref')
        d('nmea_topic', '~/nmea')
        d('global_vel_topic', '~/global_velocity')
        d('simple_heading_topic', '~/simple_heading')
        d('unicore_heading_topic', '~/heading_odometry')
        d('unicore_velocity_topic', '~/unicore_velocity')
        d('tx_cmd_topic', '~/tx_cmd')
        d('imu_topic', 'imu/data')
        d('heading_correction_angle_gps_to_imu', -3.0)  # degrees
        d('heading_frame_id', 'world_ned')
        d('gps_frame_id', 'unicore_gps')
        d('heading_enu_frame_id', 'world_enu')
        for q, v in ((0, 1000000.0), (1, 4.0), (2, 0.1), (4, 0.02), (5, 4.0), (9, 3.0)):
            d(f'epe_quality{q}', v)
        # --- output conventions / validation
        d('velocity_frame_id', 'world')
        d('fix_covariance_mode', 'hdop_scaled')
        d('strict_validation', False)
        d('heading_use_gnss_time', False)
        # --- receiver
        d('receiver', 'unicore')
        d('receiver_port', '')
        d('configure_receiver', False)
        d('save_receiver_config', False)
        d('receiver_logs', [f'{m.name}:{m.period_s:g}' for m in DEFAULT_LOGS])
        d('rover.mode_command', DEFAULT_ROVER_MODE_COMMAND)
        d('rover.rtk_timeout_s', 0)    # 0 = receiver default
        d('rover.dgps_timeout_s', 0)
        d('rover.heading_baseline_cm', 0)
        d('rover.heading_baseline_tolerance_cm', 0)
        d('query_receiver_config', True)
        d('receiver_commands', [''])
        d('save_config_after_tx_cmd', True)
        # --- RTCM transport
        d('rtcm.inputs', [''])
        d('rtcm.failover_timeout_s', 5.0)
        d('rtcm.correction_timeout_s', 5.0)
        d('diagnostics_period', 1.0)

    def p(self, name):
        return self.get_parameter(name).value

    def _processor_config(self) -> Config:
        cfg = Config()
        for name in ('gps_frame_id', 'heading_frame_id', 'heading_enu_frame_id',
                     'velocity_frame_id', 'use_gnss_time', 'heading_correction_angle_gps_to_imu',
                     'fix_covariance_mode', 'strict_validation', 'heading_use_gnss_time',
                     'epe_quality0', 'epe_quality1', 'epe_quality2', 'epe_quality4',
                     'epe_quality5', 'epe_quality9'):
            setattr(cfg, name, self.p(name))
        return cfg

    def _setup_logging(self):
        # the parsers log through Python logging ('rosout')
        level = logging.DEBUG if self.p('debug') else logging.INFO
        logging.getLogger('rosout').setLevel(level)
        if self.p('debug'):
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    # ============================================================ publishers
    def _setup_publishers(self):
        enu = self.p('heading_enu_pose_topic')
        spec = {
            'fix': (NavSatFix, self.p('fix_topic')),
            'gpsfix': (GPSFix, self.p('gpsfix_topic')),
            'time_ref': (TimeReference, self.p('time_ref_topic')),
            'global_velocity': (TwistStamped, self.p('global_vel_topic')),
            'simple_heading': (Float32, self.p('simple_heading_topic')),
            'heading_enu_pose': (PoseWithCovarianceStamped, enu),
            'heading_enu_pose_imu': (PoseWithCovarianceStamped, enu + '/imu'),
            'heading_enu_pose_imu_manual': (PoseWithCovarianceStamped, enu + '/imu/manual'),
            'heading_odometry': (Odometry, self.p('unicore_heading_topic')),
            'unicore_velocity': (TwistWithCovarianceStamped, self.p('unicore_velocity_topic')),
        }
        self.pubs = {name: self.create_publisher(t, topic, 10) for name, (t, topic) in spec.items()}
        self.nmea_pub = self.create_publisher(Sentence, self.p('nmea_topic'), 10)

    def _publish(self, name, msg):
        self.pubs[name].publish(msg)

    # ================================================================== RTCM
    def _setup_rtcm(self):
        log = self.get_logger()
        for uri in _non_empty(self.p('rtcm.inputs')):
            self.router.add_source(
                lambda on_data, uri=uri: create_source(
                    uri, log, on_data, gga_provider=lambda: self.processor.last_gga))
        self.router.set_injector(self.link.write)

    # ============================================================== receiver
    def _rover_settings(self) -> RoverSettings:
        return RoverSettings(
            mode_command=self.p('rover.mode_command').strip(),
            rtk_timeout_s=self.p('rover.rtk_timeout_s'),
            dgps_timeout_s=self.p('rover.dgps_timeout_s'),
            heading_baseline_cm=self.p('rover.heading_baseline_cm'),
            heading_baseline_tolerance_cm=self.p('rover.heading_baseline_tolerance_cm'))

    def _configuration_commands(self, configure: bool):
        """
        Build the commands for the current parameters.

        Mode and logs (if `configure`), the rover settings that are set,
        `receiver_commands`, then the save command if requested.
        """
        rover = self._rover_settings()
        cmds = []
        if configure:
            cmds += self.receiver.configuration_commands(
                rover, parse_log_list(self.p('receiver_logs')))
        cmds += self.receiver.settings_commands(rover)
        cmds += _non_empty(self.p('receiver_commands'))
        save = self.receiver.save_configuration_command()
        if cmds and save and self.p('save_receiver_config'):
            cmds.append(save)
        return cmds

    def _send_commands(self, cmds):
        for cmd in cmds:
            self.get_logger().info(f'Unicore Driver - sending: {cmd}')
            if not self.link.write_line(cmd):
                return False
        return True

    def _on_receiver_connected(self):
        """Runs on the serial thread after each (re)connect."""
        try:
            cmds = self._configuration_commands(self.p('configure_receiver'))
        except ValueError as e:  # parameters changed at runtime
            self.get_logger().error(f'Unicore Driver - invalid parameters: {e}')
            return
        if self._send_commands(cmds):
            self._query_config()

    def _query_config(self):
        query = self.receiver.config_query_command()
        if query and self.p('query_receiver_config'):
            self.link.write_line(query)

    def _on_configure_request(self, request, response):
        """~/configure_receiver: (re)send the configuration from the current
        parameters, e.g. after `ros2 param set ... receiver_logs ...`.  Works
        whatever `configure_receiver` is set to."""
        try:
            cmds = self._configuration_commands(True)
        except ValueError as e:
            response.success, response.message = False, f'invalid parameters: {e}'
            return response
        if not self.link.is_open:
            response.success, response.message = False, 'receiver not connected'
            return response
        response.success = self._send_commands(cmds)
        response.message = '; '.join(cmds) if response.success else 'serial write failed'
        if response.success:
            self._query_config()
        return response

    def _on_serial_data(self, data: bytes):
        """Runs on the serial thread for every chunk read."""
        stamp = self.get_clock().now().to_msg()
        for frame in self.framer.feed(data):
            if frame.type is FrameType.RTCM:
                continue  # a rover has no use for RTCM from its own receiver
            line = frame.text
            self._send_sentence(line + '\r\n', stamp)
            if self.receiver.is_command_response(line):
                self.get_logger().info(f'Unicore Driver - receiver replied: {line}')
                continue
            if self.receiver.is_config_report(line):
                self.get_logger().info(f'Unicore Driver - receiver config: {line}')
                continue
            try:
                if frame.type is FrameType.NMEA:
                    self.processor.handle_nmea(line, stamp)
                else:
                    self.processor.handle_unicore(line, stamp)
            except Exception as e:  # a bad line must not take the driver down
                self.get_logger().error(f'Unicore Driver - failed on {line!r}: {e}',
                                        throttle_duration_sec=5.0)

    def _send_sentence(self, data: str, stamp):
        if self.udp_socket is not None and any(p in data for p in self.gpsd_patterns):
            self.udp_socket.sendto(data.encode(), self.server_address)
        if self.nmea_pub.get_subscription_count() > 0:
            msg = Sentence()
            msg.header.stamp = stamp
            msg.header.frame_id = self.p('gps_frame_id')
            msg.sentence = data
            self.nmea_pub.publish(msg)

    def _on_tx_cmd(self, msg: String):
        """Send a raw command to the receiver, e.g. 'UNIHEADINGA COM3 0.2'.

            ros2 topic pub /<ns>/<node>/tx_cmd std_msgs/msg/String "data: 'UNLOG GPHPR'" --once
        """
        self.get_logger().info(f'Unicore Driver - Received msg: {msg.data}')
        self.link.write_line(msg.data)
        save = self.receiver.save_configuration_command()
        if self.p('save_config_after_tx_cmd') and save:
            self.link.write_line(save)

    # =================================================================== IMU
    def _on_imu(self, msg: Imu):
        self.processor.set_imu(msg.header.frame_id, msg.orientation)

    def _lookup_rotation(self, target, source):
        tf = self.tf_buffer.lookup_transform(target, source, RclpyTime())
        r = tf.transform.rotation
        return [r.x, r.y, r.z, r.w]

    # =========================================================== diagnostics
    def _publish_diagnostics(self):
        inc = self.router.incoming
        timeout = self.p('rtcm.correction_timeout_s')
        inc.roll()
        st = DiagnosticStatus()
        st.name = f'{self.get_fully_qualified_name()}: GNSS receiver'
        st.hardware_id = f'{self.receiver.name}@{self.p("device")}'
        fix = self.processor.fix_msg

        values = [
            ('receiver', self.receiver.name),
            ('serial connected', str(self.link.is_open)),
            ('serial bytes read', str(self.link.bytes_read)),
            ('fix status', str(fix.status.status)),
            ('solution', GGA_SOLUTION_NAMES.get(self.processor.gga_quality, 'unknown')),
            ('satellites used', str(self.processor.num_satellites)),
            ('horizontal std m', f'{self.processor.horizontal_std_m:.3f}'),
            ('hdop', f'{self.processor.hdop:.2f}'),
            ('receiver correction age s', _seconds(self.processor.correction_age_s)),
            ('rtcm in bytes/s', f'{inc.rate_bps:.0f}'),
        ]
        age = inc.age()
        values.append(('rtcm in age s', f'{age:.1f}' if age is not None else 'never'))
        if self.router.base_station:
            sid, lat, lon, h = self.router.base_station
            values.append(('base station', f'id {sid}: {lat:.8f}, {lon:.8f}, {h:.3f} m'))
        for t in self.router.sources:
            values.append((t.name, t.status()))

        st.level, st.message = DiagnosticStatus.OK, 'ok'
        if not self.link.is_open:
            st.level, st.message = DiagnosticStatus.ERROR, 'receiver not connected'
        elif fix.status.status == NavSatStatus.STATUS_NO_FIX:
            st.level, st.message = DiagnosticStatus.WARN, 'no fix'
        elif self.router.sources and (age is None or age > timeout):
            st.level, st.message = DiagnosticStatus.WARN, 'no RTCM corrections'
        st.values = [KeyValue(key=k, value=v) for k, v in values]

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag_pub.publish(arr)

    # ============================================================== shutdown
    def close(self):
        self.link.stop()
        self.router.stop()
        if self.udp_socket is not None:
            self.udp_socket.close()


def main(args=None):
    rclpy.init(args=args)
    node = UnicoreDriver()
    try:
        node.get_logger().info('Unicore Driver - Start node ...')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT (Ctrl-C) or SIGTERM (systemctl stop, launch)
        pass
    finally:
        # Under launch the node receives the stop signal twice (from the terminal or systemd,
        # and forwarded by launch); a repeated signal must not interrupt the cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.close()
        node.get_logger().info('Unicore Driver - Everything is shutdown now!')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
