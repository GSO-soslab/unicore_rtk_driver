"""ROS 2 node for Unicore UM98x (and other NMEA/RTCM) GNSS receivers.

Data flow
---------
                 +-------------------- receiver (serial) -------------------+
                 |                                                          |
   SerialLink thread --bytes--> StreamFramer --+-- NMEA / Unicore text --> GnssProcessor --> ROS topics
                                               |                        +-> ~/nmea, gpsd UDP
                                               +-- RTCM frames --------> RtcmRouter --> sinks (UDP/TCP/serial)
   RTCM sources (UDP/TCP/NTRIP/serial) --bytes--> RtcmRouter.inject --> SerialLink.write

Operating modes (``mode``): ``rover`` (default) or ``base``.  The receiver
class (``receiver``) turns the mode and base settings into module commands
when ``configure_receiver`` is true.  See README for parameters.
"""
import logging
import socket

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from nmea_msgs.msg import Sentence
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, TimeReference
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from unicore_rtk_driver.processor import Config, GnssProcessor
from unicore_rtk_driver.protocol.framer import FrameType, StreamFramer
from unicore_rtk_driver.receivers import (BaseStationSettings, OperatingMode, RoverSettings,
                                          create_receiver, parse_log_list,
                                          parse_rtcm_message_list)
from unicore_rtk_driver.receivers.base import DEFAULT_RTCM_MESSAGES
from unicore_rtk_driver.receivers.unicore import DEFAULT_LOGS, DEFAULT_ROVER_MODE_COMMAND
from unicore_rtk_driver.rtcm.router import RtcmRouter
from unicore_rtk_driver.rtcm.transport import create_sink, create_source
from unicore_rtk_driver.serial_link import SerialLink

try:
    from rtcm_msgs.msg import Message as RtcmMsg
except ImportError:  # optional: only needed for rtcm.ros_topic
    RtcmMsg = None


def _non_empty(values):
    return [v for v in values if v and v.strip()]


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
        self.mode = OperatingMode(self.p('mode'))

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
            f'Unicore Driver - {self.mode.value} mode, receiver={self.receiver.name}, '
            f'device={self.p("device")}, RTCM out={len(self.router.sinks)} '
            f'in={len(self.router.sources)}')

    # ================================================================ params
    def _declare_parameters(self):
        d = self.declare_parameter
        # --- serial / original parameters (names and defaults unchanged)
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
        d('mode', 'rover')
        d('receiver', 'unicore')
        d('receiver_port', '')
        d('configure_receiver', False)
        d('save_receiver_config', False)
        d('receiver_logs', [f'{m.name}:{m.period_s:g}' for m in DEFAULT_LOGS])
        d('rover.mode_command', DEFAULT_ROVER_MODE_COMMAND)
        d('receiver_commands', [''])
        d('save_config_after_tx_cmd', True)
        # --- base station
        d('base.mode_command', '')
        d('base.survey_in_min_duration_s', 120)
        d('base.survey_in_accuracy_m', 2.0)
        d('base.use_fixed_position', False)
        d('base.fixed_latitude_deg', 0.0)
        d('base.fixed_longitude_deg', 0.0)
        d('base.fixed_altitude_m', 0.0)
        d('base.rtcm_messages', [f'{m.number}:{m.period_s:g}' for m in DEFAULT_RTCM_MESSAGES])
        # --- RTCM transport
        d('rtcm.outputs', [''])
        d('rtcm.inputs', [''])
        d('rtcm.ros_topic', '')
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

    def _base_settings(self) -> BaseStationSettings:
        return BaseStationSettings(
            mode_command=self.p('base.mode_command').strip(),
            survey_in_min_duration_s=self.p('base.survey_in_min_duration_s'),
            survey_in_accuracy_m=self.p('base.survey_in_accuracy_m'),
            use_fixed_position=self.p('base.use_fixed_position'),
            fixed_latitude_deg=self.p('base.fixed_latitude_deg'),
            fixed_longitude_deg=self.p('base.fixed_longitude_deg'),
            fixed_altitude_m=self.p('base.fixed_altitude_m'),
            rtcm_messages=parse_rtcm_message_list(self.p('base.rtcm_messages')))

    def _setup_logging(self):
        # parsers log through Python logging ('rosout'), as before
        level = logging.DEBUG if self.p('debug') else logging.INFO
        logging.getLogger('rosout').setLevel(level)
        if self.p('debug'):
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    # ============================================================ publishers
    def _setup_publishers(self):
        enu = self.p('heading_enu_pose_topic')
        spec = {
            'fix': (NavSatFix, self.p('fix_topic')),
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
        for uri in _non_empty(self.p('rtcm.outputs')):
            self.router.add_sink(create_sink(uri, log))
        for uri in _non_empty(self.p('rtcm.inputs')):
            self.router.add_source(
                lambda on_data, uri=uri: create_source(
                    uri, log, on_data, gga_provider=lambda: self.processor.last_gga))
        self.router.set_injector(self.link.write)

        topic = self.p('rtcm.ros_topic')
        if topic:
            if RtcmMsg is None:
                log.error('rtcm.ros_topic is set but rtcm_msgs is not installed; ignoring it')
            elif self.mode is OperatingMode.BASE:
                pub = self.create_publisher(RtcmMsg, topic, 10)

                def publish_frame(frame):
                    msg = RtcmMsg()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.p('gps_frame_id')
                    msg.message = list(frame)
                    pub.publish(msg)
                self.router.add_callback_sink(publish_frame)
            else:
                inject = self.router.make_injector()
                self.create_subscription(RtcmMsg, topic, lambda m: inject(bytes(m.message)), 10)

        if self.mode is OperatingMode.BASE and not (self.router.sinks or topic):
            log.warn('Base mode without rtcm.outputs: corrections are not sent anywhere')

    # ============================================================== receiver
    def _configuration_commands(self, configure: bool):
        """Commands for the current parameters: mode/logs (if `configure`),
        then `receiver_commands`, then the save command if requested."""
        cmds = []
        if configure:
            base = self._base_settings()
            rover = RoverSettings(mode_command=self.p('rover.mode_command').strip())
            cmds += self.receiver.configuration_commands(
                self.mode, rover, base, parse_log_list(self.p('receiver_logs')))
            if self.mode is OperatingMode.BASE:
                self.get_logger().info('Configuring base station: ' +
                                       ' '.join(self.receiver.describe_base(base)))
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
        self._send_commands(self._configuration_commands(self.p('configure_receiver')))

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
        return response

    def _on_serial_data(self, data: bytes):
        """Runs on the serial thread for every chunk read."""
        stamp = self.get_clock().now().to_msg()
        for frame in self.framer.feed(data):
            if frame.type is FrameType.RTCM:
                self.router.publish(frame.data)
                continue
            line = frame.text
            self._send_sentence(line + '\r\n', stamp)
            if self.receiver.is_command_response(line):
                self.get_logger().info(f'Unicore Driver - receiver replied: {line}')
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
        out, inc = self.router.outgoing, self.router.incoming
        timeout = self.p('rtcm.correction_timeout_s')
        out.roll()
        inc.roll()
        st = DiagnosticStatus()
        st.name = f'{self.get_fully_qualified_name()}: GNSS receiver'
        st.hardware_id = f'{self.receiver.name}@{self.p("device")}'
        fix = self.processor.fix_msg

        values = [
            ('mode', self.mode.value),
            ('receiver', self.receiver.name),
            ('serial connected', str(self.link.is_open)),
            ('serial bytes read', str(self.link.bytes_read)),
            ('fix status', str(fix.status.status)),
            ('hdop', f'{self.processor.hdop:.2f}'),
            ('rtcm out bytes/s', f'{out.rate_bps:.0f}'),
            ('rtcm out messages', ' '.join(f'{k}:{v}' for k, v in sorted(out.message_counts.items()))),
            ('rtcm in bytes/s', f'{inc.rate_bps:.0f}'),
        ]
        age = inc.age()
        values.append(('rtcm in age s', f'{age:.1f}' if age is not None else 'never'))
        if self.router.base_station:
            sid, lat, lon, h = self.router.base_station
            values.append(('base station', f'id {sid}: {lat:.8f}, {lon:.8f}, {h:.3f} m'))
        for t in self.router.sinks + self.router.sources:
            values.append((t.name, t.status()))

        st.level, st.message = DiagnosticStatus.OK, 'ok'
        if not self.link.is_open:
            st.level, st.message = DiagnosticStatus.ERROR, 'receiver not connected'
        elif fix.status.status == NavSatStatus.STATUS_NO_FIX:
            st.level, st.message = DiagnosticStatus.WARN, 'no fix'
        elif self.router.sources and (age is None or age > timeout):
            st.level, st.message = DiagnosticStatus.WARN, 'no RTCM corrections'
        elif self.mode is OperatingMode.BASE and (out.age() is None or out.age() > timeout):
            st.level, st.message = DiagnosticStatus.WARN, 'no RTCM from receiver (survey-in running?)'
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
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.get_logger().info('Unicore Driver - Everything is shutdown now!')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
