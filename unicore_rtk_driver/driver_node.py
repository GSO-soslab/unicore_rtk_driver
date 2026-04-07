from enum import Enum
import serial
import socket 
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, TwistStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference, Imu
from nmea_msgs.msg import Sentence
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
import tf_transformations
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs
import logging
from unicore_rtk_driver import nmea_parser
from unicore_rtk_driver import unicore_parser
from builtin_interfaces.msg import Time

class StringType(Enum):
    FAILURE = 0
    NMEA = 1
    UNICORE = 2

class UnicoreDriver(Node):

    def __init__(self):
        super().__init__('unicore_rtk_driver_node')

        self.get_params()

        self.setup_ros()

        self.init()

        ### from unicore doc
        self.crcTable = [
            0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
            0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
            0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
            0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
            0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
            0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
            0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
            0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
            0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
            0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
            0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
            0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
            0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
            0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
            0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
            0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
            0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
            0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
            0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
            0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
            0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
            0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
            0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
            0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
            0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
            0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
            0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
            0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
            0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
            0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
            0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
            0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
            0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
            0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
            0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
            0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
            0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
            0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
            0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
            0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
            0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
            0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
            0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d ]

    def get_params(self):

        ### configuration
        self.declare_parameter('device', '/dev/ttyUSB0')
        self.serial_port = self.get_parameter('device').get_parameter_value().string_value

        self.declare_parameter('baudrate', 115200)
        self.baud_rate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.declare_parameter('use_gnss_time', False)
        self.use_gnss_time = self.get_parameter('use_gnss_time').get_parameter_value().bool_value

        # use UDP to send time info to gpsd
        self.declare_parameter('use_gpsd_udp', False)
        self.use_gpsd_udp = self.get_parameter('use_gpsd_udp').get_parameter_value().bool_value
        self.declare_parameter('virtual_port', 5005)
        self.virtual_port = self.get_parameter('virtual_port').get_parameter_value().integer_value

        # Declare and get parameters
        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        ### NMEA 0183 msg
        self.declare_parameter('heading_enu_pose_topic', '~/heading_enu_pose')
        self.heading_enu_pose_topic = self.get_parameter('heading_enu_pose_topic').get_parameter_value().string_value

        self.declare_parameter('fix_topic', '~/fix')
        self.fix_topic = self.get_parameter('fix_topic').get_parameter_value().string_value

        self.declare_parameter('time_ref_topic', '~/time_ref')
        self.time_ref_topic = self.get_parameter('time_ref_topic').get_parameter_value().string_value

        self.declare_parameter('nmea_topic', '~/nmea')
        self.nmea_topic = self.get_parameter('nmea_topic').get_parameter_value().string_value

        self.declare_parameter('global_vel_topic', '~/global_velocity')
        self.global_vel_topic = self.get_parameter('global_vel_topic').get_parameter_value().string_value

        self.declare_parameter('simple_heading_topic', '~/simple_heading')
        self.simple_heading_topic = self.get_parameter('simple_heading_topic').get_parameter_value().string_value

        ### Unicore msg
        self.declare_parameter('unicore_heading_topic', '~/heading_odometry')
        self.unicore_heading_topic = self.get_parameter('unicore_heading_topic').get_parameter_value().string_value

        self.declare_parameter('unicore_velocity_topic', '~/unicore_velocity')
        self.unicore_velocity_topic = self.get_parameter('unicore_velocity_topic').get_parameter_value().string_value

        self.declare_parameter('tx_cmd_topic', '~/tx_cmd')
        self.tx_cmd_topic = self.get_parameter('tx_cmd_topic').get_parameter_value().string_value

        self.declare_parameter('imu_topic', 'imu/data')
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        
        ### Frames
        self.declare_parameter('heading_frame_id', 'world_ned')
        self.heading_frame_id = self.get_parameter('heading_frame_id').get_parameter_value().string_value

        self.declare_parameter('gps_frame_id', 'unicore_gps')
        self.gps_frame_id = self.get_parameter('gps_frame_id').get_parameter_value().string_value

        self.declare_parameter('heading_enu_frame_id', 'world_enu')
        self.heading_enu_frame_id = self.get_parameter('heading_enu_frame_id').get_parameter_value().string_value


        ### epe = estimated position error
        self.declare_parameter('epe_quality0', 1000000.0)
        self.default_epe_quality0 = self.get_parameter('epe_quality0').get_parameter_value().double_value

        self.declare_parameter('epe_quality1', 4.0)
        self.default_epe_quality1 = self.get_parameter('epe_quality1').get_parameter_value().double_value

        self.declare_parameter('epe_quality2', 0.1)
        self.default_epe_quality2 = self.get_parameter('epe_quality2').get_parameter_value().double_value

        self.declare_parameter('epe_quality4', 0.02)
        self.default_epe_quality4 = self.get_parameter('epe_quality4').get_parameter_value().double_value

        self.declare_parameter('epe_quality5', 4.0)
        self.default_epe_quality5 = self.get_parameter('epe_quality5').get_parameter_value().double_value

        self.declare_parameter('epe_quality9', 3.0)
        self.default_epe_quality9 = self.get_parameter('epe_quality9').get_parameter_value().double_value

    def setup_ros(self):
        # Setup logger
        logger = logging.getLogger("rosout")
        if self.debug:
            logger.setLevel(logging.DEBUG)
        else:
            logger.setLevel(logging.INFO)

        # Publishers
        self.nmea_pub = self.create_publisher(Sentence, self.nmea_topic, 10)
        self.fix_pub = self.create_publisher(NavSatFix, self.fix_topic, 10)
        self.time_ref_pub = self.create_publisher(TimeReference, self.time_ref_topic, 10)
        self.global_vel_pub = self.create_publisher(TwistStamped, self.global_vel_topic, 10)
        self.simple_heading_pub = self.create_publisher(Float32, self.simple_heading_topic, 10)
        self.heading_enu_pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.heading_enu_pose_topic, 10)
        self.heading_enu_pose_imu_pub = self.create_publisher(PoseWithCovarianceStamped, self.heading_enu_pose_topic+'/imu', 10)

        self.unicore_heading_pub = self.create_publisher(Odometry, self.unicore_heading_topic, 10)
        self.unicore_velocity_pub = self.create_publisher(TwistWithCovarianceStamped, self.unicore_velocity_topic, 10)

        # Subscribers
        self.tx_cmd_sub = self.create_subscription(String, self.tx_cmd_topic, self.tx_cmd_callback, 10)
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)

        # Serial setup
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        self.ser.flush()

        # setup UDP for gpsd time access
        if self.use_gpsd_udp:
            self.server_address = ("127.0.0.1", self.virtual_port)
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    

        # Timer (10Hz)
        self.timer = self.create_timer(0.1, self.process)  # 1.0 / 10.0

        # TF Broadcaster
        self.tf_br = TransformBroadcaster(self)

        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def init(self):

        self.imu_rotation = None
        self.imu_frame = None
        self.imu_flag = None

        self.hdop = float("nan")

        self.valid_fix = False

        """Format for this dictionary is the fix type from a GGA message as the key, with
        each entry containing a tuple consisting of a default estimated
        position error, a NavSatStatus value, and a NavSatFix covariance value."""
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

        self.fix_msg = NavSatFix()

        self.vel_msg = TwistStamped()

    def read_nmea(self):
        """
        This function will read a line from serial stream data
        """

        if self.ser.in_waiting > 0:
            ### read one line
            try:
                data = self.ser.readline().decode('utf-8').strip()
            except Exception as e:
                print("Unicore NMEA could not deode!", flush=True)
                return False
            # data = self.ser.readline().decode('utf-8', errors='ignore')

            ### get IO time for this sentence 
            self.time_io = self.get_clock().now()

            return data
        else:
            return False

    def check_integrity(self, data):
        """This function check the data string integrity

        Args:
            data (str): the data came from serial

        Returns:
            StringType: FAILURE, NMEA, UNICORE
        """

        # check integrity
        if data.startswith('$') and '*' in data:
            return StringType.NMEA
        elif data.startswith('#') and '*' in data:
            return StringType.UNICORE
        else:
            return StringType.FAILURE

    def check_nmea_string(self, nmea_string):
        """
        This function check the nmea string, parse the data
        """
        #remove any \r\n in the string
        nmea_string = nmea_string.replace('\r', '').replace('\n', '') 

        # Split the string at the asterisk to isolate the checksum
        data, checksum = nmea_string.split('*')

        #checksum check
        c_checksum = f"{self.calculate_crc8(data):02X}"
        if(checksum == c_checksum):
            return True
        else:
            return False

    def calculate_crc8(self, nmea_string):
        """
        This function calculate the checksum.
        """

        # Calculate the checksum for the NMEA string
        checksum = 0
        # Skip the starting '$'
        for char in nmea_string[1:]:  
            checksum ^= ord(char)
        return checksum

    def parse_nmea(self, nmea_string):
        """
        This function parse the nmea string
        """

        ### check checksum
        if not self.check_nmea_string(nmea_string):
            self.get_logger().warn(
                f"Unicore Driver - Received a sentence with an invalid checksum. "
                f"nmea string was: {repr(nmea_string)}"
            )
            return False
            
        ### parse a sentence
        parsed_sentence = nmea_parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            self.get_logger().debug(
                f"Unicore Driver - Failed to parse NMEA sentence. Sentence was: {nmea_string}"
            )
            return False

        # DEBUG:
        self.get_logger().debug(f"Unicore Driver - NMEA: {nmea_string}")
        self.get_logger().debug(str(parsed_sentence))


        if 'GGA' in parsed_sentence:
            data = parsed_sentence['GGA']

            if math.isnan(data['utc_time'][0]):
                self.get_logger().warn(
                    f"Unicore Driver - Time in the NMEA GGA sentence is NOT valid"
                )
                return False
            
            ### get timestamp
            if self.use_gnss_time:
                self.fix_msg.header.stamp = Time(sec=data['utc_time'][0], nanosec=data['utc_time'][1])
            else:
                self.fix_msg.header.stamp = self.time_io.to_msg()
            self.fix_msg.header.frame_id = self.gps_frame_id

            ### get fix
            fix_type = data['fix_type']
            if not (fix_type in self.gps_qualities):
                fix_type = -1
            gps_qual = self.gps_qualities[fix_type]
            self.valid_fix = (fix_type > 0)
            ### get lat            
            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            ### get lon
            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude            
            ### get alt: altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']            
            ### get Horizontal Dilution of Precision (HDOP)
            self.hdop = data['hdop']

            ### assign fix msg: status
            self.fix_msg.status.status = gps_qual[1]
            self.fix_msg.status.service = NavSatStatus.SERVICE_GPS
            ### assign fix msg: cov type
            self.fix_msg.position_covariance_type = gps_qual[2]
            ### assign fix msg: lat/lon/alt
            self.fix_msg.latitude = latitude
            self.fix_msg.longitude = longitude
            self.fix_msg.altitude = altitude

        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']

            ### get actual GNSS time
            if math.isnan(data['utc_time'][0]):
                self.get_logger().warn("Time in the NMEA RMC sentence is NOT valid")

                return False

            ### check the gnss time and pc time, cause other NMEA msg utc_time_pc as gnss time
            if abs(data['utc_time'][0] - data['utc_time_pc'][0]) > 2.0:
                self.get_logger().warn(
                    f"Unicore Driver - System may not time sync, UTC and System time are different: "
                    f"UTC:{data['utc_time'][0]}, System:{data['utc_time_pc'][0]}"
                )

            
            ### TimeReference msg
            time_ref_msg = TimeReference()

            if self.use_gnss_time:
                time_ref_msg.header.stamp = Time(sec=data['utc_time'][0], nanosec=data['utc_time'][1])
            else:
                time_ref_msg.header.stamp = self.time_io.to_msg()            
            time_ref_msg.header.frame_id = self.gps_frame_id
            time_ref_msg.source = self.gps_frame_id
            time_ref_msg.time_ref = time_ref_msg.header.stamp
            self.time_ref_pub.publish(time_ref_msg)

            # FIXME not need if ENU speed is published through other msg
            if self.valid_fix:
                ### assign vel msg: header
                if self.use_gnss_time:
                    self.vel_msg.header.stamp = Time(sec=data['utc_time'][0], nanosec=data['utc_time'][1])
                else:
                    self.vel_msg.header.stamp = self.time_io.to_msg()            
                self.vel_msg.header.frame_id = self.heading_frame_id
                ### assign vel msg: data
                self.vel_msg.twist.linear.x = data['speed'] * math.sin(data['true_course'])
                self.vel_msg.twist.linear.y = data['speed'] * math.cos(data['true_course'])
                self.global_vel_pub.publish(self.vel_msg)

        elif 'GST' in parsed_sentence:
            data = parsed_sentence['GST']

            ### get position estimation error
            lon_std_dev = data['lon_std_dev']
            lat_std_dev = data['lat_std_dev']
            alt_std_dev = data['alt_std_dev']

            ### assign fix msg: covariance
            self.fix_msg.position_covariance[0] = (
                self.hdop * lon_std_dev) ** 2
            self.fix_msg.position_covariance[4] = (
                self.hdop * lat_std_dev) ** 2
            self.fix_msg.position_covariance[8] = (
                2 * self.hdop * alt_std_dev) ** 2  # FIXME?         

            ### publish fix
            self.fix_pub.publish(self.fix_msg)

        elif 'HPR' in parsed_sentence:
            data = parsed_sentence['HPR']

            ### get actual GNSS time
            if math.isnan(data['utc_time'][0]):
                self.get_logger().warn("Unicore Driver - Time in the NMEA HPR sentence is NOT valid")
                return False
            
            if data['fix_type'] == 0:
                self.get_logger().warn("Unicore Driver - NMEA HPR sentence is NOT valid")
                return False                

    def calculate_crc32(self, sz_buf: str) -> int:
        crc = 0
        for byte in sz_buf.encode('utf-8'):  # Encode string as bytes
            crc = self.crcTable[(crc ^ byte) & 0xff] ^ (crc >> 8)
        return '{:08x}'.format(crc)

    def check_unicore_string(self, unicore_string):
        """
        This function check the unicore string, parse the data
        """

        # remove any \r\n in the string
        unicore_string = unicore_string.replace('\r', '').replace('\n', '') 

        # split the string at the asterisk to isolate the checksum
        msg, checksum = unicore_string.split('*')

        # checksum check without the first '#'
        calc_checksum = self.calculate_crc32(msg[1:])

        # assign the pure msg
        unicore_string = msg

        # DEBUG:
        # print('given checksum:')
        # print(checksum)
        # print('calc checksum str: ')
        # print(calc_checksum)

        if(calc_checksum == checksum):
            return True
        else:
            return False

    def parse_unicore(self, unicore_string):
        """
        This function parse the unicore string
        """

        ### check checksum
        if not self.check_unicore_string(unicore_string):
            
            self.get_logger().warn(
                f"Unicore Driver - Received a sentence with an invalid checksum. "
                f"unicore string was: {repr(unicore_string)}")

            return False

        ### parse a sentence
        parsed_sentence = unicore_parser.parse_unicore_sentence(unicore_string)
        if not parsed_sentence:
            self.get_logger().warn(
                "Unicore Driver - Failed to parse UNICORE sentence. Sentence was: {unicore_string}" )
            return False
        
        # DEBUG:
        self.get_logger().debug(f"Unicore Driver - UNICORE: {unicore_string}")
        self.get_logger().debug(str(parsed_sentence))


        if 'HEAD' in parsed_sentence and 'UNIHEADINGA' in parsed_sentence:
            data = parsed_sentence['UNIHEADINGA']

            ### check the vaild status
            if data['Pose_Type'] == 0 and data['Sol_Stat'] !=0:
                self.get_logger().warn("Unicore Driver - UNICORE UNIHEADINGA sentence is not vaild")
                return 

            # print("Unicore msg - UNIHEADINGA: picth=%f, heading:%f" % 
            #       (data['Pitch']*180/3.1415926, data['Heading']*180/3.1415926))
            
            # =============================================================== #
            # Publish the heading as odometry msg in NED
            # =============================================================== #
            heading_msg = Odometry()
            heading_msg.header.stamp = self.time_io.to_msg()            
            heading_msg.header.frame_id = self.heading_frame_id
            heading_msg.child_frame_id = self.gps_frame_id
            ### assign heading msg: pose
            heading_msg.pose.pose.orientation = self.rpy_to_quaternion(
                0, 0, data['Heading'])
            heading_msg.pose.covariance[35] = data['Heading_Std'] ** 2             

            ### publish heading
            self.unicore_heading_pub.publish(heading_msg)

            # =============================================================== #
            # Publish the heading as Pose msg in ENU
            # =============================================================== #

            ### convert the heading
            # heading_enu = data['Heading'] - 90.0
            # if heading_enu < 0.0:
                # heading_enu = 270.0 - heading_enu
            heading_enu = -(data['Heading']-math.pi/2)
            ### publish the heading
            heading_enu_pose_msg = PoseWithCovarianceStamped()
            # FIXME: this heading msg not include the time
            heading_enu_pose_msg.header.stamp = self.time_io.to_msg()       
            heading_enu_pose_msg.header.frame_id = self.heading_enu_frame_id
            heading_enu_pose_msg.pose.pose.orientation = self.rpy_to_quaternion(
                0, 0, heading_enu)
            heading_enu_pose_msg.pose.covariance[35] = data['Heading_Std'] ** 2     
            self.heading_enu_pose_pub.publish(heading_enu_pose_msg)

            ###  Convert Heading to IMU frame
            ### Heading is influenced by Roll & Pitch, therefore constrain it by using IMU's Roll and Pitch
            if self.imu_flag:
                try:
                    ## 1. TF R_IMU (Rotation of IMU) to R_IMU_GPS (Rotation of IMU in GPS)
                    imu_pose_msg = PoseWithCovarianceStamped()
                    imu_pose_msg.header.stamp = heading_enu_pose_msg.header.stamp
                    imu_pose_msg.header.frame_id = self.imu_frame
                    imu_pose_msg.pose.pose.orientation = self.imu_rotation

                    imu_to_gps_tf = self.tf_buffer.lookup_transform(
                        self.gps_frame_id, self.imu_frame, RclpyTime())
                    imu_pose_in_gps = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(
                        imu_pose_msg, imu_to_gps_tf)

                    ## 2. Plug in absolute heading into R_IMU_GPS.
                    q = imu_pose_in_gps.pose.pose.orientation
                    # Get roll and pitch of R_IMU_GPS
                    roll, pitch, _ = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
                    corrected_q = tf_transformations.quaternion_from_euler(roll, pitch, heading_enu)
                    corrected_pose_msg = PoseWithCovarianceStamped()
                    corrected_pose_msg.header.stamp = heading_enu_pose_msg.header.stamp
                    corrected_pose_msg.header.frame_id = self.gps_frame_id
                    corrected_pose_msg.pose.pose.orientation = Quaternion(
                        x=corrected_q[0], y=corrected_q[1], z=corrected_q[2], w=corrected_q[3])
                    corrected_pose_msg.pose.covariance[35] = data['Heading_Std'] ** 2

                    ## 3. TF R_IMU_GPS to R_GPS_IMU (Rotation of GPS in IMU)
                    gps_to_imu_tf = self.tf_buffer.lookup_transform(
                        self.imu_frame, self.gps_frame_id, RclpyTime())
                    gps_pose_in_imu = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(
                        corrected_pose_msg, gps_to_imu_tf)
                    
                    ## 4. Publish as PoseWithCovarianceStamped msg.
                    self.heading_enu_pose_imu_pub.publish(gps_pose_in_imu)

                except Exception as e:
                    self.get_logger().warn(f"IMU-corrected heading TF failed: {e}")


            # =============================================================== #
            # Publish the heading as float msg in NED
            # =============================================================== #

            ### assign heading for simple test
            simple_heading_msg = Float32()
            simple_heading_msg.data = data['Heading']
            
            ### publish heading
            self.simple_heading_pub.publish(simple_heading_msg)

        if 'HEAD' in parsed_sentence and 'AGRICA' in parsed_sentence:
            data = parsed_sentence['AGRICA']        
 
            ### check the vaild status
            if data['Pose_Type'] == 0 or data['Speed_Vaild']:
                self.get_logger().warn("Unicore Driver - UNICORE AGRICA sentence is not vaild")
                return 

            ### get the gnss time, but wthiout nanosec
            unix_secs = unicore_parser.convert_time_argica(
                data['Year'], data['Month'], data['Day'], 
                data['Hour'], data['Minute'], data['Second'])
            
            twist_msg = TwistWithCovarianceStamped()

            ### assign heading msg: header
            if self.use_gnss_time:
                twist_msg.header.stamp = Time(sec=unix_secs, nanosec=0)
            else:
                twist_msg.header.stamp = self.time_io.to_msg()      
            twist_msg.header.frame_id = self.heading_frame_id

            ### assign the velocity and covariance
            # FIXME IS THIS VELOCITY BASED OON LOCAL SENSOR ENU FRAME?
            # FIXME OR THE SENSER BASED VELOCITY CONVERTED TO GLOBAL ENU FRAME?
            twist_msg.twist.twist.linear.x = data['Vel_East']
            twist_msg.twist.twist.linear.y = data['Vel_North']
            twist_msg.twist.twist.linear.z = data['Vel_Up']
            twist_msg.twist.covariance[0] = data['Vel_Std_East'] ** 2
            twist_msg.twist.covariance[7] = data['Vel_Std_North'] ** 2
            twist_msg.twist.covariance[14] = data['Vel_Std_Up'] ** 2

            self.unicore_velocity_pub.publish(twist_msg)

    def send_sentence(self, data):

        if "$GN" in data and self.use_gpsd_udp:
            self.udp_socket.sendto(data.encode(), self.server_address)

        if self.nmea_pub.get_subscription_count() > 0:
            msg = Sentence()
            msg.header.stamp = self.time_io.to_msg()
            msg.header.frame_id = self.gps_frame_id
            msg.sentence = data
            self.nmea_pub.publish(msg)

    def imu_callback(self, msg: Imu):
        if not self.imu_flag:
            # Get frame id 
            self.imu_frame = msg.header.frame_id
            self.imu_flag = True

        # Store rotation.
        self.imu_rotation = msg.orientation

    def tx_cmd_callback(self, data):
        """
        This function is used for receive the 'Data output Commands' and send back to GPS

        Format: "command rate"
        Examples: 
            To publish [UNIHEADING] msg at 5 Hz and send "UNIHEADINGA COM3 0.2"
                $ ros2 topic pub /my_robot/unicore_rtk/tx_cmd std_msgs/msg/String "data: 'UNIHEADINGA COM3 0.2'" --once
            To stop [UNIHEADING] msg at 5 Hz and send "UNLOG UNIHEADINGA" 
                $ ros2 topic pub /my_robot/unicore_rtk/tx_cmd std_msgs/msg/String "data: 'UNLOG UNIHEADINGA'" --once
            To stop [GNHPR] msg at 1 Hz and send "UNLOG GPHPR" 
                $ ros2 topic pub /my_robot/unicore_rtk/tx_cmd std_msgs/msg/String "data: 'UNLOG GPHPR'" --once
        """

        self.get_logger().info(f"Unicore Driver - Received msg: {data.data}")

        # enable some param
        cmd = data.data + "\r\n"
        self.ser.write(cmd.encode())

        # save the config
        cmd = "saveconfig\r\n"
        self.ser.write(cmd.encode())

    def process(self):
        ### make sure serial is open
        if not self.ser.is_open:
            return False 

        ### read one line
        data_string = self.read_nmea()
        if(data_string is False):
            return False

        ### publish the NMEA: unicore didn't have '\r\n' at the end
        self.send_sentence(data_string + "\r\n")

        ### check integrity
        data_type = self.check_integrity(data_string)

        if data_type == StringType.NMEA:
            self.parse_nmea(data_string)
        elif data_type == StringType.UNICORE:
            self.parse_unicore(data_string)
        else:
            self.get_logger().warn("Unicore Driver - not recognised sentence!")

    def close(self):
        # close gps serial port
        if self.ser.is_open:
            self.ser.close()

        # close UDP port
        if self.use_gpsd_udp:
            self.udp_socket.close()

    def rpy_to_quaternion(self, r, p, y):
        q = tf_transformations.quaternion_from_euler(r, p, y)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    
def main(args=None):
    rclpy.init(args=args)

    node = UnicoreDriver()

    try:
        node.get_logger().info("Unicore Driver - Start node ...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.get_logger().info("Unicore Driver - Everything is shutdown now!")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()