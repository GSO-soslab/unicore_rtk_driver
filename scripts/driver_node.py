#!/usr/bin/env python3

from enum import Enum
import serial
import math
import rospy
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, TwistStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from nmea_msgs.msg import Sentence
from std_msgs.msg import Float32, String
import tf
import tf.transformations
import tf2_ros

import nmea_parser
import unicore_parser

class StringType(Enum):
    FAILURE = 0
    NMEA = 1
    UNICORE = 2

class UnicoreDriver:

    def __init__(self):

        self.get_params()

        self.setup_ros()

        self.init()

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
        self.serial_port = rospy.get_param('~device', '/dev/ttyUSB0')
        self.serial_baudrate = rospy.get_param('~baudrate', 115200)

        self.use_gnss_time = rospy.get_param('~use_gnss_time', False)

        ### ros
        self.hpr_heading_topic = rospy.get_param('~hpr_heading_topic', 'unicore/heading')
        self.fix_topic = rospy.get_param('~fix_topic', 'unicore/fix')
        self.time_ref_topic = rospy.get_param('~fix_topic', 'unicore/time_reference')
        self.nmea_topic = rospy.get_param('~nmea_topic', 'unicore/nmea')
        self.global_vel_topic = rospy.get_param('~global_vel_topic', 'unicore/global_velocity')
        self.simple_heading_topic = rospy.get_param('~simple_heading_topic', 'unicore/simple_heading')

        self.tx_cmd_topic = rospy.get_param('~tx_cmd_topic', 'unicore/tx_cmd')

        self.heading_frame_id = rospy.get_param('~heading_frame_id', 'world_ned')
        self.gps_frame_id = rospy.get_param('~gps_frame_id', 'unicore_gps')

        ### epe = estimated position error
        self.default_epe_quality0 = rospy.get_param('~epe_quality0', 1000000)
        self.default_epe_quality1 = rospy.get_param('~epe_quality1', 4.0)
        self.default_epe_quality2 = rospy.get_param('~epe_quality2', 0.1)
        self.default_epe_quality4 = rospy.get_param('~epe_quality4', 0.02)
        self.default_epe_quality5 = rospy.get_param('~epe_quality5', 4.0)
        self.default_epe_quality9 = rospy.get_param('~epe_quality9', 3.0)

    def setup_ros(self):
        """
        This function is used for ros setup: publisher, subscribers, service, timer and more
        """

        ### publihser
        self.hpr_heading_pub = rospy.Publisher(self.hpr_heading_topic, PoseWithCovarianceStamped, queue_size=10)
        self.nmea_pub = rospy.Publisher(self.nmea_topic, Sentence, queue_size=10)
        self.fix_pub = rospy.Publisher(self.fix_topic, NavSatFix, queue_size=10)
        self.time_ref_pub = rospy.Publisher(self.time_ref_topic, TimeReference, queue_size=10)
        self.global_vel_pub = rospy.Publisher(self.global_vel_topic, TwistStamped, queue_size=10)
        self.simple_heading_pub = rospy.Publisher(self.simple_heading_topic, Float32, queue_size=10)

        ### subscriber
        rospy.Subscriber(self.tx_cmd_topic, String, self.tx_cmd_callback)

        ### setup the serial 
        self.ser = serial.Serial(self.serial_port, self.serial_baudrate, timeout=1)
        self.ser.flush()  

        ### timer
        rospy.Timer(rospy.Duration(1.0/10.0), self.process)

        ### tf
        self.tf_br = tf2_ros.TransformBroadcaster()

    def init(self):

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
            data = self.ser.readline().decode('utf-8').strip()

            ### get IO time for this sentence 
            self.time_io = rospy.Time.now()

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
            rospy.logwarn(
                "Received a sentence with an invalid checksum. " +
                "nmea string was: %s" % repr(nmea_string))
            return False
            
        ### parse a sentence
        parsed_sentence = nmea_parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            rospy.logdebug(
                "Failed to parse NMEA sentence. Sentence was: %s" % 
                nmea_string)
            return False
        
        if 'GGA' in parsed_sentence:
            data = parsed_sentence['GGA']

            if math.isnan(data['utc_time'][0]):
                rospy.logwarn("Time in the NMEA GGA sentence is NOT valid")
                return False
            
            ### get timestamp
            if self.use_gnss_time:
                self.fix_msg.header.stamp = rospy.Time(data['utc_time'][0], data['utc_time'][1])
            else:
                self.fix_msg.header.stamp = self.time_io
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
                rospy.logwarn("Time in the NMEA RMC sentence is NOT valid")
                return False
            
            ### TimeReference msg
            time_ref_msg = TimeReference()

            if self.use_gnss_time:
                time_ref_msg.header.stamp = rospy.Time(data['utc_time'][0], data['utc_time'][1])
            else:
                time_ref_msg.header.stamp = self.time_io            
            time_ref_msg.header.frame_id = self.gps_frame_id
            time_ref_msg.source = self.gps_frame_id
            time_ref_msg.time_ref = time_ref_msg.header.stamp
            self.time_ref_pub.publish(time_ref_msg)

            # FIXME not need if ENU speed is published through other msg
            if self.valid_fix:
                ### assign vel msg: header
                if self.use_gnss_time:
                    self.vel_msg.header.stamp = rospy.Time(data['utc_time'][0], data['utc_time'][1])
                else:
                    self.vel_msg.header.stamp = self.time_io            
                self.vel_msg.header.frame_id = "world"
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
                rospy.logwarn("Time in the NMEA HPR sentence is NOT valid")
                return False
            
            if data['fix_type'] == 0:
                rospy.logwarn("Fix in the NMEA HPR sentence is NOT valid")
                return False                
            
            heading_msg = PoseWithCovarianceStamped()

            ### assign heading msg: header
            if self.use_gnss_time:
                heading_msg.header.stamp = rospy.Time(data['utc_time'][0], data['utc_time'][1])
            else:
                heading_msg.header.stamp = self.time_io            
            heading_msg.header.frame_id = self.heading_frame_id
            ### assign heading msg: pose
            heading_msg.pose.pose.orientation = self.rpy_to_quaternion(
                0,0, data['heading'] * math.pi / 180.0)
            ### assign heading for simple test
            simple_heading_msg = Float32()
            simple_heading_msg.data = data['heading']
            
            ### publish heading
            self.hpr_heading_pub.publish(heading_msg)
            self.simple_heading_pub.publish(simple_heading_msg)

        ### publish the NMEA
        self.send_sentence(nmea_string)

        # DEBUG:
        # print("nmea: " + nmea_string)
        # print(parsed_sentence)

    def calculate_crc32(self, sz_buf: str) -> int:
        crc = 0
        for byte in sz_buf.encode('utf-8'):  # Encode string as bytes
            crc = self.crcTable[(crc ^ byte) & 0xff] ^ (crc >> 8)
        return hex(crc)

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
        # print('calc checksum: ')
        # print(str(calc_checksum)[2:])

        if(str(calc_checksum)[2:] == checksum):
            return True
        else:
            return False

    def parse_unicore(self, unicore_string):
        """
        This function parse the unicore string
        """

        ### check checksum
        if not self.check_unicore_string(unicore_string):
            rospy.logwarn(
                "Received a sentence with an invalid checksum. " +
                "unicore string was: %s" % repr(unicore_string))
            return False

        ### parse a sentence
        parsed_sentence = unicore_parser.parse_unicore_sentence(unicore_string)
        if not parsed_sentence:
            rospy.logwarn(
                "Failed to parse UNICORE sentence. Sentence was: %s" % 
                unicore_string)
            return False
        
        # DEBUG:
        print("unicore: " + unicore_string)        
        print(parsed_sentence)

        if 'HEAD' in parsed_sentence and 'UNIHEADINGA' in parsed_sentence:
            data = parsed_sentence['UNIHEADINGA']

            # assign the data
            station_id = data['StationId']
        
        if 'HEAD' in parsed_sentence and 'AGRICA' in parsed_sentence:
            data = parsed_sentence['AGRICA']        
 
             # assign the data
            GNSS = data['GNSS']

    def send_sentence(self, data):

        if self.nmea_pub.get_num_connections() > 0:
            msg = Sentence()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "unicore_gps"
            msg.sentence = data
            self.nmea_pub.publish(msg)

    def tx_cmd_callback(self, data):
        """
        This function is used for receive the 'Data output Commands' and send back to GPS

        Format: "command rate"
        Example: To publish [UNIHEADING] msg at 1 Hz and send "UNIHEADINGA COM3 1"
        Example: To stop [UNIHEADING] msg at 1 Hz and send "UNLOG UNIHEADINGA"
        """

        rospy.loginfo("Received msg: %s", data.data)

        # enable some param
        cmd = data.data + "\r\n"
        self.ser.write(cmd.encode())

        # save the config
        cmd = "saveconfig\r\n"
        self.ser.write(cmd.encode())

    def process(self, event):
        ### make sure serial is open
        if not self.ser.is_open:
            return False 

        ### read one line
        data_string = self.read_nmea()
        if(data_string is False):
            return False

        ### check integrity
        data_type = self.check_integrity(data_string)

        if data_type == StringType.NMEA:
            self.parse_nmea(data_string)
        elif data_type == StringType.UNICORE:
            self.parse_unicore(data_string)
        else:
            print("failure")

    def close(self):
        if self.ser.is_open:
            self.ser.close()

    def rpy_to_quaternion(self, r, p, y):
        quaternion = tf.transformations.quaternion_from_euler(r, p, y)
        return Quaternion(*quaternion)
    
def main():

    rospy.init_node("Unicore_RTK_Driver")

    try:
        rospy.loginfo("Start node ...")
        node = UnicoreDriver()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.shutdown()

    if rospy.is_shutdown():
        
        node.close()

        rospy.loginfo("Everything is shutdown now!")


### Call the main function
if __name__ == '__main__':
    main()