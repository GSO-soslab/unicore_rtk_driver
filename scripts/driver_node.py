#!/usr/bin/env python3

import serial
import math
import rospy
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, TwistStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
from nmea_msgs.msg import Sentence
import tf
import tf.transformations
import tf2_ros

import parser

class UnicoreDriver:
    def __init__(self):

        self.get_params()

        self.setup_ros()

        self.setup_serial()

        self.init()

    def get_params(self):

        ### configuration
        self.serial_port = rospy.get_param('~device', '/dev/ttyUSB0')
        self.serial_baudrate = rospy.get_param('~baudrate', 115200)

        self.use_gnss_time = rospy.get_param('~use_gnss_time', False)

        ### ros
        self.heading_topic = rospy.get_param('~heading_topic', 'unicore/heading')
        self.fix_topic = rospy.get_param('~fix_topic', 'unicore/fix')
        self.nmea_topic = rospy.get_param('~nmea_topic', 'unicore/nmea')
        self.global_vel_topic = rospy.get_param('~global_vel_topic', 'unicore/global_velocity')

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
        self.heading_pub = rospy.Publisher(self.heading_topic, PoseWithCovarianceStamped, queue_size=10)
        self.nmea_pub = rospy.Publisher(self.nmea_topic, Sentence, queue_size=10)
        self.fix_pub = rospy.Publisher(self.fix_topic, NavSatFix, queue_size=10)
        self.global_vel_pub = rospy.Publisher(self.global_vel_topic, TwistStamped, queue_size=10)

        ### timer
        rospy.Timer(rospy.Duration(1.0/10.0), self.process)

        ### tf
        self.tf_br = tf2_ros.TransformBroadcaster()

    def setup_serial(self):
        """
        This function will be used for setup serial port
        """

        ### setup the serial 
        try:
            self.ser = serial.Serial(self.serial_port, self.serial_baudrate, timeout=1)
            self.ser.flush()  

        except serial.SerialException as e:
            rospy.logerr("Debug: Could not open serial port %s. Reason: %s", self.serial_port, e)
            exit(1)

    def init(self):

        ### some init
        self.time_io = rospy.Time.now()
        self.time_io_ready = False
        self.time_gnss = rospy.Time.now()
        self.time_gnss_ready = False

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

        self.heading_msg = PoseWithCovarianceStamped()

        self.vel_msg = TwistStamped()

    def read_nmea(self):
        """
        This function will read a line from serial stream data
        """

        if self.ser.in_waiting > 0:
            ### read one line
            data = self.ser.readline().decode('utf-8').strip()

            ### save the IO timestamp only if this is the first sentence
            # if not self.time_gnss_ready and not self.time_io_ready:
            if not self.time_io_ready:
                self.time_io_ready = True
                self.time_io = rospy.Time.now()
                print("The real IO Time now: %s" % self.time_io.to_sec())

            # print("IO Time now: %s" % rospy.Time.now().to_sec())

            return data
        else:
            return False

    def check_nmea_string(self, nmea_string):
        """
        This function check the nmea string, parse the data
        """
        #remove any \r\n in the string
        nmea_string = nmea_string.replace('\r', '').replace('\n', '') 

        if nmea_string.startswith('$'):
            # Split the string at the asterisk to isolate the checksum
            data, checksum = nmea_string.split('*')

            #checksum check
            c_checksum = f"{self.calculate_checksum(data):02X}"
            if(checksum == c_checksum):
                return True
            else:
                return False
                            
    def calculate_checksum(self, nmea_string):
        """
        This function calculate the checksum.
        """

        # Calculate the checksum for the NMEA string
        checksum = 0
        # Skip the starting '$'
        for char in nmea_string[1:]:  
            checksum ^= ord(char)
        return checksum
            
    def send_sentence(self, data):

        if self.nmea_pub.get_num_connections() > 0:
            msg = Sentence()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "unicore_gps"
            msg.sentence = data
            self.nmea_pub.publish(msg)

    def process(self, event):

        ### read one line
        nmea_string = self.read_nmea()
        if(nmea_string is False):
            return False

        ### check checksum
        if not self.check_nmea_string(nmea_string):
            rospy.logwarn(
                "Received a sentence with an invalid checksum. " +
                "Sentence was: %s" % repr(nmea_string))
            return False
    
        ### parse a sentence
        parsed_sentence = parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            rospy.logdebug(
                "Failed to parse NMEA sentence. Sentence was: %s" % 
                nmea_string)
            return False
        
        print(parsed_sentence)
        
        if 'GGA' in parsed_sentence:
            data = parsed_sentence['GGA']

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

            ### get GNSS time
            if self.use_gnss_time:
                if math.isnan(data['utc_time'][0]):
                    rospy.logwarn("RMC: Time in the NMEA sentence is NOT valid")
                    return False
                else:
                    self.time_gnss = rospy.Time(data['utc_time'][0], data['utc_time'][1])
                    self.time_gnss_ready = True

            ### assign timestamp
            if self.time_gnss_ready and self.use_gnss_time:
                self.fix_msg.header.stamp = self.time_gnss
            else:
                self.fix_msg.header.stamp = self.time_io
            ### assign fix msg: frame id
            self.fix_msg.header.frame_id = self.gps_frame_id

            if self.valid_fix:
                ### assign vel msg: header
                if self.time_gnss_ready and self.use_gnss_time:
                    self.vel_msg.header.stamp = self.time_gnss
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

            ### assign heading msg: header
            if self.time_gnss_ready and self.use_gnss_time:
                self.heading_msg.header.stamp = self.time_gnss
            else:
                self.heading_msg.header.stamp = self.time_io            
            self.heading_msg.header.frame_id = self.heading_frame_id
            ### assign heading msg: pose
            self.heading_msg.pose.pose.orientation = self.rpy_to_quaternion(
                0,0, data['heading'] * math.pi / 180.0)
            
            ### publish heading
            self.heading_pub.publish(self.heading_msg)

            ### This the last sentence, we can set to "not_ready"
            self.time_gnss_ready = False
            self.time_io_ready = False

        ### publish the NMEA
        self.send_sentence(nmea_string)
        
    def close(self):
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