#!/usr/bin/env python3

import serial
import time
import math
from datetime import datetime, timezone

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3, TransformStamped
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Sentence
import tf
import tf.transformations
import tf2_ros

class UnicoreDriver:
    def __init__(self):

        self.get_params()

        self.setup_ros()

        self.setup_serial()

        ### some init
        self.time_io = rospy.Time.now()
        self.time_io_ready = False
        self.time_gps = rospy.Time.now()
        self.time_gps_ready = False

    def get_params(self):

        ### configuration
        self.serial_port = rospy.get_param('~device', '/dev/ttyUSB0')
        self.serial_baudrate = rospy.get_param('~baudrate', 115200)

        self.use_gps_time = rospy.get_param('~use_gps_time', False)

        ### topics
        self.heading_topic = rospy.get_param('~heading_topic', 'unicore/heading')
        self.heading_odom_topic = rospy.get_param('~heading_odom_topic', 'unicore/heading_odom')
        self.nmea_topic = rospy.get_param('~nmea_topic', 'unicore/nmea')

        ### others
        self.test_value = float(rospy.get_param('~test_value', '0.1'))

    def setup_ros(self):
        """
        This function is used for ros setup: publisher, subscribers, service, timer and more
        """

        ### publihser
        self.heading_pub = rospy.Publisher(self.heading_topic, Float32, queue_size=10)
        self.heading_odom_pub = rospy.Publisher(self.heading_odom_topic, Odometry, queue_size=10)
        self.nmea_pub = rospy.Publisher(self.nmea_topic, Sentence, queue_size=10)

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

    def read_nmea(self):
        """
        This function will read a line from serial stream data
        """

        if self.ser.in_waiting > 0:
            ### read one line
            data = self.ser.readline().decode('utf-8').strip()

            ### save the IO timestamp only if this is the first sentence
            if not self.time_gps_ready and not self.time_io_ready:
                self.time_io_ready = True
                self.time_io = rospy.Time.now()
                # print("The real IO Time now: %s" % self.time_io.to_sec())

            # print("IO Time now: %s" % rospy.Time.now().to_sec())

            return data
        else:
            return False

    def check_nmea(self, nmea_string):
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

                #publish the nmea
                # print(nmea_string)

                # Further split the data part by comma
                data_fields = data[1:].split(',')

                # Extract the header and data fields
                header = '$' + data_fields[0]
                data = data_fields[1:]
                return header, data
            else:
                print("Checksum check failed")
                return False, False
        else:
            # print("Invalid NMEA string")
            return False, False
        
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
            
    def parse_time(self, data):
        """
        This function will parse the "GNRMC" message
        """        

        # get second
        hhmmss_ss = str(data[0])
        hhmmss, fractional_seconds = hhmmss_ss.split(".")

        # get date
        old_date = data[8]
        new_date = "20" + str(old_date[4:6]) + str(old_date[2:4]) + str(old_date[0:2])
        
        # Combine the date and time into a single string (without fractional seconds)
        date_time_str = new_date + hhmmss

        # Convert the combined string into a datetime object
        date_time_obj = datetime.strptime(date_time_str, '%Y%m%d%H%M%S')

        # Add the fractional seconds to the datetime object
        date_time_obj = date_time_obj.replace(microsecond=int(fractional_seconds) * 10000)

        # Convert the datetime object to Unix timestamp (including fractional seconds)
        gps_unix_time = date_time_obj.timestamp()

        # save this time
        self.time_gps = rospy.Time.from_sec(gps_unix_time)
        self.time_gps_ready = True

        # print("GPS Time now: %s sec" % gps_unix_time)

    def parse_heading(self, data):
        """
        This function will parse the "GNHPR" message
        """
        # parse the data
        rtk_heading = float(data[1])
        rtk_pitch = float(data[2])
        rtk_qf = float(data[4])
        rtk_sat_n = float(data[5])      

        ### publish the heading odometry message
        heading_odom_msg = Odometry()

        # determine which timestamp to use
        if self.time_gps_ready and self.use_gps_time:
            heading_odom_msg.header.stamp = self.time_gps

        else:
            heading_odom_msg.header.stamp = self.time_io

        heading_odom_msg.header.frame_id = "world_ned"
        heading_odom_msg.child_frame_id = "unicore_gps"
        heading_odom_msg.pose.pose.orientation = self.rpy_to_quaternion(0,0, rtk_heading * math.pi / 180.0)
        self.heading_odom_pub.publish(heading_odom_msg)

        ### publish the heading message
        heading_msg = Float32()
        heading_msg.data = rtk_heading

        # publish if it has Subscribers
        if self.heading_pub.get_num_connections() > 0:
            self.heading_pub.publish(heading_msg)

        # GNHPR is the last sentence, we can set to "not_ready"
        self.time_gps_ready = False
        self.time_io_ready = False

    def send_sentence(self, data):

        msg = Sentence()
        msg.header.stamp = self.time_io
        msg.header.frame_id = "unicore_gps"
        msg.sentence = data
        self.nmea_pub.publish(msg)

    def process(self, event):

        # read one line
        nmea_string = self.read_nmea()
        if(nmea_string is not False):

            header,cdata = self.check_nmea(nmea_string)

            # parse the header and data items
            if header == '$GNRMC':
                self.parse_time(cdata)
            
            # parse the heading message
            if header == '$GNHPR':
                self.parse_heading(cdata)
            # send as ros nmea sentence
            else:
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