import serial
import time
from datetime import datetime, timezone


class UnicoreGPS:

    def __init__(self):
        try:
            # port_name = '/dev/tty.usbserial-DU0D6GOD'
            port_name = '/dev/ttyUSB0'
            baud_rate = 115200
            self.ser = serial.Serial(port_name, baud_rate, timeout=1)
            self.ser.flush()  # Clear the input and output buffers
            print(f"Connected to {port_name}")

        except serial.SerialException as e:
            print(f"Error: Could not open serial port {port_name}. Reason: {e}")
            # Handle the error, for example, exit the program
            exit(1)

    #read a line
    def read_nmea(self):
        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode('utf-8').strip()  # Read the data
            # print(data)
            return data
        else:
            return False

    #check nmea
    def check_nmea(self, nmea_string):
        nmea_string = nmea_string.replace('\r', '').replace('\n', '') #remove any \r\n in the string
        if nmea_string.startswith('$'):
            # Split the string at the asterisk to isolate the checksum
            data, checksum = nmea_string.split('*')
            #checksum check
            c_checksum = f"{self.calculate_checksum(data):02X}"
            if(checksum == c_checksum):

                #publish the nmea
                print(nmea_string)

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
        # Calculate the checksum for the NMEA string
        checksum = 0
        for char in nmea_string[1:]:  # Skip the starting '$'
            checksum ^= ord(char)
        return checksum
    
    def close(self):
        self.ser.close()

    ##parse unicore nmea
    def parse_unicore_nmea(self, data):
        # print(data)
        hhmmss_ss = str(data[0])

                
        current_date = datetime.now(timezone.utc).strftime('%Y%m%d')

        # Split the time part to extract the fractional seconds
        hhmmss, fractional_seconds = hhmmss_ss.split(".")

        # Combine the date and time into a single string (without fractional seconds)
        date_time_str = current_date + hhmmss

        # Convert the combined string into a datetime object
        date_time_obj = datetime.strptime(date_time_str, '%Y%m%d%H%M%S')

        # Add the fractional seconds to the datetime object
        date_time_obj = date_time_obj.replace(microsecond=int(fractional_seconds) * 10000)

        # Convert the datetime object to Unix timestamp (including fractional seconds)
        rtk_unix_time = date_time_obj.timestamp()
        

        rtk_heading = float(data[1])
        rtk_pitch = float(data[2])
        rtk_qf = float(data[4])
        rtk_sat_n = float(data[5])

        # print("unix time=" , rtk_unix_time, "sec")
        # print("heading=" , rtk_heading, "deg")

##################################################################
gps =UnicoreGPS()

try:
    while True:
        nmea_string = gps.read_nmea()
        if(nmea_string is not False):
            header,cdata = gps.check_nmea(nmea_string)
            # print(header)
            if header =='$GNHPR':
                gps.parse_unicore_nmea(cdata)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Script terminated by user (Ctrl+C).")
    gps.close()
