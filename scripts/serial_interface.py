import serial

class SerialInterface:
    def __init__(self, port_name, baud_rate):
        self.ser = serial.Serial(port_name, baud_rate, timeout=1)  # Replace with your port and baud rate
        self.ser.flush()

    def send_nmea(self, data):
        message = data + '\r\n'
        self.ser.write(message.encode('utf-8'))

    def read_nmea(self):
        if self.ser.in_waiting > 0:
            data = self.ser.readline().decode('utf-8').strip()  # Read the data
            return data
        else:
            return False

    def close(self):
        self.ser.close()

    # def serial_receive_nmea(self)