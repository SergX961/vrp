import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from threading import Thread
import time
import libscrc
import struct


class Sensor(Node):
    sensor_idx = 0x6
    rqstHum = [sensor_idx, 0x03, 0x00, 0x00, 0x00, 0x06, 0xFF, 0xFF]


    def __init__(self):
        super().__init__('HT_sensor')
        self.tx_ = self.create_publisher(
            UInt8MultiArray,
            '/booblik/rs485Rx',
            10
        )
        self.rx_ = self.create_subscription(
            UInt8MultiArray,
            '/booblik/rs485Tx',
            self.recieve_callback,
            10
        )

        self.sendThread = Thread(
            target=self.request_thread, daemon=True).start()


    def check_crc (self, data):
        size = len(data)
        if (size < 3):
            return False
        
        crc = libscrc.modbus(bytes(data[0:(size - 2)]))
        crcLow = crc & 0xFF
        crcHigh = crc >> 8
        if (crcLow == data[size - 2]) and (crcHigh == data[size - 1]):
            return True
        else:
            return False

    def check_idx (self, data):
        if data[0] == self.sensor_idx:
            return True
        else:
            return False


    def parce (self, data):
        try:
            oxygen_saturation = struct.unpack(">f", data[3:7])[0]
            oxygen_concentration = struct.unpack(">f", data[7:11])[0]
            temperature = struct.unpack(">f", data[11:15])[0]
            return (oxygen_saturation, oxygen_concentration, temperature)
        except:
            print("parce error")
            return (0.0, 0.0, 0.0)

    def recieve_callback(self, msg):
        data = msg.data

        if self.check_crc(data) == False:
            print("crc parse error")
            return

        if self.check_idx(data) == False:
            return
        
        print(self.parce(data))


    def get_rqst_data_msg (self):
        l = self.rqstHum
        crc = libscrc.modbus(bytes(l[0:6]))
        crcLow = crc & 0xFF
        crcHigh = crc >> 8
        l[6] = crcLow
        l[7] = crcHigh
        return l


    def request_thread (self):
        while True:
            request_message = self.get_rqst_data_msg()
            self.send_message(request_message)
            time.sleep(1)

    
    def send_message(self, message):
        try:
            msg = UInt8MultiArray()

            for item in message:
                msg.data.append(item)
            
            self.tx_.publish(msg)
        except Exception as e:
            print("Error send message: {e}")


def main(args=None):
    rclpy.init(args=args)
    sensor = Sensor()
    rclpy.spin(sensor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
