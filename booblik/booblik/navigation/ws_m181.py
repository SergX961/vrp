# import troykahat
import struct
from dataclasses import dataclass
import serial
from threading import Thread
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
import pynmea2
import io

format_time = 'BBBB'
format_latlon = "=BBiic"
format_angle = "=BBhhhhc"
format_quat = '=BBhhhhc'


def parseLanLon(packet: bytearray):
    s = struct.unpack(format_latlon, packet)
    lon = s[2]//1e7 + (s[2] % 1e7)/1e5/60
    lat = s[3]//1e7 + (s[3] % 1e7)/1e5/60
    print(lat, lon)
    return lat, lon


def parseAngle(packet: bytearray):
    s = struct.unpack(format_angle, packet)
    roll = s[2] / 32768 * 180
    pitch = s[3] / 32768 * 180
    yaw = s[4] / 32768 * 180
    return roll, pitch, yaw


def parseQuat(packet: bytearray):
    s = struct.unpack(format_angle, packet)
    q1 = s[2] / 32768
    q2 = s[3] / 32768
    q3 = s[4] / 32768
    q0 = s[5] / 32768
    return q0, q1, q2, q3


@dataclass
class GpsConfig:
    port: str
    baudrate: int


class GpsImuNode(Node):
    config: GpsConfig

    def __init__(self, name='ws_m181'):
        super().__init__(name)
        self.config = GpsConfig('/dev/ttyUSB0', 115200)
        self.nav_ = self.create_publisher(
            NavSatFix,
            '/booblik/sensors/gps/navsat/fix',
            10)
        
        self.imu_ = self.create_publisher(
            Imu,
            '/booblik/sensors/imu/imu/data',
            10)
        self.imu_

        Thread(target=self._readLoop, daemon=True).start()


    def _readLoop(self):
        ser = serial.Serial(
            self.config.port,
            self.config.baudrate,
            timeout=3
        )  # open serial port
        while True:
            try:
                raw_data = ser.readline().decode()#read data
                data = pynmea2.parse(raw_data)#parse data
                
                #match input nmea packet
                if data.sentence_type == "GGA":
                    nav = NavSatFix()
                    nav.latitude = data.latitude
                    nav.longitude = data.longitude
                    self.nav_.publish(nav)
                    print(data.latitude, " ", data.longitude)

            except Exception as e:
                pass


    def parsePacket(self, packet: bytearray):
        if packet[1] == 0x59:
            q0, q1, q2, q3 = parseQuat(packet)
            imu = Imu()
            imu.orientation.x = q0
            imu.orientation.y = q1
            imu.orientation.z = q2
            imu.orientation.w = q3
            self.imu_.publish(imu)
        elif packet[1] == 0x57:
            lat, lon = parseLanLon(packet)
            nav = NavSatFix()
            nav.latitude = lat
            nav.longitude = lon
            self.nav_.publish(nav)


def main(args=None):
    rclpy.init(args=args)
    task = GpsImuNode()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
