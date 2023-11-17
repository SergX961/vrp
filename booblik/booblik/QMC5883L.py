from threading import Thread
import rclpy
from rclpy.node import Node
import time
import raspy_qmc5883l
from sensor_msgs.msg import Imu


class QMC5883LNode(Node):
    def __init__(self, name='QMC5883L'):
        super().__init__(name)
        Thread(target=self._readLoop, daemon=True).start()
        while 1:
            try:
                self.sensor = raspy_qmc5883l.QMC5883L()
                #TODO read from config? 
                self.sensor.calibration = [[1.0817261189833043, -0.06705906178799911, -485.7272567957916], 
                      [-0.06705906178799906, 1.0550242422352802, -2953.8769005789645], 
                      [0.0, 0.0, 1.0]]
                break
            except:
                print("Init Error. Try init again...")
                time.sleep(0.1)
        
        self.imu_ = self.create_publisher(
            Imu,
            '/booblik/sensors/imu/imu/data',
            10)
            

    def _readLoop(self):
        imu = Imu()
        while True:
            try:
                #TODO convert to quat?
                bearing = self.sensor.get_bearing()
                imu.header(bearing)
                print(bearing)
                self.imu_.publish(imu)
                time.sleep(0.1)
            except:
                print("Except: Reques error")

def main(args=None):
    rclpy.init(args=args)
    task = QMC5883LNode()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
