import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import socket


ethRS485_converter_address = "192.168.0.7"
ethRS485_converter_port = 26


class ETHRS485(Node):
    def __init__(self):
        super().__init__('rs485')
        
        self.rx_ = self.create_subscription(
            UInt8MultiArray,
            '/booblik/rs485Rx',
            self.request_callback,
            10
        )
        self.tx_ = self.create_publisher(
            UInt8MultiArray,
            '/booblik/rs485Tx',
            10
        )
        
        self.socket_ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_.settimeout(1)
        self.socket_.connect((ethRS485_converter_address, ethRS485_converter_port))
        print("Успешное подключение к серверу")


    def request_callback(self, data:UInt8MultiArray):
        try:
            self.socket_.sendall(data.data)
            data = self.socket_.recv(1024)
            self.tx_answer(list(data))
        except Exception as e:
            print("Request error {e}")
            print(e)

    
    def tx_answer (self, data):
        try:
            msg = UInt8MultiArray()
            
            for i in data:
                msg.data.append(i)

            self.tx_.publish(msg)
        except Exception as e:
            print("Error send message")
            print(e)
            self.tx_.publish(UInt8MultiArray())#return empty


def main(args=None):
    rclpy.init(args=args)
    task = ETHRS485()
    rclpy.spin(task)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
