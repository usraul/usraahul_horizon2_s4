#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from srv_pkg.srv import Adval

class AngleConversionClient(Node):
    def __init__(self):
        super().__init__('angle_conversion_client')
        self.cli = self.create_client(Adval, 'angle_conversion')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again ....')

    def send_request(self, a_values):
        req = Adval.Request()
        for i in range(len(a_values)):
            req.angle_input[i].data = a_values[i]

        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()

def main(args=None):
        rclpy.init(args=args)
        conversion_client = AngleConversionClient()
        
        angle_values = [0.12, 9.80, 0.34]

        response = conversion_client.send_request(angle_values)
        print("response: " , response)

        conversion_client.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
     main()