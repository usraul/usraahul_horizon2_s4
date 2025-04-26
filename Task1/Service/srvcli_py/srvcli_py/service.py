#!/usr/bin/env python3

import rclpy
import time
import math
from srv_pkg.srv import Adval
from rclpy.node import Node

class AngleConversion(Node):
    def __init__(self):
        super().__init__('angle_conversion_service')
        self.srv = self.create_service(Adval, 'angle_conversion', self.angle_conversion_callback)

    def angle_conversion_callback(self,request, response):
        a_values = []
        for i in range(len(request.angle_input)):
            a_values.append(request.angle_input[i].data)
        
        result = []
        Ax = a_values[0]
        Ay = a_values[1]
        Az = a_values[2]
        result.append(math.atan(Ax / math.sqrt(Ay**2 + Az**2)))
        result.append(Ay/Az)

        for i in range(len(result)):
            response.angle_output[i].data = result[i]

        time.sleep(2)
        print("Conversion performed")
        return response
    
def main(args=None):
    rclpy.init(args=args)
    conversion=AngleConversion()
    rclpy.spin(conversion)
    rclpy.shutdown()

if __name__ == '__main___':
    main()