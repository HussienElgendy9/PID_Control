import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
#from race.msg import pid_input
import numpy as np
import time

class control(Node):
    def __init__(self):
        super().__init__("control_Node")
        self.get_logger().info("control_Node is running!")
        self.create_subscription(Float32,"error",self.scan_callback,10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped,"/drive",10)
        self.kp = 5.0
        self.kd = 0.09
        self.servo_offset = 18.5
        self.prev_error = 0.0
        self.vel_input = 25.0
        self.prev_time = time.time() #chat gpt
        #self.angle = 0.0
        self.error = 0

    def scan_callback(self,data:Float32):
        self.error =data.data
        self.control()

    def control(self):
        #relative_velocity = self.odom_linear_velocity
            current_time = time.time() 
            dt = current_time - self.prev_time 
            self.prev_time = current_time 
            d_error = (self.error-self.prev_error)/dt
            self.prev_error = self.error

            speed = 0
            angle = ((self.kp * self.error)+(self.kd * d_error)) * (180.0 / np.pi)
            if angle >= 0 and angle < 10:
                speed =1.5
                self.vel_input = speed
            elif angle >= 10 and angle < 20:
                speed = 1.0
                self.vel_input = speed
            else:
                speed = 0.5 
                self.vel_input = speed
            
            msg = AckermannDriveStamped()
            msg.drive.speed = self.vel_input
            msg.drive._steering_angle = angle
            self.publisher_.publish(msg)
            self.get_logger().info(f"angle = {angle}")


def main(args=None):
    rclpy.init(args=args)
    Node = control()
    rclpy.spin(Node)
    control.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
