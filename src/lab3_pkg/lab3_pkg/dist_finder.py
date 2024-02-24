import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math
#from lab3_pkg import pid_input
#from race.msg import pid_input
import numpy as np

class distFinder(Node):
    def __init__(self):
        super().__init__("distFinder_Node")
        self.get_logger().info("Dist finder is running!")
        self.subscribeLaser_ = self.create_subscription(LaserScan,"/scan",self.scan_Callback,10)
        self.publishRace_ = self.create_publisher(Float32,"error",10)
        self.angle_min = 0
        self.angle_max = 0
        self.angle_increment = 0
        self.range = []
    
    #Input: data: Lidar Scan data 
    #       theta: the angle at which the distance is require
    #Output:distance of scan at angle theta
    def getRange (self,data: LaserScan,theta=50) ->float:
        self.angle_min = data.angle_min * (180.0 / np.pi)
        self.angle_max = data.angle_max * (180.0 / np.pi)
        self.angle_increment = data.angle_increment * (180.0 / np.pi)
        self.range = data.ranges
        # Calculate the index corresponding to the given angle theta
        index = int((theta - self.angle_min) / self.angle_increment)

        # Return the distance at the calculated index
        self.get_logger().info(f"index = {index}, the length of the array = {len(self.range)}, the min angle = {self.angle_min},the angle increment = {self.angle_increment} and the max angle = {self.angle_max}")
        if index<= len(self.range):
            return self.range[index]
        else:
            return 0.0


    def scan_Callback(self,data):#,msg:pid_input):
        theta = 50
        a = self.getRange(data,50)
        b = self.getRange(data,0)
        alpha = np.arctan(((a * np.cos(theta)) - b) / (a * np.sin(theta))) #chat gpt #will replace np.cos(90)) with math.cos(90))
        Dt = b * np.cos(alpha) #AB
        
        L = 1 #Lookhead Distance
        Dt_future = Dt + L * np.sin(alpha)
        self.get_logger().info(f"Dt = {Dt}, DT_future = {Dt_future}")
        error_msg = Float32()
        error_msg.data = 1 - Dt_future
        #msg.pid_vel = vel
        self.publishRace_.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    dist_finder = distFinder()
    rclpy.spin(dist_finder)
    dist_finder.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
