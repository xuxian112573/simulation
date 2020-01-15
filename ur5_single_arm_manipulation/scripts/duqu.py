import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import control_msgs.JointTrajectoryControllerState 


def get_scan(self):
        msg = rospy.wait_for_message("arm_controller/state", State)
        self.scan_filter = []
        self.scan_filter.append(msg.actual.positions)
                
                



    

if __name__ == '__main__':
    rospy.init_node('turtlebot_scan')
    get_scan()
