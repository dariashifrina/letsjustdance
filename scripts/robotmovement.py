#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Point
import moveit_commander
import math
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from operator import itemgetter
import graph

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

class RobotMovement:

        def __init__(self):
            #Once everything is initialized will set to true
            self.initialized = False

            #set up Subscriber and Publishers
            #TODO link correct function for Sub
            # # set up ROS / OpenCV bridge
            self.bridge = cv_bridge.CvBridge()

            self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
            self.navigator = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            rospy.Subscriber("/scan", LaserScan, self.process_scan)

            self.odometry = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
            self.position = None
            rospy.sleep(1)
            self.initialized = True
            self.navigate_graph([[0,0], [0,3], [0,6], [-3,3]])



        def image_callback(self, msg):

            self.view = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        def odometry_callback(self, data):
            self.position = data.pose.pose

        #helper function for rotating bot. used to survey the numbers on the blocks.
        def turn_around(self, angle):
            relative_angle = (angle * math.pi) / 180
            angular_speed = 0.2
            twister = Twist()
            twister.angular.z = -abs(angular_speed)
            self.navigator.publish(twister)
            current_angle = 0
            firstTime = rospy.Time.now().to_sec()
            while current_angle < relative_angle:
                curTime = rospy.Time.now().to_sec() - firstTime
                current_angle = angular_speed*(curTime)
            twister.angular.z = 0
            self.navigator.publish(twister)

        def travel(self, x, y):
            while True:
                goal = Point()
                speed = Twist()
                goal.x = x
                goal.y = y
                rot_q = self.position.orientation
                (roll, pitch,  theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
                inc_x = goal.x - self.position.position.x
                inc_y = goal.y - self.position.position.y

                angle_to_goal = math.atan2(inc_y, inc_x)
                distance = pow((pow(inc_x, 2) + pow(inc_y,2)), 0.5)
                if distance < 0.1:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    self.navigator.publish(speed)
                    break
                if abs(angle_to_goal- theta) > 0.3:
                    speed.linear.x = 0.0
                    prop_control = 0.3
                    if((angle_to_goal - theta) > 3.5):
                        speed.angular.z = prop_control * -1
                    elif((angle_to_goal - theta) > -1.7 and (angle_to_goal - theta) < 0):
                        speed.angular.z = prop_control * -1
                    else:
                        speed.angular.z = prop_control
                else:
                    speed.linear.x = 0.5
                    speed.angular.z = 0.0

                self.navigator.publish(speed)

        #takes in a list like: [[0,0], [0,3], [0,6], [-3,3]] and navigates thru the items one at a time
        def navigate_graph(self, graphlist):
            for pair in graphlist:
                self.travel(pair[0],pair[1])
        #publish laser scan data to self.directly_ahead node. used in cv dumbbell algorithms to approximate best position for pickup
        def process_scan(self, data):
            self.directly_ahead = data.ranges[0]

        #call this to make robot stoop and open grasp. ready to grip dumbbell -> needs fixing. it doesnt pick it up well...
        def run(self):
            rospy.spin()

if __name__ == '__main__':

        example_nodes = [(0,0),(3,0),(0,3),(3,3), (0,6)]
        g = graph.Graph(example_nodes)
        print(g.plan_path((0,0), (0,6)))
        rospy.init_node('robotmovement')
        robot_movement = RobotMovement()
        robot_movement.run()
