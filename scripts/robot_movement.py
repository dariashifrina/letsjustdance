#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


example_nodes = [(0,0), (0,1), (1,0), (1,1)]

class Graph:
        def __init__(self, nodes):
                # this makes an undirected graph, we could do a directed one too?
                # take in a list of nodes stored as tuples
                self.edges = {} # list of edges to be used in bfs
             
                for node in nodes:
                        # for movement purposes, we want edges[node] = [left_node, right_node, up_node, down_node]
                        # edges[node][i] = -1 if movement in that direction is impossible

                        new = [-1]*4

                        for point in nodes:
                                if node[0] in point or node[1] in point:
                                        if point[0] == node[0]-1 and point[1] == node[1]:
                                                new[0] = point
                                        if point[0] == node[0] + 1 and point[1] == node[1]:
                                                new[1] = point
                                        if point[1] == node[1]+1 and point[0] == node[0]:
                                                new[2] = point
                                        if point[1] == node[1]-1 and point[0] == node[0]:
                                                new[3] = point
                        self.edges[node] = new

        def remove_edge(self, src, goal):
                # method to remove edge that is no longer navigable(i.e traffic cone blocking, etc)
                # remove src -> goal edge:
                direction = self.edges[src].index(goal)
                self.edges[src][direction] = -1

                direction = self.edges[goal].index(src)
                self.edges[goal][direction] = -1

        #def get_path(self, src, goal):
        



class Follower:

        def __init__(self):

                self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
                lin = Vector3(0.03,0,0)
                ang = Vector3()
                self.twist = Twist(linear=lin,angular=ang)
        
                # set up ROS / OpenCV bridge
                self.bridge = cv_bridge.CvBridge()

                # initalize the debugging window
                cv2.namedWindow("window", 1)

                # subscribe to the robot's RGB camera data stream
                self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)

        def image_callback(self, msg):

                # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                # TODO: define the upper and lower bounds for what should be considered 'yellow'
                lower_yellow = numpy.array([20, 100, 100]) #TODO
                upper_yellow = numpy.array([40, 255, 255]) #TODO
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                # this erases all pixels that aren't yellow
                h, w, d = image.shape
                search_top = int(3*h/4)
                search_bot = int(3*h/4 + 20)
                mask[0:search_top, 0:w] = 0
                mask[search_bot:h, 0:w] = 0

                # using moments() function, the center of the yellow pixels is determined
                M = cv2.moments(mask)
                # if there are any yellow pixels found
                if M['m00'] > 0:
                        # center of the yellow pixels in the image
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])

                        # a red circle is visualized in the debugging window to indicate
                        # the center point of the yellow pixels
                        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                        # TODO: based on the location of the line (approximated
                        #       by the center of the yellow pixels), implement
                        #       proportional control to have the robot follow
                        #       the yellow line
                        e = cx - w/2
                        if e<-10 or e>10:
                                self.twist.angular.z = -0.001*e
                                self.twist_pub.publish(self.twist)

                        

                # shows the debugging window
                cv2.imshow("window", image)
                cv2.waitKey(3)


        def run(self):
                rospy.spin()
                
if __name__ == '__main__':
        rospy.init_node('line_follower')
        #follower = Follower()
        #follower.run()