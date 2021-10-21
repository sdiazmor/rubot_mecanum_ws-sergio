#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class nexus:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('nexus_control', anonymous=True)
        # Define Params
        self.x_goal = rospy.get_param("~x")
        self.y_goal = rospy.get_param("~y")
        self.f_goal = rospy.get_param("~f")
        self.q_goal = quaternion_from_euler(0,0,self.f_goal)
        
        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic '/odom'. self.update_pose is called
        # when a message of type Odometry is received.
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_odom)

        self.odom = Odometry()
        self.rate = rospy.Rate(10)

    def update_odom(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.odom = data
        self.x_pose = round(self.odom.pose.pose.position.x, 2)
        self.y_pose = round(self.odom.pose.pose.position.y, 2)
        self.z_pose = round(self.odom.pose.pose.position.z, 2)
        self.orientation_q = self.odom.pose.pose.orientation
        self.orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]
        self.rpw = euler_from_quaternion (self.orientation_list)
        self.yaw = self.rpw[2]
        
    def euclidean_distance(self, goal_odom):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_odom.pose.pose.position.x - self.x_pose), 2) +
                    pow((goal_odom.pose.pose.position.y - self.y_pose), 2))

    def linear_vel(self, goal_odom, constant=1):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_odom)

    def steering_angle(self, goal_odom):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_odom.pose.pose.position.y - self.y_pose, goal_odom.pose.pose.position.x - self.x_pose)

    def angular_vel(self, goal_odom, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_odom) - self.yaw)

    def move2pose(self):
        """Moves the turtle to the goal."""
        goal_odom = Odometry()

        # Get the input from the user.
        goal_odom.pose.pose.position.x = self.x_goal
        goal_odom.pose.pose.position.y = self.y_goal
        goal_odom.pose.pose.orientation.x = self.q_goal[0]
        goal_odom.pose.pose.orientation.y = self.q_goal[1]
        goal_odom.pose.pose.orientation.z= self.q_goal[2]
        goal_odom.pose.pose.orientation.w= self.q_goal[3]

        rospy.loginfo("goal : " + str(goal_odom))
        
        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.01

        vel_msg = Twist()

        while self.euclidean_distance(goal_odom) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_odom)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_odom)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        go = nexus()
        go.move2pose()
    except rospy.ROSInterruptException:
        pass