#!/usr/bin/env python

import rospy
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from mavros_msgs.msg import PositionTarget, State, HomePosition

def ego_to_mavros():
    rospy.init_node("position_command_to_mavros")

    # Publisher for MAVROS PositionTarget commands
    mavros_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
    pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    vel_unst = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
    home_pub = rospy.Publisher('/mavros/home_position/set', HomePosition, queue_size=10)


    def position_command_callback(msg):
        """
        Callback function to process PositionCommand messages and convert them to MAVROS-compatible commands.
        """
        # Extract position, velocity, and acceleration from PositionCommand
        header = msg.header
        header.stamp.secs = 0
        header.stamp.nsecs = 0
        header.frame_id = 'map'
        position = msg.position
        velocity = msg.velocity
        acceleration = msg.acceleration
        yaw = msg.yaw
        yaw_rate = msg.yaw_dot


        # Create a PositionTarget message
        mavros_cmd = PositionTarget()
        pose = PoseStamped()
        vel = TwistStamped()
        home = HomePosition()
        unstvel = Twist()

        # Set frame and type mask for velocity control with yaw
        mavros_cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Assign velocity values and yaw
        mavros_cmd.header = header
        #mavros_cmd.header.frame_id = "base_link"
        mavros_cmd.position = position
        mavros_cmd.velocity = velocity
        mavros_cmd.acceleration_or_force = acceleration
        mavros_cmd.yaw = yaw
        mavros_cmd.yaw_rate = yaw_rate

        pose.header = header
        #pose.header.frame_id = "base_link"
        pose.pose.position.x = position.x
        pose.pose.position.y = position.y
        pose.pose.position.z = position.z

        vel.header = header
        #vel.header.frame_id = "base_link"
        vel.twist.linear.x = velocity.x
        vel.twist.linear.y = velocity.y
        vel.twist.linear.z = velocity.z

        unstvel.linear.x = velocity.x
        unstvel.linear.y = velocity.y
        unstvel.linear.z = velocity.z

        home.header = header
        #home.header.frame_id = "map"
        home.geo.latitude = 0
        home.geo.longitude = 0
        home.geo.altitude = 0
        home.position.x = 0
        home.position.y = 0
        home.position.z = 0
        home.orientation.x = 0
        home.orientation.y = 0
        home.orientation.z = 0
        home.orientation.w =0
        home.approach.x = 0
        home.approach.y = 0
        home.approach.z = 0

        # Publish the converted command
        mavros_pub.publish(mavros_cmd)
        #pos_pub.publish(pose)
        #vel_pub.publish(vel)
        #home_pub.publish(home)


    # Subscriber to PositionCommand topicthe
    rospy.Subscriber('/position_cmd', PositionCommand, position_command_callback)

    rospy.spin()



if __name__ == "__main__":
    try:
        ego_to_mavros()
    except rospy.ROSInterruptException:
        pass


