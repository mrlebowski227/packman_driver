#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

from opcua import ua, Server
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt32, Int16, Float32, Byte

STATE_LIST = ["NOT_READY", "STANDBY", "ENABLE", "EXECUTE", "FAULT", "LEFT_AXIS_ERROR", "RIGHT_AXIS_ERROR"]

class RosHandler(object):
    def __init__(self):
        self.setup_opcua_server()

        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=5)
        self.pub_status = rospy.Publisher('/robot_status', Int16, queue_size=5)
        self.pub_batv = rospy.Publisher('/voltage', Float32, queue_size=5)
        self.pub_toolstate = rospy.Publisher('/tool_state', Int16, queue_size=5)
        
        rospy.Subscriber('/cmd_vel', Twist, self.updateTwist)
        rospy.Subscriber('/enable_drives', Bool, self.updateEnable)
        rospy.Subscriber('/toggle_tool', Int16, self.updateTool)
        
        self.twist_data = Twist()
        self.odom = Odometry()
        self.enable = Bool()
        self.tool_state = Int16()
        self.connection = Bool()
        self.voltage = Float32()
        self.robot_status = Int16()
        
        self.i = UInt32()

    def setup_opcua_server(self):
        # Setting up server
        rospy.loginfo("Setting up OPCUA server")
        server_string = "opc.tcp://%s:%s%s" % (rospy.get_param('~ip', "192.168.0.20"), rospy.get_param('~port', "5554"), rospy.get_param('~namespace', "/"))
        rospy.loginfo("OPCUA on: " + server_string)
        server = Server()
        server.set_endpoint(server_string)

        # Creating objects for odometry, twist and states
        uri_odom = "http://packman.ros/odom"
        uri_twist = "http://packman.ros/twist"
        uri_state = "http://packman.ros/state"

        idx_odom = server.register_namespace(uri_odom)
        idx_twist = server.register_namespace(uri_twist)
        idx_state = server.register_namespace(uri_state)

        opc_objects = server.get_objects_node()

        odom_object = opc_objects.add_object(idx_odom, "Odometry")
        twist_object = opc_objects.add_object(idx_twist, "Twist")
        state_object = opc_objects.add_object(idx_state, "States")

        # Setting variables which will be handled by middleman
        self.var_state_robot_state = state_object.add_variable(idx_state, "state_robot_state", 0)
        self.var_state_tool_state = state_object.add_variable(idx_state, "state_tool_state", 0)
        self.var_state_bat_volt = state_object.add_variable(idx_state, "state_bat_volt", 0.0)

        self.var_odom_pose_position_x = odom_object.add_variable(idx_odom, "odom_position_x", 0.0)
        self.var_odom_pose_position_y = odom_object.add_variable(idx_odom, "odom_position_y", 0.0)
        self.var_odom_pose_position_z = odom_object.add_variable(idx_odom, "odom_position_z", 0.0)

        self.var_odom_pose_orientation_x = odom_object.add_variable(idx_odom, "odom_orientation_x", 0.0)
        self.var_odom_pose_orientation_y = odom_object.add_variable(idx_odom, "odom_orientation_y", 0.0)
        self.var_odom_pose_orientation_z = odom_object.add_variable(idx_odom, "odom_orientation_z", 0.0)
        
        self.var_odom_twist_linear_x = odom_object.add_variable(idx_odom, "odom_twist_x", 0.0)
        self.var_odom_twist_linear_y = odom_object.add_variable(idx_odom, "odom_twist_y", 0.0)
        self.var_odom_twist_angular_z = odom_object.add_variable(idx_odom, "odom_twist_z", 0.0)

        # Setting all writable for middleman
        self.var_state_robot_state.set_writable()
        self.var_state_tool_state.set_writable()
        self.var_state_bat_volt.set_writable()

        self.var_odom_pose_position_x.set_writable()
        self.var_odom_pose_position_y.set_writable()
        self.var_odom_pose_position_z.set_writable()
        
        self.var_odom_pose_orientation_x.set_writable()
        self.var_odom_pose_orientation_y.set_writable()
        self.var_odom_pose_orientation_z.set_writable()
        
        self.var_odom_twist_linear_x.set_writable()
        self.var_odom_twist_linear_y.set_writable()
        self.var_odom_twist_angular_z.set_writable()

        # Setting up all methods for the driver to send
        self.var_twist_linear_x = twist_object.add_variable(idx_twist, "twist_linear_x", 0.0)
        self.var_twist_angular_z = twist_object.add_variable(idx_twist, "twist_angular_z", 0.0)

        self.var_state_toggle_tool = state_object.add_variable(idx_state, "state_toggle_tool", 0)
        self.var_state_enable_drives = state_object.add_variable(idx_state, "state_enable_drives", 0)

        server.start()
        
        rospy.loginfo("Ready")
        
    def updateTwist(self, data):
        if self.enable != 1:
            rospy.logerr("Robot actuators not enabled, ignoring twist command!")
        else:
            self.var_twist_linear_x.set_value(data.linear.x)
            self.var_twist_angular_z.set_value(data.angular.z)
    
    def updateEnable(self, data):
        self.enable = data.data
        self.var_state_enable_drives.set_value(data.data)
    
    def updateTool(self, data):
        self.var_state_toggle_tool.set_value(data.data)
    
    def doMain(self):
        # Update odom data
        self.odom.header.seq = self.i
        self.i.data += 1
        
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = rospy.get_param('~odom_frame', 'odom')
        self.odom.child_frame_id = rospy.get_param('~base_frame', 'base_footprint')
        
        self.odom.pose.pose.position.x = float(self.var_odom_pose_position_x.get_value())
        self.odom.pose.pose.position.y = float(self.var_odom_pose_position_y.get_value())
        self.odom.pose.pose.orientation.z = float(self.var_odom_pose_orientation_z.get_value())
        
        self.odom.twist.twist.linear.x = float(self.var_odom_twist_linear_x.get_value())
        self.odom.twist.twist.linear.y = float(self.var_odom_twist_linear_y.get_value())
        self.odom.twist.twist.angular.z = float(self.var_odom_twist_angular_z.get_value())
        
        # Update statuses
        self.tool_state.data = int(self.var_state_tool_state.get_value())
        self.voltage.data = float(self.var_state_bat_volt.get_value())
        self.robot_status.data = int(self.var_state_robot_state.get_value())
        
        # Publish all
        self.pub_odom.publish(self.odom)
        self.pub_status.publish(self.robot_status)
        self.pub_batv.publish(self.voltage)


if __name__ == '__main__':
    rospy.init_node('packman_node', anonymous=False)

    ros = RosHandler()
    
    r = rospy.Rate(int(rospy.get_param('~rate', 50)))
    
    while(not rospy.is_shutdown()):
        ros.doMain()
        r.sleep()