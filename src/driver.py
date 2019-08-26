#!/usr/bin/env python

import rospy
import json
import socket
import sys

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, UInt32, Int16, Float32, Byte

STATE_LIST = ["NOT_READY", "STANDBY", "ENABLE", "EXECUTE", "FAULT", "LEFT_AXIS_ERROR", "RIGHT_AXIS_ERROR"]

class RosHandler(object):
    def __init__(self, json_class, socket):
        self.json_class = json_class
        self.socket = socket
        
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=5)
        self.pub_status = rospy.Publisher('/robot_status', Byte, queue_size=5)
        self.pub_batv = rospy.Publisher('/voltage', Float32, queue_size=5)
        self.pub_toolstate = rospy.Publisher('/tool_state', Int16, queue_size=5)
        self.pub_connection = rospy.Publisher('/connection', Bool, queue_size=5)
        
        rospy.Subscriber('/cmd_vel', Twist, self.updateTwist)
        rospy.Subscriber('/enable', Bool, self.updateEnable)
        rospy.Subscriber('/request_tool_state', Int16, self.updateTool)
        
        self.twist_data = Twist()
        self.odom = Odometry()
        self.enable = Bool()
        self.tool_state = Int16()
        self.connection = Bool()
        self.voltage = Float32()
        self.robot_status = Int16()
        
        self.i = UInt32()
        
    def updateTwist(self, data):
        self.twist_data.linear.x = data.linear.x
        self.twist_data.linear.y = 0
        self.twist_data.linear.z = 0
        
        self.twist_data.angular.x = 0
        self.twist_data.angular.y = 0
        self.twist_data.angular.z = data.angular.z
    
    def updateEnable(self, data):
        self.enable = data.data
    
    def updateTool(self, data):
        self.tool_state = data.data
    
    def doMain(self):
        # Update the JSON object with current commands
        self.json_class.updateJsonSend(self.twist_data.linear.x, 
                                       self.twist_data.angular.z,
                                       self.enable.data,
                                       self.tool_state.data)
        # Send JSON to PLC                            
        self.socket.sendData(json.dumps(self.json_class.json_send, sort_keys=True))
        # Receive response of PLC
        self.socket.receiveData()
        
        # Update odom data
        self.odom.Header.uint32.seq = self.i
        self.i.data += self.i.data
        
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = rospy.get_param('~odom_frame', 'odom')
        self.odom.child_frame_id = rospy.get_param('~base_frame', 'base_footprint')
        
        self.odom.twist.twist.linear.x = float(self.socket.json_receive["odom"]["x"])
        self.odom.twist.twist.linear.y = float(self.socket.json_receive["odom"]["y"])
        self.odom.twist.twist.angular.z = float(self.socket.json_receive["odom"]["z"])
        
        # Update statuses
        self.tool_state.data = int(self.socket.json_receive["status_tool_state"])
        self.connection.data = bool(self.socket.json_receive["status_connection_alive"])
        self.voltage.data = float(self.socket.json_receive["status_battery_v"])
        self.robot_status.data = bytes(self.socket.json_receive["status_robot_state"])
        
        # Publish all
        self.pub_odom.publish(self.odom)
        self.pub_status.publish(self.robot_status)
        self.pub_batv.publsih(self.voltage)
        self.pub_connection.publish(self.connection)
 
       
class JSONHanlding(object):
    def __init__(self):
        self.json_send = {
                            "twist_x": 0.0,
                            "twist_z": 0.0,
                            "enable": 0,
                            "tool_state": 0
                         } 
    
    def updateJsonSend(self, a, b, c, d):
        self.json_send["twist_x"] = a
        self.json_send["twist_y"] = b
        self.json_send["enable"] = c
        self.json_send["tool_state"] = d
        
    def updateJsonReceived(self, string):
        self.json_receive = json.load(string)
                                        
                                        
class Socket(object):
    def __init__(self, ip, port):
        rospy.loginfo("Connecting to packman.")
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((ip, port))
        rospy.loginfo("Connected!")
        
        self.json_receive = {
                                "odom_x": 0,
                                "odom_y": 0,
                                "odom_z": 0,
                                "status_connection_alive": 0,
                                "status_robot_state": 0,
                                "status_battery_v": 0,
                                "status_tool_state": 0
                            } 
       
    def sendData(self, string):
        self.client.sendall(string.encode('utf-8'))
        
    def receiveData(self):
        self.json_receive = json.load(self.client.recv(4096))
       
       
if __name__ == '__main__':
    rospy.init_node('packman_node', anonymous=False)
    
    ros = RosHandler(JSONHanlding(), Socket(rospy.get_param('~ip', '192.168.1.1'), int(rospy.get_param('~port', 55555))))
    
    r = rospy.Rate(int(rospy.get_param('~rate', 50)))
    
    while(not rospy.is_shutdown()):
        ros.doMain()
        r.sleep()