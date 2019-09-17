import rospy
from opcua import ua, Server


class RosHandler(object):
    def __init__(self, ip, port):
        server = Server("opc.tcp://" + ip + ":" + port + namespace)

if __name__ == '__main__':
    rospy.init_node('opcua_server', anonymous=False)

    while (not rospy.is_shutdown()):
        ros.doMain()
        r.sleep()