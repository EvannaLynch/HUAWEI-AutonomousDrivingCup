#!/usr/bin/env python

# cmd1.data=from 0 to 180
# cmd2.data=from 0 to 255
# cmd3.data=1 CW
# cmd3.data=2 CCW
# cmd3.data=0 brake
# cmd3.data=3 cut off power
# servo 0 is at right and 180 is at left

import rospy
import numpy as np

from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
import math
import tf

class base_controller():
    def __init__(self, mode):
        self.servoCmdMsg = UInt8()
        self.motorSpdCmdMsg = UInt8()
        self.motorModeCmdMsg = UInt8()
        self.odomMsg = Odometry()
        self.servoCmdMsg.data = 79

        rospy.init_node('base_controller', anonymous=True)
        self.servoCmdPub = rospy.Publisher('servoCmd', UInt8, queue_size=1)
        self.motorSpdCmdPub = rospy.Publisher('motorSpdCmd', UInt8, queue_size=1)
        self.motorModeCmdPub = rospy.Publisher('motorModeCmd', UInt8, queue_size=1)
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.odom_br =  tf.TransformBroadcaster()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.v = 0.0
        self.vth = 0.0
        self.delta = 0.0
        self.l = 0.58
        self.cur_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.scale = 121.75
        if mode == "PID":
            self.KP = 1
            self.KI = 1
            self.KD = 1
            self.error = [0, 0, 0]
            rospy.Subscriber('cmd_vel', Twist, self.cmdPIDCallback)
        else: 
            rospy.Subscriber('cmd_vel', Twist, self.cmdCallback)
        rospy.Subscriber('rpm', Int16, self.rpmCallback)
        self.rate = rospy.Rate(20)
        rospy.on_shutdown(self._shutdown)
    
    def cmdCallback(self, msg):
        _servoCmdMsg = msg.angular.z * 2 + 79
        self.servoCmdMsg.data = min(max(0, _servoCmdMsg), 180)
        _motorSpdCmdMsg = msg.linear.x
        if _motorSpdCmdMsg:
            self.motorSpdCmdMsg.data = min(abs(_motorSpdCmdMsg) * self.scale, 255)
            self.motorModeCmdMsg.data = 1.5 - 0.5*np.sign(_motorSpdCmdMsg) 
        else:
            self.stopMotor()

    def cmdPIDCallback(self, msg):
        _servoCmdMsg = msg.angular.z * 2 + 79
        self.servoCmdMsg.data = min(max(0, _servoCmdMsg), 180)
        _motorSpdCmdMsg = msg.linear.x
        self.error[2] = self.error[1]
        self.error[1] = self.error[0]
        self.error[0] = _motorSpdCmdMsg - self.odomMsg.twist.twist.linear.x
        if _motorSpdCmdMsg:
            _motorSpdCmdMsg *= self.scale
            _motorSpdCmdMsg += self.KP*(self.error[0]-self.error[1])+self.KI*self.error[0] \
              +self.KD*(self.error[0]-2*self.error[1]+self.error[2])
            self.motorSpdCmdMsg.data = min(abs(_motorSpdCmdMsg), 255)
            self.motorModeCmdMsg.data = 1.5 - 0.5*np.sign(_motorSpdCmdMsg) 
        else:
            self.stopMotor()

    def rpmCallback(self, msg):
        self.cur_time = rospy.Time.now()

        self.dt = (self.cur_time-self.last_time).to_sec()
        self.delta=  (self.servoCmdMsg.data - 79) * 3.1415/(180*1.8)
        self.v = msg.data * 0.09 * 2 * np.pi / 60
        self.vx = self.v * math.cos(self.theta)
        self.vy = self.v * math.sin(self.theta)
        self.vth = self.v * math.tan(self.delta)/self.l

        self.x += self.vx * self.dt
        self.y += self.vy * self.dt
        self.theta += self.vth * self.dt

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)


        self.odom_br.sendTransform((self.x, self.y, 0),
                     odom_quat,
                     self.cur_time,
                     "base_link",
                     "odom")


        self.odomMsg.header.stamp = self.cur_time
        self.odomMsg.header.frame_id = "odom"

        # set the position
        self.odomMsg.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*odom_quat))

        self.odomMsg.child_frame_id = "base_link"
        self.odomMsg.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))
        self.odomPub.publish(self.odomMsg)     
        self.last_time = self.cur_time

    def stopMotor(self):
        self.motorSpdCmdMsg.data = 0
        self.motorModeCmdMsg.data = 0
    
    def _shutdown(self):
        self.servoCmdMsg.data = 79
        self.stopMotor()
        self.servoCmdPub.publish(self.servoCmdMsg)
        self.motorSpdCmdPub.publish(self.motorSpdCmdMsg)
        self.motorModeCmdPub.publish(self.motorModeCmdMsg)

    
    def spin(self):
        while not rospy.is_shutdown():
            self.servoCmdPub.publish(self.servoCmdMsg)
            self.motorSpdCmdPub.publish(self.motorSpdCmdMsg)
            self.motorModeCmdPub.publish(self.motorModeCmdMsg)
            self.rate.sleep()
        
if __name__=="__main__":
    mode = rospy.get_param('mode', 'PID')
    baseController = base_controller(mode)
    baseController.spin()
