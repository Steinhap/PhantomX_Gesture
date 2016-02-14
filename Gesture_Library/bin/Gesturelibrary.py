#!/usr/bin/env python

"""
A node which subscribes to the armGesture topic of type ./ 
with the PhantomX_Reactor arm
By: Phillip Steinhart
"""
import rospy

from math import radians 

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from arbotix_msgs.srv import Relax
from arbotix_python.joints import*


class gestureLibrary():
	
	def __init__(self):
        self.t_delta = rospy.Duration(1.0/rospy.get_param("~diagnostic_rate", 1.0))
        self.t_next = rospy.Time.now() + self.t_delta
        self.pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=5)

 	def sendGesture(self, joints, controllers):
        """ Publish diagnostics. """    
        now = rospy.Time.now()
        if now > self.t_next:
            # create message
            msg = DiagnosticArray()
            msg.header.stamp = now
            for controller in controllers:
                d = controller.getDiagnostics()
                if d:
                    msg.status.append(d)
            for joint in joints:
                d = joint.getDiagnostics()
                if d:
                    msg.status.append(d)
            # publish and update stats
            self.pub.publish(msg)
            self.t_next = now + self.t_delta
 



if __name__ == '__main__':
	app = wx.PySimpleApp()
	frame = wx.Frame(None, title='Hello World')
	frame.Show()
	app.MainLoop()
