#!/usr/bin/env python

"""
A simple interface which can perform a series of poses 
with the PhantomX_Reactor arm

By: Phillip Steinhart

"""



import rospy
import wx 

from math import radians 

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from arbotix_msgs.srv import Relax
from arbotix_python.joints import*


class gestureLibrary(wx.Frame):
	def __init__(self, parent, debug= false):
		print wx.version() 
		joint_defaults = getJointsFromURDF()
        	i = 0
        	dynamixels = rospy.get_param('/arbotix/dynamixels', dict())
        	self.servos = list()
        	self.publishers = list()
        	self.relaxers = list()

 		joints = rospy.get_param('/arbotix/joints', dict())
        	# create publishers
        	for name in sorted(joints.keys()):
        	    # pull angles
         	   min_angle, max_angle = getJointLimits(name, joint_defaults)
         	   # create publisher
          	  self.publishers.append(rospy.Publisher(name+'/command', Float64, queue_size=5))
           	 if rospy.get_param('/arbotix/joints/'+name+'/type','dynamixel') == 'dynamixel':
                	self.relaxers.append(rospy.ServiceProxy(name+'/relax', Relax))
            	else:
             	   self.relaxers.append(None)
 



if __name__ == '__main__':
	app = wx.PySimpleApp()
	frame = wx.Frame(None, title='Hello World')
	frame.Show()
	app.MainLoop()
