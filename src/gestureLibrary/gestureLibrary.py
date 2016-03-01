#!/usr/bin/env python

""" 
  This Node will subsrcibes to the /armGesture topic of type std_msgs/String
  
  The current commands that have been implemented are "Wave" and "Point left"
"""

import rospy
import wx
import time

from math import radians

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Float64
from arbotix_msgs.srv import Relax
from arbotix_python.joints import *

width = 325
arm_elbow_r= rospy.Publisher('/arm_elbow_flex_joint_right/command', Float64, queue_size=5)
arm_elbow_l= rospy.Publisher('/arm_elbow_flex_joint_left/command', Float64, queue_size=5)
arm_shldr_l= rospy.Publisher('/arm_shoulder_lift_joint_left/command', Float64, queue_size=5)
arm_shldr_r= rospy.Publisher('/arm_shoulder_lift_joint_right/command', Float64, queue_size=5)
wrist_flx= rospy.Publisher('/arm_wrist_flex_joint/command', Float64, queue_size=5)
gripper= rospy.Publisher('/gripper_joint/command', Float64, queue_size=5)
arm_shldr_pan= rospy.Publisher('/arm_shoulder_pan_joint/command', Float64, queue_size = 5)
gsture_pub = rospy.Publisher('/armGesture', String, queue_size = 5)


class controllerGUI(wx.Frame):
    TIMER_ID = 1000

    def __init__(self, parent, debug= False):  
        wx.Frame.__init__(self, parent, -1, "Gesture Library", style = wx.DEFAULT_FRAME_STYLE & ~ (wx.RESIZE_BORDER | wx.MAXIMIZE_BOX))
       	sizer = wx.GridBagSizer(5,5)
        panel = wx.Panel(self,size=(320,250))

        self.button1 = wx.Button(panel, id=-1, label='Wave', pos=(8, 8), size=(175, 28))
        self.button1.Bind(wx.EVT_BUTTON, self.button1Click)
        
        self.button2 = wx.Button(panel, id=-1, label='Point Left', pos=(8, 38), size=(175, 28))
        self.button2.Bind(wx.EVT_BUTTON, self.button2Click)
        
        self.button3 = wx.Button(self, id=-1, label='Point Right', pos=(8, 68), size=(175, 28))
        self.button3.Bind(wx.EVT_BUTTON, self.button3Click)
        
        self.button4 = wx.Button(self, id=-1, label='Point Up', pos=(8, 98), size=(175, 28))
        self.button4.Bind(wx.EVT_BUTTON, self.button4Click)
        
        sizer.Add(panel,(0,0),wx.GBSpan(1,1),wx.EXPAND|wx.TOP|wx.BOTTOM|wx.LEFT,5)

        arm_elbow_r.publish(0.0)
	arm_elbow_l.publish(0.0)
	arm_shldr_l.publish(0.0) 
	arm_shldr_r.publish(0.0) 
	wrist_flx.publish(0.0) 
	arm_shldr_pan.publish(1.04) 
        
	    
        # now we can subscribe
        rospy.Subscriber('armGesture', String, self.stateCb)

        # timer for output
        self.timer = wx.Timer(self, self.TIMER_ID)
        self.timer.Start(50)
        wx.EVT_CLOSE(self, self.onClose)

        self.SetSizerAndFit(sizer)
        self.Show(True)

    def onClose(self, event):
        self.timer.Stop()
        self.Destroy()

    def enableSliders(self, event):
        servo = event.GetId()
        if event.IsChecked(): 
            self.servos[servo].position.Enable()
        else:
            self.servos[servo].position.Disable()
            self.relaxers[servo]()


    def stateCb(self, msg):
    	
    	if msg.data in ["wave", "Wave"]:
		arm_elbow_r.publish(1.0)
		arm_elbow_l.publish(-1.0)
		time.sleep(1)
		arm_elbow_r.publish(-1.0)
		arm_elbow_l.publish(1.0)
		time.sleep(1)
		arm_elbow_r.publish(1.0)
		arm_elbow_l.publish(-1.0)
		time.sleep(1)
		arm_elbow_r.publish(-1.0)
		arm_elbow_l.publish(1.0)
		time.sleep(1)
		arm_elbow_r.publish(0.0)
		arm_elbow_l.publish(0.0)
		
	if msg.data in ["point left", "Point Left"]:
		arm_shldr_pan.publish(-.75)
		
		arm_shldr_r.publish(-1.0)
		arm_shldr_l.publish(1.0)

    def button1Click(self,event):
    	gsture_pub.publish("Wave")
    	
    def button2Click(self, event):
    	gsture_pub.publish("Point Left")
    	
    def button3Click(self, event):
    	gsture_pub.publish("Point Right")
  
    def button4Click(self, event):
    	gsture_pub.publish("Point up")

        

    	

if __name__ == '__main__':
    # initialize GUI
    rospy.init_node('gestureLibrary')
    app = wx.PySimpleApp()
    frame = controllerGUI(None, True)
    app.MainLoop()
