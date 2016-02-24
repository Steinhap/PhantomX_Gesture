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

class servoSlider():
    def __init__(self, parent, min_angle, max_angle, name, i):
        self.name = name
        if name.find("_joint") > 0: # remove _joint for display name
            name = name[0:-6]
        self.position = wx.Slider(parent, -1, 0, int(min_angle*100), int(max_angle*100), wx.DefaultPosition, (150, -1), wx.SL_HORIZONTAL)
        self.enabled = wx.CheckBox(parent, i, name+":")
        self.enabled.SetValue(False)
        self.position.Disable()
	self.synched = 0

    def setPosition(self, angle):
        self.position.SetValue(int(angle*100))

    def getPosition(self):
        return self.position.GetValue()/100.0

class controllerGUI(wx.Frame):
    TIMER_ID = 1000

    def __init__(self, parent, debug = False):  
        wx.Frame.__init__(self, parent, -1, "Gesture Library ", style = wx.DEFAULT_FRAME_STYLE & ~ (wx.RESIZE_BORDER | wx.MAXIMIZE_BOX))
        sizer = wx.GridBagSizer(5,5)

        # Move Base
        drive = wx.StaticBox(self, -1, 'Move Base')
        drive.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        driveBox = wx.StaticBoxSizer(drive,orient=wx.VERTICAL) 
        self.movebase = wx.Panel(self,size=(width,width-20))
        self.movebase.SetBackgroundColour('WHITE')
        self.movebase.Bind(wx.EVT_MOTION, self.onMove)  
        wx.StaticLine(self.movebase, -1, (width/2, 0), (1,width), style=wx.LI_VERTICAL)
        wx.StaticLine(self.movebase, -1, (0, width/2), (width,1))
        driveBox.Add(self.movebase)        
        sizer.Add(driveBox,(0,0),wx.GBSpan(1,1),wx.EXPAND|wx.TOP|wx.BOTTOM|wx.LEFT,5)
        self.forward = 0
        self.turn = 0
        self.X = 0
        self.Y = 0
        arm_elbow_r.publish(0.0)
	arm_elbow_l.publish(0.0)
	arm_shldr_l.publish(0.0) 
	arm_shldr_r.publish(0.0) 
	wrist_flx.publish(0.0) 
	arm_shldr_pan.publish(1.04) 
        

        # Move Servos
        servo = wx.StaticBox(self, -1, 'Move Servos')
        servo.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        servoBox = wx.StaticBoxSizer(servo,orient=wx.VERTICAL) 
        servoSizer = wx.GridBagSizer(5,5)

        joint_defaults = getJointsFromURDF()
        
        i = 0
        dynamixels = rospy.get_param("/arbotix/dynamixels", dict())
       
	
        self.servos = list()
        self.publishers = list()
        self.relaxers = list()
        
	self.synched = list()


      
	    
        # add everything
        servoBox.Add(servoSizer) 
        sizer.Add(servoBox, (0,1), wx.GBSpan(1,1), wx.EXPAND|wx.TOP|wx.BOTTOM|wx.RIGHT,5)
        self.Bind(wx.EVT_CHECKBOX, self.enableSliders)
        # now we can subscribe
        rospy.Subscriber('armGesture', String, self.stateCb)

        # timer for output
        self.timer = wx.Timer(self, self.TIMER_ID)
        self.timer.Start(50)
        wx.EVT_CLOSE(self, self.onClose)

        # bind the panel to the paint event
        wx.EVT_PAINT(self, self.onPaint)
        self.dirty = 1
        self.onPaint()

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

   
    def onPaint(self, event=None):
        # this is the wx drawing surface/canvas
        dc = wx.PaintDC(self.movebase)
        dc.Clear()
        # draw crosshairs
        dc.SetPen(wx.Pen("black",1))
        dc.DrawLine(width/2, 0, width/2, width)
        dc.DrawLine(0, width/2, width, width/2)
        dc.SetPen(wx.Pen("red",2))
        dc.SetBrush(wx.Brush('red', wx.SOLID))
        dc.SetPen(wx.Pen("black",2))
        dc.DrawCircle((width/2) + self.X*(width/2), (width/2) - self.Y*(width/2), 5)  

    def onMove(self, event=None):
        if event.LeftIsDown():        
            pt = event.GetPosition()
            if pt[0] > 0 and pt[0] < width and pt[1] > 0 and pt[1] < width:
                self.forward = ((width/2)-pt[1])/2
                self.turn = (pt[0]-(width/2))/2 
                self.X = (pt[0]-(width/2.0))/(width/2.0)
                self.Y = ((width/2.0)-pt[1])/(width/2.0)        
        else:
            self.forward = 0; self.Y = 0
            self.turn = 0; self.X = 0
        self.onPaint()          
        

    	

if __name__ == '__main__':
    # initialize GUI
    rospy.init_node('gestureLibrary')
    app = wx.PySimpleApp()
    frame = controllerGUI(None, True)
    app.MainLoop()
