
#!/usr/bin/env python

""" 
This node will subscribe armGesture topic of type std_msgs/String and publish the proper commands
to the joint/Command topics to move the arm as desired.  

"""

import rospy
import wx

from math import radians

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from arbotix_msgs.srv import Relax
from arbotix_python.joints import *

width = 325

class gestureLibrary():
    TIMER_ID = 100

    def __init__(self, parent, debug = False):  

        joint_defaults = getJointsFromURDF()
        
        i = 0
        dynamixels = rospy.get_param('/arbotix/dynamixels', dict())
        self.servos = list()
        self.publishers = list()
        self.relaxers = list()

        joints = rospy.get_param('/arbotix/joints', dict())
        # create sliders and publishers
        for name in sorted(joints.keys()):
            # pull angles
            min_angle, max_angle = getJointLimits(name, joint_defaults)
            # create publisher
            self.publishers.append(rospy.Publisher(name+'/command', Float64, queue_size=5))
            if rospy.get_param('/arbotix/joints/'+name+'/type','dynamixel') == 'dynamixel':
                self.relaxers.append(rospy.ServiceProxy(name+'/relax', Relax))
            else:
                self.relaxers.append(None)
            # create slider
            s = servoSlider(self, min_angle, max_angle, name, i)
            servoSizer.Add(s.enabled,(i,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)   
            servoSizer.Add(s.position,(i,1), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
            self.servos.append(s)
            i += 1

        # now we can subscribe
        rospy.Subscriber('joint_states', JointState, self.stateCb)

    def onClose(self, event):
        self.timer.Stop()
        self.Destroy()

    def stateCb(self, msg):        
        for servo in self.servos:
            if not servo.enabled.IsChecked():
                try:
                    idx = msg.name.index(servo.name)
                    servo.setPosition(msg.position[idx])
                except: 
                    pass

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
        
    def onTimer(self, event=None):
        # send joint updates
        for s,p in zip(self.servos,self.publishers):
            if s.enabled.IsChecked():
                d = Float64()
                d.data = s.getPosition()
                p.publish(d)

if __name__ == '__main__':
    # initialize GUI
    rospy.init_node('controllerGUI')
    app = wx.PySimpleApp()
    frame = controllerGUI(None, True)
    app.MainLoop()

Status API Training Shop Blog About Pricing
© 2016 GitHub, Inc. Terms Privacy Security Contact Help
