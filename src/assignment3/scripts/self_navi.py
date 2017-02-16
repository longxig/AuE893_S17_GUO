#!/usr/bin/env python

'''
Author: Longxiang Guo
Latest Update: 20170215
All rights reserved
'''

import rospy
import math
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan

def chunks(list,n):
    return (list[i:i+n] for i in xrange(0, len(list), n))

class Navigate():
    def __init__(self):

        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.OdomCallback, queue_size=10)
        self.bumper_subscriber = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.BumperCallback, queue_size=10)
        self.depth_subscriber = rospy.Subscriber('/scan', LaserScan, self.DepthCallback, queue_size=10)

        self.odom = Odometry()
        self.bumper = BumperEvent()
        self.depth = LaserScan()
        self.maxspeed = 0.3
        self.minspeed = 0.07
        self.minrotate = 0.06
        self.mindis = [10, 10, 10, 10, 10, 10, 10, 10]
        self.minpos = [-26.25, -18.75, -11.25, -3.75, 3.75, 11.25, 18.75, 26.25]
        self.avgdis = [10, 10, 10, 10, 10, 10, 10, 10]
        self.avgwidth = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        self.absmindis = 10
        self.disthres = [0.5, 0.7, 1.0]
        self.finalmin = 10
        self.finalminpos = 0
    
    def OdomCallback(self, msg):
        self.odom = msg
    
    def BumperCallback(self, msg):
        self.bumper = msg

    def DepthCallback(self, msg):
        self.depth = msg
        self.Sort()

    def Sort(self):
        '''
        self.mindis = []
        self.minpos = []
        self.avgdis = []
        '''
        n = 80
        ranges = list(self.depth.ranges)
        total = len(ranges)
        for i in range(0, total):
            if math.isnan(ranges[i]):
                ranges[i] = 10
        self.finalmin = min(ranges)
        self.finalminpos = ranges.index(self.finalmin)*self.depth.angle_increment + self.depth.angle_min
        chunk_range = list(chunks(ranges, n))
        for j in range(0, 7):
            minvalue = min(chunk_range[j])
            self.mindis[j] = minvalue
            #self.mindis.append(minvalue)
            minindices = chunk_range[j].index(minvalue)
            minpos = self.depth.angle_min + (j*n + minindices + 1) * self.depth.angle_increment
            self.minpos[j] = minpos
            #self.minpos.append(minpos)
            avgvalue = sum(chunk_range[j]) / len(chunk_range[j])
            self.avgdis[j] = avgvalue
            #self.avgdis.append(avgvalue)
            avgwid = avgvalue * math.sin(n*self.depth.angle_increment)
            self.avgwidth[j] = avgwid
            #self.avgwidth.append(avgwid)
            #rospy.loginfo("minvalue: %f, minpos: %f, avgvalue: %f, avgwid: %f", minvalue,minpos, avgvalue,avgwid )
        self.absmindis = min(self.mindis)
        if min(self.mindis) < self.disthres[0]:
            self.maxspeed = 0.05
            self.minspeed = 0.0
        elif min(self.mindis) < self.disthres[1]:
            self.maxspeed = 0.1
            self.minspeed = 0.05
        elif min(self.mindis) < self.disthres[2]:
            self.maxspeed = 0.12
            self.minspeed = 0.07
        else:
            self.maxspeed = 0.2
            self.minspeed = 0.07

    def getDistance(self, x, y):
        xd = x - self.odom.pose.pose.position.x
        yd = y - self.odom.pose.pose.position.y
        return math.sqrt(xd*xd+yd*yd)

    def quad2rad(self):
        x = self.odom.pose.pose.orientation.x
        y = self.odom.pose.pose.orientation.y
        z = self.odom.pose.pose.orientation.z
        w = self.odom.pose.pose.orientation.w
        return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

    def targetangle(self, x, y):
        return math.atan2(y-self.odom.pose.pose.position.y, x-self.odom.pose.pose.position.x)

    def rotateangle(self, x, y):
        currentangle = self.quad2rad()
        finalangle = self.targetangle(x, y)
        return math.atan2(math.sin(finalangle-currentangle), math.cos(finalangle-currentangle))
    
    def simplerotate(self, angle):
        vel_msg = Twist()
        rate = rospy.Rate(10)
        angletogo = math.radians(angle)
        coveredangle = 0.
        while not rospy.is_shutdown():
            starttime = rospy.Time.from_sec(time.time())
            startseconds = starttime.to_sec()
            vel_msg.linear.x = 0.
            vel_msg.angular.z = 0.1*(angletogo-coveredangle)
            #rospy.loginfo("angletogo: %f, coveredangle: %f", angletogo,coveredangle)
            if abs(coveredangle) < abs(angletogo):
                if abs(vel_msg.angular.z) < self.minrotate:
                    if vel_msg.angular.z > 0:
                        vel_msg.angular.z = self.minrotate
                    elif vel_msg.angular.z < 0:
                        vel_msg.angular.z = -1*self.minrotate
                self.cmd_vel.publish(vel_msg)
            else:
                break
            rate.sleep()
            endtime = rospy.Time.from_sec(time.time())
            endsecconds = endtime.to_sec()
            coveredangle += (endsecconds - startseconds) * vel_msg.angular.z
        self.cmd_vel.publish(Twist())
        #rospy.loginfo("Rotate Complete")
        rate.sleep()

    def simplemove(self, distance):
        vel_msg = Twist()
        rate = rospy.Rate(10)
        covereddistance = 0.
        while not rospy.is_shutdown():
            starttime = rospy.Time.from_sec(time.time())
            startseconds = starttime.to_sec()
            vel_msg.angular.z = 0.
            vel_msg.linear.x = 0.1*(distance-covereddistance)
            #rospy.loginfo("distancetogo: %f, covereddistance: %f", distance, covereddistance)
            if abs(covereddistance) < abs(distance):
                if vel_msg.linear.x > self.maxspeed:
                    vel_msg.linear.x = self.maxspeed
                elif vel_msg.linear.x < self.minspeed:
                    vel_msg.linear.x = self.minspeed
                self.cmd_vel.publish(vel_msg)
            else:
                break
            rate.sleep()
            endtime = rospy.Time.from_sec(time.time())
            endsecconds = endtime.to_sec()
            covereddistance += (endsecconds - startseconds) * vel_msg.linear.x
        self.cmd_vel.publish(Twist())
        #rospy.loginfo("Rotate Complete")
        rate.sleep()

    def MoveTo(self, x, y, tolerance):

        vel_msg = Twist()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.getDistance(x, y) > tolerance and self.absmindis > self.disthres[2]:
            
            if self.getDistance(x, y) > 1:
                thres = 1.4
            elif self.getDistance(x, y) > 0.5:
                thres = 0.8
            else:
                thres = 0.5

            if abs(self.rotateangle(x, y)) > thres:
                vel_msg.linear.x = 0.
                vel_msg.angular.z = 0.1*self.rotateangle(x, y)

                if abs(vel_msg.angular.z) < self.minrotate:
                    if vel_msg.angular.z > 0:
                        vel_msg.angular.z = self.minrotate
                    elif vel_msg.angular.z < 0:
                        vel_msg.angular.z = -1*self.minrotate
            else:
                vel_msg.linear.x = 0.1*self.getDistance(x, y)
                vel_msg.angular.z = 0.2*self.rotateangle(x, y)
                if vel_msg.linear.x > self.maxspeed:
                    vel_msg.linear.x = self.maxspeed
                elif vel_msg.linear.x < self.minspeed:
                    vel_msg.linear.x = self.minspeed

            self.cmd_vel.publish(vel_msg)
            rate.sleep()
            #rospy.loginfo("Distance: %f,  Angle: %f", self.getDistance(x, y), self.rotateangle(x, y))

        self.cmd_vel.publish(Twist())
        rate.sleep()

    def MoveToSlow(self, x, y, tolerance):
        vel_msg = Twist()
        rate = rospy.Rate(10)
        while (not rospy.is_shutdown()) and (self.getDistance(x, y) > tolerance) and (self.absmindis > self.disthres[1]) and (self.absmindis <= self.disthres[2]):

            if self.getDistance(x, y) > 1:
                thres = 2.4
            elif self.getDistance(x, y) > 0.5:
                thres = 0.25
            else:
                thres = 0.5
            
            if abs(self.rotateangle(x, y)) > thres:
                vel_msg.linear.x = 0.
                vel_msg.angular.z = 0.1*self.rotateangle(x, y)

                if abs(vel_msg.angular.z) < self.minrotate:
                    if vel_msg.angular.z > 0:
                        vel_msg.angular.z = self.minrotate
                    elif vel_msg.angular.z < 0:
                        vel_msg.angular.z = -1*self.minrotate
            else:
                vel_msg.linear.x = 0.1*self.getDistance(x, y)
                vel_msg.angular.z = 0.2*self.rotateangle(x, y)
                if vel_msg.linear.x > self.maxspeed:
                    vel_msg.linear.x = self.maxspeed
                elif vel_msg.linear.x < self.minspeed:
                    vel_msg.linear.x = self.minspeed

            self.cmd_vel.publish(vel_msg)
            rate.sleep()
            #rospy.loginfo("Distance: %f,  Angle: %f", self.getDistance(x, y), self.rotateangle(x, y))

        self.cmd_vel.publish(Twist())
        rate.sleep()
    
    def IntelMove(self, x, y, tolerance):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.getDistance(x, y) < tolerance:
                break
            rospy.loginfo("Closest: %f", self.absmindis)
            if self.absmindis > self.disthres[2]:
                rospy.loginfo("Full speed")
                self.MoveTo(x, y, tolerance)
            elif self.absmindis > self.disthres[1]:


                if self.finalminpos > 0:
                    togoangle = -1*(40 - math.degrees(self.finalminpos))
                    self.simplerotate(togoangle)
                elif self.finalminpos < 0:
                    togoangle = 40 + math.degrees(self.finalminpos)
                    self.simplerotate(togoangle)

                '''
                availableangle = []
                availabledis = []
                for i in range(0, len(self.avgwidth)):
                    #rospy.loginfo("average width: %f", self.avgwidth[i])
                    #rospy.loginfo("Closest Obj: %f", self.absmindis)   
                    if self.avgwidth[i] > 0.3:
                        availableangle.append(math.radians(-26.25+7.5*i))
                        availabledis.append(self.avgdis[i])

                if len(availableangle) > 0:
                    rospy.loginfo("Half speed")
                    maxdistance = max(availabledis)
                    maxind = availabledis.index(maxdistance)
                    targetangle = availableangle[maxind]
                    #rospy.loginfo("Max dis: %f", maxdistance)
                    rospy.loginfo("Target angle: %f", targetangle)
                    self.simplerotate(math.degrees(targetangle))
                    self.MoveTo(x, y, tolerance)
                    rate.sleep()
                else:
                    rospy.loginfo("Half speed rotate")
                    if self.mindis.index(self.absmindis) <= 4:
                        self.simplerotate(7.5)
                    else:
                        self.simplerotate(-7.5)
                '''
            else:
                rospy.loginfo("Only rotate")
                if self.mindis.index(self.absmindis) <= 4:
                    self.simplerotate(7.5)
                else:
                    self.simplerotate(-7.5)
                rate.sleep()

    def shutdown(self):
        rospy.loginfo("Stop Turtlebot")
        self.cmd_vel.publish(Twist())
        time.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('self_navi', anonymous=False)
        navi = Navigate()
        time.sleep(0.1)
        navi.IntelMove(4, 4, 0.15)
        time.sleep(0.1)
        navi.IntelMove(2, -2.5, 0.15)
        time.sleep(0.1)
        navi.IntelMove(-3, -3, 0.15)
        time.sleep(0.1)
        navi.IntelMove(-2, 1, 0.15)
        time.sleep(0.1)
    except rospy.ROSInterruptException:
        rospy.loginfo("Node Terminated")
