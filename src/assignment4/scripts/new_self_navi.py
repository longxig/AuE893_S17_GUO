#!/usr/bin/env python

'''
Author: Longxiang Guo
Latest Update: 20170215
All rights reserved
'''

import rospy
import math
import numpy as np
import time
import pid
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def chunks(list,n):
    return (list[i:i+n] for i in xrange(0, len(list), n))

def closest(list, Number):
    aux = []
    for valor in list:
        aux.append(abs(Number-valor))

    return aux.index(min(aux))

class Navigate():
    def __init__(self):

        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.OdomCallback, queue_size=10)
        self.depth_subscriber = rospy.Subscriber('/mybot/laser/scan', LaserScan, self.DepthCallback, queue_size=10)
        self.odom = Odometry()
        self.depth = LaserScan()
        self.mindis = np.zeros(15)
        self.minpos = np.zeros(15)
        self.avgdis = np.zeros(15)
        self.avgwidth = np.zeros(15)
        self.absmindis = 10
        self.globalmin = 30
        self.globalmax = 0.1
        self.globalminpos = 0
        self.globalmaxpos = 0
        self.maxavgpos = 0
        self.latecontroller1 = pid.PID(0.2, 0.005, 0.002, -0.1, 0.1)
    
    def OdomCallback(self, msg):
        self.odom = msg
        self.maxrotate = 1 - self.getspd()

    def DepthCallback(self, msg):
        self.depth = msg
        self.Sort()

    def Sort(self):

        n = 48
        ranges = list(self.depth.ranges)
        total = len(ranges)
        for i in range(total):
            ranges[i] = min(ranges[i], 25)
        
        self.globalmin = min(ranges)
        self.globalmax = max(ranges)
        self.globalminpos = ranges.index(self.globalmin)*self.depth.angle_increment + self.depth.angle_min
        self.globalmaxpos = ranges.index(self.globalmax)*self.depth.angle_increment + self.depth.angle_min
        chunk_range = list(chunks(ranges, n))
        for j in range(15):
            minvalue = min(chunk_range[j])
            self.mindis[j] = minvalue
            minindices = chunk_range[j].index(minvalue)

            minpos = self.depth.angle_min + (j*n + minindices + 1) * self.depth.angle_increment
            self.minpos[j] = minpos

            avgvalue = sum(chunk_range[j]) / len(chunk_range[j])
            self.avgdis[j] = avgvalue

            avgwid = avgvalue * math.sin(n*self.depth.angle_increment)
            self.avgwidth[j] = avgwid
            #rospy.loginfo("minvalue: %f, minpos: %f, avgvalue: %f, avgwid: %f", minvalue,minpos, avgvalue,avgwid )

    def getDistance(self, x, y):
        xd = x - self.odom.pose.pose.position.x
        yd = y - self.odom.pose.pose.position.y
        return math.sqrt(xd*xd+yd*yd)
    
    def getspd(self):
        vx = self.odom.twist.twist.linear.x
        vy = self.odom.twist.twist.linear.y
        return math.sqrt(vx*vx+vy*vy)

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
                if abs(vel_msg.angular.z) < 0.1:
                    if vel_msg.angular.z > 0:
                        vel_msg.angular.z = 0.1
                    elif vel_msg.angular.z < 0:
                        vel_msg.angular.z = -1*0.1
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

    def IntelMove(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.
        vel_msg.linear.y = 0.
        vel_msg.linear.z = 0.
        vel_msg.angular.x = 0.
        vel_msg.angular.y = 0.
        vel_msg.angular.z = 0.
        spd = 0.
        rot = 0.
        speedlimit = 0.4
        rotatelimit = speedlimit + 0.3

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            maxavg = max(self.avgdis)
            maxavgpos = [m for m, n in enumerate(self.avgdis) if n == maxavg] 
            minavg = min(self.avgdis)
            minavgpos = np.argmin(self.avgdis)
            middle = 7
            target = maxavgpos[closest(maxavgpos,middle)]
            targetangle = math.radians((target - 7) * 12)
            avoidangle = self.globalminpos
            avoidzone = math.radians((np.argmin(self.avgdis) - 7) * 12)

            if self.globalmin > 0.3:
                #spd = 1.5 * self.globalmin
                spd = speedlimit
            elif self.globalmin > 0.15:
                spd = 2 * self.globalmin
            else:
                spd = 0
                rospy.loginfo("Too Close ")
            if minavg < 0.5:
                #rot = self.latecontroller1.update_PID(avoidangle)
                rot =  0.2 * avoidangle + 0.1 * avoidzone
                rospy.loginfo("Avoidance")
            else:
                rot = -0.35 * targetangle
            
            if maxavg < 0.6: 
                self.simplerotate(180)

            vel_msg.linear.x = min(speedlimit, spd)
            rotatemax = rotatelimit - vel_msg.linear.x
            absrot = abs(rot)
            temprot = min(absrot, rotatemax)

            if rot > 0:
                vel_msg.angular.z = temprot
            else:
                vel_msg.angular.z = -1 * temprot
            rospy.loginfo("target: %f , avoid: %f , spd: %f , rot: %f ", targetangle, avoidangle, vel_msg.linear.x, vel_msg.angular.z)
            self.cmd_vel.publish(vel_msg)
            rate.sleep()

        self.cmd_vel.publish(Twist())
    def shutdown(self):
        rospy.loginfo("Stop Turtlebot")
        self.cmd_vel.publish(Twist())
        time.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('self_navi', anonymous=False)
        navi = Navigate()

        while not rospy.is_shutdown():
            time.sleep(0.1)
            navi.IntelMove()


    except rospy.ROSInterruptException:
        rospy.loginfo("Node Terminated")
