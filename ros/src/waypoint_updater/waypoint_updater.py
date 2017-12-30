#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped ####
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Bool, Int32 ####

import math


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint',Int32,self.traffic_cb) ####
        rospy.Subscriber('/current_velocity',TwistStamped, self.twist_cb) ####
        rospy.Subscriber('/vehicle/dbw_enabled',Bool,self.dbw_enabled_cb) ####

        ######## Publisher

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        ####
        self.base_waypoints = Lane()
        self.current_pose = PoseStamped()
        self.current_twist = TwistStamped()
        self.base_waypoints_cb = False
        self.current_pose_cb = False
        self.current_twist_cb = False
        self.wp_num = 0
        self.wp_front = None
        self.red_light_index = -1
        self.desired_vel = 0.0
        self.max_vel = rospy.get_param('/waypoint_loader/velocity')*1000./3600.#10.0
        self.ramp_dist = 40
        self.acceleration = 0.125

        #dbw
        self.dbw = False
        self.dbw_init = False



        rospy.spin()

    def waypoints_pblsh(self): ####

        if(self.dbw & (len(self.base_waypoints.waypoints) > 0) & self.base_waypoints_cb & self.current_pose_cb):
            front_index = self.nearest_front()


            self.set_linear_velocity(front_index)
            
            #rospy.loginfo("Current wp: %d",front_index)
            #rospy.loginfo("Current wp x: %f", self.base_waypoints.waypoints[front_index].pose.pose.position.x)
            #rospy.loginfo("Current wp y: %f", self.base_waypoints.waypoints[front_index].pose.pose.position.y)
            #rospy.loginfo("current lin twist: %f",self.base_waypoints.waypoints[front_index].twist.twist.linear.x)
            #rospy.loginfo("current x: %f",self.current_pose.pose.position.x)
            #rospy.loginfo("current y: %f",self.current_pose.pose.position.y)
            #rospy.loginfo("red light wp index: %d",self.red_light_index)


            self.base_waypoints.waypoints[front_index].twist.twist.linear.x = self.desired_vel
            final_waypoints = Lane()
            final_waypoints.header = self.base_waypoints.header
            for i in range (front_index, front_index+LOOKAHEAD_WPS):
                ci = i%self.wp_num
                final_waypoints.waypoints.append(self.base_waypoints.waypoints[ci])
            self.final_waypoints_pub.publish(final_waypoints)

        else:
            self.wp_front = None

    def nearest_front(self): ####
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        wp_x = self.base_waypoints.waypoints[0].pose.pose.position.x
        wp_y = self.base_waypoints.waypoints[0].pose.pose.position.y
        min_dist = math.sqrt(abs(wp_x-current_x)**2 + abs(wp_y-current_y)**2)
        min_index = 0

        if self.wp_front is None:
            lookup_start = 0
            lookup_end = len(self.base_waypoints.waypoints)
        elif self.wp_front + LOOKAHEAD_WPS > len(self.base_waypoints.waypoints):
            if self.wp_front >= len(self.base_waypoints.waypoints):
                lookup_start = 0
                lookup_end = lookup_start + LOOKAHEAD_WPS
            else:
                lookup_start = self.wp_front
                lookup_end = len(self.base_waypoints.waypoints)
        else:
            lookup_start = self.wp_front
            lookup_end = lookup_start + LOOKAHEAD_WPS


        for i in range(lookup_start, lookup_end):
            wp_x = self.base_waypoints.waypoints[i].pose.pose.position.x
            wp_y = self.base_waypoints.waypoints[i].pose.pose.position.y
            dist_i = math.sqrt(abs(wp_x-current_x)**2 + abs(wp_y-current_y)**2)
            if (dist_i < min_dist):
                min_dist = dist_i
                min_index = i

        wp_front = min_index
        wp_1 = (min_index - 1)%self.wp_num
        wp_2 = min_index%self.wp_num
        x1 = self.base_waypoints.waypoints[wp_1].pose.pose.position.x
        x2 = self.base_waypoints.waypoints[wp_2].pose.pose.position.x
        y1 = self.base_waypoints.waypoints[wp_1].pose.pose.position.y
        y2 = self.base_waypoints.waypoints[wp_2].pose.pose.position.y
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        Rx = x-x1
        Ry = y-y1
        dx = x2-x1
        dy = y2-y1
        seg_length = math.sqrt(dx**2 + dy**2)
        u = (Rx*dx + Ry*dy)/seg_length

        if abs(u) > seg_length:
            wp_front += 1

        self.wp_front = wp_front%self.wp_num
        return wp_front%self.wp_num

    def set_linear_velocity(self, index): ####
        wheel_base = rospy.get_param('~wheel_base', 2.8498)

        if(self.red_light_index >= 0):
            x_1 = self.base_waypoints.waypoints[self.red_light_index].pose.pose.position.x
            x_2 = self.base_waypoints.waypoints[index].pose.pose.position.x
            y_1 = self.base_waypoints.waypoints[self.red_light_index].pose.pose.position.y
            y_2 = self.base_waypoints.waypoints[index].pose.pose.position.y

            dist = math.sqrt(abs(x_1-x_2)**2 + abs(y_1-y_2)**2)
            
            if (dist < 0.3):
                self.desired_vel = 0.
            elif (dist < self.ramp_dist):
                self.desired_vel = max(self.max_vel*dist/self.ramp_dist, 0.5)
        else:
            self.desired_vel = max(self.desired_vel + self.acceleration, 0.5)

        self.desired_vel = min(self.desired_vel, self.max_vel)



    def pose_cb(self, msg): ####
        # TODO: Implement
        if not self.current_pose_cb:
            self.current_pose_cb = True
        self.current_pose = msg
        self.waypoints_pblsh()

    def twist_cb(self,msg): ####
        if not self.current_twist_cb:
            self.current_twist_cb = True
        self.current_twist = msg 

    def waypoints_cb(self, msg): ####
        # TODO: Implement
        if not self.base_waypoints_cb:
            self.base_waypoints_cb = True
        self.base_waypoints = msg
        self.wp_num = len(msg.waypoints)

    def traffic_cb(self, msg): ####
        # TODO: Callback for /traffic_waypoint message. Implement
        self.red_light_index = msg.data

    def obstacle_cb(self, msg): ####
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def dbw_enabled_cb(self, msg):
        if not self.dbw_init:
            self.dbw_init = True
        self.dbw = msg.data
            

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
