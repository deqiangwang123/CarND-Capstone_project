#!/usr/bin/env python
import rospy
import math
import copy
from enum import Enum
import sys
from std_msgs.msg import Bool, Int32, Float32
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''
BUF_SAFE = 0.5
LOOKAHEAD_WPS = 50
UNKNOWN = -1
MAX_dist = sys.maxint

class State(Enum):
    ACCELERATION = 1
    DECELERATION = 2

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.curr_pos_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.curr_vel_cb)
        rospy.Subscriber('/obstacle_waypoints', PoseStamped, self.obstacle_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        self.final_waypoints_publisher = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.final_waypoints = None

        self.current_position = None
        self.current_velocity_mps = None

        self.lane = None
        self.n_waypoints = None


        self.next_stopline_waypoint = UNKNOWN
        self.current_state = State.DECELERATION
        self.is_state_change = True
        self.final_waypoints = None

        self.max_velocity_mps = rospy.get_param('/waypoint_loader/velocity') / 3.6
        self.acceleration_limit_mps = rospy.get_param('~accel_limit', 1.)

        self.deceleration_limit_max_mps = -rospy.get_param('~decel_limit', -5.)
        self.deceleration_limit_min_mps = min(1.0, -rospy.get_param('~decel_limit', -5.) / 2.)

        self.loop()

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub_wps_final()
            rate.sleep()

    def filter_waypoints_behind(self, waypoint_idx):
        d_x = self.current_position.x - self.lane.waypoints[waypoint_idx].pose.pose.position.x
        d_y = self.current_position.y - self.lane.waypoints[waypoint_idx].pose.pose.position.y
        n_x = self.lane.waypoints[(waypoint_idx + 1) % self.n_waypoints].pose.pose.position.x - \
             self.lane.waypoints[waypoint_idx].pose.pose.position.x
        n_y = self.lane.waypoints[(waypoint_idx + 1) % self.n_waypoints].pose.pose.position.y - \
             self.lane.waypoints[waypoint_idx].pose.pose.position.y
        dp = d_x*n_x + d_y*n_y
        return dp > 0.0

    def accelerate(self, lane, current_waypoint_idx):
        current_velocity = self.current_velocity_mps
        tr_vel = self.current_velocity_mps
        acceleration = self.acceleration_limit_mps
        i = 0
        while tr_vel < self.max_velocity_mps or i < LOOKAHEAD_WPS:
            srt_pos = self.current_position
            end_pos = self.lane.waypoints[
                (current_waypoint_idx + i) % self.n_waypoints].pose.pose.position
            distance = self.dist_pos(srt_pos, end_pos)
            tr_vel = math.sqrt(current_velocity**2.0 + 2.0*acceleration*distance)
            if tr_vel > self.max_velocity_mps:
                tr_vel = self.max_velocity_mps
            current_waypoint = copy.deepcopy(
                self.lane.waypoints[(current_waypoint_idx + i) % self.n_waypoints])
            current_waypoint.twist.twist.linear.x = tr_vel
            lane.waypoints.append(current_waypoint)
            i += 1

    def get_closest_wp_idx(self):
        minimal_distance = MAX_dist
        waypoints = self.lane.waypoints
        waypoint_idx = UNKNOWN
        for i in range(self.n_waypoints):
            distance = self.dist_pos(self.current_position, waypoints[i].pose.pose.position)
            if distance < minimal_distance:
                minimal_distance = distance
                waypoint_idx = i
        if self.filter_waypoints_behind(waypoint_idx):
            waypoint_idx = (waypoint_idx + 1) % self.n_waypoints
        return waypoint_idx

    def decelerate(self, lane, current_waypoint_idx):
        tr_vel = self.current_velocity_mps
        current_velocity = self.current_velocity_mps
        distance = self.dist_pos(self.current_position, self.lane.waypoints[
            self.next_stopline_waypoint].pose.pose.position) - BUF_SAFE
        acceleration = current_velocity ** 2.0/(2.0*distance)
        i = 0
        while tr_vel > 0.0 or i < LOOKAHEAD_WPS:
            end_pos = self.lane.waypoints[
                (current_waypoint_idx + i) % self.n_waypoints].pose.pose.position
            srt_pos = self.current_position
            distance = self.dist_pos(srt_pos, end_pos)
            tr_vel_exp = current_velocity ** 2.0 - 2.0 * acceleration * distance
            if tr_vel_exp <= 0:
                tr_vel = 0
            else:
                tr_vel = math.sqrt(tr_vel_exp)
            current_waypoint = copy.deepcopy(
                self.lane.waypoints[(current_waypoint_idx + i) % self.n_waypoints])
            current_waypoint.twist.twist.linear.x = tr_vel
            lane.waypoints.append(current_waypoint)
            i += 1

    def cnt_curr_state(self, lane, waypoint_idx, cv):
        k_ = 0
        while k_ < len(self.final_waypoints):
            if self.final_waypoints[k_].pose.pose.position == self.lane.waypoints[waypoint_idx].pose.pose.position:
                break
            k_ += 1

        for i in range(k_, len(self.final_waypoints)):
            current_waypoint = copy.deepcopy(self.lane.waypoints[(waypoint_idx + i - k_) % self.n_waypoints])
            current_waypoint.twist.twist.linear.x = self.final_waypoints[i].twist.twist.linear.x
            lane.waypoints.append(current_waypoint)

        for i in range(len(lane.waypoints), LOOKAHEAD_WPS):
            current_waypoint = copy.deepcopy(self.lane.waypoints[(waypoint_idx + i) % self.n_waypoints])
            current_waypoint.twist.twist.linear.x = cv
            lane.waypoints.append(current_waypoint)

    def pub_wps_final(self):
        if self.lane is None or self.current_position is None or self.current_velocity_mps is None:
            return
        waypoint_idx = self.get_closest_wp_idx()
        lane = Lane()
        lane.header.stamp = rospy.Time.now()
        if self.current_state == State.ACCELERATION:
            if self.next_stopline_waypoint != UNKNOWN:
                srt_pos = self.current_position
                end_pos = self.lane.waypoints[self.next_stopline_waypoint].pose.pose.position
                brk_dist = self.dist_pos(srt_pos, end_pos) - BUF_SAFE
                min_brk_dist = 0.5 * self.current_velocity_mps ** 2 / self.deceleration_limit_max_mps
                max_brk_dist = 0.5 * self.current_velocity_mps ** 2 / self.deceleration_limit_min_mps
                if max_brk_dist >= brk_dist >= min_brk_dist:
                    self.current_state = State.DECELERATION
                    self.is_state_change = True

        elif self.current_state == State.DECELERATION:
            if self.next_stopline_waypoint == UNKNOWN:
                self.current_state = State.ACCELERATION
                self.is_state_change = True
        else:
            rospy.logerr("State doesn't exist: WayUpdater Node")

        if self.current_state == State.ACCELERATION and self.is_state_change:
            self.accelerate(lane, waypoint_idx)
        elif self.current_state == State.ACCELERATION and not self.is_state_change:
            self.cnt_curr_state(lane, waypoint_idx, self.max_velocity_mps)
        elif self.current_state == State.DECELERATION and self.is_state_change:
            self.decelerate(lane, waypoint_idx)
        elif self.current_state == State.DECELERATION and not self.is_state_change:
            self.cnt_curr_state(lane, waypoint_idx, 0)
        else:
            rospy.logerr("State doesn't exist: WayUpdater Node")
        self.is_state_change = False
        self.final_waypoints = copy.deepcopy(lane.waypoints)
        self.final_waypoints_publisher.publish(lane)


    def curr_pos_cb(self, msg):
        self.current_position = msg.pose.position

    def curr_vel_cb(self, msg):
        self.current_velocity_mps = msg.twist.linear.x

    def waypoints_cb(self, waypoints):
        self.lane = waypoints
        self.n_waypoints = len(self.lane.waypoints)

    def traffic_cb(self, msg):
        self.next_stopline_waypoint = msg.data
        if self.next_stopline_waypoint != UNKNOWN and self.n_waypoints is not None:
            self.next_stopline_waypoint = (self.next_stopline_waypoint - 5 + self.n_waypoints) % self.n_waypoints

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def dist_pos(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 +(p1.y - p2.y)**2 + (p1.z - p2.z)**2)

    def distance_wp(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
