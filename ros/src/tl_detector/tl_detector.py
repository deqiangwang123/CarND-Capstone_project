#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight, Waypoint
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import sys
import math
import yaml
from timeit import default_timer as timer

dist_max = sys.maxint
freq_op = 10.0

STATE_COUNT_THRESHOLD = 3
UNKNOWN = -1


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.processing_tm = 0

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        if self.processing_tm > 0:
            self.processing_tm -= freq_op/ 100
            return

        self.has_image = True
        self.camera_image = msg
        detection_start_time = timer()
        light_wp, state = self.process_traffic_lights()
        detection_end_time = timer()
        self.processing_tm = detection_end_time - detection_start_time

        '''
        Publish upcoming red lights at camera frequency.
        Each prediction state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            if state == TrafficLight.RED or state == TrafficLight.YELLOW:
                light_wp = light_wp
            else:
                light_wp = UNKNOWN
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if (not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Get classification
        prediction = self.light_classifier.get_classification(cv_image)
        rospy.logdebug("traffic light state: %d", light.state)
        return prediction

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        light = None

        dst_safe = 300
        stop_line_location = None

        if self.pose:
            car_index = self.get_closest_waypoint(self.pose.pose.position)
            car_position = self.waypoints.waypoints[car_index].pose.pose.position
            light_index = self.get_closest_light(car_position)
            if light_index != UNKNOWN:
                light_waypoints_idx = self.get_closest_waypoint(self.lights[light_index].pose.pose.position)
                light_position = self.waypoints.waypoints[light_waypoints_idx].pose.pose.position

                if light_waypoints_idx > car_index:
                    dst_light = self.get_dist(car_position, light_position)
                    if dst_light < dst_safe:
                        light = self.lights[light_index]
                        stop_line_idx = self.get_closest_stop_line(light_position)
                        stop_line_location = self.get_stop_line_locations()[stop_line_idx].pose.pose
                        light_waypoints = self.get_closest_waypoint(stop_line_location.position)
        if light and stop_line_location:
            state = self.get_light_state(light)
            return light_waypoints, state
        return -1, TrafficLight.UNKNOWN


    def get_closest_waypoint(self, pose):
        return self.get_closest_index(pose, self.waypoints.waypoints)

    def get_closest_stop_line(self, pose):
        return self.get_closest_index(pose, self.get_stop_line_locations())

    def get_closest_light(self, pose):
        return self.get_closest_index(pose, self.lights)

    def get_stop_line_locations(self):
        stop_line_locations = []
        for light_position in self.config['stop_line_positions']:
            p = Waypoint()
            p.pose.pose.position.x = light_position[0]
            p.pose.pose.position.y = light_position[1]
            p.pose.pose.position.z = 0.0
            stop_line_locations.append(p)
        return stop_line_locations

    def get_dist(self, point1, point2):
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2)


    def get_closest_index(self, pose, positions):
        minimal_distance = dist_max
        index = UNKNOWN

        for i in range(len(positions)):
            distance = self.get_dist(pose, positions[i].pose.pose.position)
            if distance < minimal_distance:
                minimal_distance = distance
                index = i

        return index



if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
