from styx_msgs.msg import TrafficLight
import cv2
import numpy as np
import tensorflow as tf
import rospy
import os
import yaml

img_path = os.path.dirname(os.path.realpath(__file__)) + '/../../../../test_images/simulator/'
m_wdt = 300
m_hgt = 300
r_image = False

class TLClassifier(object):
    def __init__(self):
        self.session = None
        self.model_gb = None
        self.image_counter = 0
        self.classes = {1: TrafficLight.RED,
                        2: TrafficLight.YELLOW,
                        3: TrafficLight.GREEN,
                        4: TrafficLight.UNKNOWN}

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.load_model(self.get_model_path())

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        class_idx, prb = self.predict(image)

        if class_idx is not None:
            rospy.logdebug("class: %d, prb: %f", class_idx, prb)
        return class_idx

    def load_model(self, model_path):
        config = tf.ConfigProto()
        config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1
        self.model_gb = tf.Graph()
        with tf.Session(graph=self.model_gb, config=config) as sess:
            self.session = sess
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as model_file:
                serialized_graph = model_file.read()
                graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(graph_def, name='')

    def predict(self, img_np_, score_thresh=0.5):
        image_tensor = self.model_gb.get_tensor_by_name('image_tensor:0')
        detection_boxes = self.model_gb.get_tensor_by_name('detection_boxes:0')
        detection_scores = self.model_gb.get_tensor_by_name('detection_scores:0')
        detection_classes = self.model_gb.get_tensor_by_name('detection_classes:0')
        img_np_ = self.preprocess_img(img_np_)
        (boxes, scores, classes) = self.session.run(
            [detection_boxes, detection_scores, detection_classes],
            feed_dict={image_tensor: np.expand_dims(img_np_, axis=0)})
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)
        boxes = np.squeeze(boxes)
        for i, box in enumerate(boxes):
            if scores[i] > score_thresh:
                light_class = self.classes[classes[i]]
                self.save_image(img_np_, light_class)
                rospy.logdebug("Traffic Light detected by the model : %d Class", light_class)
                return light_class, scores[i]
            else:
                self.save_image(img_np_, TrafficLight.UNKNOWN)
        return None, None

    def preprocess_img(self, img):
        img = cv2.resize(img, (m_wdt, m_hgt))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    def save_image(self, image, light_class):
        if r_image:
            bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imwrite(os.path.join(img_path, "image_%04i_%d.jpg" % (self.image_counter, light_class)), bgr_image)
            self.image_counter += 1

    def get_model_path(self):
        return os.path.dirname(os.path.realpath(__file__)) + self.config['detection_model']

    def resize_image(self, image):
        hgt, wdt = image.shape[:2]
        if m_hgt < hgt or m_wdt < wdt:
            scaling_factor = m_hgt / float(hgt)
            if m_wdt / float(wdt) < scaling_factor:
                scaling_factor = m_wdt / float(wdt)
            image = cv2.resize(image, None, fx=scaling_factor, fy=scaling_factor, interpolation=cv2.INTER_AREA)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image
