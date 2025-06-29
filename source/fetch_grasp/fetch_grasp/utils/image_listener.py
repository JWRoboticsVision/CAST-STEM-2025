import numpy as np
import cv2

# ROS imports
import ros_numpy
import rospy
import tf
import tf2_ros
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel

from .ros import ros_qt_to_rt


class FetchImageListener:
    def __init__(self):
        # Topics for subscriobtion
        self._color_topic = "/head_camera/rgb/image_raw"
        self._depth_topic = "/head_camera/depth_registered/image_raw"
        self._caminfo_topic = "/head_camera/rgb/camera_info"
        # frames for tf transformation
        self._base_frame = "base_link"
        self._camera_frame = "head_camera_rgb_optical_frame"

        self._tf_listener = tf.TransformListener()

        fs = [
            message_filters.Subscriber(self._color_topic, Image, queue_size=1),
            message_filters.Subscriber(self._depth_topic, Image, queue_size=1),
        ]
        ts = message_filters.ApproximateTimeSynchronizer(fs, 4, 0.1)
        ts.registerCallback(self._ts_callback)

        self._rgb = None
        self._depth = None
        self._cam_RT = None
        self._rgb_frame_id = None
        self._rgb_frame_stamp = None
        self._cam_K, self._im_height, self._im_width = self._get_camera_K(self._caminfo_topic)

    def _ts_callback(self, *msgs):
        if any(msg is None for msg in msgs):
            return
        cam_RT = self._get_camera_RT()
        if cam_RT is None:
            return
        self._rgb = self._image_msg_to_np(msgs[0])
        self._depth = self._image_msg_to_np(msgs[1])
        self._rgb_frame_id = msgs[0].header.frame_id
        self._rgb_frame_stamp = msgs[0].header.stamp
        self._cam_RT = cam_RT

    def _get_camera_K(self, caminfo_topic):
        msg = rospy.wait_for_message(caminfo_topic, CameraInfo)
        cam_model = PinholeCameraModel()
        cam_model.fromCameraInfo(msg)
        K = cam_model.intrinsicMatrix().astype("float32")
        img_height, img_width = cam_model.height, cam_model.width
        return K, img_height, img_width

    def _image_msg_to_np(self, image_msg):
        img = ros_numpy.numpify(image_msg)
        if image_msg.encoding == "rgb8":
            pass
        elif image_msg.encoding == "bgr8":
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        elif image_msg.encoding == "32FC1":
            pass
        elif image_msg.encoding == "16UC1":
            img = img.astype(np.float32) / 1000.0  # Convert to meters
        else:
            rospy.logwarn("Unsupported image encoding: {}".format(image_msg.encoding))

        return img

    def _get_camera_RT(self):
        try:
            trans, rot = self._tf_listener.lookupTransform(self._base_frame, self._camera_frame, rospy.Time(0))
            return ros_qt_to_rt(rot, trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to get camera transform from {} to {}".format(self._base_frame, self._camera_frame))
            return None

    def get_data(self):
        if self._rgb is None:
            return None, None, None, None
        return self._rgb, self._depth, self._cam_RT, self._cam_K

    @property
    def cam_K(self):
        return self._cam_K

    @property
    def im_width(self):
        return self._im_width

    @property
    def im_height(self):
        return self._im_height

    @property
    def rgb_frame_id(self):
        return self._rgb_frame_id

    @property
    def rgb_frame_stamp(self):
        return self._rgb_frame_stamp
