from pathlib import Path
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

PUB_TOPICS = {
    "colorTopic": "/dummy_camera/color/image_raw",
    "depthTopic": "/dummy_camera/depth/image_raw",
}


def read_rgb_image(image_path):
    return cv2.cvtColor(cv2.imread(str(image_path)), cv2.COLOR_BGR2RGB)


def read_depth_image(image_path):
    return cv2.imread(str(image_path), cv2.IMREAD_ANYDEPTH)


class RecordingPublisher:
    def __init__(self, recording_dir, fps=30) -> None:
        # Initialize the Node
        self._init_node("recording_publisher")
        self._bridge = CvBridge()
        self._rate = rospy.Rate(fps)
        self._color_files = sorted(Path(recording_dir).glob("color/*.jpg"))
        self._depth_files = sorted(Path(recording_dir).glob("depth/*.png"))

        assert len(self._color_files) == len(self._depth_files), "Color and depth files must match in number"
        self._num_frames = len(self._color_files)
        self._frame_id = -1

        # Define publishers
        self._colorPub = rospy.Publisher(PUB_TOPICS["colorTopic"], Image, queue_size=1)
        self._depthPub = rospy.Publisher(PUB_TOPICS["depthTopic"], Image, queue_size=1)

    def _init_node(self, node_name):
        self._node = rospy.init_node(node_name, anonymous=True)
        rospy.loginfo(f"Node initialized: {node_name}")

    def _get_next_frame(self):
        self._frame_id = (self._frame_id + 1) % self._num_frames
        color = cv2.imread(str(self._color_files[self._frame_id]), cv2.IMREAD_COLOR)
        depth = cv2.imread(str(self._depth_files[self._frame_id]), cv2.IMREAD_ANYDEPTH)
        return color, depth

    def run(self):
        rospy.loginfo("Start publishing recording frames...")
        while not rospy.is_shutdown():
            # Read and publish video frames, or loop if video is over

            try:
                color_frame, depth_frame = self._get_next_frame()
                if color_frame is None or depth_frame is None:
                    continue
                # Convert OpenCV images to ROS messages
                color_msg = self._bridge.cv2_to_imgmsg(color_frame)
                depth_msg = self._bridge.cv2_to_imgmsg(depth_frame)
                stamp = rospy.Time.now()
                color_msg.header.stamp = stamp
                depth_msg.header.stamp = stamp

                # Publish the messages
                self._colorPub.publish(color_msg)
                self._depthPub.publish(depth_msg)

            except CvBridgeError as e:
                rospy.logerr(e)

            self._rate.sleep()


def main():
    camera_publisher = RecordingPublisher(recording_dir)
    camera_publisher.run()


if __name__ == "__main__":
    recording_dir = "demo/recordings/demo_video"
    main()
