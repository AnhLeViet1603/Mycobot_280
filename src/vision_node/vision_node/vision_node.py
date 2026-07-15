"""Detect the color of cubes on the conveyor using OpenCV HSV thresholding.

Subscribes to the camera image, finds the largest colored blob (red / blue /
green), classifies it, and publishes a ``DetectedObject`` with the blob
centroid. Optionally republishes an annotated debug image.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from custom_interfaces.msg import DetectedObject


# HSV ranges per color. Hue is 0-179 in OpenCV. Red wraps around 0/180 so it
# needs two ranges.
HSV_RANGES = {
    'red': [((0, 100, 60), (10, 255, 255)),
            ((170, 100, 60), (179, 255, 255))],
    'green': [((40, 80, 40), (85, 255, 255))],
    'blue': [((100, 100, 40), (130, 255, 255))],
}


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('min_area', 300.0)
        self.declare_parameter('publish_debug', True)

        # Detection region of interest (pixels). Only blobs inside this box are
        # reported, so with the camera mounted above the pusher a cube is
        # "seen" only as it passes right in front of the paddle. Set a small
        # central box. -1 on a max means "to the image edge".
        self.declare_parameter('roi_enabled', True)
        self.declare_parameter('roi_x_min', 220)
        self.declare_parameter('roi_x_max', 420)
        self.declare_parameter('roi_y_min', 210)
        self.declare_parameter('roi_y_max', 270)

        self.min_area = self.get_parameter('min_area').value
        self.publish_debug = self.get_parameter('publish_debug').value
        image_topic = self.get_parameter('image_topic').value
        self.roi_enabled = self.get_parameter('roi_enabled').value
        self.roi_x_min = self.get_parameter('roi_x_min').value
        self.roi_x_max = self.get_parameter('roi_x_max').value
        self.roi_y_min = self.get_parameter('roi_y_min').value
        self.roi_y_max = self.get_parameter('roi_y_max').value

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, image_topic, self.on_image, 10)
        self.pub = self.create_publisher(
            DetectedObject, '/detected_object', 10)
        self.debug_pub = (
            self.create_publisher(Image, '/vision/debug_image', 10)
            if self.publish_debug else None)

        self.get_logger().info(
            f'vision_node ready: subscribing [{image_topic}], '
            f'min_area {self.min_area}')

    def _roi_bounds(self, w, h):
        x1 = self.roi_x_max if self.roi_x_max >= 0 else w
        y1 = self.roi_y_max if self.roi_y_max >= 0 else h
        x0 = max(0, min(self.roi_x_min, w))
        y0 = max(0, min(self.roi_y_min, h))
        return x0, min(x1, w), y0, min(y1, h)

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge failed: {exc}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        h, w = hsv.shape[:2]
        x0, x1, y0, y1 = self._roi_bounds(w, h)

        best = None  # (area, color, cx, cy, contour)
        for color, ranges in HSV_RANGES.items():
            mask = None
            for lo, hi in ranges:
                m = cv2.inRange(hsv, np.array(lo), np.array(hi))
                mask = m if mask is None else cv2.bitwise_or(mask, m)

            if self.roi_enabled:
                # Zero everything outside the ROI so blobs are only found there.
                mask[:y0, :] = 0
                mask[y1:, :] = 0
                mask[:, :x0] = 0
                mask[:, x1:] = 0

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < self.min_area:
                    continue
                if best is None or area > best[0]:
                    m = cv2.moments(c)
                    if m['m00'] == 0:
                        continue
                    cx = m['m10'] / m['m00']
                    cy = m['m01'] / m['m00']
                    best = (area, color, cx, cy, c)

        if best is not None:
            _, color, cx, cy, contour = best
            out = DetectedObject()
            out.class_name = color
            # Position in image pixels (x, y); z carries the blob area so
            # downstream nodes can gauge distance/size if needed.
            out.position.x = float(cx)
            out.position.y = float(cy)
            out.position.z = float(best[0])
            self.pub.publish(out)

            if self.debug_pub is not None:
                cv2.drawContours(frame, [contour], -1, (0, 255, 255), 2)
                cv2.circle(frame, (int(cx), int(cy)), 4, (0, 0, 255), -1)
                cv2.putText(
                    frame, color, (int(cx) - 20, int(cy) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if self.debug_pub is not None:
            if self.roi_enabled:
                cv2.rectangle(frame, (x0, y0), (x1, y1), (255, 0, 0), 1)
            dbg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            dbg.header = msg.header
            self.debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
