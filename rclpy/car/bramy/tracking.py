import itertools
import threading
import time
from traceback import print_exc

import cv2
import onnxruntime as ort
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

import rclpy
from bramy.utils.utils import from_numpy, non_max_suppression, preprocess, scale_coords
from rclpy.node import Node


#640 640
class tracking(Node):
    def __init__(self):
        super().__init__('tracking')
        self.angle = 0
        self.get_logger().info("load model")
        self.model = ort.InferenceSession(
            r"/home/pi/Downloads/alexnet.onnx", providers=["CPUExecutionProvider"])
        self.get_logger().info("load model complete")
        self._rgb_image = None
        self._depth_image = None
        self.bridge = CvBridge()
        self.output_names = [x.name for x in self.model.get_outputs()]
        self.inputName = self.model.get_inputs()[0].name
        self.threshold = 0.3



        self.publisher_ = self.create_publisher(Float32MultiArray, 'cordinates', 1)
        self.create_subscription(
            Image,
            'color_image',
            self._rgb_callback,
            1  # QoS: queue size
        )

        # self.create_subscription(
        #     Image,
        #     'depth_image',
        #     self._depth_callback,
        #     1  # QoS: queue size
        # )
        
        threading.Thread(target=self.detect, daemon=True).start()

    def _rgb_callback(self, msg):
        """
        Callback for the color image topic.
        Converts the ROS Image message to OpenCV format (BGR8).
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self._rgb_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting RGB image: {e}')
            self._rgb_image = None

    # def _depth_callback(self, msg):
    #     """
    #     Callback for the depth image topic.
    #     Converts the ROS Image message to OpenCV format (passthrough).
    #     """
    #     try:
    #         # Use "passthrough" to retain the original format of the depth image (e.g., 16UC1, 32FC1)
    #         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
    #         self._depth_image = cv_image
    #     except Exception as e:
    #         self.get_logger().error(f'Error converting Depth image: {e}')
    #         self._depth_image = None
    
    def detect(self):
        while 1:
            prev_time = time.time()
            try:
                if self._rgb_image is not None:
                    rgb_img = self._rgb_image.copy()
                    rgb_img = cv2.flip(rgb_img, 1)
                    # depth_img = self._depth_image.copy()
                    im0, img = preprocess(320, rgb_img)
                    pred = self.model.run(self.output_names, {self.inputName: img})
                    pred = from_numpy(pred[0])
                    pred = pred.float()
                    pred = non_max_suppression(pred, self.threshold, 0.4)
                    pred_boxes = []
                    for det in pred:
                        if det is not None and len(det):
                            det[:, :4] = scale_coords(
                                img.shape[2:], det[:, :4], im0.shape).round()
                            for *x, conf, cls_id in det:
                                x1, y1 = int(x[0]), int(x[1])
                                x2, y2 = int(x[2]), int(x[3])

                                # cx = int((x1 + x2) / 2)
                                # cy = int((y1 + y2) / 2)
                                pred_boxes.append((x1, y1, x2, y2, conf))
                                cv2.rectangle(rgb_img, (x1, y1), (x2, y2), (0,255,0), -1, cv2.LINE_AA)  # filled

                    cv2.imshow("123", rgb_img)
                    cv2.waitKey(1)
                    msg = Float32MultiArray()
                    msg.data = list(itertools.chain.from_iterable(pred_boxes))
                    self.publisher_.publish(msg)
            except:  # noqa: E722
                print_exc()
            current_time = time.time()
            fps = 1.0 / (current_time - prev_time)
            prev_time = current_time
            self.get_logger().info(f'FPS : {fps}')

def main(arg=None):
    rclpy.init()
    node = tracking()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
