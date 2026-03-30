import os
from typing import List, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32, Int32MultiArray

from tflite_runtime.interpreter import Interpreter

class VisionDetectorNode(Node):
    def __init__(self):
        super().__init__('vision_detector_node')
        self.bridge = CvBridge()

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detected_topic', '/vision/detected')
        self.declare_parameter('confidence_topic', '/vision/confidence')
        self.declare_parameter('bbox_topic', '/vision/bbox')
        self.declare_parameter('class_id_topic', '/vision/class_id')
        self.declare_parameter('debug_image_topic', '/vision/debug_image')
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('target_class_id', -1)
        self.declare_parameter('target_label', 'drone')
        self.declare_parameter('image_timeout_sec', 1.0)
        self.declare_parameter('model_path', '')
        self.declare_parameter('score_threshold', 0.20)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('max_detections', 2)
        self.declare_parameter('process_every_n_frames', 3)
        self.declare_parameter('input_size', 640)
        self.declare_parameter('use_rgb', True)
        self.declare_parameter('print_output_shape_once', True)
        self.declare_parameter('log_every_n_inferences', 10)
        self.declare_parameter('debug_log_rejected_candidates', True)
        self.declare_parameter('prefer_objectness_when_class_scores_empty', True)

        image_topic = str(self.get_parameter('image_topic').value)
        detected_topic = str(self.get_parameter('detected_topic').value)
        confidence_topic = str(self.get_parameter('confidence_topic').value)
        bbox_topic = str(self.get_parameter('bbox_topic').value)
        class_id_topic = str(self.get_parameter('class_id_topic').value)
        debug_image_topic = str(self.get_parameter('debug_image_topic').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.target_class_id = int(self.get_parameter('target_class_id').value)
        self.target_label = str(self.get_parameter('target_label').value)
        self.image_timeout_sec = max(
            0.1, float(self.get_parameter('image_timeout_sec').value)
        )
        model_param = str(self.get_parameter('model_path').value)
        self.score_threshold = float(self.get_parameter('score_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.max_detections = int(self.get_parameter('max_detections').value)
        self.process_every_n_frames = max(
            1, int(self.get_parameter('process_every_n_frames').value)
        )
        self.input_size = int(self.get_parameter('input_size').value)
        self.use_rgb = bool(self.get_parameter('use_rgb').value)
        self.print_output_shape_once = bool(
            self.get_parameter('print_output_shape_once').value
        )
        self.log_every_n_inferences = max(
            1, int(self.get_parameter('log_every_n_inferences').value)
        )
        self.debug_log_rejected_candidates = bool(
            self.get_parameter('debug_log_rejected_candidates').value
        )
        self.prefer_objectness_when_class_scores_empty = bool(
            self.get_parameter('prefer_objectness_when_class_scores_empty').value
        )

        if model_param:
            self.model_path = model_param
        else:
            pkg_share = get_package_share_directory('yolo_detection')
            self.model_path = os.path.join(
                pkg_share,
                'models',
                'best_full_integer_quant.tflite',
            )

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10,
        )
        self.detect_pub = self.create_publisher(Bool, detected_topic, 10)
        self.conf_pub = self.create_publisher(Float32, confidence_topic, 10)
        self.bbox_pub = self.create_publisher(Int32MultiArray, bbox_topic, 10)
        self.class_id_pub = self.create_publisher(Int32, class_id_topic, 10)
        self.debug_image_pub = None
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(Image, debug_image_topic, 10)

        self.frame_count = 0
        self.inference_count = 0
        self.output_shape_printed = False
        self.last_image_time = self.get_clock().now()
        self.last_detection_state = None
        self.timeout_notified = False
        self.watchdog_timer = self.create_timer(0.2, self.watchdog_callback)

        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f'Model file not found: {self.model_path}')

        self.interpreter = Interpreter(model_path=self.model_path)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        inp = self.input_details[0]
        self.input_index = inp['index']
        self.input_dtype = inp['dtype']

        self.input_scale, self.input_zero_point = self._safe_quantization_params(inp)
        self.input_h, self.input_w = self._resolve_input_size(inp)

        self.get_logger().info(
            'VisionDetectorNode started. '
            f'image_topic={image_topic}, model={self.model_path}, '
            f'input=({self.input_w},{self.input_h}), dtype={self.input_dtype}, '
            f'score_threshold={self.score_threshold}, iou_threshold={self.iou_threshold}, '
            f'target_label={self.target_label}, target_class_id={self.target_class_id}'
        )

    def _safe_quantization_params(self, tensor_detail) -> Tuple[float, int]:
        scale, zero_point = tensor_detail.get('quantization', (0.0, 0))
        if scale in (None, 0.0):
            return 0.0, 0
        return float(scale), int(zero_point)

    def _resolve_input_size(self, tensor_detail) -> Tuple[int, int]:
        try:
            return int(tensor_detail['shape'][1]), int(tensor_detail['shape'][2])
        except Exception:
            return self.input_size, self.input_size

    def preprocess(self, image_bgr: np.ndarray) -> Tuple[np.ndarray, int, int]:
        orig_h, orig_w = image_bgr.shape[:2]
        img = cv2.resize(image_bgr, (self.input_w, self.input_h))

        if self.use_rgb:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        if self.input_dtype == np.float32:
            img = img.astype(np.float32) / 255.0
        else:
            img = img.astype(np.float32) / 255.0
            if self.input_scale > 0.0:
                img = img / self.input_scale + self.input_zero_point
            img = np.clip(
                img,
                np.iinfo(self.input_dtype).min,
                np.iinfo(self.input_dtype).max,
            ).astype(self.input_dtype)

        return np.expand_dims(img, axis=0), orig_w, orig_h

    def dequantize_output(self, output: np.ndarray, output_detail) -> np.ndarray:
        scale, zero_point = self._safe_quantization_params(output_detail)
        if scale > 0.0 and np.issubdtype(output.dtype, np.integer):
            return (output.astype(np.float32) - zero_point) * scale
        return output.astype(np.float32)

    def box_iou(self, box1: Sequence[float], box2: Sequence[float]) -> float:
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])

        inter_w = max(0.0, x2 - x1)
        inter_h = max(0.0, y2 - y1)
        inter = inter_w * inter_h

        area1 = max(0.0, box1[2] - box1[0]) * max(0.0, box1[3] - box1[1])
        area2 = max(0.0, box2[2] - box2[0]) * max(0.0, box2[3] - box2[1])
        union = area1 + area2 - inter + 1e-6
        return float(inter / union)

    def nms(
        self,
        boxes: np.ndarray,
        scores: np.ndarray,
        max_detections: int,
        iou_threshold: float,
    ) -> List[int]:
        if len(boxes) == 0:
            return []

        indices = np.argsort(scores)[::-1]
        keep = []

        while len(indices) > 0 and len(keep) < max_detections:
            current = int(indices[0])
            keep.append(current)

            remaining = []
            for idx in indices[1:]:
                iou = self.box_iou(boxes[current], boxes[int(idx)])
                if iou < iou_threshold:
                    remaining.append(int(idx))

            indices = np.array(remaining, dtype=np.int32)

        return keep

    def normalize_prediction_tensor(self, pred: np.ndarray) -> np.ndarray:
        if pred.ndim == 3 and pred.shape[0] == 1:
            pred = pred[0]

        if pred.ndim == 1:
            pred = np.expand_dims(pred, axis=0)

        if pred.ndim == 2 and pred.shape[0] < pred.shape[1]:
            pred = pred.T

        return pred

    def decode_predictions(
        self,
        outputs: List[np.ndarray],
        orig_w: int,
        orig_h: int,
    ) -> Tuple[List[List[int]], List[float], List[int], dict]:
        pred = self.normalize_prediction_tensor(outputs[0])

        if pred.ndim != 2 or pred.shape[1] < 5:
            self.get_logger().warn(f'Unexpected output shape: {pred.shape}')
            return [], [], [], {}

        boxes = []
        scores = []
        class_ids = []
        best_candidate = {
            'score': -1.0,
            'class_id': -1,
            'objectness': -1.0,
            'class_score': -1.0,
        }

        for row in pred:
            x, y, w, h = [float(v) for v in row[:4]]
            remainder = row[4:]
            objectness = -1.0
            best_class_score = -1.0

            if remainder.size == 1:
                score = float(remainder[0])
                class_id = 0
            elif remainder.size >= 2:
                objectness = float(remainder[0])
                cls_scores = remainder[1:]
                best_class = int(np.argmax(cls_scores))
                best_class_score = float(cls_scores[best_class])

                if (
                    self.prefer_objectness_when_class_scores_empty and
                    objectness > 0.0 and
                    best_class_score <= 1e-6
                ):
                    # Some single-class or quantized exports expose objectness as the
                    # usable confidence while class scores stay zeroed out.
                    score = objectness
                elif objectness <= 1.0 and best_class_score <= 1.0:
                    score = objectness * best_class_score
                else:
                    score = max(objectness, best_class_score)
                class_id = best_class
            else:
                continue

            if score > best_candidate['score']:
                best_candidate = {
                    'score': float(score),
                    'class_id': int(class_id),
                    'objectness': float(objectness),
                    'class_score': float(best_class_score),
                }

            if self.target_class_id >= 0 and class_id != self.target_class_id:
                continue

            if score < self.score_threshold:
                continue

            if max(abs(x), abs(y), abs(w), abs(h)) <= 2.0:
                x *= self.input_w
                y *= self.input_h
                w *= self.input_w
                h *= self.input_h

            x1 = x - w / 2.0
            y1 = y - h / 2.0
            x2 = x + w / 2.0
            y2 = y + h / 2.0

            scale_x = orig_w / self.input_w
            scale_y = orig_h / self.input_h

            x1 = int(max(0, min(orig_w - 1, x1 * scale_x)))
            y1 = int(max(0, min(orig_h - 1, y1 * scale_y)))
            x2 = int(max(0, min(orig_w - 1, x2 * scale_x)))
            y2 = int(max(0, min(orig_h - 1, y2 * scale_y)))

            if x2 <= x1 or y2 <= y1:
                continue

            boxes.append([x1, y1, x2, y2])
            scores.append(float(score))
            class_ids.append(class_id)

        return boxes, scores, class_ids, best_candidate

    def run_inference(
        self,
        image_bgr: np.ndarray,
    ) -> Tuple[bool, float, List[int], int]:
        input_tensor, orig_w, orig_h = self.preprocess(image_bgr)

        self.interpreter.set_tensor(self.input_index, input_tensor)
        self.interpreter.invoke()

        outputs = []
        for output_detail in self.output_details:
            raw_output = self.interpreter.get_tensor(output_detail['index'])
            outputs.append(self.dequantize_output(raw_output, output_detail))

        if self.print_output_shape_once and not self.output_shape_printed:
            for i, out in enumerate(outputs):
                self.get_logger().info(
                    f'Output[{i}] shape={out.shape}, dtype={out.dtype}'
                )
            self.output_shape_printed = True

        boxes, scores, class_ids, best_candidate = self.decode_predictions(
            outputs, orig_w, orig_h
        )

        if len(boxes) == 0:
            if self.debug_log_rejected_candidates and best_candidate:
                self.get_logger().info(
                    'No accepted detection. '
                    f'best_candidate_score={best_candidate["score"]:.3f}, '
                    f'class_id={best_candidate["class_id"]}, '
                    f'objectness={best_candidate["objectness"]:.3f}, '
                    f'class_score={best_candidate["class_score"]:.3f}, '
                    f'threshold={self.score_threshold:.3f}, '
                    f'target_class_id={self.target_class_id}'
                )
            return False, 0.0, [-1, -1, -1, -1], -1

        keep = self.nms(
            boxes=np.array(boxes),
            scores=np.array(scores),
            max_detections=self.max_detections,
            iou_threshold=self.iou_threshold,
        )

        if len(keep) == 0:
            return False, 0.0, [-1, -1, -1, -1], -1

        best_idx = keep[0]
        return (
            True,
            float(scores[best_idx]),
            boxes[best_idx],
            int(class_ids[best_idx]),
        )

    def publish_debug_frame(
        self,
        frame: np.ndarray,
        detected: bool,
        confidence: float,
        bbox: Sequence[int],
        class_id: int,
        source_msg: Image,
    ) -> None:
        if self.debug_image_pub is None:
            return

        debug_frame = frame.copy()
        label = f'detected={detected} conf={confidence:.2f} class={class_id}'
        color = (0, 200, 0) if detected else (0, 0, 255)

        if detected and bbox[0] >= 0:
            cv2.rectangle(
                debug_frame,
                (int(bbox[0]), int(bbox[1])),
                (int(bbox[2]), int(bbox[3])),
                color,
                2,
            )
        cv2.putText(
            debug_frame,
            label,
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2,
            cv2.LINE_AA,
        )

        debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
        debug_msg.header = source_msg.header
        self.debug_image_pub.publish(debug_msg)

    def publish_detection_state(
        self,
        detected: bool,
        confidence: float,
        bbox: Sequence[int],
        class_id: int,
    ) -> None:
        detect_msg = Bool()
        detect_msg.data = bool(detected)
        self.detect_pub.publish(detect_msg)

        conf_msg = Float32()
        conf_msg.data = float(confidence)
        self.conf_pub.publish(conf_msg)

        bbox_msg = Int32MultiArray()
        bbox_msg.data = [int(v) for v in bbox]
        self.bbox_pub.publish(bbox_msg)

        class_id_msg = Int32()
        class_id_msg.data = int(class_id)
        self.class_id_pub.publish(class_id_msg)

        self.last_detection_state = (
            bool(detected),
            float(confidence),
            [int(v) for v in bbox],
            int(class_id),
        )

    def publish_no_detection(self) -> None:
        self.publish_detection_state(False, 0.0, [-1, -1, -1, -1], -1)

    def watchdog_callback(self) -> None:
        elapsed = (
            self.get_clock().now() - self.last_image_time
        ).nanoseconds / 1e9

        if elapsed <= self.image_timeout_sec:
            self.timeout_notified = False
            return

        expected_state = (False, 0.0, [-1, -1, -1, -1], -1)
        if self.last_detection_state != expected_state:
            self.publish_no_detection()

        if not self.timeout_notified:
            self.get_logger().warn(
                f'No image received for {elapsed:.2f}s. Publishing no-drone state.'
            )
            self.timeout_notified = True

    def image_callback(self, msg: Image) -> None:
        self.last_image_time = self.get_clock().now()
        self.timeout_notified = False
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge convert failed: {exc}')
            return

        detected, confidence, bbox, class_id = self.run_inference(frame)
        self.inference_count += 1

        self.publish_detection_state(detected, confidence, bbox, class_id)

        self.publish_debug_frame(frame, detected, confidence, bbox, class_id, msg)

        if self.inference_count % self.log_every_n_inferences == 0 or detected:
            self.get_logger().info(
                'vision result: '
                f'detected={detected}, confidence={confidence:.3f}, '
                f'class_id={class_id}, bbox={bbox}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
