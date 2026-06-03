from __future__ import annotations

from pathlib import Path
import time
import cv2
import os

from ament_index_python.packages import get_package_share_directory
from bridge_interfaces.msg import VisionDetection, VisionDetectionArray
import rclpy
from rclpy.node import Node

from .detect_burger_pieces import classify_burger_piece # for camera check in burger_assembly_test.py

from .camera_utils import ManagedCamera
from .debug_utils import DetectionDebugWriter
from .model_utils import (
    DetectedObject,
    YoloNcnnDetector,
    default_model_path,
    resolve_model_path,
)
from .rule_based_detection import (
    detect_yellow_block,
)
from .stop_sign import classify_stop_sign_visibility
from .timing_utils import FixedRateScheduler
from .traffic_light import classify_traffic_light_color


# User-facing COCO class filter.
CLASSES_OF_INTEREST = [
    "traffic light",
    "stop sign",
    "person",
    "burger piece"
]

DEFAULT_CLASS_FILTER = ",".join(CLASSES_OF_INTEREST)


def classify_person_face_lighting(person_crop) -> tuple[str, float]:
    if person_crop.size == 0:
        return "unknown", 0.0

    crop_height, crop_width = person_crop.shape[:2]
    face_height = max(1, crop_height // 3)
    face_left = crop_width // 4
    face_right = max(face_left + 1, crop_width - face_left)
    face_crop = person_crop[:face_height, face_left:face_right]
    if face_crop.size == 0:
        return "unknown", 0.0

    brightness = float(face_crop.mean()) / 255.0
    if brightness < 0.25:
        return "dim", min(1.0, (0.25 - brightness) / 0.25)
    if brightness > 0.75:
        return "bright", min(1.0, (brightness - 0.75) / 0.25)
    return "normal", max(0.0, 1.0 - (abs(brightness - 0.5) / 0.25))


# ---------------------------------------------------------------------------
# Customer face matcher
# ---------------------------------------------------------------------------

CUSTOMER_A_IMAGE_PATH = "/ros2_ws/src/vision/data/customer_a.jpeg"
CUSTOMER_B_IMAGE_PATH = "/ros2_ws/src/vision/data/customer_b.jpeg"

FACE_MATCH_SIZE      = (128, 128)   # resize all crops to this before comparing
FACE_MATCH_THRESHOLD = 0.40         # minimum template match score to claim a match


class CustomerFaceMatcher:
    """
    Compares a live person crop against two stored reference images using
    template matching (spatial features). Returns 'A', 'B', or 'unknown'.
    """
    def __init__(self, path_a: str, path_b: str) -> None:
        self._ref_a = self._load(path_a, "A")
        self._ref_b = self._load(path_b, "B")

    @staticmethod
    def _load(path: str, label: str):
        if not os.path.exists(path):
            print(f"[FaceMatcher] WARNING: reference image for Customer {label} "
                  f"not found at {path}")
            return None
        img = cv2.imread(path)
        if img is None:
            print(f"[FaceMatcher] WARNING: could not read image at {path}")
            return None
        img  = cv2.resize(img, FACE_MATCH_SIZE)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return gray

    def _get_pattern(self, crop):
        if crop is None or crop.size == 0:
            return None
        resized = cv2.resize(crop, FACE_MATCH_SIZE)
        gray    = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        return gray

    def match(self, person_crop) -> tuple[str, float]:
        if self._ref_a is None and self._ref_b is None:
            return "unknown", 0.0

        live_pattern = self._get_pattern(person_crop)
        if live_pattern is None:
            return "unknown", 0.0

        score_a = -1.0
        if self._ref_a is not None:
            res_a = cv2.matchTemplate(self._ref_a, live_pattern, cv2.TM_CCOEFF_NORMED)
            score_a = float(res_a[0][0])

        score_b = -1.0
        if self._ref_b is not None:
            res_b = cv2.matchTemplate(self._ref_b, live_pattern, cv2.TM_CCOEFF_NORMED)
            score_b = float(res_b[0][0])

        if score_a > score_b and score_a > FACE_MATCH_THRESHOLD:
            return "A", score_a
        elif score_b > score_a and score_b > FACE_MATCH_THRESHOLD:
            return "B", score_b
        else:
            return "unknown", max(score_a, score_b)


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_node")

        self._share_data_dir = Path(get_package_share_directory("vision")) / "data"
        self._source_data_dir = Path("/ros2_ws/src/vision/data")
        model_default = default_model_path(self._source_data_dir, self._share_data_dir)

        self.declare_parameter("camera_device", "/dev/video10")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 15.0)
        self.declare_parameter("process_rate_hz", 4.0)
        self.declare_parameter("model_path", str(model_default))
        self.declare_parameter("model_imgsz", 416)
        self.declare_parameter("confidence_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.7)
        self.declare_parameter("max_detections", 20)
        self.declare_parameter("class_filter", DEFAULT_CLASS_FILTER)
        self.declare_parameter("ncnn_threads", 4)
        self.declare_parameter("reconnect_delay_sec", 1.0)
        self.declare_parameter("log_interval_sec", 5.0)
        self.declare_parameter("debug_save_enabled", False)
        self.declare_parameter("debug_output_dir", "/runtime_output/vision")
        self.declare_parameter("debug_save_rate_hz", 1.0)
        self.declare_parameter("debug_save_only_on_detection", True)
        self.declare_parameter("debug_save_latest", True)
        self.declare_parameter("debug_save_timestamped", False)
        self.declare_parameter("debug_max_timestamped_images", 100)

        self._camera_device = str(self.get_parameter("camera_device").value)
        self._camera_width = int(self.get_parameter("camera_width").value)
        self._camera_height = int(self.get_parameter("camera_height").value)
        self._camera_fps = float(self.get_parameter("camera_fps").value)
        self._process_rate_hz = float(self.get_parameter("process_rate_hz").value)
        self._model_imgsz = int(self.get_parameter("model_imgsz").value)
        self._confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self._iou_threshold = float(self.get_parameter("iou_threshold").value)
        self._max_detections = int(self.get_parameter("max_detections").value)
        self._class_filter = str(self.get_parameter("class_filter").value)
        self._ncnn_threads = int(self.get_parameter("ncnn_threads").value)
        self._reconnect_delay_sec = max(0.1, float(self.get_parameter("reconnect_delay_sec").value))
        self._log_interval_sec = max(1.0, float(self.get_parameter("log_interval_sec").value))

        self._publisher = self.create_publisher(VisionDetectionArray, "/vision/detections", 10)
        self._camera = ManagedCamera(
            device=self._camera_device,
            width=self._camera_width,
            height=self._camera_height,
            fps=self._camera_fps,
            reconnect_delay_sec=self._reconnect_delay_sec,
            log_interval_sec=self._log_interval_sec,
            logger=self.get_logger(),
        )
        self._debug_writer = DetectionDebugWriter(
            enabled=bool(self.get_parameter("debug_save_enabled").value),
            output_dir=str(self.get_parameter("debug_output_dir").value),
            save_rate_hz=float(self.get_parameter("debug_save_rate_hz").value),
            save_only_on_detection=bool(self.get_parameter("debug_save_only_on_detection").value),
            save_latest=bool(self.get_parameter("debug_save_latest").value),
            save_timestamped=bool(self.get_parameter("debug_save_timestamped").value),
            max_timestamped_images=int(self.get_parameter("debug_max_timestamped_images").value),
            logger=self.get_logger(),
        )
        self._last_loop_summary = 0.0

        model_path = resolve_model_path(
            raw_path=str(self.get_parameter("model_path").value),
            source_data_dir=self._source_data_dir,
            share_data_dir=self._share_data_dir,
        )
        self._detector = YoloNcnnDetector(
            model_path=model_path,
            input_size=self._model_imgsz,
            confidence_threshold=self._confidence_threshold,
            iou_threshold=self._iou_threshold,
            max_detections=self._max_detections,
            class_filter=self._class_filter,
            ncnn_threads=self._ncnn_threads,
        )
        self.get_logger().info(
            "Loaded NCNN YOLO model path=%s max_imgsz=%d classes=%d filter=%s ncnn_threads=%s confidence=%.2f yellow_block=%s"
            % (
                model_path,
                self._model_imgsz,
                self._detector.class_count,
                self._class_filter or "all",
                self._ncnn_threads if self._ncnn_threads > 0 else "auto",
                self._confidence_threshold,
                "on",
            )
        )

        # -------------------------------------------------------------------
        # Initialize Face Matcher at startup
        # -------------------------------------------------------------------
        self._face_matcher = CustomerFaceMatcher(
            CUSTOMER_A_IMAGE_PATH,
            CUSTOMER_B_IMAGE_PATH,
        )

    def _infer_yolo_detections(self, frame) -> list[DetectedObject]:
        return self._detector.predict(frame)

    def _detect_yellow_block(self, frame):
        return detect_yellow_block(frame)

    def _build_detection_msg(self, detected_object: DetectedObject) -> VisionDetection:
        detection = VisionDetection()
        detection.class_name = detected_object.class_name
        detection.confidence = float(detected_object.confidence)
        detection.x = int(detected_object.x)
        detection.y = int(detected_object.y)
        detection.width = int(detected_object.width)
        detection.height = int(detected_object.height)
        for attribute in detected_object.attributes:
            detection.attribute_names.append(attribute.name)
            detection.attribute_values.append(attribute.value)
            detection.attribute_scores.append(float(attribute.score))
        return detection

    def _build_detection_array_msg(
        self,
        capture_stamp,
        image_width: int,
        image_height: int,
        detected_objects: list[DetectedObject],
    ) -> VisionDetectionArray:
        message = VisionDetectionArray()
        message.header.stamp = capture_stamp
        message.header.frame_id = "vision_camera"
        message.image_width = int(image_width)
        message.image_height = int(image_height)
        for detected_object in detected_objects:
            message.detections.append(self._build_detection_msg(detected_object))
        return message

    def run(self) -> None:
        scheduler = FixedRateScheduler(self._process_rate_hz)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if not self._camera.ensure():
                scheduler.reset()
                continue

            scheduler.wait_until_ready()

            ok, frame = self._camera.read() # frame is a numpy array
            if not ok or frame is None:
                self._camera.handle_read_failure()
                scheduler.reset()
                continue

            capture_stamp = self.get_clock().now().to_msg()
            inference_start = time.monotonic()
            try:
                yolo_detections = self._infer_yolo_detections(frame)
                yellow_block_detections, yellow_block_overlays = self._detect_yellow_block(frame)
                
                for detection in yolo_detections:
                    object_crop = frame[
                        detection.y : detection.y + detection.height,
                        detection.x : detection.x + detection.width,
                    ]

                    if detection.class_name == "traffic light":
                        traffic_light_crop = object_crop
                        color_label, color_score = classify_traffic_light_color(traffic_light_crop)
                        detection.add_attribute("color", color_label, color_score)

                    elif detection.class_name == "stop sign":
                        stop_sign_crop = object_crop
                        visibility_label, visibility_score = classify_stop_sign_visibility(stop_sign_crop)
                        detection.add_attribute("visibility", visibility_label, visibility_score)
                        
                    elif detection.class_name == "person":
                        person_crop = object_crop
                        face_lighting_label, face_lighting_score = classify_person_face_lighting(person_crop)
                        detection.add_attribute("face_lighting", face_lighting_label, face_lighting_score)
                        
                        # -------------------------------------------------------------------
                        # Add customer matching attribute to the person detection
                        # -------------------------------------------------------------------
                        customer_id, match_score = self._face_matcher.match(person_crop)
                        detection.add_attribute("customer_id", customer_id, float(match_score))

                    elif detection.class_name == "burger piece":
                        piece_crop = object_crop
                        result = classify_burger_piece(piece_crop)
                        detection.add_attribute("burger_type",  result["label"],  result["score"])
                        detection.add_attribute("patty_score",  str(result["patty_score"]),  result["patty_score"])
                        detection.add_attribute("bun_score",    str(result["bun_score"]),    result["bun_score"])
                
                all_detections = yolo_detections + yellow_block_detections

                message = self._build_detection_array_msg(
                    capture_stamp=capture_stamp,
                    image_width=frame.shape[1],
                    image_height=frame.shape[0],
                    detected_objects=all_detections,
                )
                self._publisher.publish(message)
                self._debug_writer.maybe_write(
                    frame_bgr=frame,
                    detected_objects=all_detections,
                    debug_overlays=yellow_block_overlays,
                )
                yolo_count = len(yolo_detections)
                yellow_block_count = len(yellow_block_detections)
                detection_count = len(message.detections)
            except Exception as exc:
                self.get_logger().error(f"Vision inference failed for one frame: {exc}")
                yolo_count = 0
                yellow_block_count = 0
                detection_count = 0
            inference_ms = (time.monotonic() - inference_start) * 1000.0

            now = time.monotonic()
            if now - self._last_loop_summary >= self._log_interval_sec:
                self._last_loop_summary = now
                self.get_logger().info(
                    "Vision frame %dx%d total=%.1fms preprocess=%.1fms ncnn=%.1fms postprocess=%.1fms yolo=%d yellow_block=%d total=%d target_rate=%.1fHz"
                    % (
                        frame.shape[1],
                        frame.shape[0],
                        inference_ms,
                        self._detector.last_preprocess_ms,
                        self._detector.last_inference_ms,
                        self._detector.last_postprocess_ms,
                        yolo_count,
                        yellow_block_count,
                        detection_count,
                        self._process_rate_hz,
                    )
                )

            scheduler.schedule_next()

        self._camera.release()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()