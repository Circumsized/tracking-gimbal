from picamera2 import Picamera2
import cv2
import time
import os


class YuNetCamera:
    def __init__(
        self,
        model_path="/home/jr/models/face_detection_yunet_2023mar.onnx",
        capture_size=(1640, 1232),
        # capture_size=(2304, 1296),        
        stream_size=(640, 480),
    ):
        if not os.path.exists(model_path):
            raise FileNotFoundError(model_path)

        self.capture_size = capture_size
        self.stream_size = stream_size

        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": self.capture_size, "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()

        self.picam2.set_controls({
            "AeEnable": True,
            "AwbEnable": True,
            "FrameDurationLimits": (20000, 20000)
        })

        time.sleep(1)

        self.detector = cv2.FaceDetectorYN.create(
            model_path,
            "",
            self.stream_size,
            score_threshold=0.6,
            nms_threshold=0.3,
            top_k=5000,
        )

    def read(self):
        frame = self.picam2.capture_array()
        frame = cv2.resize(frame, self.stream_size)

        # Picamera2 输出 RGB，OpenCV/YuNet 使用 BGR
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        return frame

    def detect_faces(self, frame):
        self.detector.setInputSize(self.stream_size)
        _, faces = self.detector.detect(frame)

        if faces is None:
            return [], None

        result = []

        for f in faces:
            x, y, w, h = f[:4].astype(int)

            result.append({
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "cx": x + w // 2,
                "cy": y + h // 2,
                "area": w * h,
            })

        # 默认选择面积最大的人脸作为跟踪目标
        selected_face = max(result, key=lambda face: face["area"])

        return result, selected_face
