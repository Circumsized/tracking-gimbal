from flask import Flask, Response
import cv2
import threading
import time


class MJPEGStreamer:
    def __init__(self, host="0.0.0.0", port=5000):
        self.app = Flask(__name__)
        self.host = host
        self.port = port
        self.frame = None
        self.lock = threading.Lock()

        self.app.add_url_rule("/", "index", self.index)
        self.app.add_url_rule("/video", "video", self.video)

    def update_frame(self, frame):
        with self.lock:
            self.frame = frame.copy()

    def index(self):
        return """
        <html>
        <head><title>Gimbal Tracking</title></head>
        <body>
            <h2>YuNet Face Tracking + TFmini</h2>
            <img src="/video" width="640">
        </body>
        </html>
        """

    def generate(self):
        while True:
            with self.lock:
                if self.frame is None:
                    time.sleep(0.02)
                    continue
                frame = self.frame.copy()

            ok, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + buffer.tobytes()
                + b"\r\n"
            )
            time.sleep(0.03)

    def video(self):
        return Response(
            self.generate(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    def start(self):
        threading.Thread(
            target=lambda: self.app.run(
                host=self.host,
                port=self.port,
                threaded=True,
                use_reloader=False,
            ),
            daemon=True,
        ).start()
