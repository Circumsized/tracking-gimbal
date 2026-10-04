#!/usr/bin/env python3
import time
import cv2

from camera_yunet import YuNetCamera
from tfmini_uart import TFminiReader
from gimbal_can import GimbalCAN
from mjpeg_stream import MJPEGStreamer


STREAM_W = 640
STREAM_H = 480

CENTER_X = STREAM_W // 2
CENTER_Y = STREAM_H // 2

YAW_DIRECTION = -1
PITCH_DIRECTION = 1

YAW_GAIN = 0.003
PITCH_GAIN = 0.003

MAX_YAW_SPEED = 100.0
MAX_PITCH_SPEED = 100.0

MIN_YAW_SPEED = 0.02
MIN_PITCH_SPEED = 0.02

DEADZONE_X = 15
DEADZONE_Y = 15

CONTROL_PERIOD = 0.02


def clamp(x, lo, hi):
    return max(lo, min(x, hi))


def calc_speed(error, gain, min_speed, max_speed, deadzone, direction=1):
    if abs(error) <= deadzone:
        return 0.0

    vel = abs(error) * gain
    vel = clamp(vel, min_speed, max_speed)

    if error < 0:
        vel = -vel

    return vel * direction


def draw_overlay(frame, face, distance):
    h, w = frame.shape[:2]
    cx0, cy0 = w // 2, h // 2

    cross_color = (0, 0, 255)

    if face is not None:
        x = face["x"]
        y = face["y"]
        fw = face["w"]
        fh = face["h"]

        face_cx = face["cx"]
        face_cy = face["cy"]

        err_x = face_cx - cx0
        err_y = face_cy - cy0

        if x <= cx0 <= x + fw and y <= cy0 <= y + fh:
            cross_color = (0, 255, 0)

        cv2.rectangle(frame, (x, y), (x + fw, y + fh), (0, 255, 0), 2)

        cv2.putText(frame, f"faces={face['count']}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.putText(frame, f"err_x={err_x}, err_y={err_y}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    else:
        cv2.putText(frame, "No face", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    cv2.line(frame, (cx0 - 20, cy0), (cx0 + 20, cy0), cross_color, 2)
    cv2.line(frame, (cx0, cy0 - 20), (cx0, cy0 + 20), cross_color, 2)

    if distance is None:
        distance_text = "Distance: -- cm"
    else:
        distance_text = f"Distance: {distance} cm"

    text_size, _ = cv2.getTextSize(distance_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    text_x = w - text_size[0] - 10

    cv2.putText(frame, distance_text, (text_x, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    return frame


def main():
    camera = YuNetCamera()
    lidar = TFminiReader()
    streamer = MJPEGStreamer()
    gimbal = GimbalCAN()

    lidar.start()
    streamer.start()

    last_control_time = time.time()

    print("Starting speed-mode face tracking...")
    print("Browser: http://<raspberrypi_ip>:5000")
    print("Press Ctrl+C to stop.")

    try:
        gimbal.enable()
        time.sleep(0.5)
        gimbal.stop()

        while True:
            frame = camera.read()
            face = camera.detect_largest_face(frame)
            distance = lidar.get_distance()

            now = time.time()

            if now - last_control_time >= CONTROL_PERIOD:
                if face is not None:
                    err_x = face["cx"] - CENTER_X
                    err_y = face["cy"] - CENTER_Y

                    yaw_vel = calc_speed(
                        err_x,
                        YAW_GAIN,
                        MIN_YAW_SPEED,
                        MAX_YAW_SPEED,
                        DEADZONE_X,
                        YAW_DIRECTION
                    )

                    pitch_vel = calc_speed(
                        err_y,
                        PITCH_GAIN,
                        MIN_PITCH_SPEED,
                        MAX_PITCH_SPEED,
                        DEADZONE_Y,
                        PITCH_DIRECTION
                    )

                    gimbal.send_speed(yaw_vel, pitch_vel)

                else:
                    gimbal.stop()

                last_control_time = now

            display_frame = draw_overlay(frame, face, distance)
            streamer.update_frame(display_frame)

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        try:
            gimbal.stop()
            time.sleep(0.2)
            gimbal.disable()
            gimbal.close()
            lidar.stop()
        except Exception as e:
            print(f"Shutdown error: {e}")

        print("Exit.")


if __name__ == "__main__":
    main()
