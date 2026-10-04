import serial
import threading
import time


class TFminiReader:
    def __init__(self, port="/dev/serial0", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.distance = None
        self.strength = None
        self.running = False
        self.lock = threading.Lock()

    def _read_frame(self, ser):
        while self.running:
            b = ser.read(1)
            if b != b"\x59":
                continue

            b2 = ser.read(1)
            if b2 != b"\x59":
                continue

            frame = b + b2 + ser.read(7)
            if len(frame) != 9:
                return None

            checksum = sum(frame[0:8]) & 0xFF
            if checksum != frame[8]:
                return None

            distance = frame[2] + (frame[3] << 8)
            strength = frame[4] + (frame[5] << 8)

            valid = strength > 100 and strength != 65535 and distance != 0
            if not valid:
                return None

            return distance, strength

    def _loop(self):
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            ser.reset_input_buffer()

            while self.running:
                result = self._read_frame(ser)
                if result is None:
                    continue

                distance, strength = result

                with self.lock:
                    self.distance = distance
                    self.strength = strength

        except Exception as e:
            print(f"TFmini error: {e}")

    def start(self):
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def get_distance(self):
        with self.lock:
            return self.distance

    def stop(self):
        self.running = False
