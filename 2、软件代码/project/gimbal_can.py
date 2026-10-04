import struct
import time
import can


class GimbalCAN:
    def __init__(self, channel="can0"):
        self.bus = can.interface.Bus(channel=channel, interface="socketcan")

        # 速度模式：control mode = 2
        self.yaw_id = 0x201
        self.pitch_id = 0x202

        self.enable_cmd = bytes([0xFF] * 7 + [0xFC])
        self.disable_cmd = bytes([0xFF] * 7 + [0xFD])

    def send(self, can_id, data):
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        self.bus.send(msg, timeout=0.05)

    def enable(self):
        self.send(self.yaw_id, self.enable_cmd)
        time.sleep(0.02)
        self.send(self.pitch_id, self.enable_cmd)

    def disable(self):
        self.stop()
        time.sleep(0.05)
        self.send(self.yaw_id, self.disable_cmd)
        time.sleep(0.02)
        self.send(self.pitch_id, self.disable_cmd)

    def pack_speed(self, vel):
        # 速度模式：8字节中只用前4字节float，小端
        return struct.pack("<f", float(vel)) + bytes([0x00, 0x00, 0x00, 0x00])

    def send_yaw_speed(self, vel):
        self.send(self.yaw_id, self.pack_speed(vel))

    def send_pitch_speed(self, vel):
        self.send(self.pitch_id, self.pack_speed(vel))

    def send_speed(self, yaw_vel, pitch_vel):
        self.send_yaw_speed(yaw_vel)
        time.sleep(0.005)
        self.send_pitch_speed(pitch_vel)

    def stop(self):
        self.send_speed(0.0, 0.0)

    def close(self):
        self.bus.shutdown()
