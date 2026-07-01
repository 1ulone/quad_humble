import rclpy
from rclpy.node import Node
import serial
import time
import math


class RobstrideNode(Node):
    def __init__(self):
        super().__init__('robstride_node')

        self.port = '/dev/ttyCH341USB0'
        self.baudrate = 921600  # confirmed in manual's connection dialog, p.8
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

        self.motor_id = 0x01   # target motor CAN_ID, default per param 0x200a (p.10)
        self.host_id = 0x00    # our host id, default per param 0x200b CAN_MASTER (p.10)

        # Target motion — start small and confirm movement before going bigger
        self.target_angle_rad = math.radians(30.0)
        self.kp = 5.0
        self.kd = 0.5

        self.enable_motor()
        self.timer = self.create_timer(0.02, self.send_command)  # 50 Hz
        self.get_logger().info("Node initialized and timer started.")

    def float_to_uint(self, x, x_min, x_max, bits):
        span = x_max - x_min
        offset = x_min
        if x > x_max:
            x = x_max
        elif x < x_min:
            x = x_min
        return int(((x - offset) * ((1 << bits) - 1)) / span)

    def send_can_packet(self, comm_type, target_id, data16, data):
        # Real 29-bit CAN ID: bit28-24=comm_type, bit23-8=data16, bit7-0=target_id
        # (manual pages 18-19)
        real_id = (comm_type << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)

        # Serial-frame encoding of the extended CAN ID: (real_id << 3) | 0x04
        # Reverse-engineered from the manual's own worked example (pages 14-15)
        encoded_id = (real_id << 3) | 0x04

        payload = bytearray([
            (encoded_id >> 24) & 0xFF,
            (encoded_id >> 16) & 0xFF,
            (encoded_id >> 8) & 0xFF,
            encoded_id & 0xFF,
            8,  # DLC
        ]) + data

        frame = bytearray([0x41, 0x54]) + payload + bytearray([0x0D, 0x0A])
        self.ser.write(frame)
        self.get_logger().info(frame.hex())
        self.get_logger().info(self.ser.read())

    def enable_motor(self):
        # Type 3: Enable. data16 field = host CAN ID, payload = 0 (manual p.20, 26)
        self.send_can_packet(3, self.motor_id, self.host_id, bytearray(8))
        self.get_logger().info("Sent Enable Frame (Type 3)")
        time.sleep(0.1)

    def stop_motor(self):
        # Type 4: Stop. data16 field = host CAN ID, payload = 0 (manual p.20, 27)
        self.send_can_packet(4, self.motor_id, self.host_id, bytearray(8))
        self.get_logger().info("Sent Stop Frame (Type 4)")

    def send_command(self):
        P_MIN, P_MAX = -12.5, 12.5      # ~-4pi..4pi (manual p.19, 25)
        V_MIN, V_MAX = -44.0, 44.0
        KP_MIN, KP_MAX = 0.0, 500.0
        KD_MIN, KD_MAX = 0.0, 5.0
        T_MIN, T_MAX = -17.0, 17.0

        torque_ff = 0.0
        velocity = 0.0

        p_int = self.float_to_uint(self.target_angle_rad, P_MIN, P_MAX, 16)
        v_int = self.float_to_uint(velocity, V_MIN, V_MAX, 16)
        kp_int = self.float_to_uint(self.kp, KP_MIN, KP_MAX, 16)
        kd_int = self.float_to_uint(self.kd, KD_MIN, KD_MAX, 16)
        t_int = self.float_to_uint(torque_ff, T_MIN, T_MAX, 16)

        data = bytearray(8)
        data[0] = p_int >> 8
        data[1] = p_int & 0xFF
        data[2] = v_int >> 8
        data[3] = v_int & 0xFF
        data[4] = kp_int >> 8
        data[5] = kp_int & 0xFF
        data[6] = kd_int >> 8
        data[7] = kd_int & 0xFF

        try:
            # Type 1: control command. data16 field = torque_ff (manual p.19, 26)
            self.send_can_packet(1, self.motor_id, t_int, data)
        except serial.SerialTimeoutException:
            self.get_logger().warn("Serial write timed out")

    def destroy_node(self):
        try:
            self.stop_motor()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobstrideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
