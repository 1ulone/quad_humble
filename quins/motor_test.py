import rclpy
from rclpy.node import Node
import serial
import time

class RobstrideNode(Node):
    def __init__(self):
        super().__init__('robstride_node')
        # Point to your confirmed device path
        self.port = '/dev/ttyCH341USB0'
        self.baudrate = 115200 # Adjust if your firmware uses a different rate
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        
        self.motor_id = 0x01
        self.timer = self.create_timer(0.1, self.send_command)

    def float_to_uint(self, x, x_min, x_max, bits):
        span = x_max - x_min
        offset = x_min
        if x > x_max: x = x_max
        elif x < x_min: x = x_min
        return int(((x - offset) * ((1 << bits) - 1)) / span)

    def send_command(self):
        angle, velocity, kp, kd, torque = 0.0, 0.0, 0.0, 0.0, 0.0

        P_MIN, P_MAX = -12.5, 12.5
        V_MIN, V_MAX = -30.0, 30.0
        KP_MIN, KP_MAX = 0.0, 500.0
        KD_MIN, KD_MAX = 0.0, 5.0
        T_MIN, T_MAX = -12.0, 12.0

        p_int = self.float_to_uint(angle, P_MIN, P_MAX, 16)
        v_int = self.float_to_uint(velocity, V_MIN, V_MAX, 12)
        kp_int = self.float_to_uint(kp, KP_MIN, KP_MAX, 12)
        kd_int = self.float_to_uint(kd, KD_MIN, KD_MAX, 12)
        t_int = self.float_to_uint(torque, T_MIN, T_MAX, 12)

        data = bytearray(8)
        data[0] = p_int >> 8
        data[1] = p_int & 0xFF
        data[2] = v_int >> 4
        data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)
        data[4] = kp_int & 0xFF
        data[5] = kd_int >> 4
        data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)
        data[7] = t_int & 0xFF

        # Serial frame format: [ID (1 byte)] + [8 bytes data]
        # This assumes your hardware serial bridge expects a simple 9-byte packet
        packet = bytearray([self.motor_id]) + data
        
        try:
            self.ser.write(packet)
            self.get_logger().info(f"Serial command sent to {self.port}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

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
