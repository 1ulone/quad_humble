import rclpy
from rclpy.node import Node
import can

class RobstrideNode(Node):
    def __init__(self):
        super().__init__('robstride_node')
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.motor_id = 0x01  # Change if your motor ID is different
        self.timer = self.create_timer(0.1, self.send_command)

    def float_to_uint(self, x, x_min, x_max, bits):
        span = x_max - x_min
        offset = x_min
        if x > x_max: x = x_max
        elif x < x_min: x = x_min
        return int(((x - offset) * ((1 << bits) - 1)) / span)

    def send_command(self):
        # Target inputs
        angle = 0.0       # rad
        velocity = 0.0    # rad/s
        kp = 0.0
        kd = 0.0
        torque = 0.0      # Nm

        # Motor limits (Verify with your specific RobStride datasheet)
        P_MIN = -12.5; P_MAX = 12.5
        V_MIN = -30.0; V_MAX = 30.0
        KP_MIN = 0.0; KP_MAX = 500.0
        KD_MIN = 0.0; KD_MAX = 5.0
        T_MIN = -12.0; T_MAX = 12.0

        # Bit packing
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

        msg = can.Message(arbitration_id=self.motor_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            self.get_logger().info(f"Command sent to ID {self.motor_id}")
        except can.CanError:
            self.get_logger().error("CAN message failed to send")

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
