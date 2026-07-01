import rclpy
from rclpy.node import Node
import serial
import time

class RobstrideNode(Node):
    def __init__(self):
        super().__init__('robstride_node')
        self.port = '/dev/ttyCH341USB0'
        # Baud rate must be 921600 per Robstride official specs[cite: 1]
        self.baudrate = 921600 
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        
        self.motor_id = 0x01
        
        # Mandatory Enable sequence[cite: 1]
        self.enable_motor()
        
        self.timer = self.create_timer(0.1, self.send_command)
        self.get_logger().info("Node initialized and timer started.")

    def float_to_uint(self, x, x_min, x_max, bits):
        span = x_max - x_min
        offset = x_min
        if x > x_max: x = x_max
        elif x < x_min: x = x_min
        return int(((x - offset) * ((1 << bits) - 1)) / span)

    def send_can_packet(self, comm_type, motor_id, data):
        # Format bits 28-24 as Comm Type, bits 7-0 as Motor ID[cite: 1]
        can_id = (comm_type << 24) | (motor_id & 0xFF)
        
        # Bridge Protocol: [4 bytes ID] + [1 byte DLC] + [8 bytes DATA]
        frame = bytearray([
            (can_id >> 24) & 0xFF, (can_id >> 16) & 0xFF,
            (can_id >> 8) & 0xFF,  (can_id & 0xFF),
            8 # DLC = 8
        ]) + data
        self.ser.write(frame)

    def enable_motor(self):
        # Communication Type 3: Enable motor[cite: 1]
        self.send_can_packet(3, self.motor_id, bytearray(8))
        self.get_logger().info("Sent Enable Frame (Type 3)")
        time.sleep(0.1)

    def send_command(self):
        self.get_logger().info("send_command triggered")
        # Target values
        angle, velocity, kp, kd, torque = 0.0, 0.0, 0.0, 0.0, 0.0

        # Limits defined in manual[cite: 1]
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

        # Communication Type 1: Control Instruction[cite: 1]
        try:
            self.send_can_packet(1, self.motor_id, data)
        except serial.SerialTimeoutException:
            self.get_logger().warn("Serial write timed out")

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
