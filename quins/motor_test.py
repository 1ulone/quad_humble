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

    def send_can_packet(self, comm_type, motor_id, torque_int, data):
        # ID Structure: CommType (5 bits) | Torque (16 bits) | Motor ID (8 bits)[cite: 1]
        can_id = (comm_type << 24) | ((torque_int & 0xFFFF) << 8) | (motor_id & 0xFF)
        
        # Construct the internal payload
        payload = bytearray([
            (can_id >> 24) & 0xFF, (can_id >> 16) & 0xFF,
            (can_id >> 8) & 0xFF,  (can_id & 0xFF),
            8 # DLC
        ]) + data
        
        # Wrap in official protocol: Header [0x41, 0x54] + Payload + Footer [0x0D, 0x0A][cite: 1]
        frame = bytearray([0x41, 0x54]) + payload + bytearray([0x0D, 0x0A])
        
        self.ser.write(frame)

    def enable_motor(self):
        # Communication Type 3: Enable motor[cite: 1]
        # Torque is 0 for enable packet
        self.send_can_packet(3, self.motor_id, 0, bytearray(8))
        self.get_logger().info("Sent Enable Frame (Type 3)")
        time.sleep(0.1)

    def send_command(self):
        # Target values
        angle, velocity, kp, kd, torque = 0.0, 0.0, 0.0, 0.0, 0.0

        # Correct Limits defined in the C-code examples[cite: 1]
        P_MIN, P_MAX = -12.5, 12.5
        V_MIN, V_MAX = -44.0, 44.0
        KP_MIN, KP_MAX = 0.0, 500.0
        KD_MIN, KD_MAX = 0.0, 5.0
        T_MIN, T_MAX = -17.0, 17.0

        # ALL control variables are cast to 16 bits[cite: 1]
        p_int = self.float_to_uint(angle, P_MIN, P_MAX, 16)
        v_int = self.float_to_uint(velocity, V_MIN, V_MAX, 16)
        kp_int = self.float_to_uint(kp, KP_MIN, KP_MAX, 16)
        kd_int = self.float_to_uint(kd, KD_MIN, KD_MAX, 16)
        t_int = self.float_to_uint(torque, T_MIN, T_MAX, 16)

        # 8-byte payload contains only Pos, Vel, Kp, Kd[cite: 1]
        data = bytearray(8)
        data[0] = p_int >> 8
        data[1] = p_int & 0xFF
        data[2] = v_int >> 8
        data[3] = v_int & 0xFF
        data[4] = kp_int >> 8
        data[5] = kp_int & 0xFF
        data[6] = kd_int >> 8
        data[7] = kd_int & 0xFF

        # Communication Type 1: Control Instruction. Pass t_int into the ID[cite: 1]
        try:
            self.send_can_packet(1, self.motor_id, t_int, data)
            # Log the full serial frame to verify header/footer inclusion
            self.get_logger().info(f"send data :{data}") 
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
