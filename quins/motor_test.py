import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import serial
import time
import math

class RobstrideTunerNode(Node):
    def __init__(self):
        super().__init__('robstride_tuner_node')

        self.port = '/dev/ttyCH341USB0'
        self.baudrate = 921600 
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

        self.motor_id = 127
        self.host_id = 253

        # Declare ROS 2 parameters for dynamic tuning
        self.declare_parameter('target_angle_rad', 0.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('velocity', 0.44)
        self.declare_parameter('torque_ff', 0.0)

        # Register callback to log parameter changes
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.enable_motor()
        self.timer = self.create_timer(0.02, self.send_command)
        self.get_logger().info("Tuner node initialized. Open rqt to tune.")

    def parameters_callback(self, params):
        for param in params:
            if param.name == "target_angle_rad":
                self.get_logger().info(f"Updated {param.name} to {(param.value / 360) * 6.25}")
            else:
                self.get_logger().info(f"Updated {param.name} to {param.value}")
        return SetParametersResult(successful=True)

    def float_to_uint(self, x, x_min, x_max, bits):
        span = x_max - x_min
        offset = x_min
        if x > x_max: x = x_max
        elif x < x_min: x = x_min
        return int(((x - offset) * ((1 << bits) - 1)) / span)

    def send_can_packet(self, comm_type, target_id, data16, data):
        real_id = (comm_type << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)
        encoded_id = (real_id << 3) | 0x04

        payload = bytearray([
            (encoded_id >> 24) & 0xFF,
            (encoded_id >> 16) & 0xFF,
            (encoded_id >> 8) & 0xFF,
            encoded_id & 0xFF,
            8,
        ]) + data

        frame = bytearray([0x41, 0x54]) + payload + bytearray([0x0D, 0x0A])
        self.ser.write(frame)

    def enable_motor(self):
        self.send_can_packet(3, self.motor_id, self.host_id, bytearray(8))
        time.sleep(0.1)

    def stop_motor(self):
        self.send_can_packet(4, self.motor_id, self.host_id, bytearray(8))

    def send_command(self):
        # Fetch current parameter values dynamically from rqt
        target_angle_rad = (self.get_parameter('target_angle_rad').value / 360) * 6.25 
        kp = self.get_parameter('kp').value
        kd = self.get_parameter('kd').value
        velocity = self.get_parameter('velocity').value
        torque_ff = self.get_parameter('torque_ff').value

        clamp = min(6.25, max(-6.25, target_angle_rad))

        P_MIN, P_MAX = -12.5, 12.5
        V_MIN, V_MAX = -44.0, 44.0
        KP_MIN, KP_MAX = 0.0, 500.0
        KD_MIN, KD_MAX = 0.0, 5.0
        T_MIN, T_MAX = -17.0, 17.0

        p_int = self.float_to_uint(clamp, P_MIN, P_MAX, 16)
        v_int = self.float_to_uint(velocity, V_MIN, V_MAX, 16)
        kp_int = self.float_to_uint(kp, KP_MIN, KP_MAX, 16)
        kd_int = self.float_to_uint(kd, KD_MIN, KD_MAX, 16)
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
            self.send_can_packet(1, self.motor_id, t_int, data)
        except serial.SerialTimeoutException:
            pass

    def destroy_node(self):
        try:
            self.stop_motor()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RobstrideTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
