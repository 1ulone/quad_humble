import rclpy
from rclpy.node import Node
# from rcl_interfaces.msg import SetParametersResult
import serial
import time
import math
import tkinter as tk
import threading

KP = 1.0
KD = 0.2
VELOCITY = 0.44
TORQUE = 0.0

P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -44.0, 44.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -17.0, 17.0

class MotorData():
    def __init__(self, id):
        self.motor_id = id
        self.angle = 0.0

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        self.port = '/dev/ttyCH341USB0'
        self.baudrate = 921600 
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

        self.host_id = 253
        self.motors = []

        self.motors.append(MotorData(1))
        self.motors.append(MotorData(2))

        # Register callback to log parameter changes
        # self.add_on_set_parameters_callback(self.parameters_callback)

        for m in self.motors:
            self.enable_motor(m.motor_id)

        # self.timer = self.create_timer(0.02, self.send_command)
        self.get_logger().info("Programm Initialized. Ready for Tuning / Control.")

    # def parameters_callback(self, params):
    #     for param in params:
    #         if param.name == "angle_by_degree":
    #             self.get_logger().info(f"Updated {param.name} to {(param.value / 360) * 6.25}")
    #         else:
    #             self.get_logger().info(f"Updated {param.name} to {param.value}")
    #     return SetParametersResult(successful=True)

    # NOTE: float to integer, input into raw stuff
    def float_to_uint(self, x, x_min, x_max, bits):
        span = x_max - x_min
        offset = x_min
        if x > x_max: x = x_max
        elif x < x_min: x = x_min
        return int(((x - offset) * ((1 << bits) - 1)) / span)

    # NOTE: sending message to motor 
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

    # NOTE: enable motor 
    def enable_motor(self, target_id):
        self.send_can_packet(3, target_id, self.host_id, bytearray(8))
        time.sleep(0.1)

    # NOTE: stop motor 
    def stop_motor(self, target_id):
        self.send_can_packet(4, target_id, self.host_id, bytearray(8))


    # NOTE: sending command 
    def send_command(self, target_motor):
        # Fetch current parameter values dynamically from rqt
        pi2 = math.pi * 2
        angle_by_degree = (target_motor.angle / 360) * pi2 
        clamped_angle = min(pi2, max(-pi2, angle_by_degree))

        p_int = self.float_to_uint(clamped_angle, P_MIN, P_MAX, 16)
        v_int = self.float_to_uint(VELOCITY, V_MIN, V_MAX, 16)
        kp_int = self.float_to_uint(KP, KP_MIN, KP_MAX, 16)
        kd_int = self.float_to_uint(KD, KD_MIN, KD_MAX, 16)
        t_int = self.float_to_uint(TORQUE, T_MIN, T_MAX, 16)

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
            self.send_can_packet(1, target_motor.motor_id, t_int, data)
        except serial.SerialTimeoutException:
            pass

    def destroy_node(self):
        try:
            for m in self.motors:
                self.stop_motor(m.motor_id)
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()

    root = tk.Tk()
    root.title("Motor Tuner")
    root.geometry("500x480")

    tk.Label(root, text="Motor 1 Angle by Degree").pack()
    m1 = tk.DoubleVar(value=node.motors[0].angle)
    tk.Entry(root, textvariable=m1).pack()

    tk.Label(root, text="Motor 2 Angle by Degree").pack()
    m2 = tk.DoubleVar(value=node.motors[1].angle)
    tk.Entry(root, textvariable=m2).pack()

    # tk.Label(root, text="Motor 3 Angle by Degree").pack()
    # m3 = tk.DoubleVar(value=node.motors[2].angle)
    # tk.Entry(root, textvariable=m3).pack()

    def apply_angle(event=None):
        try:
            node.motors[0].angle = float(m1.get())
            node.send_command(node.motors[0])
        except ValueError:
            pass

    submit_button = tk.Button(root, text="Send Angle", command=apply_angle)
    submit_button.pack()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
