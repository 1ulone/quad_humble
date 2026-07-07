import rclpy
from rclpy.node import Node
import time
import math
import serial
import tkinter as tk
from tkinter import ttk
import threading

from quins_jetson.zero_point_init import get_angle_relative, reset_zero_position

PORT = '/dev/ttyCH341USB0'

KP = 1.0
KD = 0.2
VELOCITY = 0.44
TORQUE = 0.0

P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -44.0, 44.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -17.0, 17.0

HOST_ID = 253


class MotorData():
    def __init__(self, id):
        self.motor_id = id
        self.angle = 0.0
        self.enabled = False
        self.last_fault = None
        self.zero_point = 0.0
        self.prev_zero_point = 0.0

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # self.port = PORT
        # self.baudrate = 921600
        # self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        #
        # self.host_id = HOST_ID
        self.motors = []

        self.motors.append(MotorData(3))
        self.motors.append(MotorData(4))

        # for m in self.motors:
        #     self.enable_motor(m)
        #     time.sleep(0.2)
        #
        # self.get_logger().info("Program Initialized. Ready for Tuning / Control.")
        # self.send_command(self.motors[0])
        # self.send_command(self.motors[1])

    def float_to_uint(self, x, x_min, x_max, bits):
        span = x_max - x_min
        offset = x_min
        if x > x_max:
            x = x_max
        elif x < x_min:
            x = x_min
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
        self.ser.reset_input_buffer()
        self.ser.write(frame)

        return self.read_response()

    def read_response(self):
        # Response frame: 41 54 [4-byte encoded id] [DLC] [8 data bytes] 0d 0a = 17 bytes
        raw = self.ser.read(17)
        if len(raw) != 17 or raw[0:2] != b'\x41\x54' or raw[-2:] != b'\x0d\x0a':
            return None
        return self.decode_response(raw)

    def decode_response(self, raw):
        encoded_id = int.from_bytes(raw[2:6], 'big')
        real_id = (encoded_id - 0x04) >> 3

        comm_type = (real_id >> 24) & 0x1F
        data16 = (real_id >> 8) & 0xFFFF
        target_id = real_id & 0xFF
        data = raw[7:15]

        result = {'comm_type': comm_type, 'target_id': target_id, 'data16': data16, 'raw_data': data}

        if comm_type == 2:
            # Type 2 feedback: bit8-15 = current CAN ID, bit16-21 = fault, bit22-23 = mode
            can_id = data16 & 0xFF
            fault_bits = (data16 >> 8) & 0x3F
            mode = (data16 >> 14) & 0x03
            result.update({
                'can_id': can_id,
                'fault_bits': fault_bits,
                'mode': mode,
                'angle_raw': int.from_bytes(data[0:2], 'big'),
                'velocity_raw': int.from_bytes(data[2:4], 'big'),
                'torque_raw': int.from_bytes(data[4:6], 'big'),
                'temp_raw': int.from_bytes(data[6:8], 'big'),
            })

        return result

    def enable_motor(self, m: MotorData):
        resp = self.send_can_packet(3, m.motor_id, self.host_id, bytearray(8))
        if resp is None:
            self.get_logger().warn(f"Motor {m.motor_id}: no response to Enable frame")
            m.enabled = False
            return
        if resp.get('fault_bits'):
            self.get_logger().warn(f"Motor {m.motor_id}: fault bits {resp['fault_bits']:#x} on enable")
        m.enabled = True
        self.get_logger().info(f"Motor {m.motor_id}: enabled, mode={resp.get('mode')}")

    def stop_motor(self, m: MotorData):
        self.send_can_packet(4, m.motor_id, self.host_id, bytearray(8))
        m.enabled = False

    def set_zero_position(self, m: MotorData):


        data = bytearray(8)
        data[0] = 1
        resp = self.send_can_packet(6, m.motor_id, self.host_id, data)
        if resp is None:
            self.get_logger().warn(f"Motor {m.motor_id}: no response to Set Zero frame")
            return
        self.get_logger().info(f"Motor {m.motor_id}: zero position set")

    def send_command(self, target_motor: MotorData):

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

        resp = self.send_can_packet(1, target_motor.motor_id, t_int, data)

        if resp is None:
            self.get_logger().warn(f"Motor {target_motor.motor_id}: no feedback frame received")
            target_motor.enabled = False
            return

        fault_bits = resp.get('fault_bits', 0)
        if fault_bits:
            if target_motor.last_fault != fault_bits:
                self.get_logger().warn(f"Motor {target_motor.motor_id}: fault bits {fault_bits:#x}")
            target_motor.last_fault = fault_bits

        mode = resp.get('mode')
        if mode == 0:  # Reset mode
            self.get_logger().warn(f"Motor {target_motor.motor_id}: dropped to RESET mode, re-enabling")
            self.enable_motor(target_motor)

    def destroy_node(self):
        for m in self.motors:
            try:
                self.stop_motor(m)
            except Exception as e:
                self.get_logger().error(f"Motor {m.motor_id}: failed to send stop on shutdown: {e}")
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)
    node = MotorController()

    root = tk.Tk()
    root.title("Motor Tuner")
    root.geometry("500x480")

    motor_input = []
    zeropoint_input = []

    def apply_angle(event=None):
        try:
            for motor in node.motors:
                motor.angle = float(motor_input[id].get())
        except ValueError:
            return

        def send_cmd():
            for motor in node.motors:
                node.send_command(motor)

        threading.Thread(target=send_cmd, daemon=True).start()

    def apply_zero_point(event=None):
        try:
            for id, motor in enumerate(node.motors):
                motor.zero_point = float(zeropoint_input[id])
                motor.angle = get_angle_relative(motor.prev_zero_point, motor.zero_point)
        except ValueError:
            return

        def send_both():
            for motor in node.motors:
                if motor.zero_point == motor.prev_zero_point:
                    continue

                node.send_command(motor)
                time.sleep(0.5)

                node.set_zero_position(motor)
                time.sleep(0.5)

                motor.angle = 0
                motor.prev_zero_point = motor.zero_point

        threading.Thread(target=send_both, daemon=True).start()

    def jog_motor(id, isRight):
        newAngle = min(360, max(-360, node.motors[id].angle+10 if isRight==True else node.motors[id].angle-10))
        node.motors[id].angle = newAngle

        def send_cmd(d):
            node.send_command(node.motors[d])

        threading.Thread(target=lambda: send_cmd(id), daemon=True).start()

    def set_current_zp(id):
        def send_cmd(motor):
            node.set_zero_position(motor)

        threading.Thread(target=lambda: send_cmd(node.motors[id]), daemon=True).start()



    grid_container = tk.Frame(root)
    grid_container.pack(pady=10, padx=10)

    for id, motor in enumerate(node.motors):
        motor_group = ttk.LabelFrame(grid_container, text=f"Motor {id+1}", padding=15)
        motor_group.grid(row=0, column=id)

        # NOTE: set Angle by Degree Input 
        ttk.Label(motor_group, text=f"Motor {id+1} Angle (rad)").pack()
        motor_input.append(tk.DoubleVar(value=motor.angle))
        ttk.Entry(motor_group, textvariable=motor_input[id]).pack(fill='x')

        # NOTE: set Zero point via Angle
        ttk.Label(motor_group, text=f"Set Motor {id+1} Zero Point").pack()
        zeropoint_input.append(tk.DoubleVar(value=motor.zero_point))
        ttk.Entry(motor_group, textvariable=zeropoint_input[id]).pack(fill='x')

        jm = ttk.Frame(motor_group)
        jm.pack(side="top", pady=10)

        ttk.Label(jm, text=f"Jog Motor {id+1}").pack()
        ttk.Button(jm, text="+", command=lambda: jog_motor(id, True)).pack(side="left", padx=5)
        ttk.Button(jm, text="-", command=lambda: jog_motor(id, False)).pack(side="left", padx=5)

        zpca = tk.Button(motor_group, text="Set Zero Point from Current Angle", command=lambda: set_current_zp(id))
        zpca.pack()

    angle_subbutton = tk.Button(grid_container, text="Send Angle", command=apply_angle)
    angle_subbutton.grid(row=3, column=0, columnspan=2, sticky="nsew", pady=(20, 5))

    zero_subbutton = tk.Button(grid_container, text="Set Zero Point", command=apply_zero_point)
    zero_subbutton.grid(row=4, column=0, columnspan=2, sticky="nsew", pady=5)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()


    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
