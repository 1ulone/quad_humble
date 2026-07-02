from rclpy.node import Node
import rclpy
import tkinter as tk
import threading
from quins.motor_controller import MotorData, MotorController

class Tuner(Node):
    def __init__(self):
        super().__init__('quins_tuner')

def main(args=None):
    rclpy.init(args=args)
    node = Tuner()

    root = tk.Tk()
    root.title("Motor Tuner")
    root.geometry("500x480")

    tk.Label(root, text="Motor 1 Angle by Degree").pack()
    m1 = tk.DoubleVar(value=0.0)
    tk.Entry(root, textvariable=m1).pack()

    tk.Label(root, text="Motor 1 Angle by Degree").pack()
    m2 = tk.DoubleVar(value=0.0)
    tk.Entry(root, textvariable=m2).pack()

    tk.Label(root, text="Motor 1 Angle by Degree").pack()
    m3 = tk.DoubleVar(value=0.0)
    tk.Entry(root, textvariable=m3).pack()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
