import rclpy
import tkinter as tk

def main(args=None):
    rclpy.init(args=args)

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

    root.mainloop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
