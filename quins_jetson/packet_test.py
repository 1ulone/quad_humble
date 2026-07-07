import serial
import time

PORT = '/dev/ttyCH341USB0'
BAUDRATE = 921600
HOST_ID = 253

ser = serial.Serial(PORT, BAUDRATE, timeout=0.2)

def send_can_packet(comm_type, target_id, data16, data):
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
    ser.write(frame)
    return frame

def get_device_id(target_id):
    # Type 0: Get Device ID. Request: comm_type=0, data16=host_id, target_id=motor_id, data=0
    ser.reset_input_buffer()
    frame = send_can_packet(0, target_id, HOST_ID, bytearray(8))
    print(f"Sent to ID {target_id}: {frame.hex()}")

    time.sleep(0.1)
    response = ser.read(64)
    print(f"Raw response: {response.hex() if response else '(nothing received)'}")
    print("---")

get_device_id(3)
get_device_id(4)

ser.close()
