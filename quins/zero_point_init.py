import serial
import struct
import time
import os
import json

PORT = 'COM6'
BAUDRATE = 921600
HOST_ID = 253
MOTOR_ID = 3

MECH_OFFSET_INDEX = 0x2005
BACKUP_FILE = f"mechoffset_backup_{MOTOR_ID}.json"


def build_frame(comm_type, target_id, data16, data):
    real_id = (comm_type << 24) | ((data16 & 0xFFFF) << 8) | (target_id & 0xFF)
    encoded_id = (real_id << 3) | 0x04

    payload = bytearray([
        (encoded_id >> 24) & 0xFF,
        (encoded_id >> 16) & 0xFF,
        (encoded_id >> 8) & 0xFF,
        encoded_id & 0xFF,
        8,
    ]) + data

    return bytearray([0x41, 0x54]) + payload + bytearray([0x0D, 0x0A])


def read_param(ser, motor_id, host_id, index):
    # Type 17: Single Parameter Read. Request payload: Byte0-1=index (low byte first), Byte2-7=0
    data = bytearray(8)
    data[0] = index & 0xFF
    data[1] = (index >> 8) & 0xFF

    frame = build_frame(17, motor_id, host_id, data)
    ser.reset_input_buffer()
    ser.write(frame)
    time.sleep(0.05)

    raw = ser.read(17)
    if len(raw) != 17 or raw[0:2] != b'\x41\x54' or raw[-2:] != b'\x0d\x0a':
        return None

    encoded_id = int.from_bytes(raw[2:6], 'big')
    real_id = (encoded_id - 0x04) >> 3
    comm_type = (real_id >> 24) & 0x1F

    if comm_type != 17:
        return None

    payload = raw[7:15]
    resp_index = payload[0] | (payload[1] << 8)
    value = struct.unpack('<f', payload[4:8])[0]

    return resp_index, value


def main():
    if os.path.exists(BACKUP_FILE):
        print(f"{BACKUP_FILE} already exists. Not overwriting. Exiting.")
        return

    ser = serial.Serial(PORT, BAUDRATE, timeout=0.2)

    result = read_param(ser, MOTOR_ID, HOST_ID, MECH_OFFSET_INDEX)
    ser.close()

    if result is None:
        print(f"Motor {MOTOR_ID}: no valid response to parameter read.")
        return

    resp_index, mech_offset = result

    if resp_index != MECH_OFFSET_INDEX:
        print(f"Motor {MOTOR_ID}: unexpected index in response ({resp_index:#x}), not saving.")
        return

    with open(BACKUP_FILE, 'w') as f:
        json.dump({'motor_id': MOTOR_ID, 'index': MECH_OFFSET_INDEX, 'mech_offset': mech_offset}, f)

    print(f"Motor {MOTOR_ID}: MechOffset={mech_offset} saved to {BACKUP_FILE}")


if __name__ == '__main__':
    main()
