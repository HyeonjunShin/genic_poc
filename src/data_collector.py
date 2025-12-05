import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, UInt8MultiArray
import struct
import numpy as np
import argparse
import os
from datetime import datetime
import csv

PACKET_SIZE = 141

def unpack_packet(msg):
    raw = bytes(msg.data)

    if len(raw) < PACKET_SIZE:
        raise ValueError(f"Invalid packet size {len(raw)} (expected {PACKET_SIZE})")

    # 1) header (4 bytes)
    header = raw[0:4]  # bytes 타입

    # 2) data_size (uint16, offset=4)
    data_size = struct.unpack_from("<H", raw, 4)[0]

    # 3) tick (uint32, offset=6)
    tick = struct.unpack_from("<I", raw, 6)[0]

    # 4) flag (bool, stored as uint8, offset=10)
    flag = struct.unpack_from("<?", raw, 10)[0]

    # 5) mic1 (32 * uint16, offset=11 → length 64 bytes)
    mic1 = np.frombuffer(raw, dtype="<u2", count=32, offset=11)

    # 6) mic2 (32 * uint16, offset=75 → length 64 bytes)
    mic2 = np.frombuffer(raw, dtype="<u2", count=32, offset=11+64)

    # 7) crc (uint16, offset=139)
    crc = struct.unpack_from("<H", raw, 11+64+64)[0]

    return {
        "header": header,
        "data_size": data_size,
        "tick": tick,
        "flag": flag,
        "mic1": mic1.copy(),   # numpy는 view라서 copy 추천
        "mic2": mic2.copy(),
        "crc": crc,
    }

def checkPath(path: str) -> bool:
    if "/" in path or "\\" in path:
        _, ext = os.path.splitext(path)
        return ext == ""
    return False

def checkDirectory(path):
    if not os.path.isdir(path):
        print(f"Make the directories: {path}")
        os.makedirs(path, exist_ok=True)
    else:
        print(f"Alread exist the directory: {path}")

class Listener(Node):
    def __init__(self, path):
        super().__init__('data_collector_node')
        self.path = path

        self.maskDetect = 0
        self.robotStatus = 0
        self.isSave = False
        self.tempData = []
        self.startTime = None  

        topics = self.get_topic_names_and_types()
        print("=== Active Topics ===")
        for name, types in topics:
            print(f"{name} : {types}")

        self.create_subscription(Int32, '/ethercat/mask_detect', self.maskDetectCallback, 10)
        self.create_subscription(Int32, '/robot/step_status', self.robotStatusCallback, 10)
        self.create_subscription(UInt8MultiArray, '/sensor/raw', self.piezoSensorCallback, 10)


    def maskDetectCallback(self, msg):
        self.maskDetect = msg.data

        if self.maskDetect == 100 and not self.isSave:
            self.isSave = True
            self.startTime = datetime.now()
            print("recording started")

    def robotStatusCallback(self, msg):
        self.robotStatus = msg.data

        if self.robotStatus == 7 and self.isSave:
            self.isSave = False
            print("recording finished")
            self.saveData()

    def piezoSensorCallback(self, msg):
        if not self.isSave:
            return

        packet = unpack_packet(msg)

        # store every frame → absolutely no loss
        self.tempData.append([
            packet["tick"],
            self.robotStatus,
            packet["mic1"],
            packet["mic2"]
        ])

    def saveData(self):
        if not self.tempData:
            print("No data to save.")
            return

        # File name: YYYYMMDD_HHMMSS.csv
        filename = self.startTime.strftime("%Y%m%d_%H%M%S") + ".csv"
        save_path = os.path.join(self.path, filename)

        print(f"Saving {len(self.tempData)} frames to {save_path}")

        # CSV saving
        with open(save_path, "w", newline="") as f:
            writer = csv.writer(f)

            # 데이터 작성
            for tick, status, mic1, mic2 in self.tempData:
                writer.writerow([tick, status] + mic1.tolist() + mic2.tolist())

        print("Clearing tempData...")
        self.tempData = []
        self.startTime = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", type=str, help="For the save file location.", default="./")

    args = parser.parse_args()
    path = args.path
    if(not checkPath(path)):
        print("Path is not available.")
        exit(1)
    print("path for the data save is", path)
    
    path = os.path.join(path, "output")
    checkDirectory(path)
        
    rclpy.init()
    rclpy.spin(Listener(path))
    rclpy.shutdown()

if __name__ == "__main__":
    main()
