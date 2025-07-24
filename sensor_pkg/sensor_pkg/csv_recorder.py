#!/home/crisp_laptop/miniconda3/envs/ros2_env/bin/python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
from datetime import datetime
import threading
import serial
import numpy as np
import os

BASE_SAVE_DIR = '/home/crisp_laptop/robotiq_ws/src/sensor_pkg/sensor_pkg/recorded_data'

FORCE_LEN = 12  # 4 taxels * 3 axes

def parse_sensor_line(line):
    try:
        parts = line.split()
        if len(parts) != FORCE_LEN:
            return None
        values = list(map(float, parts))
        return values
    except Exception:
        return None

class SerialReaderThread(threading.Thread):
    def __init__(self, port, baud, buffer, lock, running_flag, calib_flag, calib_buffer):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.buffer = buffer
        self.lock = lock
        self.running_flag = running_flag
        self.calib_flag = calib_flag
        self.calib_buffer = calib_buffer
        self.ser = None
        self.latest_value = None

    def run(self):
        last_time = datetime.now().timestamp()
        count = 0
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            while True:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    values = parse_sensor_line(line)
                    if values is not None:
                        self.latest_value = values
                        # Calibration collection
                        if self.calib_flag['calibrating'] and len(self.calib_buffer) < 100:
                            self.calib_buffer.append(values)
                        # Normal recording
                        if self.running_flag['recording']:
                            timestamp = datetime.now().isoformat()
                            with self.lock:
                                self.buffer.append([timestamp] + values)
                        count += 1  # Only count valid sensor readings
                        current_time = datetime.now().timestamp()
                        if current_time - last_time >= 1.0:
                            frequency = count / (current_time - last_time)
                            print(f"[{self.port}] Reading frequency: {frequency:.1f} Hz")
                            count = 0
                            last_time = current_time
        except Exception as e:
            print(f"Serial thread error on {self.port}: {e}")
        finally:
            if self.ser:
                self.ser.close()

class CSVRecorder(Node):
    def __init__(self, save_dir):
        super().__init__('csv_recorder')
        self.recording_flag = {'recording': False}
        self.calib_flag0 = {'calibrating': False}
        self.calib_flag1 = {'calibrating': False}
        self.data0 = []
        self.data1 = []
        self.lock0 = threading.Lock()
        self.lock1 = threading.Lock()
        self.save_dir = save_dir
        self.calib_buffer0 = []
        self.calib_buffer1 = []
        self.calib0 = None
        self.calib1 = None
        self.calib_in_progress = False
        self.calib_logged = False

        self.get_logger().info(f"Saving data to: {self.save_dir}")

        self.subscription = self.create_subscription(
            String,
            '/start_recording',
            self.control_callback,
            10)

        # Start serial reader threads
        self.serial_thread0 = SerialReaderThread('/dev/ttyACM0', 115200, self.data0, self.lock0, self.recording_flag, self.calib_flag0, self.calib_buffer0)
        self.serial_thread1 = SerialReaderThread('/dev/ttyACM1', 115200, self.data1, self.lock1, self.recording_flag, self.calib_flag1, self.calib_buffer1)
        self.serial_thread0.start()
        self.serial_thread1.start()
        self.get_logger().info('CSV Recorder node with 2 serial threads started.\nDon\'t start recording until the frequency rate of both sensors print here.')

        # Timer for calibration check
        self.calib_timer = self.create_timer(0.1, self.calib_check)

    def control_callback(self, msg):
        command = msg.data.lower()
        print(f"Received command: {command}")
        if command == 'start' and not self.recording_flag['recording']:
            self.get_logger().info('Started recording.')
            self.recording_flag['recording'] = True
            with self.lock0:
                self.data0.clear()
            with self.lock1:
                self.data1.clear()
        elif command == 'stop' and self.recording_flag['recording']:
            self.get_logger().info('Stopped recording. Saving CSV...')
            self.recording_flag['recording'] = False
            self.save_csv()
        elif command == 'calibrate':
            self.get_logger().info('Calibration check: collecting 100 samples...')
            self.calib_buffer0.clear()
            self.calib_buffer1.clear()
            self.calib_flag0['calibrating'] = True
            self.calib_flag1['calibrating'] = True
            self.calib_in_progress = True
            self.calib_logged = False

    def calib_check(self):
        if self.calib_in_progress:
            if len(self.calib_buffer0) >= 100 and len(self.calib_buffer1) >= 100:
                self.calib_flag0['calibrating'] = False
                self.calib_flag1['calibrating'] = False
                self.calib0 = np.mean(self.calib_buffer0[:100], axis=0)
                self.calib1 = np.mean(self.calib_buffer1[:100], axis=0)
                if not self.calib_logged:
                    self.get_logger().info(f'Sensor 0 calibration: {np.round(self.calib0, 3)}')
                    self.get_logger().info(f'Sensor 1 calibration: {np.round(self.calib1, 3)}')
                    self.calib_logged = True
                self.calib_in_progress = False

    def save_csv(self):
        axes = ['x', 'y', 'z']
        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        save_dir = self.save_dir
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        with self.lock0:
            if self.data0:
                filename0 = f'{save_dir}/sensor0_data_{now}.csv'
                with open(filename0, 'w', newline='') as f:
                    writer = csv.writer(f)
                    header = ['timestamp'] + [f"{axes[i % 3]}{i // 3 + 1}" for i in range(FORCE_LEN)] + [f"{axes[i % 3]}{i // 3 + 1}_calib" for i in range(FORCE_LEN)]
                    writer.writerow(header)
                    for row in self.data0:
                        timestamp = row[0]
                        values = np.array(row[1:])
                        if self.calib0 is not None:
                            recalib = values - self.calib0
                        else:
                            recalib = [None]*FORCE_LEN
                        writer.writerow([timestamp] + list(values) + list(recalib))
                self.get_logger().info(f'Data0 saved to {filename0}')
        with self.lock1:
            if self.data1:
                filename1 = f'{save_dir}/sensor1_data_{now}.csv'
                with open(filename1, 'w', newline='') as f:
                    writer = csv.writer(f)
                    header = ['timestamp'] + [f"{axes[i % 3]}{i // 3 + 1}" for i in range(FORCE_LEN)] + [f"{axes[i % 3]}{i // 3 + 1}_calib" for i in range(FORCE_LEN)]
                    writer.writerow(header)
                    for row in self.data1:
                        timestamp = row[0]
                        values = np.array(row[1:])
                        if self.calib1 is not None:
                            recalib = values - self.calib1
                        else:
                            recalib = [None]*FORCE_LEN
                        writer.writerow([timestamp] + list(values) + list(recalib))
                self.get_logger().info(f'Data1 saved to {filename1}')

def main(args=None):
    rclpy.init(args=args)
    last_folder = input("Enter the name of the subfolder to save data in: ").strip()
    save_dir = os.path.join(BASE_SAVE_DIR, last_folder)
    print(save_dir)
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    node = CSVRecorder(save_dir)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
