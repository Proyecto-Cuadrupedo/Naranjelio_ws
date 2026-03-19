import sys
import numpy as np
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget, QLabel
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QColor, QFont
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from threading import Thread, Lock
from collections import deque
import time
import math

import csv

class IMUPlotter(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'imu_plotter')
        QWidget.__init__(self)
        
        self.csv_filename = 'data/log.csv'
        self.start_time = time.time()
        # Subscriber
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'IMU',
            self.imu_callback,
            10
        )
        
        with open('log.csv', mode = 'a', newline='') as csvfile:
            writer= csv.DictWriter(csvfile,fieldnames={'time','ax','ay','az', 'vx','vy','vz','comp'})
            writer.writeheader()
        
        self.compp=0
        
        
        self.imu_data_lock = Lock()
        self.history_length = 1000
        self.smoothing_window = 10  # Size of the moving average window
        self.time_stamps = deque(maxlen=self.history_length)
        self.imu_data_buffers = [deque(maxlen=self.history_length) for _ in range(6)]  # Buffers for the first six sensors

        # Header Label
        self.header_label = QLabel("Naranjelio")
        self.header_label.setAlignment(Qt.AlignCenter)
        self.header_label.setFont(QFont('Arial', 24))
        self.header_label.setStyleSheet("QLabel { color : orange; }")

        # Compass label
        self.compass_label = QLabel("Compass Heading: 0°")
        self.compass_label.setAlignment(Qt.AlignCenter)
        self.compass_label.setStyleSheet("QLabel { color : white; }")
        
        
        #qgq label
        self.qgq_label = QLabel("QGQ Value: 0.00")
        self.qgq_label.setAlignment(Qt.AlignLeft)
        self.qgq_label.setFont(QFont('Arial', 18))
        self.qgq_label.setStyleSheet("QLabel { color : white; }")
        

        # Create a figure and a set of subplots for the first six sensors only
        self.figure, self.axes = plt.subplots(2, 3, figsize=(15, 10))
        self.figure.patch.set_facecolor('#000000')  # Set the background color of the figure to black
        self.titles = ["X Acceleration", "Y Acceleration", "Z Acceleration", 
                       "X Angular Velocity", "Y Angular Velocity", "Z Angular Velocity"]
        self.units = ["Force (g)","Force (g)","Force (g)",
                      "deg/s","deg/s","deg/s"]
        for ax, title in zip(self.axes.flatten(), self.titles):
            ax.set_facecolor('#000000')
            ax.tick_params(axis='x', colors='white')
            ax.tick_params(axis='y', colors='white')
            ax.xaxis.label.set_color('white')
            ax.yaxis.label.set_color('white')
            ax.title.set_color('orange')
            ax.set_title(title)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Measurement Units')

        self.canvas = FigureCanvas(self.figure)
        self.canvas.setStyleSheet("background-color: black;")

        # Use QVBoxLayout to manage the layout of widgets in the GUI
        layout = QVBoxLayout()
        layout.addWidget(self.header_label)
        layout.addWidget(self.canvas)
        layout.addWidget(self.compass_label)
        layout.addWidget(self.qgq_label)
        self.setLayout(layout)
        self.setStyleSheet("background-color: black;")

        # Timer to update the plot and compass heading in the GUI thread
        self.plot_timer = QTimer(self)
        self.plot_timer.setInterval(10)  # Update the plot every 100 ms
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start()

        self.show()
        
        
        
        

    def imu_callback(self, msg):
        with self.imu_data_lock:
            current_time = time.time() - self.start_time  # Get current time for the x-axis
            self.time_stamps.append(current_time)
            for i in range(6):  # Update only the first six sensors
                self.imu_data_buffers[i].append(msg.data[i])
            # Calculate compass heading
            self.update_compass_heading(msg.data[6], msg.data[7])
            with open(self.csv_filename, mode = 'a', newline='') as csvfile:
                writer= csv.DictWriter(csvfile,fieldnames={'time','ax','ay','az', 'vx','vy','vz','comp'})
                logdata={'time':current_time,'ax':msg.data[0],'ay':msg.data[1],'az':msg.data[2], 'vx':msg.data[3],'vy':msg.data[4],'vz':msg.data[5],'comp':self.compp}
                writer.writerow(logdata)
            self.update_qgq(msg.data)
            

    def update_qgq(self,datos):
       
        qgq_value = datos[0]*100-abs(datos[1]*3)-(datos[2]-1)*5  # Simple example formula: modify as needed
        self.qgq_label.setText(f"QGQ Value: {qgq_value:.2f}")

    def update_compass_heading(self, x, y):
        heading_radians = math.atan2(y, x)
        heading_degrees = math.degrees(heading_radians)
        self.compp=heading_degrees
        if heading_degrees < 0:
            heading_degrees += 360
        self.compass_label.setText(f"Compass Heading: {heading_degrees:.2f}°")

    def update_plot(self):
        with self.imu_data_lock:
            time_array = np.array(self.time_stamps)
            try:
                for i, ax in enumerate(self.axes.flatten()):
                    ax.clear()
                    ax.set_facecolor('#000000')  # Ensure subplot backgrounds stay black
                    ax.tick_params(axis='x', colors='white')
                    ax.tick_params(axis='y', colors='white')
                    ax.xaxis.label.set_color('white')
                    ax.yaxis.label.set_color('white')
                    ax.title.set_color('orange')
                    ax.set_title(self.titles[i])
                    ax.set_xlabel('Time (s)')
                    ax.set_ylabel(self.units[i])
                    raw_data = np.array(self.imu_data_buffers[i])
                    if len(raw_data) >= self.smoothing_window:
                        # Compute moving average
                        smoothed_data = np.convolve(raw_data, np.ones(self.smoothing_window)/self.smoothing_window, mode='valid')
                        if len(smoothed_data) > 0:
                            data_min = np.min(smoothed_data)
                            data_max = np.max(smoothed_data)
                            if data_min == data_max:  # Check if all values are the same
                                data_min -= 0.5  # Adjust min down by arbitrary value
                                data_max += 0.5  # Adjust max up by arbitrary value
                            padding = (data_max - data_min) * 0.5  # Adjust padding percentage if needed
                            ax.set_ylim([data_min - padding, data_max + padding])
                            ax.plot(time_array[-len(smoothed_data):], smoothed_data, 'r-')  # Plot smoothed data
                    else:
                        if len(raw_data) > 0:
                            data_min = np.min(raw_data)
                            data_max = np.max(raw_data)
                            if data_min == data_max:  # Check if all values are the same
                                data_min -= 0.5  # Adjust min down by arbitrary value
                                data_max += 0.5  # Adjust max up by arbitrary value
                            padding = (data_max - data_min) * 0.5
                            ax.set_ylim([data_min - padding, data_max + padding])
                            ax.plot(time_array, raw_data, 'r-')  # Plot raw data if not enough for smoothing
                    self.canvas.draw()
                    
                    
                    
            except Exception as e:
                self.get_logger().error(f'Failed to update plot: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    imu_plotter = IMUPlotter()
    thread = Thread(target=rclpy.spin, args=(imu_plotter,), daemon=True)
    thread.start()
    app.exec_()
    imu_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
