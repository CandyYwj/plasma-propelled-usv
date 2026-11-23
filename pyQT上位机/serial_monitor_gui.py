import sys
from PySide6.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QWidget, QLabel, 
                             QComboBox, QPushButton, QHBoxLayout, QCheckBox, QGridLayout, QSplitter, QSizePolicy, QGroupBox, QLineEdit)
from PySide6.QtCore import QThread, Signal, QIODevice, QTime, Qt, QTimer, QSize
import serial.tools.list_ports
import serial
import time
import re
import numpy as np # Import numpy for min/max calculations
from queue import Queue # 新增

import matplotlib
# Use the QtAgg backend for embedding in PySide6
matplotlib.use('QtAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar
from PySide6.QtGui import QIcon

# Enable Chinese characters in Matplotlib
matplotlib.rcParams['font.sans-serif'] = ['SimHei']
matplotlib.rcParams['axes.unicode_minus'] = False


class SerialReaderThread(QThread):
    error_occurred = Signal(str)

    def __init__(self, serial_port, data_queue):
        super().__init__()
        self.serial_port = serial_port
        self.data_queue = data_queue
        self._running = True

    def run(self):
        print("Serial reader thread started.")
        while self._running and self.serial_port.isOpen():
            try:
                data = self.serial_port.read_until(b'\r\n').decode('utf-8', errors='ignore').strip()
                if data:
                    self.data_queue.put(data)
            except serial.SerialException as e:
                error_msg = f"串口读取错误: {e}"
                print(error_msg)
                self.error_occurred.emit(error_msg)
                self._running = False # Stop the thread on serial error
            except Exception as e:
                error_msg = f"未知错误: {e}"
                print(error_msg)
                self.error_occurred.emit(error_msg)
                self._running = False # Stop the thread on other errors
        print("Serial reader thread stopped.")

    def stop(self):
        self._running = False
        self.wait()


class SerialMonitorGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("智水监测者：基于等离子体推进的河道智慧监测系统")
        self.setGeometry(100, 100, 800, 600) # Increased window size for plot

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Use a QVBoxLayout for the main layout within the central widget
        main_layout = QVBoxLayout(central_widget) # Set central_widget as parent of main_layout

        # --- Control Area Widget ---
        control_widget = QWidget()
        control_layout = QVBoxLayout(control_widget) # Use QVBoxLayout for controls

        # --- Top Row Layout (Serial Connection and Measurement) ---
        top_row_layout = QHBoxLayout()
        control_layout.addLayout(top_row_layout)

        # --- Serial Connection and Measurement Group ---
        self.conn_meas_group_box = QGroupBox("串口连接与测量")
        # Set a background color for the connection and measurement group box
        self.conn_meas_group_box.setStyleSheet("QGroupBox { background-color: #e8e8e8; border: 1px solid gray; border-radius: 5px; margin-top: 1ex;} QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 3px 0 3px;}")
        # Use a single horizontal layout to fit all elements in one row within the group box
        conn_meas_inner_layout = QHBoxLayout(self.conn_meas_group_box)

        self.com_label = QLabel("选择 COM 口:")
        self.com_label.setStyleSheet("font-size: 11pt;")
        conn_meas_inner_layout.addWidget(self.com_label)

        self.com_combobox = QComboBox()
        self.com_combobox.setStyleSheet("font-size: 11pt;")
        conn_meas_inner_layout.addWidget(self.com_combobox, 3) # Give COM combobox some stretch priority

        # Add Baud Rate selection
        self.baud_label = QLabel("波特率:")
        self.baud_label.setStyleSheet("font-size: 11pt;")
        conn_meas_inner_layout.addWidget(self.baud_label)

        self.baud_combobox = QComboBox()
        self.baud_combobox.setStyleSheet("font-size: 11pt;")
        # Populate with common baud rates
        self.baud_combobox.addItems(["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"])
        self.baud_combobox.setCurrentText("115200") # Set a default value
        conn_meas_inner_layout.addWidget(self.baud_combobox, 1) # Give baud combobox less stretch priority

        # Modify Connect button text and add to layout
        self.connect_button = QPushButton("打开串口") # Changed text to match screenshot
        self.connect_button.clicked.connect(self.connect_serial)
        self.connect_button.setStyleSheet("font-size: 11pt;")
        conn_meas_inner_layout.addWidget(self.connect_button)

        # Add Measurement buttons to the same horizontal layout
        self.start_button = QPushButton("开始测量")
        self.start_button.clicked.connect(self.start_measurement)
        self.start_button.setEnabled(False)
        self.start_button.setStyleSheet("font-size: 11pt;")
        conn_meas_inner_layout.addWidget(self.start_button)

        self.stop_button = QPushButton("停止测量")
        self.stop_button.clicked.connect(self.stop_measurement)
        self.stop_button.setEnabled(False)
        self.stop_button.setStyleSheet("font-size: 11pt;")
        conn_meas_inner_layout.addWidget(self.stop_button)

        # No extra stretch needed in conn_meas_inner_layout as it's the only widget in top_row_layout

        # Add the Serial Connection and Measurement Group Box to the top row layout
        top_row_layout.addWidget(self.conn_meas_group_box)

        # Removed the stretch that was making the group box take full width - the inner layout handles this now
        # top_row_layout.addStretch()

        # Call populate_com_ports after connect_button is created (already called here, moving later)
        # self.populate_com_ports()

        # --- Status Label (in its own row) ---
        status_layout = QHBoxLayout()
        control_layout.addLayout(status_layout)
        self.status_label = QLabel("状态: 未连接")
        self.status_label.setStyleSheet("font-size: 11pt;")
        status_layout.addWidget(self.status_label)
        status_layout.addStretch() # Add stretch to prevent status label from expanding horizontally

        # --- History Data Control Group (Create it here) ---
        self.history_group_box = QGroupBox("历史数据控制")
        # Set a different background color for the history group box
        self.history_group_box.setStyleSheet("QGroupBox { background-color: #d0f0d0; border: 1px solid gray; border-radius: 5px; margin-top: 1ex;} QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 3px 0 3px;}")

        history_control_layout = QVBoxLayout(self.history_group_box)

        self.show_history_checkbox = QCheckBox("显示历史波形")
        self.show_history_checkbox.setChecked(True)
        self.show_history_checkbox.stateChanged.connect(self.toggle_history)
        history_control_layout.addWidget(self.show_history_checkbox)
        self.show_history_checkbox.setStyleSheet("font-size: 11pt;")

        # Add toggle button for plot type
        self.plot_type_button = QPushButton("切换到TMP/TDS显示")
        self.plot_type_button.setCheckable(True)  # Make it a toggle button
        self.plot_type_button.clicked.connect(self.toggle_plot_type)
        history_control_layout.addWidget(self.plot_type_button)
        self.plot_type_button.setStyleSheet("font-size: 11pt;")

        self.clear_history_button = QPushButton("清除历史数据")
        self.clear_history_button.clicked.connect(self.clear_history)
        history_control_layout.addWidget(self.clear_history_button)
        self.clear_history_button.setStyleSheet("font-size: 11pt;")

        # --- Bottom Row Splitter (Direction Control, Parameter Display, and History Control) ---
        # Use a QSplitter for dynamic resizing of these three modules
        self.bottom_splitter = QSplitter(Qt.Horizontal) # Horizontal splitter for the bottom row
        control_layout.addWidget(self.bottom_splitter) # Add the splitter to the main control layout

        # --- Direction Control Area ---
        # This group box is already created and configured.
        self.direction_group_box = QGroupBox("等离子体驱动") # Create QGroupBox with title
        self.direction_group_box.setStyleSheet("QGroupBox { background-color: #e0f2f7; border: 1px solid gray; border-radius: 5px; margin-top: 1ex;} QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 3px 0 3px;}")

        direction_control_layout = QGridLayout(self.direction_group_box) # Use QGridLayout within the group box

        # Direction buttons
        self.forward_button = QPushButton("")
        self.forward_button.setStyleSheet("QPushButton { background-color: white; border: 1px solid gray; border-radius: 5px; } QPushButton:pressed { background-color: #a0a0a0; }")
        self.forward_button.setFixedSize(40, 40)
        self.forward_button.setIcon(QIcon("icons/arrow_up.png")) # Placeholder for up arrow icon
        self.forward_button.setIconSize(QSize(40, 40)) # Set icon size to fill button
        self.left_button = QPushButton("")
        self.left_button.setStyleSheet("QPushButton { background-color: white; border: 1px solid gray; border-radius: 5px; } QPushButton:pressed { background-color: #a0a0a0; }")
        self.left_button.setFixedSize(40, 40)
        self.left_button.setIcon(QIcon("icons/arrow_left.png")) # Placeholder for left arrow icon
        self.left_button.setIconSize(QSize(40, 40)) # Set icon size to fill button
        self.right_button = QPushButton("")
        self.right_button.setStyleSheet("QPushButton { background-color: white; border: 1px solid gray; border-radius: 5px; } QPushButton:pressed { background-color: #a0a0a0; }")
        self.right_button.setFixedSize(40, 40)
        self.right_button.setIcon(QIcon("icons/arrow_right.png")) # Placeholder for right arrow icon
        self.right_button.setIconSize(QSize(40, 40)) # Set icon size to fill button
        self.backward_button = QPushButton("") # This button will not send commands
        self.backward_button.setStyleSheet("QPushButton { background-color: white; border: 1px solid gray; border-radius: 5px; } QPushButton:pressed { background-color: #a0a0a0; }")
        self.backward_button.setFixedSize(40, 40)
        self.backward_button.setIcon(QIcon("icons/arrow_down.png")) # Placeholder for down arrow icon
        self.backward_button.setIconSize(QSize(40, 40)) # Set icon size to fill button

        # Connect signals (Backward button does not send command)
        self.forward_button.pressed.connect(lambda: self.send_direction_command(0x06))  # 前进（功率增加）
        self.left_button.pressed.connect(lambda: self.send_direction_command(0x08))     # 左转
        self.right_button.pressed.connect(lambda: self.send_direction_command(0x09))    # 右转

        # Add buttons directly to the grid layout with center alignment
        direction_control_layout.addWidget(self.forward_button, 0, 1, Qt.AlignCenter) # Top center, Center aligned
        direction_control_layout.addWidget(self.left_button, 1, 0, Qt.AlignCenter)    # Middle left, Center aligned
        direction_control_layout.addWidget(self.backward_button, 1, 1, Qt.AlignCenter) # Middle center, Center aligned
        direction_control_layout.addWidget(self.right_button, 1, 2, Qt.AlignCenter)   # Middle right, Center aligned

        # Set column stretch for uniform horizontal spacing
        direction_control_layout.setColumnStretch(0, 1)
        direction_control_layout.setColumnStretch(1, 1)
        direction_control_layout.setColumnStretch(2, 1)

        # Set spacing for the grid layout
        direction_control_layout.setHorizontalSpacing(10) # Adjust horizontal spacing as needed
        direction_control_layout.setVerticalSpacing(10) # Adjust vertical spacing as needed

        # Set row stretch for balanced vertical spacing
        direction_control_layout.setRowStretch(0, 1) # Give first row some stretch
        direction_control_layout.setRowStretch(1, 1) # Give second row some stretch

        # Add longitude and latitude input fields and auto-navigation button
        self.lon_label_input = QLabel("目标经度:")
        self.lon_label_input.setStyleSheet("font-size: 11pt;")
        self.lon_input = QLineEdit()
        self.lon_input.setStyleSheet("font-size: 11pt;")
        self.lat_label_input = QLabel("目标纬度:")
        self.lat_label_input.setStyleSheet("font-size: 11pt;")
        self.lat_input = QLineEdit()
        self.lat_input.setStyleSheet("font-size: 11pt;")
        self.auto_nav_button = QPushButton("自动巡航")
        self.auto_nav_button.setStyleSheet("font-size: 11pt;")
        self.auto_nav_button.clicked.connect(self.start_auto_navigation)

        # Add plasma thruster control buttons
        self.thruster_on_button = QPushButton("开启推进器")
        self.thruster_on_button.setStyleSheet("font-size: 11pt;")
        self.thruster_on_button.clicked.connect(lambda: self.send_thruster_command(0x04))
        
        self.increase_power_button = QPushButton("增加功率")
        self.increase_power_button.setStyleSheet("font-size: 11pt;")
        self.increase_power_button.clicked.connect(lambda: self.send_thruster_command(0x07))
        
        self.decrease_power_button = QPushButton("减速")
        self.decrease_power_button.setStyleSheet("font-size: 11pt;")
        self.decrease_power_button.clicked.connect(lambda: self.send_thruster_command(0x06))
        
        self.thruster_off_button = QPushButton("关闭推进器")
        self.thruster_off_button.setStyleSheet("font-size: 11pt;")
        self.thruster_off_button.clicked.connect(lambda: self.send_thruster_command(0x05))

        # Add new widgets to the grid layout
        direction_control_layout.addWidget(self.lon_label_input, 2, 0)
        direction_control_layout.addWidget(self.lon_input, 2, 1, 1, 2) # Span 2 columns
        direction_control_layout.addWidget(self.lat_label_input, 3, 0)
        direction_control_layout.addWidget(self.lat_input, 3, 1, 1, 2) # Span 2 columns
        direction_control_layout.addWidget(self.auto_nav_button, 4, 0, 1, 3) # Span 3 columns
        
        # Add thruster control buttons to the grid layout
        direction_control_layout.addWidget(self.thruster_on_button, 5, 0, 1, 2) # Span 2 columns
        direction_control_layout.addWidget(self.thruster_off_button, 5, 2) # Single column
        direction_control_layout.addWidget(self.increase_power_button, 6, 0, 1, 2) # Span 2 columns
        direction_control_layout.addWidget(self.decrease_power_button, 6, 2) # Single column

        # Optional: Set size policy for buttons to keep them a reasonable size
        size_policy = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.forward_button.setSizePolicy(size_policy)
        self.left_button.setSizePolicy(size_policy)
        self.right_button.setSizePolicy(size_policy)
        self.backward_button.setSizePolicy(size_policy)

        # Add the direction control group box to the bottom splitter
        self.bottom_splitter.addWidget(self.direction_group_box)

        # --- Parameter Display Area ---
        # Wrap parameter labels in a Group Box for clearer separation
        self.params_group_box = QGroupBox("参数实时显示")
        # Set a different background color for the parameter group box
        self.params_group_box.setStyleSheet("QGroupBox { background-color: #f0f0d0; border: 1px solid gray; border-radius: 5px; margin-top: 1ex;} QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 3px 0 3px;}")

        param_display_layout = QGridLayout(self.params_group_box) # Use QGridLayout within the group box

        # Labels for parameters
        self.tds_label = QLabel("TDS: -- ppm")
        param_display_layout.addWidget(self.tds_label, 0, 0) # Row 0, Col 0
        self.tds_label.setStyleSheet("font-size: 12pt;")

        self.tmp_label = QLabel("TMP: -- °C")
        param_display_layout.addWidget(self.tmp_label, 0, 1) # Row 0, Col 1
        self.tmp_label.setStyleSheet("font-size: 12pt;")

        self.longitude_label = QLabel("经度: --")
        param_display_layout.addWidget(self.longitude_label, 1, 0) # Row 1, Col 0
        self.longitude_label.setStyleSheet("font-size: 12pt;")

        self.latitude_label = QLabel("纬度: --")
        self.latitude_label.setStyleSheet("font-size: 12pt;")
        param_display_layout.addWidget(self.latitude_label, 1, 1) # Row 1, Col 1

        # Add QLabel for current measured value and cross-section area to the grid layout
        self.current_value_label = QLabel("河岸宽度: --") # Initialize without unit
        self.current_value_label.setStyleSheet("font-size: 12pt;") # Keep consistent font size with other params
        param_display_layout.addWidget(self.current_value_label, 2, 0) # Row 2, Col 0

        self.cross_section_area_label = QLabel("断面面积: -- m²") # Initialize with default text
        self.cross_section_area_label.setStyleSheet("font-size: 12pt;") # Keep consistent font size
        param_display_layout.addWidget(self.cross_section_area_label, 2, 1) # Row 2, Col 1

        # Add labels for IMU data
        self.imu_acc_label = QLabel("IMU 加速度: --")
        self.imu_acc_label.setStyleSheet("font-size: 12pt;")
        param_display_layout.addWidget(self.imu_acc_label, 3, 0, 1, 2) # Span 2 columns

        self.imu_gyro_label = QLabel("IMU 角速度: --")
        self.imu_gyro_label.setStyleSheet("font-size: 12pt;")
        param_display_layout.addWidget(self.imu_gyro_label, 4, 0, 1, 2) # Span 2 columns

        self.imu_angle_label = QLabel("IMU 欧拉角: --")
        self.imu_angle_label.setStyleSheet("font-size: 12pt;")
        param_display_layout.addWidget(self.imu_angle_label, 5, 0, 1, 2) # Span 2 columns

        # Set spacing for the grid layout
        param_display_layout.setHorizontalSpacing(60) # Keep existing spacing
        param_display_layout.setVerticalSpacing(15) # Keep existing spacing

        # Add stretch to the grid layout for better spacing
        param_display_layout.setColumnStretch(0, 1) # Add stretch to columns for even distribution
        param_display_layout.setColumnStretch(1, 1)
        param_display_layout.setRowStretch(3, 1) # Add stretch below the last row of labels

        # Add the Parameter Display Group Box to the bottom splitter
        self.bottom_splitter.addWidget(self.params_group_box)

        # --- History Data Control Group (Moved to bottom row) ---
        # self.history_group_box is already created at the top, we just add it to a different layout
        # Add the History Data Control Group to the bottom splitter
        self.bottom_splitter.addWidget(self.history_group_box)

        # Set initial sizes for the bottom splitter's widgets (optional, adjust as needed)
        # Example: Equal initial sizes
        self.bottom_splitter.setSizes([1, 1, 1]) # Adjust these proportions as needed

        # Removed the fixed spacing as splitter handles spacing with its handles
        # bottom_row_layout.setSpacing(30)

        # Call populate_com_ports after all widgets are created
        self.populate_com_ports()

        # --- Plot Area Widget ---
        plot_widget = QWidget()
        plot_layout = QVBoxLayout(plot_widget)

        # Set the background color of the plot_widget to white
        plot_widget.setStyleSheet("background-color: white;")

        # Matplotlib Figure and Canvas
        self.figure = Figure()
        # Create two subplots: 2 rows, 1 column
        self.ax_laser = self.figure.add_subplot(211)
        # Set background color for the laser plot axes
        self.ax_laser.set_facecolor('#f8f8f8') # Very light grey background
        # Create second plot (bottom)
        self.ax_ultrasonic = self.figure.add_subplot(212, sharex=self.ax_laser) # Share the x-axis for aligned time
        # Set background color for the ultrasonic plot axes
        self.ax_ultrasonic.set_facecolor('#f8f8f8') # Very light grey background

        self.canvas = FigureCanvasQTAgg(self.figure)
        plot_layout.addWidget(self.canvas)

        # Add Navigation Toolbar to the plot widget
        self.toolbar = NavigationToolbar(self.canvas, plot_widget) # Parent is plot_widget
        plot_layout.addWidget(self.toolbar)

        # --- QSplitter ---
        self.splitter = QSplitter(Qt.Vertical) # Vertical splitter
        self.splitter.addWidget(control_widget)
        self.splitter.addWidget(plot_widget)

        # Set initial sizes (e.g., 1 part for controls, 4 parts for plot)
        self.splitter.setSizes([int(self.height() * 0.2), int(self.height() * 0.8)])

        # Add the splitter to the main layout
        main_layout.addWidget(self.splitter)

        # Plotting data
        self.times = [] # Laser time data
        self.distances = [] # Laser distance data
        self.ultrasonic_distances = [] # Ultrasonic distance data
        # Add TMP and TDS data lists
        self.tmp_values = [] # TMP values
        self.tds_values = [] # TDS values

        # Flag to control plot type (0:距离, 1:TMP/TDS, 2:轨迹)
        self.plot_mode = 0
        # 经纬度历史数据
        self.lon_history = []
        self.lat_history = []

        # Parameters data (store latest values)
        self.tds = None
        self.tmp = None
        self.longitude = None
        self.latitude = None

        # Parameters data (store latest values for IMU)
        self.imu_acc = None
        self.imu_gyro = None
        self.imu_angle = None

        self.start_time = None # Initialize to None, set on connect

        # Initialize plot appearance for all subplots
        self.ax_laser.set_ylabel('距离 (m)', fontsize=12)
        self.ax_laser.set_title('河岸宽度实时显示', fontsize=14, ha='center', y=1.05)
        self.ax_laser.grid(True, linestyle='--', alpha=0.7)
        self.ax_laser.tick_params(labelbottom=False)

        self.ax_ultrasonic.set_xlabel('时间 (s)', fontsize=12)
        self.ax_ultrasonic.set_ylabel('距离 (m)', fontsize=12)
        self.ax_ultrasonic.set_title('河流断面实时显示', fontsize=14, ha='center', y=1.05)
        self.ax_ultrasonic.grid(True, linestyle='--', alpha=0.7)
        # 不在此处添加legend

        self.figure.tight_layout()

        self.serial_port = None
        self.reader_thread = None

        # Timer for sending 'begin' command
        self.command_timer = QTimer()
        self.command_timer.timeout.connect(self.send_begin_command)
        # Set interval to 1000 ms (1 second)
        self.command_timer.setInterval(1000) # 1 second interval

        # Flag to control history display
        self.show_history = True

        # 启动时直接用统一的方式显示初始页面，且保证为距离显示
        self.plot_mode = -1
        self.toggle_plot_type()

        # 新增：数据队列
        self.data_queue = Queue()
        # 新增：定时处理队列数据
        self.process_timer = QTimer()
        self.process_timer.timeout.connect(self.process_data_queue)
        self.process_timer.start(50)  # 每50ms处理一次
        # 初始化数据缓冲区，修复AttributeError
        self.data_buffer = ""

        # Add data frame tracking
        self.current_frame_data = {
            'laser': None,
            'ultrasonic': None,
            'tmp': None,
            'tds': None,
            'longitude': None,
            'latitude': None,
            'imu_acc': None,
            'imu_gyro': None,
            'imu_angle': None
        }
        self.frame_complete = False

    def toggle_history(self, state):
        self.show_history = bool(state)
        if not self.show_history:
            # Clear the plot but keep the data
            self.ax_laser.clear()
            self.ax_ultrasonic.clear()
            self.canvas.draw()
        else:
            # Restore the plot with existing data
            self.ax_laser.set_data(self.times, self.distances)
            self.ax_ultrasonic.set_data(self.times, self.ultrasonic_distances)
            self.canvas.draw()

    def clear_history(self):
        self.times = []
        self.distances = []
        self.ultrasonic_distances = []
        # Clear TMP and TDS data
        self.tmp_values = []
        self.tds_values = []
        # Clear data from all lines
        self.ax_laser.clear()
        self.ax_ultrasonic.clear()
        # Reset parameter display labels (保留这些，它们是参数显示标签，不是被删除的当前测量值和断面面积标签)
        self.tds_label.setText("TDS: -- ppm")
        self.tmp_label.setText("TMP: -- °C")
        self.longitude_label.setText("经度: --")
        self.latitude_label.setText("纬度: --")
        # Reset the view limits of all subplots (keep x-axis starting from 0)
        self.ax_laser.set_xlim(0, 10)
        self.ax_laser.set_ylim(0, 10) # Adjust initial Y limit to meters
        self.ax_ultrasonic.set_xlim(0, 10)
        self.ax_ultrasonic.set_ylim(0, 10) # Adjust initial Y limit to meters

        # Re-apply grid after clearing the axes
        self.ax_laser.grid(True, linestyle='--', alpha=0.7)
        self.ax_ultrasonic.grid(True, linestyle='--', alpha=0.7)

        self.canvas.draw()

    def populate_com_ports(self):
        ports = serial.tools.list_ports.comports()
        self.com_combobox.clear()
        if not ports:
            self.com_combobox.addItem("无可用串口")
            self.connect_button.setEnabled(False)
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(False)
        else:
            for port, desc, hwid in sorted(ports):
                self.com_combobox.addItem(port)
            self.connect_button.setEnabled(True)
            self.start_button.setEnabled(False) # Still disabled until connected
            self.stop_button.setEnabled(False)

    def connect_serial(self):
        selected_port = self.com_combobox.currentText()
        if selected_port == "无可用串口":
             self.status_label.setText("状态: 请选择有效串口")
             return

        # If already connected, disconnect first
        if self.serial_port and self.serial_port.isOpen():
            self.disconnect_serial() # Disconnect first if already connected
            return # Exit connect_serial after disconnecting

        try:
            # Close existing connection if any
            if self.serial_port and self.serial_port.isOpen():
                self.disconnect_serial() # Disconnect first if already connected

            # Attempt to open the selected port with longer timeout
            self.serial_port = serial.Serial(
                selected_port, 
                int(self.baud_combobox.currentText()), 
                timeout=1.0,  # Increased timeout to 1 second
                write_timeout=1.0  # Added write timeout
            )
            # Clear input buffer to discard any residual data
            self.serial_port.flushInput() # For pySerial version < 3.0 use self.serial_port.flushInput()
            # print("Serial input buffer cleared.") # Debug print

            self.status_label.setText(f"状态: 已连接到 {selected_port}")
            self.connect_button.setText("断开连接")

            self.com_combobox.setEnabled(False) # Disable COM port selection after connecting
            self.start_button.setEnabled(True) # Enable Start button after connecting
            self.stop_button.setEnabled(False) # Stop button starts disabled
            print(f"成功连接到 {selected_port}")

            # Reset data and start time for new plot
            self.clear_history()
            self.start_time = time.monotonic() # Use monotonic time
            self.canvas.draw()

            # Start data reading thread, passing the queue
            self.reader_thread = SerialReaderThread(self.serial_port, self.data_queue)
            self.reader_thread.error_occurred.connect(self.handle_error)
            self.reader_thread.start()

        except serial.SerialException as e:
            self.status_label.setText(f"状态: 连接失败 - {e}")
            print(f"连接失败: {e}")
            self.serial_port = None
            self.com_combobox.setEnabled(True)
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(False)

    def disconnect_serial(self):
        # Stop the command timer first (if it were active, though we removed it from start_measurement)
        # if self.command_timer.isActive():
        #     self.command_timer.stop()

        # Stop the reader thread first
        if self.reader_thread and self.reader_thread.isRunning():
            self.reader_thread.stop()
            self.reader_thread = None

        # Close the serial port
        if self.serial_port and self.serial_port.isOpen():
            try:
                self.serial_port.close()
                print("Serial port closed.") # Debug print for closing
            except Exception as e:
                print(f"Error closing serial port: {e}")
            self.serial_port = None # Set to None after closing
            self.start_time = None # Reset start_time on disconnect

        # Update GUI elements
        self.status_label.setText("状态: 未连接")
        self.connect_button.setText("打开串口") # Changed back to "打开串口"

        self.com_combobox.setEnabled(True)
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        # Reset parameter display on disconnect
        self.tds_label.setText("TDS: -- ppm")
        self.tmp_label.setText("TMP: -- °C")
        self.longitude_label.setText("经度: --")
        self.latitude_label.setText("纬度: --")
        # Reset IMU parameter display on disconnect
        self.imu_acc_label.setText("IMU 加速度: --")
        self.imu_gyro_label.setText("IMU 角速度: --")
        self.imu_angle_label.setText("IMU 欧拉角: --")
        self.cross_section_area_label.setText("断面面积: -- m²")
        print("已断开连接") # Keep this print

    def start_measurement(self):
        if not (self.serial_port and self.serial_port.isOpen()):
            self.status_label.setText("状态: 未连接，无法发送命令")
            return
            
        try:
            # Send begin command once
            self.send_begin_command()
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
            self.status_label.setText("状态: 已连接，测量已开始")
            print("Sent 'begin' command once.")
        except Exception as e:
            print(f"Error starting measurement: {e}")
            self.status_label.setText(f"状态: 启动测量失败 - {e}")
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)

    def stop_measurement(self):
        try:
            # Add check if serial port is open before sending
            if self.serial_port and self.serial_port.is_open:
                # Send "end" command without line endings
                self.serial_port.write("end".encode())
                print("Sent: end") # Add debug print for sent command
            else:
                print("Serial port not open, cannot send end command.") # Optional: Add message if not connected

            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self.status_label.setText("状态: 已连接，测量已停止")
            print("Measurement stopped.")
        except Exception as e:
            print(f"Error stopping measurement: {e}")
            self.status_label.setText(f"状态: 停止测量失败 - {e}")

    def send_begin_command(self):
        """发送开始测量命令"""
        if not (self.serial_port and self.serial_port.is_open):
            return

        try:
            # Changed to send just "begin" without line endings
            self.serial_port.write("begin".encode())
            print("Sent: begin") # Update debug print
        except Exception as e:
            print(f"Error sending begin command: {e}")
            pass

    def send_direction_command(self, command):
        """发送方向控制命令"""
        if not (self.serial_port and self.serial_port.is_open):
            self.status_label.setText("状态: 未连接，无法发送命令")
            return

        try:
            # Send the actual direction command as hex byte without line endings
            self.serial_port.write(bytes([command]))
            print(f"Sent direction command: 0x{command:02X}") # Debug print for sent command
            # Update status bar (optional)
            # self.status_label.setText(f"状态: 已发送方向命令: 0x{command:02X}")

        except Exception as e:
            print(f"Error sending direction command: {e}")
            self.status_label.setText(f"状态: 发送方向命令失败 - {e}")

    def send_thruster_command(self, command):
        """发送推进器控制命令"""
        if not (self.serial_port and self.serial_port.is_open):
            self.status_label.setText("状态: 未连接，无法发送命令")
            return

        try:
            # Send the thruster command as hex byte without line endings
            self.serial_port.write(bytes([command]))
            print(f"Sent thruster command: 0x{command:02X}") # Debug print for sent command
            # Update status bar (optional)
            # self.status_label.setText(f"状态: 已发送推进器命令: 0x{command:02X}")

        except Exception as e:
            print(f"Error sending thruster command: {e}")
            self.status_label.setText(f"状态: 发送推进器命令失败 - {e}")

    def toggle_plot_type(self):
        """切换显示距离图、TMP/TDS图、历史轨迹星位图，三者互不影响"""
        self.plot_mode = (self.plot_mode + 1) % 3
        self.refresh_current_plot()

    def refresh_current_plot(self):
        # 切换页面或刷新时，先清空整个figure
        self.figure.clf()
        if self.plot_mode == 0:
            # 距离显示
            self.ax_laser = self.figure.add_subplot(211)
            self.ax_ultrasonic = self.figure.add_subplot(212, sharex=self.ax_laser)
            self.ax_ultrasonic.set_visible(True)

            # Set Matplotlib titles for distance plots
            self.ax_laser.set_title('河岸宽度实时显示', fontsize=14, ha='center')
            self.ax_ultrasonic.set_title('河流断面实时显示', fontsize=14, ha='center')

            self.ax_laser.set_ylabel('距离 (m)', fontsize=12)
            self.ax_laser.grid(True, linestyle='--', alpha=0.7)
            self.ax_laser.tick_params(labelbottom=False)
            
            # 优化绘图性能：使用Line2D对象而不是plot
            self.laser_line, = self.ax_laser.plot([], [], color='#0057b7', linewidth=2, label='河岸宽度')
            self.ultrasonic_line, = self.ax_ultrasonic.plot([], [], color='#e37b40', linewidth=2, label='河流断面')
            
            # 添加填充
            self.laser_fill = self.ax_laser.fill_between([], [], [], alpha=0.6, color='#0057b7')
            self.ultrasonic_fill = self.ax_ultrasonic.fill_between([], [], [], alpha=0.6, color='#e37b40')

            self.ax_laser.legend(loc='upper right')
            self.ax_ultrasonic.legend(loc='upper right')

            # 设置坐标轴标签
            self.ax_ultrasonic.set_xlabel('时间 (s)', fontsize=12)
            self.ax_ultrasonic.set_ylabel('距离 (m)', fontsize=12)
            self.ax_ultrasonic.grid(True, linestyle='--', alpha=0.7)

            # 更新数据
            if self.show_history and self.times:
                # 更新线条数据
                self.laser_line.set_data(self.times, self.distances)
                self.ultrasonic_line.set_data(self.times, self.ultrasonic_distances)
                
                # 更新填充数据
                self.laser_fill.remove()
                self.ultrasonic_fill.remove()
                self.laser_fill = self.ax_laser.fill_between(self.times, 0, self.distances, alpha=0.6, color='#0057b7')
                self.ultrasonic_fill = self.ax_ultrasonic.fill_between(self.times, 0, self.ultrasonic_distances, alpha=0.6, color='#e37b40')

                # 优化坐标轴范围设置
                if len(self.times) > 1:
                    # 时间轴范围
                    time_range = max(self.times) - min(self.times)
                    x_min = max(0, min(self.times) - time_range * 0.05)
                    x_max = max(self.times) + time_range * 0.05
                    
                    # 激光距离范围
                    valid_laser = [d for d in self.distances if not np.isnan(d)]
                    if valid_laser:
                        laser_min = min(valid_laser)
                        laser_max = max(valid_laser)
                        laser_range = laser_max - laser_min
                        y_min_laser = max(0, laser_min - laser_range * 0.1)
                        y_max_laser = laser_max + laser_range * 0.1
                        self.ax_laser.set_ylim(y_min_laser, y_max_laser)
                    
                    # 超声波距离范围
                    valid_ultrasonic = [d for d in self.ultrasonic_distances if not np.isnan(d)]
                    if valid_ultrasonic:
                        ultrasonic_min = min(valid_ultrasonic)
                        ultrasonic_max = max(valid_ultrasonic)
                        ultrasonic_range = ultrasonic_max - ultrasonic_min
                        y_min_ultrasonic = max(0, ultrasonic_min - ultrasonic_range * 0.1)
                        y_max_ultrasonic = ultrasonic_max + ultrasonic_range * 0.1
                        self.ax_ultrasonic.set_ylim(y_min_ultrasonic, y_max_ultrasonic)
                    
                    # 设置x轴范围
                    self.ax_laser.set_xlim(x_min, x_max)
                    self.ax_ultrasonic.set_xlim(x_min, x_max)

            self.plot_type_button.setText("切换到TMP/TDS显示")

        elif self.plot_mode == 1:
            # TMP/TDS显示
            self.ax_laser = self.figure.add_subplot(211)
            self.ax_ultrasonic = self.figure.add_subplot(212, sharex=self.ax_laser)
            self.ax_ultrasonic.set_visible(True)

            # 设置标题和标签
            self.ax_laser.set_title('TMP实时显示', fontsize=14, ha='center')
            self.ax_laser.set_ylabel('温度 (°C)', fontsize=12)
            self.ax_ultrasonic.set_title('TDS实时显示', fontsize=14, ha='center')
            self.ax_ultrasonic.set_ylabel('TDS (ppm)', fontsize=12)
            self.ax_ultrasonic.set_xlabel('时间 (s)', fontsize=12)

            # 优化绘图性能：使用Line2D对象
            self.tmp_line, = self.ax_laser.plot([], [], color='#2ecc71', linewidth=2, label='TMP')
            self.tds_line, = self.ax_ultrasonic.plot([], [], color='#e74c3c', linewidth=2, label='TDS')
            
            # 添加填充
            self.tmp_fill = self.ax_laser.fill_between([], [], [], alpha=0.6, color='#2ecc71')
            self.tds_fill = self.ax_ultrasonic.fill_between([], [], [], alpha=0.6, color='#e74c3c')

            self.ax_laser.grid(True, linestyle='--', alpha=0.7)
            self.ax_ultrasonic.grid(True, linestyle='--', alpha=0.7)
            self.ax_laser.legend(loc='upper right')
            self.ax_ultrasonic.legend(loc='upper right')

            # 更新数据
            if self.show_history and self.times:
                # 更新线条数据
                self.tmp_line.set_data(self.times, self.tmp_values)
                self.tds_line.set_data(self.times, self.tds_values)
                
                # 更新填充数据
                self.tmp_fill.remove()
                self.tds_fill.remove()
                self.tmp_fill = self.ax_laser.fill_between(self.times, 0, self.tmp_values, alpha=0.6, color='#2ecc71')
                self.tds_fill = self.ax_ultrasonic.fill_between(self.times, 0, self.tds_values, alpha=0.6, color='#e74c3c')

                # 优化坐标轴范围设置
                if len(self.times) > 1:
                    # 时间轴范围
                    time_range = max(self.times) - min(self.times)
                    x_min = max(0, min(self.times) - time_range * 0.05)
                    x_max = max(self.times) + time_range * 0.05
                    
                    # TMP范围
                    valid_tmp = [t for t in self.tmp_values if not np.isnan(t)]
                    if valid_tmp:
                        tmp_min = min(valid_tmp)
                        tmp_max = max(valid_tmp)
                        tmp_range = tmp_max - tmp_min
                        y_min_tmp = tmp_min - tmp_range * 0.1
                        y_max_tmp = tmp_max + tmp_range * 0.1
                        self.ax_laser.set_ylim(y_min_tmp, y_max_tmp)
                    
                    # TDS范围
                    valid_tds = [t for t in self.tds_values if not np.isnan(t)]
                    if valid_tds:
                        tds_min = min(valid_tds)
                        tds_max = max(valid_tds)
                        tds_range = tds_max - tds_min
                        y_min_tds = tds_min - tds_range * 0.1
                        y_max_tds = tds_max + tds_range * 0.1
                        self.ax_ultrasonic.set_ylim(y_min_tds, y_max_tds)
                    
                    # 设置x轴范围
                    self.ax_laser.set_xlim(x_min, x_max)
                    self.ax_ultrasonic.set_xlim(x_min, x_max)

            self.plot_type_button.setText("切换到轨迹显示")

        else:
            # 轨迹显示
            self.ax_laser = self.figure.add_subplot(111)
            self.ax_laser.set_title('历史轨迹定位图', fontsize=14, ha='center')
            
            if self.lon_history and self.lat_history and len(self.lon_history) > 1:
                # 获取起点作为中心
                center_lon, center_lat = self.lon_history[0], self.lat_history[0]

                # 定义半径（米）
                radii_m = [0.1 * i for i in range(1, 10)] + [1.0 + 0.1 * i for i in range(9)]

                # 近似转换因子（米到度）
                meters_to_degrees = 1 / 100000.0

                # 绘制同心圆
                from matplotlib.patches import Circle
                for r_m in radii_m:
                    radius_deg = r_m * meters_to_degrees
                    circle = Circle((center_lon, center_lat), radius_deg, color='gray', linestyle='--', fill=False)
                    self.ax_laser.add_patch(circle)

                # 绘制轨迹
                self.ax_laser.plot(self.lon_history, self.lat_history, color='#8e44ad', linestyle='-', marker='o', markersize=4, label='历史轨迹')
                self.ax_laser.scatter(self.lon_history[0], self.lat_history[0], color='red', s=80, label='起点', zorder=5)

                # 设置标签
                self.ax_laser.set_xlabel('经度', fontsize=12)
                self.ax_laser.set_ylabel('纬度', fontsize=12)
                self.ax_laser.grid(True, linestyle='--', alpha=0.7)
                self.ax_laser.legend(loc='upper right')

                # 设置等比例
                self.ax_laser.set_aspect('equal', adjustable='box')

                # 优化坐标轴范围
                min_lon, max_lon = min(self.lon_history), max(self.lon_history)
                min_lat, max_lat = min(self.lat_history), max(self.lat_history)

                # 添加缓冲区
                lon_range = max_lon - min_lon
                lat_range = max_lat - min_lat
                lon_buffer = lon_range * 0.1 if lon_range > 0 else 0.00001
                lat_buffer = lat_range * 0.1 if lat_range > 0 else 0.00001

                self.ax_laser.set_xlim(min_lon - lon_buffer, max_lon + lon_buffer)
                self.ax_laser.set_ylim(min_lat - lat_buffer, max_lat + lat_buffer)
            else:
                self.ax_laser.text(0.5, 0.5, '暂无轨迹数据', ha='center', va='center', fontsize=16, transform=self.ax_laser.transAxes)
            
            self.plot_type_button.setText("切换到距离显示")

        # 优化布局
        self.figure.tight_layout()
        # 使用draw_idle()代替draw()以提高性能
        self.canvas.draw_idle()

    def handle_data(self, data):
        # 兼容旧信号，不再使用
        pass

    def process_data_queue(self):
        while not self.data_queue.empty():
            data = self.data_queue.get()
            self.data_buffer += data + "\n"
            if "end" in data:
                self.handle_frame(self.data_buffer)
                self.data_buffer = ""

    def handle_frame(self, frame_data):
        # 复制原 handle_data 的帧处理部分
        print(f"End of frame detected. Processing buffered data:\n{frame_data}")
        current_time = time.monotonic() - self.start_time
        for line in frame_data.strip().split("\n"):
            if not line or "end" in line:
                continue
            print(f"Processing line from buffer: {line}")
            # 使用更精确的正则表达式匹配
            laser_match = re.search(r"laser=(\d+)", line)
            ultrasonic_match = re.search(r"distance=(\d+)", line)
            tmp_match = re.search(r"TMP=([\d.]+)", line)
            tds_match = re.search(r"TDS=([\d.]+)", line)
            # 修改GPS数据解析的正则表达式
            latitude_match = re.search(r"GPS_LAT=([\d.]+)([NS])", line)
            longitude_match = re.search(r"GPS_LON=([\d.]+)([EW])", line)
            imu_acc_match = re.search(r"IMU_ACC:([\d.-]+),([\d.-]+),([\d.-]+)", line)
            imu_gyro_match = re.search(r"IMU_GYRO:([\d.-]+),([\d.-]+),([\d.-]+)", line)
            imu_angle_match = re.search(r"IMU_ANGLE:([\d.-]+),([\d.-]+),([\d.-]+)", line)
            if laser_match:
                try:
                    distance_mm = int(laser_match.group(1))
                    distance_m = distance_mm / 1000.0
                    self.current_frame_data['laser'] = distance_m
                    self.current_value_label.setText(f"河岸宽度: {distance_m:.3f} m")
                except ValueError as e:
                    print(f"Error parsing laser distance value: {e} from line: {line}")
            elif ultrasonic_match:
                try:
                    distance_mm = int(ultrasonic_match.group(1))
                    distance_m = distance_mm / 1000.0
                    self.current_frame_data['ultrasonic'] = distance_m
                except ValueError as e:
                    print(f"Error parsing ultrasonic distance value: {e} from line: {line}")
            elif tmp_match:
                try:
                    tmp_value = float(tmp_match.group(1))
                    self.current_frame_data['tmp'] = tmp_value
                    self.tmp_label.setText(f"TMP: {tmp_value:.1f} °C")
                except ValueError as e:
                    print(f"Error parsing TMP value: {e} from line: {line}")
            elif tds_match:
                try:
                    tds_value = float(tds_match.group(1))
                    self.current_frame_data['tds'] = tds_value
                    self.tds_label.setText(f"TDS: {tds_value:.1f} ppm")
                except ValueError as e:
                    print(f"Error parsing TDS value: {e} from line: {line}")
            # 修改经纬度解析逻辑
            if longitude_match:
                try:
                    lon_value = float(longitude_match.group(1))
                    lon_direction = longitude_match.group(2)
                    if lon_direction == 'W':
                        lon_value = -lon_value # 西经为负
                    self.current_frame_data['longitude'] = lon_value
                    self.longitude_label.setText(f"经度: {lon_value:.6f}°")
                except Exception as e:
                    print(f"Error parsing longitude value: {e} from line: {line}")
            if latitude_match:
                try:
                    lat_value = float(latitude_match.group(1))
                    lat_direction = latitude_match.group(2)
                    if lat_direction == 'S':
                        lat_value = -lat_value # 南纬为负
                    self.current_frame_data['latitude'] = lat_value
                    self.latitude_label.setText(f"纬度: {lat_value:.6f}°")
                except Exception as e:
                    print(f"Error parsing latitude value: {e} from line: {line}")
            # 优化IMU数据显示格式
            if imu_acc_match:
                try:
                    acc_x = float(imu_acc_match.group(1))
                    acc_y = float(imu_acc_match.group(2))
                    acc_z = float(imu_acc_match.group(3))
                    self.current_frame_data['imu_acc'] = (acc_x, acc_y, acc_z)
                    self.imu_acc_label.setText(f"IMU 加速度 (g): X={acc_x:.3f}, Y={acc_y:.3f}, Z={acc_z:.3f}")
                except ValueError as e:
                    print(f"Error parsing IMU ACC value: {e} from line: {line}")
            if imu_gyro_match:
                try:
                    gyro_x = float(imu_gyro_match.group(1))
                    gyro_y = float(imu_gyro_match.group(2))
                    gyro_z = float(imu_gyro_match.group(3))
                    self.current_frame_data['imu_gyro'] = (gyro_x, gyro_y, gyro_z)
                    self.imu_gyro_label.setText(f"IMU 角速度 (dps): X={gyro_x:.3f}, Y={gyro_y:.3f}, Z={gyro_z:.3f}")
                except ValueError as e:
                    print(f"Error parsing IMU GYRO value: {e} from line: {line}")
            if imu_angle_match:
                try:
                    angle_x = float(imu_angle_match.group(1))
                    angle_y = float(imu_angle_match.group(2))
                    angle_z = float(imu_angle_match.group(3))
                    self.current_frame_data['imu_angle'] = (angle_x, angle_y, angle_z)
                    self.imu_angle_label.setText(f"IMU 欧拉角 (deg): 横滚={angle_x:.3f}, 俯仰={angle_y:.3f}, 航向={angle_z:.3f}")
                except ValueError as e:
                    print(f"Error parsing IMU ANGLE value: {e} from line: {line}")
        # 计算断面面积
        if len(self.times) > 1 and len(self.times) == len(self.ultrasonic_distances):
            distances_m = [d for d in self.ultrasonic_distances]
            time_intervals = np.diff(self.times)
            valid_distances = [d for d in distances_m if not np.isnan(d)]
            total_area = 0.0
            valid_segment_count = 0
            for i in range(len(time_intervals)):
                if not np.isnan(distances_m[i]) and not np.isnan(distances_m[i+1]) and not np.isnan(time_intervals[i]):
                    total_area += (distances_m[i] + distances_m[i+1]) / 2 * abs(time_intervals[i])
                    valid_segment_count += 1
            if valid_segment_count > 0:
                self.cross_section_area_label.setText(f"断面面积: {total_area:.2f} m²")
            else:
                self.cross_section_area_label.setText("断面面积: -- m²")
        else:
            self.cross_section_area_label.setText("断面面积: -- m²")
        # 只有TDS字段不为None时才追加数据并刷新图表，否则丢弃该帧
        if self.current_frame_data['tds'] is not None:
            self.times.append(current_time)
            self.distances.append(self.current_frame_data['laser'] if self.current_frame_data['laser'] is not None else np.nan)
            self.ultrasonic_distances.append(self.current_frame_data['ultrasonic'] if self.current_frame_data['ultrasonic'] is not None else np.nan)
            self.tmp_values.append(self.current_frame_data['tmp'] if self.current_frame_data['tmp'] is not None else np.nan)
            self.tds_values.append(self.current_frame_data['tds'])
            if self.current_frame_data['longitude'] is not None:
                self.lon_history.append(self.current_frame_data['longitude'])
            if self.current_frame_data['latitude'] is not None:
                self.lat_history.append(self.current_frame_data['latitude'])
            self.refresh_current_plot()
        else:
            print("本帧缺少TDS，已丢弃，不参与绘图。")
        # 重置帧数据
        self.current_frame_data = {k: None for k in self.current_frame_data}

    def handle_error(self, error_msg):
        print(f"Handling error in GUI thread: {error_msg}")
        self.status_label.setText(f"状态: {error_msg}")
        if self.serial_port and self.serial_port.isOpen():
             self.disconnect_serial()

    def closeEvent(self, event):
        self.disconnect_serial()
        event.accept()

    def start_auto_navigation(self):
        """发送自动巡航命令"""
        if not (self.serial_port and self.serial_port.is_open):
            self.status_label.setText("状态: 未连接，无法发送命令")
            return

        try:
            lon_str = self.lon_input.text().strip()
            lat_str = self.lat_input.text().strip()

            if not lon_str or not lat_str:
                self.status_label.setText("状态: 请输入经度和纬度")
                return

            try:
                # 解析经度
                lon_match = re.search(r"^([\d.]+)([EW])$", lon_str.upper()) # 匹配数字和方向，转大写方便判断
                if lon_match:
                    longitude = float(lon_match.group(1))
                    lon_direction = lon_match.group(2)
                    if lon_direction == 'W':
                        longitude = -longitude # 西经为负
                else:
                    # 如果没有方向，尝试直接解析为浮点数（假定已是带符号的十进制）
                    longitude = float(lon_str)

                # 解析纬度
                lat_match = re.search(r"^([\d.]+)([NS])$", lat_str.upper()) # 匹配数字和方向，转大写方便判断
                if lat_match:
                    latitude = float(lat_match.group(1))
                    lat_direction = lat_match.group(2)
                    if lat_direction == 'S':
                        latitude = -latitude # 南纬为负
                else:
                    # 如果没有方向，尝试直接解析为浮点数（假定已是带符号的十进制）
                    latitude = float(lat_str)

                command = f"NAV:{longitude},{latitude}"
                self.serial_port.write(command.encode())
                self.status_label.setText(f"状态: 已发送自动巡航命令: {command}")
                print(f"Sent auto-navigation command: {command}")
            except ValueError:
                self.status_label.setText("状态: 无效的经纬度格式")
                print("Invalid longitude or latitude format.")

        except Exception as e:
            print(f"Error sending auto-navigation command: {e}")
            self.status_label.setText(f"状态: 发送自动巡航命令失败 - {e}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SerialMonitorGUI()
    window.show()
    sys.exit(app.exec()) 