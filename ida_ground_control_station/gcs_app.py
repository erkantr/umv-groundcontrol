import sys
import os
import time
import threading
import math
from collections import deque
import pyqtgraph as pg
from PyQt5.QtCore import QUrl, Qt, QTimer, pyqtSignal, QObject, pyqtSlot, QDateTime
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QPushButton, QLabel,
                             QTextEdit, QHBoxLayout, QMessageBox, QGridLayout,
                             QProgressBar, QGroupBox, QComboBox, QDoubleSpinBox, QDialog, QFormLayout, QFrame, QScrollArea, QLineEdit, QTabWidget)
from PyQt5.QtGui import QPixmap, QIcon, QTransform, QTextCursor, QFont
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage
from PyQt5.QtWebChannel import QWebChannel
# pyqtgraph import
import pyqtgraph as pg

# Pixhawk bağlantısı için import edilecek (bu satırları aktif edin)
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import serial.tools.list_ports

# Attitude Indicator import
from attitude_indicator import AttitudeIndicator

class MapBridge(QObject):
    """Harita ve Python arasında köprü görevi gören sınıf"""
    updateVehiclePosition = pyqtSignal(float, float, float)
    addWaypoint = pyqtSignal(float, float)
    waypoint_from_user = pyqtSignal(float, float)
    waypoint_removed = pyqtSignal(int)  # Waypoint silme sinyali
    clearMap = pyqtSignal()

    @pyqtSlot(float, float)
    def add_waypoint_to_ui(self, lat, lng):
        """JavaScript tarafından haritaya çift tıklandığında çağrılır."""
        self.waypoint_from_user.emit(lat, lng)
    
    @pyqtSlot(int)
    def remove_waypoint_from_ui(self, index):
        """JavaScript tarafından waypoint silindiğinde çağrılır."""
        self.waypoint_removed.emit(index)

class GCSApp(QWidget):
    log_message_received = pyqtSignal(str)
    connection_status_changed = pyqtSignal(bool, str)
    status_message_received = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.vehicle = None
        self.is_connected = False
        self.waypoints = []
        self.current_heading = 0
        
        # SERVO_OUTPUT_RAW cache - DroneKit channels güncellenmiyor
        self.servo_output_cache = {}
        
        # Grafik veri bufferları (son 100 veri noktası)
        self.max_data_points = 100
        self.time_data = deque(maxlen=self.max_data_points)
        self.speed_data = deque(maxlen=self.max_data_points)
        self.speed_setpoint_data = deque(maxlen=self.max_data_points)
        self.heading_data = deque(maxlen=self.max_data_points)
        self.heading_setpoint_data = deque(maxlen=self.max_data_points)
        self.thruster_left_data = deque(maxlen=self.max_data_points)
        self.thruster_right_data = deque(maxlen=self.max_data_points)
        self.start_time = time.time()
        
        self.initUI()
        self.log_message_received.connect(self.update_log_safe)
        self.connection_status_changed.connect(self.on_connection_status_changed)
        self.status_message_received.connect(self.update_status_safe)

    def initUI(self):
        self.setWindowTitle('IDA Yer Kontrol İstasyonu')
        self.setGeometry(100, 100, 1400, 900)

        # Ana layout
        main_layout = QHBoxLayout()
        self.setLayout(main_layout)

        # Sol taraf (Sidebar) - Kaydırmalı (sadece dikey)
        sidebar_scroll = QScrollArea()
        sidebar_widget = QWidget()
        sidebar_layout = QVBoxLayout(sidebar_widget)
        
        sidebar_scroll.setWidget(sidebar_widget)
        sidebar_scroll.setWidgetResizable(True)
        sidebar_scroll.setMaximumWidth(400)
        sidebar_scroll.setMinimumWidth(400)  # Sabit genişlik
        sidebar_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        sidebar_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        # Widget'in genişliğini sabitle
        sidebar_widget.setMaximumWidth(380)  # Scroll bar için biraz yer bırak
        sidebar_widget.setMinimumWidth(380)

        # Sağ taraf (Harita)
        map_layout = QVBoxLayout()
        
        # Web View (Harita)
        self.web_view = QWebEngineView()
        self.setup_web_channel()
        map_layout.addWidget(self.web_view)

        # Bağlantı Paneli
        connection_frame = QFrame()
        connection_frame.setFrameShape(QFrame.StyledPanel)
        connection_layout = QVBoxLayout(connection_frame)
        sidebar_layout.addWidget(connection_frame)

        connection_title = QLabel("Bağlantı")
        connection_title.setFont(QFont('Arial', 14, QFont.Bold))
        connection_layout.addWidget(connection_title)
        
        # Bağlantı tipi seçimi
        self.connection_type = QComboBox()
        self.connection_type.addItems(["Serial (USB)", "UDP (Kablosuz)", "TCP (WiFi)"])
        self.connection_type.currentTextChanged.connect(self.on_connection_type_changed)
        
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["57600", "115200", "38400", "19200", "9600"])
        self.baud_combo.setCurrentText("57600")  # Telemetri modülleri için varsayılan
        
        # UDP/TCP için IP ve Port alanları
        self.ip_input = QDoubleSpinBox()
        self.ip_input.setDecimals(0)
        self.ip_input.setRange(0, 999)
        self.ip_input.setValue(127)
        self.ip_input.setVisible(False)
        
        self.udp_port_input = QDoubleSpinBox()
        self.udp_port_input.setDecimals(0)
        self.udp_port_input.setRange(1000, 65535)
        self.udp_port_input.setValue(14550)
        self.udp_port_input.setVisible(False)
        
        self.refresh_button = QPushButton("Portları Yenile")
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button = QPushButton("  BAĞLAN")
        self.connect_button.clicked.connect(self.toggle_connection)
        
        connection_grid = QGridLayout()
        connection_grid.addWidget(QLabel("Tip:"), 0, 0)
        connection_grid.addWidget(self.connection_type, 0, 1)
        connection_grid.addWidget(QLabel("Port:"), 1, 0)
        connection_grid.addWidget(self.port_combo, 1, 1)
        connection_grid.addWidget(QLabel("Baud:"), 2, 0)
        connection_grid.addWidget(self.baud_combo, 2, 1)
        connection_grid.addWidget(QLabel("IP:"), 3, 0)
        connection_grid.addWidget(self.ip_input, 3, 1)
        connection_grid.addWidget(QLabel("UDP Port:"), 4, 0)
        connection_grid.addWidget(self.udp_port_input, 4, 1)
        connection_grid.addWidget(self.refresh_button, 5, 0)
        connection_grid.addWidget(self.connect_button, 5, 1)
        connection_layout.addLayout(connection_grid)

        # Telemetri Paneli
        telemetry_frame = QFrame()
        telemetry_frame.setFrameShape(QFrame.StyledPanel)
        telemetry_layout = QGridLayout(telemetry_frame)
        sidebar_layout.addWidget(telemetry_frame)
        
        telemetry_title = QLabel("Telemetri")
        telemetry_title.setFont(QFont('Arial', 14, QFont.Bold))
        telemetry_layout.addWidget(telemetry_title, 0, 0, 1, 4)
        self.telemetry_values = {
            "Hız:": QLabel("N/A"), "Hız Setpoint:": QLabel("N/A"),
            "Yükseklik:": QLabel("N/A"), "Heading:": QLabel("N/A"), 
            "Heading Setpoint:": QLabel("N/A"), "Pitch:": QLabel("N/A"),
            "Yaw:": QLabel("N/A"), "GPS Fix:": QLabel("N/A"),
            "Mod:": QLabel("N/A"), "Batarya:": QLabel("N/A")
        }
        row = 1
        for label, value_widget in self.telemetry_values.items():
            telemetry_layout.addWidget(QLabel(label), row, 0)
            telemetry_layout.addWidget(value_widget, row, 1)
            row += 1
        self.battery_progress = QProgressBar()
        telemetry_layout.addWidget(QLabel("Batarya Seviyesi:"), row, 0)
        telemetry_layout.addWidget(self.battery_progress, row, 1)
        row += 1
        thruster_layout = QVBoxLayout()
        self.thruster_labels = []
        for i in range(2):
            thruster_label = QLabel(f"T{i+1}: 0%")
            thruster_label.setStyleSheet("border: 1px solid gray; padding: 5px; font-size: 10px; font-weight: bold;")
            thruster_label.setMinimumHeight(25)
            thruster_label.setWordWrap(False)
            thruster_layout.addWidget(thruster_label)
            self.thruster_labels.append(thruster_label)
        telemetry_layout.addWidget(QLabel("Thruster'lar:"), row, 0)
        telemetry_layout.addLayout(thruster_layout, row, 1, 1, 2)
        self.attitude_indicator = AttitudeIndicator()
        self.attitude_indicator.setFixedSize(120, 120)
        telemetry_layout.addWidget(self.attitude_indicator, 1, 2, 4, 2)

        # --- GRAFİKLER PANELİ ---
        self.graph_frame = QFrame()
        self.graph_frame.setFrameShape(QFrame.StyledPanel)
        self.graph_frame.setStyleSheet("QFrame { border: 2px solid #aaa; border-radius: 8px; background: #fff; padding: 2px; }")
        graph_layout = QVBoxLayout(self.graph_frame)
        graph_layout.setContentsMargins(6, 6, 6, 6)
        graph_layout.setSpacing(2)
        graph_title = QLabel("Grafikler")
        graph_title.setFont(QFont('Arial', 11, QFont.Bold))
        graph_title.setStyleSheet("color: #333; padding-bottom: 2px;")
        graph_layout.addWidget(graph_title)
        # pyqtgraph plot
        self.pwm_plot = pg.PlotWidget()
        self.pwm_plot.setFixedHeight(180)
        self.pwm_plot.setBackground('w')
        self.pwm_plot.setMenuEnabled(False)
        self.pwm_plot.setMouseEnabled(x=False, y=False)
        self.pwm_plot.hideButtons()
        self.pwm_plot.setLabel('left', 'PWM', color='#333', size='9pt')
        self.pwm_plot.setLabel('bottom', 'Zaman (s)', color='#333', size='9pt')
        self.pwm_plot.setTitle("")
        self.pwm_plot.showGrid(x=True, y=True, alpha=0.12)
        self.pwm_plot.getAxis('left').setPen(pg.mkPen('#bbb', width=1))
        self.pwm_plot.getAxis('bottom').setPen(pg.mkPen('#bbb', width=1))
        self.pwm_plot.getAxis('left').setTextPen(pg.mkPen('#333'))
        self.pwm_plot.getAxis('bottom').setTextPen(pg.mkPen('#333'))
        self.pwm_plot.setYRange(900, 2100, padding=0)
        # Legend minimalist
        legend = self.pwm_plot.addLegend(offset=(10,10), labelTextColor='#444', brush=pg.mkBrush(255,255,255,200))
        legend.setColumnCount(2)
        legend.setMaximumHeight(22)
        # Plot curve'leri (daha yumuşak renkler)
        self.left_curve = self.pwm_plot.plot(pen=pg.mkPen('#1976d2', width=2), name="Sol (CH2)")
        self.right_curve = self.pwm_plot.plot(pen=pg.mkPen('#d32f2f', width=2), name="Sağ (CH1)")
        graph_layout.addWidget(self.pwm_plot)
        # PWM veri bufferları
        self.pwm_time_buffer = []
        self.pwm_left_buffer = []
        self.pwm_right_buffer = []
        self.pwm_max_points = 120  # 2 dakikalık pencere (1s'de 1 örnek)
        # --- GRAFİK PANELİNİ LOG FRAME'DEN ÖNCE EKLE ---
        sidebar_layout.addWidget(self.graph_frame)
        
        # Log Paneli
        log_frame = QFrame()
        log_frame.setFrameShape(QFrame.StyledPanel)
        log_layout = QVBoxLayout(log_frame)
        sidebar_layout.addWidget(log_frame)
        log_title = QLabel("Sistem Logları")
        log_title.setFont(QFont('Arial', 14, QFont.Bold))
        log_layout.addWidget(log_title)
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        log_layout.addWidget(self.log_display)
        
        # Durum Paneli
        status_frame = QFrame()
        status_frame.setFrameShape(QFrame.StyledPanel)
        status_layout = QVBoxLayout(status_frame)
        sidebar_layout.addWidget(status_frame)
        status_title = QLabel("Durum")
        status_title.setFont(QFont('Arial', 14, QFont.Bold))
        status_layout.addWidget(status_title)
        self.status_label = QLabel("Sistem hazır. Bağlantı bekleniyor...")
        self.status_label.setWordWrap(True)
        status_layout.addWidget(self.status_label)

        # Grafikler Paneli
        graphs_frame = QFrame()
        graphs_frame.setFrameShape(QFrame.StyledPanel)
        graphs_layout = QVBoxLayout(graphs_frame)
        sidebar_layout.addWidget(graphs_frame)
        graphs_title = QLabel("Temel Grafikler")
        graphs_title.setFont(QFont('Arial', 14, QFont.Bold))
        graphs_layout.addWidget(graphs_title)
        
        # Hız grafiği
        self.speed_plot = pg.PlotWidget(title="Hız (m/s)")
        self.speed_plot.setLabel('left', 'Hız', units='m/s')
        self.speed_plot.setLabel('bottom', 'Zaman', units='s')
        self.speed_plot.setMaximumHeight(120)
        self.speed_curve = self.speed_plot.plot(pen='b', name='Gerçek')
        self.speed_setpoint_curve = self.speed_plot.plot(pen='r', name='Setpoint')
        self.speed_plot.addLegend()
        graphs_layout.addWidget(self.speed_plot)
        
        # Heading grafiği
        self.heading_plot = pg.PlotWidget(title="Heading/Yaw (°)")
        self.heading_plot.setLabel('left', 'Açı', units='°')
        self.heading_plot.setLabel('bottom', 'Zaman', units='s')
        self.heading_plot.setMaximumHeight(120)
        self.heading_curve = self.heading_plot.plot(pen='g', name='Gerçek')
        self.heading_setpoint_curve = self.heading_plot.plot(pen='orange', name='Setpoint')
        self.heading_plot.addLegend()
        graphs_layout.addWidget(self.heading_plot)
        
        # Thruster kuvvet grafiği
        self.thruster_plot = pg.PlotWidget(title="Thruster Kuvveti (%)")
        self.thruster_plot.setLabel('left', 'Güç', units='%')
        self.thruster_plot.setLabel('bottom', 'Zaman', units='s')
        self.thruster_plot.setMaximumHeight(120)
        self.thruster_left_curve = self.thruster_plot.plot(pen='cyan', name='Sol')
        self.thruster_right_curve = self.thruster_plot.plot(pen='magenta', name='Sağ')
        self.thruster_plot.addLegend()
        graphs_layout.addWidget(self.thruster_plot)

        # Görev Kontrol Paneli
        mission_control_frame = QFrame()
        mission_control_frame.setFrameShape(QFrame.StyledPanel)
        mission_control_layout = QVBoxLayout(mission_control_frame)
        sidebar_layout.addWidget(mission_control_frame)
        mission_title = QLabel("Görev Kontrolü")
        mission_title.setFont(QFont('Arial', 14, QFont.Bold))
        mission_control_layout.addWidget(mission_title)
        
        mode_buttons_layout = QHBoxLayout()
        self.stabilize_button = QPushButton("STABILIZE")
        self.auto_button = QPushButton("AUTO")
        self.guided_button = QPushButton("GUIDED")
        mode_buttons_layout.addWidget(self.stabilize_button)
        mode_buttons_layout.addWidget(self.auto_button)
        mode_buttons_layout.addWidget(self.guided_button)
        mission_control_layout.addLayout(mode_buttons_layout)
        
        self.stabilize_button.clicked.connect(lambda: self.set_vehicle_mode("STABILIZE"))
        self.auto_button.clicked.connect(lambda: self.set_vehicle_mode("AUTO"))
        self.guided_button.clicked.connect(lambda: self.set_vehicle_mode("GUIDED"))

        mission_buttons_layout = QHBoxLayout()
        self.upload_mission_button = QPushButton("Rotayı Gönder")
        self.read_mission_button = QPushButton("Rotayı Oku")
        self.clear_mission_button = QPushButton("Rotayı Temizle")
        mission_buttons_layout.addWidget(self.upload_mission_button)
        mission_buttons_layout.addWidget(self.read_mission_button)
        mission_buttons_layout.addWidget(self.clear_mission_button)
        mission_control_layout.addLayout(mission_buttons_layout)
        
        self.upload_mission_button.clicked.connect(self.send_mission_to_vehicle)
        self.read_mission_button.clicked.connect(self.read_mission_from_vehicle)
        self.clear_mission_button.clicked.connect(self.clear_mission)

        self.mode_buttons = [self.stabilize_button, self.auto_button, self.guided_button, 
                           self.upload_mission_button, self.read_mission_button, self.clear_mission_button]
        for btn in self.mode_buttons:
            btn.setEnabled(False)

        # ... (main_layout'a widget'ların eklenmesi)
        main_layout.addWidget(sidebar_scroll)
        main_layout.addWidget(self.web_view, 1) # Haritayı daha geniş yap

        # Timer'lar
        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.timeout.connect(self.update_telemetry)
        self.attitude_timer = QTimer(self)
        self.attitude_timer.timeout.connect(self.update_attitude)
        self.motor_timer = QTimer(self)
        self.motor_timer.timeout.connect(self.update_motor_simulation)
        self.motor_timer.start(1000)  # Motor simülasyonu 1s'de bir - daha az CPU kullanımı
        
        # Grafikler için timer
        self.graphs_timer = QTimer(self)
        self.graphs_timer.timeout.connect(self.update_graphs)
        self.graphs_timer.start(250)  # 4Hz grafik güncelleme - smooth görünüm

        self.refresh_ports()
    
    def setup_web_channel(self):
        self.bridge = MapBridge(self)
        self.channel = QWebChannel(self.web_view.page())
        self.web_view.page().setWebChannel(self.channel)
        self.channel.registerObject('py_bridge', self.bridge)
        
        # JavaScript'ten gelen waypoint ekleme/silme isteklerini yakala
        self.bridge.waypoint_from_user.connect(self.add_waypoint_to_list)
        self.bridge.waypoint_removed.connect(self.remove_waypoint_from_list)

        map_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'map.html'))
        self.web_view.setUrl(QUrl.fromLocalFile(map_path))

    def toggle_connection(self):
        if self.is_connected:
            self.disconnect_from_vehicle()
        else:
            self.connect_to_vehicle()

    def connect_to_vehicle(self):
        connection_type = self.connection_type.currentText()
        
        if connection_type == "UDP (Kablosuz)":
            ip = int(self.ip_input.value())
            udp_port = int(self.udp_port_input.value())
            connection_string = f"udp:127.0.0.1:{udp_port}"
            self.status_message_received.emit(f"UDP bağlantısı kuruluyor... ({connection_string})")
        elif connection_type == "TCP (WiFi)":
            ip = int(self.ip_input.value())
            connection_string = f"tcp:192.168.1.{ip}:5760"
            self.status_message_received.emit(f"TCP bağlantısı kuruluyor... ({connection_string})")
        else:  # Serial
            port = self.port_combo.currentText()
            baud = int(self.baud_combo.currentText())
            if not port or "bulunamadı" in port:
                self.status_message_received.emit("Geçerli bir port seçilmedi.")
                return
            connection_string = port
            self.status_message_received.emit(f"Serial bağlantı kuruluyor... ({port}, {baud})")
        
        threading.Thread(target=self._connect_thread, args=(connection_string, connection_type), daemon=True).start()

    def _connect_thread(self, connection_string, connection_type):
        try:
            if connection_type == "Serial (USB)":
                baud = int(self.baud_combo.currentText())
                # Telemetri modülleri için optimize edilmiş ayarlar
                self.vehicle = connect(connection_string, baud=baud, wait_ready=True, 
                                     heartbeat_timeout=15, timeout=60)
            else:  # UDP veya TCP
                # Kablosuz bağlantılar için daha kısa timeout
                self.vehicle = connect(connection_string, wait_ready=True, 
                                     heartbeat_timeout=10, timeout=30)
            
            self.connection_status_changed.emit(True, f"Pixhawk'a başarıyla bağlanıldı! ({connection_type})")
            
            # Dinleyicileri ekle
            self.vehicle.add_attribute_listener('location.global_relative_frame', self.location_callback)
            self.vehicle.add_attribute_listener('heading', self.heading_callback)

        except Exception as e:
            self.connection_status_changed.emit(False, f"Bağlantı hatası: {e}")
            self.log_message_received.emit(f"Pixhawk bağlantısı başarısız: {e}")
            # Bağlantı başarısız olsa bile uygulama çalışmaya devam etsin
            self.vehicle = None

    @pyqtSlot(bool, str)
    def on_connection_status_changed(self, connected, message):
        self.is_connected = connected
        self.status_message_received.emit(message)
        self.log_message_received.emit(message)
        
        if connected:
            self.connect_button.setText("BAĞLANTIYI KES")
            self.connect_button.setStyleSheet("background-color: red; color: white; font-weight: bold;")
            self.telemetry_timer.start(500)  # 500ms = 2Hz - daha hızlı güncelleme
            self.attitude_timer.start(50)   # 50ms = 20Hz - daha smooth attitude
            
            # SERVO_OUTPUT_RAW mesajını request et
            self.request_servo_output()
            
            # Bağlandıktan hemen sonra bir kez telemetri güncelle
            QTimer.singleShot(100, self.update_telemetry)  # 100ms sonra
            for btn in self.mode_buttons:
                btn.setEnabled(True)
        else:
            self.connect_button.setText("BAĞLAN")
            self.connect_button.setStyleSheet("")
            self.telemetry_timer.stop()
            self.attitude_timer.stop()
            self.graphs_timer.stop()  # Grafik timer'ını durdur
            for btn in self.mode_buttons:
                btn.setEnabled(False)
            self.vehicle = None
            
            # Cache'i temizle
            self.servo_output_cache.clear()

    def request_servo_output(self):
        """ArduPilot'tan SERVO_OUTPUT_RAW mesajını request et"""
        if self.vehicle:
            try:
                # SERVO_OUTPUT_RAW mesajını aktif et
                msg = self.vehicle.message_factory.request_data_stream_encode(
                    self.vehicle._master.target_system,
                    self.vehicle._master.target_component,
                    2,  # MAV_DATA_STREAM_EXTENDED_STATUS
                    2,  # 2 Hz rate
                    1   # start_stop (1=start)
                )
                self.vehicle.send_mavlink(msg)
                self.log_message_received.emit("SERVO_OUTPUT_RAW stream başlatıldı (2Hz)")
                
                # Servo function'ları da alalım
                QTimer.singleShot(2000, self.log_servo_functions)  # 2 saniye sonra
                
            except Exception as e:
                self.log_message_received.emit(f"SERVO_OUTPUT request hatası: {e}")

    def log_servo_functions(self):
        """Servo function parametrelerini logla"""
        if self.vehicle and hasattr(self.vehicle, 'parameters'):
            try:
                # Vehicle tipi ve versiyonu
                vehicle_type = getattr(self.vehicle, '_vehicle_type', 'UNKNOWN')
                version = getattr(self.vehicle, 'version', 'UNKNOWN')
                
                self.log_message_received.emit(f"Vehicle: {vehicle_type}, Version: {version}")
                
                # Servo function'ları
                servo1_func = self.vehicle.parameters.get('SERVO1_FUNCTION', None)
                servo2_func = self.vehicle.parameters.get('SERVO2_FUNCTION', None)
                servo3_func = self.vehicle.parameters.get('SERVO3_FUNCTION', None)
                servo4_func = self.vehicle.parameters.get('SERVO4_FUNCTION', None)
                
                # Frame class ve type
                frame_class = self.vehicle.parameters.get('FRAME_CLASS', None)
                frame_type = self.vehicle.parameters.get('FRAME_TYPE', None)
                
                self.log_message_received.emit(f"Frame: CLASS={frame_class}, TYPE={frame_type}")
                self.log_message_received.emit(f"Servo Functions: S1={servo1_func}, S2={servo2_func}, S3={servo3_func}, S4={servo4_func}")
                
                # Motor çıkış kanalları kontrol et
                self.debug_all_servo_channels()
                
            except Exception as e:
                self.log_message_received.emit(f"Vehicle info okuma hatası: {e}")

    def debug_all_servo_channels(self):
        """Tüm servo kanallarını debug et"""
        if self.vehicle:
            try:
                # DroneKit channels attribute'u kontrol et
                self.log_message_received.emit("=== CHANNELS DEBUG ===")
                
                if hasattr(self.vehicle, 'channels'):
                    if self.vehicle.channels is not None:
                        self.log_message_received.emit(f"✓ vehicle.channels type: {type(self.vehicle.channels)}")
                        
                        # Channels içindeki attribute'ları kontrol et
                        channels_attrs = dir(self.vehicle.channels)
                        self.log_message_received.emit(f"Channels attributes: {[attr for attr in channels_attrs if not attr.startswith('_')]}")
                        
                        # Eğer channels bir dict-like object ise, içeriğini göster
                        try:
                            channels_dict = dict(self.vehicle.channels)
                            self.log_message_received.emit(f"Channels dict: {channels_dict}")
                        except:
                            self.log_message_received.emit("Channels dict'e çevrilemedi")
                        
                        # Override durumu
                        if hasattr(self.vehicle.channels, 'overrides'):
                            self.log_message_received.emit(f"Overrides mevcut: {self.vehicle.channels.overrides}")
                        else:
                            self.log_message_received.emit("Overrides attribute yok")
                            
                    else:
                        self.log_message_received.emit("✗ vehicle.channels = None")
                else:
                    self.log_message_received.emit("✗ vehicle.channels attribute yok")
                
                # Servo output raw message listener ekle - CACHE'E KAYDET
                def servo_output_listener(vehicle, name, message):
                    # Servo değerlerini parse et
                    servo_values = [
                        message.servo1_raw, message.servo2_raw, message.servo3_raw, message.servo4_raw,
                        message.servo5_raw, message.servo6_raw, message.servo7_raw, message.servo8_raw
                    ]
                    active_servos = [(i+1, val) for i, val in enumerate(servo_values) if val != 0]
                    self.log_message_received.emit(f"✓ SERVO_OUTPUT_RAW alındı! Aktif: {active_servos}")
                    
                    # Cache'e kaydet - DroneKit channels güncellenmiyor
                    for i, val in enumerate(servo_values):
                        if val != 0:  # Sadece aktif kanalları kaydet
                            self.servo_output_cache[str(i+1)] = val
                                
                self.vehicle.on_message('SERVO_OUTPUT_RAW')(servo_output_listener)
                self.log_message_received.emit("✓ SERVO_OUTPUT_RAW listener eklendi")
                
                # Periyodik channel durumu kontrol et
                QTimer.singleShot(5000, self.periodic_channel_check)  # 5 saniye sonra
                
            except Exception as e:
                self.log_message_received.emit(f"Servo debug hatası: {e}")
                import traceback
                self.log_message_received.emit(f"Stack trace: {traceback.format_exc()}")

    def periodic_channel_check(self):
        """Periyodik olarak channel durumunu kontrol et"""
        if self.vehicle:
            try:
                # Cache durumunu kontrol et
                ch1 = self.servo_output_cache.get('1', 'YOK')
                ch2 = self.servo_output_cache.get('2', 'YOK') 
                ch3 = self.servo_output_cache.get('3', 'YOK')
                ch4 = self.servo_output_cache.get('4', 'YOK')
                
                cache_count = len([x for x in [ch1, ch2, ch3, ch4] if x != 'YOK'])
                self.log_message_received.emit(f"Cache check: CH1={ch1}, CH2={ch2}, CH3={ch3}, CH4={ch4} (Aktif: {cache_count})")
                    
                # 10 saniye sonra tekrar kontrol et
                QTimer.singleShot(10000, self.periodic_channel_check)
                
            except Exception as e:
                self.log_message_received.emit(f"Periyodik check hatası: {e}")

    def change_mode(self, mode_name):
        """Vehicle modunu değiştir"""
        if self.vehicle:
            try:
                self.vehicle.mode = VehicleMode(mode_name)
                self.log_message_received.emit(f"Mod değiştirildi: {mode_name}")
            except Exception as e:
                self.log_message_received.emit(f"Mod değiştirme hatası: {e}")

    def test_thruster(self, side):
        """Manuel thruster test - PWM komutları gönder"""
        if not self.vehicle:
            self.log_message_received.emit("Thruster test: Vehicle bağlı değil")
            return
            
        try:
            # Önce channels.overrides'ı initialize et
            if not hasattr(self.vehicle.channels, 'overrides'):
                self.log_message_received.emit("channels.overrides initialize ediliyor...")
                self.vehicle.channels.overrides = {}
            
            # Mevcut override durumunu logla
            current_overrides = getattr(self.vehicle.channels, 'overrides', {})
            self.log_message_received.emit(f"Mevcut overrides: {current_overrides}")
            
            if side == 'left':
                # Sol thruster test: Kanal 2'ye 1600μs (60% güç)
                self.vehicle.channels.overrides['2'] = 1600
                self.log_message_received.emit("Sol thruster test: CH2=1600μs GÖNDER")
                
                # Doğrulama
                if '2' in self.vehicle.channels.overrides:
                    self.log_message_received.emit(f"Sol override doğrulandı: CH2={self.vehicle.channels.overrides['2']}")
                else:
                    self.log_message_received.emit("SOL OVERRIDE BAŞARISIZ!")
                    
            elif side == 'right':
                # Sağ thruster test: Kanal 1'e 1600μs (60% güç)  
                self.vehicle.channels.overrides['1'] = 1600
                self.log_message_received.emit("Sağ thruster test: CH1=1600μs GÖNDER")
                
                # Doğrulama
                if '1' in self.vehicle.channels.overrides:
                    self.log_message_received.emit(f"Sağ override doğrulandı: CH1={self.vehicle.channels.overrides['1']}")
                else:
                    self.log_message_received.emit("SAĞ OVERRIDE BAŞARISIZ!")
            
            # Override sonrası durumu logla
            final_overrides = getattr(self.vehicle.channels, 'overrides', {})
            self.log_message_received.emit(f"Test sonrası overrides: {final_overrides}")
                
        except Exception as e:
            self.log_message_received.emit(f"Thruster test hatası: {e}")
            import traceback
            self.log_message_received.emit(f"Stack trace: {traceback.format_exc()}")

    def stop_thrusters(self):
        """Tüm thruster'ları durdur"""
        if not self.vehicle:
            return
            
        try:
            # Her iki kanala da nötr PWM gönder
            self.vehicle.channels.overrides['1'] = 1500  # Sağ thruster nötr
            self.vehicle.channels.overrides['2'] = 1500  # Sol thruster nötr
            self.log_message_received.emit("Thruster'lar durduruldu: CH1=CH2=1500μs")
            
        except Exception as e:
            self.log_message_received.emit(f"Thruster durdurma hatası: {e}")

    def disconnect_from_vehicle(self):
        if self.vehicle:
            self.vehicle.close()
        self.on_connection_status_changed(False, "Bağlantı sonlandırıldı.")

    def refresh_ports(self):
        self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)
        if not ports:
            self.port_combo.addItem("Port bulunamadı")

    def update_telemetry(self):
        if not self.is_connected or not self.vehicle:
            return

        try:
            # GERÇEK ARDUPİLOT VERİLERİ - sadece mevcut olanları al
            speed = getattr(self.vehicle, 'groundspeed', None)
            heading = getattr(self.vehicle, 'heading', None)  
            mode = getattr(self.vehicle.mode, 'name', None) if hasattr(self.vehicle, 'mode') else None
            
            # Location güvenli alım
            alt = None
            if hasattr(self.vehicle, 'location') and self.vehicle.location:
                if hasattr(self.vehicle.location, 'global_relative_frame'):
                    alt = getattr(self.vehicle.location.global_relative_frame, 'alt', None)
            
            # Armed durumu
            armed = getattr(self.vehicle, 'armed', None)
            
            # Veri eksikse: gösterme 
            if speed is None:
                self.telemetry_values["Hız:"].setText("Veri yok")
            else:
                self.telemetry_values["Hız:"].setText(f"{speed:.1f} m/s")
                
            if heading is None:
                self.telemetry_values["Heading:"].setText("Veri yok")
            else:
                self.telemetry_values["Heading:"].setText(f"{heading}°")
                
            if alt is None:
                self.telemetry_values["Yükseklik:"].setText("Veri yok") 
            else:
                self.telemetry_values["Yükseklik:"].setText(f"{alt:.1f} m")
                
            if mode is None:
                self.telemetry_values["Mod:"].setText("UNKNOWN")
            else:
                self.telemetry_values["Mod:"].setText(mode)
            
            # GERÇEK SETPOINT VERİLERİ - ArduPilot'tan al
            if mode in ["AUTO", "GUIDED", "RTL"]:
                # Heading Setpoint: Aktif waypoint varsa bearing hesapla
                if len(self.waypoints) > 0 and heading is not None:
                    target_heading = self.get_target_heading_from_mission(heading)
                    self.telemetry_values["Heading Setpoint:"].setText(f"{target_heading:.0f}°")
                else:
                    self.telemetry_values["Heading Setpoint:"].setText("Veri yok")

                # Hız Setpoint: WPNAV_SPEED veya WP_SPEED parametresinden al (cm/s → m/s)
                wpnav_speed = self.vehicle.parameters.get('WPNAV_SPEED', None)
                wp_speed = self.vehicle.parameters.get('WP_SPEED', None)
                speed_param = wpnav_speed if wpnav_speed is not None else wp_speed
                if speed_param is not None:
                    speed_ms = speed_param / 100.0  # cm/s → m/s
                    self.telemetry_values["Hız Setpoint:"].setText(f"{speed_ms:.2f} m/s")
                else:
                    self.telemetry_values["Hız Setpoint:"].setText("Veri yok")
            else:  # MANUAL, STABILIZE, etc.
                # Manuel modda: setpoint YOK
                self.telemetry_values["Hız Setpoint:"].setText("MANUAL")
                self.telemetry_values["Heading Setpoint:"].setText("MANUAL")
            
            self.telemetry_values["Yükseklik:"].setText(f"{alt:.1f} m")
            self.telemetry_values["Heading:"].setText(f"{heading}°")
            
            # Attitude - basit
            pitch_deg = math.degrees(self.vehicle.attitude.pitch)
            yaw_deg = math.degrees(self.vehicle.attitude.yaw)
            self.telemetry_values["Pitch:"].setText(f"{pitch_deg:.1f}°")
            self.telemetry_values["Yaw:"].setText(f"{yaw_deg:.1f}°")
            
            self.telemetry_values["Mod:"].setText(mode)

            if self.vehicle.battery and self.vehicle.battery.level is not None:
                battery_level = self.vehicle.battery.level
                self.telemetry_values["Batarya:"].setText(f"{battery_level}%")
                self.battery_progress.setValue(battery_level)

            if self.vehicle.gps_0:
                fix_str = f"{self.vehicle.gps_0.fix_type}D Fix ({self.vehicle.gps_0.satellites_visible} uydu)"
                self.telemetry_values["GPS Fix:"].setText(fix_str)
            
            # Rota label'ını güncelle
            if heading is not None and hasattr(self, 'current_heading_label'):
                if mode == "MANUAL":
                    self.current_heading_label.setText("Mevcut Rota: MANUAL")
                else:
                    self.current_heading_label.setText(f"Mevcut Rota: {heading}°")
            
            # Grafik veri bufferlarını güncelle
            current_time = time.time() - self.start_time  # Uygulama başlangıcından itibaren saniye
            self.time_data.append(current_time)
            
            # Hız verilerini ekle
            self.speed_data.append(speed if speed is not None else 0)
            
            # Hız setpoint hesapla
            speed_setpoint = 0
            if mode in ["AUTO", "GUIDED", "RTL"]:
                if groundspeed_setpoint is not None:
                    speed_setpoint = groundspeed_setpoint
                else:
                    # Alternatif: vehicle.parameters'dan WPNAV_SPEED al
                    wpnav_speed = getattr(self.vehicle.parameters, 'WPNAV_SPEED', None)
                    if wpnav_speed:
                        speed_setpoint = wpnav_speed / 100.0  # cm/s → m/s
            self.speed_setpoint_data.append(speed_setpoint)
            
            # Heading verilerini ekle
            self.heading_data.append(heading if heading is not None else 0)
            
            # Heading setpoint hesapla
            heading_setpoint = 0
            if mode in ["AUTO", "GUIDED", "RTL"]:
                if target_bearing is not None:
                    heading_setpoint = target_bearing
                elif nav_bearing is not None:
                    heading_setpoint = nav_bearing
            self.heading_setpoint_data.append(heading_setpoint)
            
            # Thruster verilerini ekle (PWM'den güç hesapla)
            left_power = 0
            right_power = 0
            if self.servo_output_cache:
                left_pwm = self.servo_output_cache.get('2')  # Sol thruster (CH2)
                right_pwm = self.servo_output_cache.get('1')  # Sağ thruster (CH1)
                
                if left_pwm is not None:
                    left_diff = abs(left_pwm - 1500)
                    left_power = min(100, left_diff / 5.0) if left_diff > 25 else 0
                    if left_pwm < 1475:  # Geri yön
                        left_power = -left_power
                        
                if right_pwm is not None:
                    right_diff = abs(right_pwm - 1500)
                    right_power = min(100, right_diff / 5.0) if right_diff > 25 else 0
                    if right_pwm < 1475:  # Geri yön
                        right_power = -right_power
                        
            self.thruster_left_data.append(left_power)
            self.thruster_right_data.append(right_power)

        
        except Exception as e:
            self.log_message_received.emit(f"Telemetri okuma hatası: {e}")

    def update_motor_simulation(self):
        """Thruster güçleri - gerçek bağlantıda Pixhawk'tan PWM değerleri alınır"""
        if not hasattr(self, 'thruster_labels'):
            return
        if self.is_connected and self.vehicle:
            try:
                right_pwm = self.servo_output_cache.get('1')
                left_pwm = self.servo_output_cache.get('2')
                # None kontrolü yap
                if left_pwm is not None and right_pwm is not None:
                    # Debug: Tüm PWM kanallarını kontrol et
                    all_channels_debug = []
                    for ch in range(1, 9):
                        pwm_val = self.servo_output_cache.get(str(ch))
                        if pwm_val is not None:
                            all_channels_debug.append(f"CH{ch}:{pwm_val}")
                    debug_msg = f"Cache PWM: {', '.join(all_channels_debug)} | SERVO1_FUNC=74(Sağ), SERVO2_FUNC=73(Sol)"
                    self.log_message_received.emit(debug_msg)
                    # Marine thruster PWM: 1500=neutral, 1000=tam geri, 2000=tam ileri
                    def calculate_thruster_power(pwm):
                        if pwm is None:
                            return 0, "NEUTRAL"
                        diff = pwm - 1500
                        power = abs(diff) / 5.0
                        power = max(0, min(100, power))
                        if abs(diff) <= 25:
                            direction = "NEUTRAL"
                            power = 0
                        elif diff > 25:
                            direction = "GERİ"
                        else:
                            direction = "İLERİ"
                        return power, direction
                    left_power, left_dir = calculate_thruster_power(left_pwm)
                    right_power, right_dir = calculate_thruster_power(right_pwm)
                    # Sol motor ilk (UI sırası), Sağ motor ikinci
                    motor_data = [(left_power, left_dir, left_pwm, "Sol", "CH2(73)"),
                                  (right_power, right_dir, right_pwm, "Sağ", "CH1(74)")]
                    # --- PWM GRAFİK BUFFER ---
                    now = time.time()
                    if not hasattr(self, 'pwm_time_start'):
                        self.pwm_time_start = now
                    t = now - self.pwm_time_start
                    # Grafik buffer'ına label'a yazılacak PWM değerlerini ekle
                    self.pwm_time_buffer.append(t)
                    self.pwm_left_buffer.append(left_pwm)
                    self.pwm_right_buffer.append(right_pwm)
                    if len(self.pwm_time_buffer) > self.pwm_max_points:
                        self.pwm_time_buffer = self.pwm_time_buffer[-self.pwm_max_points:]
                        self.pwm_left_buffer = self.pwm_left_buffer[-self.pwm_max_points:]
                        self.pwm_right_buffer = self.pwm_right_buffer[-self.pwm_max_points:]
                    self.left_curve.setData(self.pwm_time_buffer, self.pwm_left_buffer)
                    self.right_curve.setData(self.pwm_time_buffer, self.pwm_right_buffer)
                    # --- LABEL'LARA YAZ ---
                    for i, (power, direction, pwm_val, side, channel_info) in enumerate(motor_data):
                        if direction == "NEUTRAL":
                            color = "gray"
                        elif power < 30:
                            color = "green"
                        elif power < 70:
                            color = "orange"
                        else:
                            color = "red"
                        real_pwm = pwm_val if pwm_val else 0
                        self.thruster_labels[i].setText(f"{side}: {power:.0f}% {direction} ({real_pwm}μs)")
                        self.thruster_labels[i].setStyleSheet(f"border: 1px solid {color}; padding: 5px; font-size: 10px; font-weight: bold; color: {color};")
                    return
                else:
                    for i in range(2):
                        side = "Sol" if i == 0 else "Sağ"
                        self.thruster_labels[i].setText(f"{side}: VERİ BEKLENİYOR")
                        self.thruster_labels[i].setStyleSheet("border: 1px solid orange; padding: 5px; font-size: 10px; font-weight: bold; color: orange;")
                    return
            except Exception as e:
                self.log_message_received.emit(f"Thruster cache okunamadı: {e}")
                for i in range(2):
                    side = "Sol" if i == 0 else "Sağ"
                    self.thruster_labels[i].setText(f"{side}: HATA")
                    self.thruster_labels[i].setStyleSheet("border: 1px solid red; padding: 5px; font-size: 10px; font-weight: bold; color: red;")
                return
        
        # SADECE GERÇEK VERİ - SİMÜLASYON YOK
        if not self.is_connected or not self.vehicle:
            # Bağlantı yoksa: hiçbir şey gösterme
            for i in range(2):
                side = "Sol" if i == 0 else "Sağ"
                self.thruster_labels[i].setText(f"{side}: BAĞLANTI YOK")
                self.thruster_labels[i].setStyleSheet("border: 1px solid gray; padding: 5px; font-size: 10px; font-weight: bold; color: gray;")
            return
            
        # Cache boşsa: bekle  
        if not self.servo_output_cache or '1' not in self.servo_output_cache or '2' not in self.servo_output_cache:
            for i in range(2):
                side = "Sol" if i == 0 else "Sağ"
                self.thruster_labels[i].setText(f"{side}: VERİ BEKLENİYOR")
                self.thruster_labels[i].setStyleSheet("border: 1px solid orange; padding: 5px; font-size: 10px; font-weight: bold; color: orange;")
            return

    def update_graphs(self):
        """Grafikleri günceller"""
        try:
            # Eğer veri yoksa grafikleri güncelleme
            if len(self.time_data) == 0:
                return
            
            # Veri listelerini numpy array'e çevir
            time_array = list(self.time_data)
            
            # Hız grafiği güncelleme
            if len(self.speed_data) > 0:
                speed_array = list(self.speed_data)
                speed_setpoint_array = list(self.speed_setpoint_data)
                self.speed_curve.setData(time_array, speed_array)
                self.speed_setpoint_curve.setData(time_array, speed_setpoint_array)
            
            # Heading grafiği güncelleme
            if len(self.heading_data) > 0:
                heading_array = list(self.heading_data)
                heading_setpoint_array = list(self.heading_setpoint_data)
                self.heading_curve.setData(time_array, heading_array)
                self.heading_setpoint_curve.setData(time_array, heading_setpoint_array)
            
            # Thruster grafiği güncelleme
            if len(self.thruster_left_data) > 0:
                thruster_left_array = list(self.thruster_left_data)
                thruster_right_array = list(self.thruster_right_data)
                self.thruster_left_curve.setData(time_array, thruster_left_array)
                self.thruster_right_curve.setData(time_array, thruster_right_array)
                
        except Exception as e:
            self.log_message_received.emit(f"Grafik güncelleme hatası: {e}")

    def location_callback(self, vehicle, attr_name, value):
        if value and self.vehicle.heading is not None:
            # Koordinat doğruluğu için debug log (sadece 10 saniyede bir)
            import time
            current_time = time.time()
            if not hasattr(self, '_last_coord_log') or current_time - self._last_coord_log > 10:
                self._last_coord_log = current_time
                precision_info = f"Koordinat: {value.lat:.6f}, {value.lon:.6f}, Heading: {self.vehicle.heading}°"
                self.log_message_received.emit(f"📍 {precision_info}")
            
            self.bridge.updateVehiclePosition.emit(value.lat, value.lon, self.vehicle.heading)

    def heading_callback(self, vehicle, attr_name, value):
        self.current_heading = value
    
    def get_target_heading_from_mission(self, current_heading):
        """Mission waypoint'lerinden target heading hesapla"""
        if len(self.waypoints) > 0 and self.vehicle and hasattr(self.vehicle, 'location'):
            # Bir sonraki waypoint'e doğru hedef açıyı hesapla
            current_loc = self.vehicle.location.global_relative_frame
            next_wp = self.waypoints[0]  # İlk waypoint'i hedef al
            
            # Basit bearing hesaplama
            dlat = next_wp["lat"] - current_loc.lat
            dlon = next_wp["lng"] - current_loc.lon
            target_heading = math.degrees(math.atan2(dlon, dlat)) % 360
            return target_heading
        else:
            # Waypoint yoksa hafif sapma simüle et
            return (current_heading + 15) % 360

    def update_attitude(self):
        """Attitude indicator'ı günceller."""
        if not self.is_connected or not self.vehicle:
            return
        
        try:
            # Dronekit'ten attitude bilgilerini al
            if hasattr(self.vehicle, 'attitude'):
                pitch = self.vehicle.attitude.pitch  # radyan
                roll = self.vehicle.attitude.roll    # radyan
                self.attitude_indicator.set_attitude(pitch, roll)
        except Exception as e:
            self.log_message_received.emit(f"Attitude güncelleme hatası: {e}")
    
    def add_waypoint_to_list(self, lat, lng):
        """Haritadan gelen waypoint ekleme isteğini işler."""
        waypoint_num = len(self.waypoints) + 1
        self.waypoints.append({"lat": lat, "lng": lng, "num": waypoint_num})
        self.log_message_received.emit(f"Yeni Waypoint #{waypoint_num} eklendi: {lat:.6f}, {lng:.6f}")
        
        # Haritaya da waypoint'i çizmesi için sinyal gönder
        self.bridge.addWaypoint.emit(lat, lng)
    
    def remove_waypoint_from_list(self, index):
        """Haritadan gelen waypoint silme isteğini işler."""
        if 0 <= index < len(self.waypoints):
            removed_wp = self.waypoints.pop(index)
            self.log_message_received.emit(f"Waypoint #{removed_wp['num']} silindi")
            
            # Kalan waypoint'lerin numaralarını güncelle
            for i, wp in enumerate(self.waypoints):
                wp['num'] = i + 1

    @pyqtSlot(str)
    def update_log_safe(self, message):
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.log_display.append(f"[{timestamp}] {message}")

    @pyqtSlot(str)
    def update_status_safe(self, message):
        self.status_label.setText(f"  Durum: {message}")

    def set_vehicle_mode(self, mode):
        if not self.is_connected or not self.vehicle:
            self.status_message_received.emit("Mod değiştirmek için araç bağlantısı gerekli.")
            return
        
        self.status_message_received.emit(f"{mode} moduna geçiliyor...")
        threading.Thread(target=self._set_mode_thread, args=(mode,), daemon=True).start()

    def _set_mode_thread(self, mode):
        try:
            self.vehicle.mode = VehicleMode(mode)
            self.status_message_received.emit(f"Araç modu başarıyla {mode} olarak ayarlandı.")
            self.log_message_received.emit(f"Mod değiştirildi: {mode}")
        except Exception as e:
            self.status_message_received.emit(f"Mod değiştirme hatası: {e}")
            self.log_message_received.emit(f"HATA: Mod değiştirilemedi - {e}")

    def send_mission_to_vehicle(self):
        if not self.is_connected or not self.vehicle:
            self.status_message_received.emit("Rota göndermek için araç bağlantısı gerekli.")
            return
        if not self.waypoints:
            self.status_message_received.emit("Gönderilecek rota (waypoint) bulunmuyor.")
            return
        
        self.status_message_received.emit("Rota araca gönderiliyor...")
        threading.Thread(target=self._upload_mission_thread, daemon=True).start()

    def _upload_mission_thread(self):
        try:
            from dronekit import Command
            from pymavlink import mavutil
            
            cmds = self.vehicle.commands
            cmds.clear()

            for wp in self.waypoints:
                cmd = Command(
                    0, 0, 0, 
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 0, 0, 0, 0, 0,
                    wp["lat"], wp["lng"], 0
                )
                cmds.add(cmd)
            
            self.log_message_received.emit(f"{len(self.waypoints)} komut araca yükleniyor...")
            cmds.upload() 
            self.status_message_received.emit("Rota başarıyla gönderildi!")
            self.log_message_received.emit("Misyon araca yüklendi.")

        except Exception as e:
            self.status_message_received.emit(f"Rota gönderme hatası: {e}")
            self.log_message_received.emit(f"HATA: Rota gönderilemedi - {e}")

    def read_mission_from_vehicle(self):
        """Aracın bellekindeki misyonu okur (Mission Planner'daki Read butonu işlevi)"""
        if not self.is_connected or not self.vehicle:
            self.status_message_received.emit("Rota okumak için araç bağlantısı gerekli.")
            return
        
        self.status_message_received.emit("Aracın bellekindeki rota okunuyor...")
        threading.Thread(target=self._read_mission_thread, daemon=True).start()

    def _read_mission_thread(self):
        """Aractan misyonu okuma thread'i"""
        try:
            # Önce mevcut misyonu temizle
            self.waypoints = []
            self.bridge.clearMap.emit()
            
            # Aracın komutlarını indir
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()  # İndirmenin tamamlanmasını bekle
            
            waypoint_count = 0
            # Her komutu kontrol et
            for i, cmd in enumerate(cmds):
                # Sadece waypoint komutlarını al (NAV_WAYPOINT)
                if cmd.command == 16:  # MAV_CMD_NAV_WAYPOINT
                    waypoint_count += 1
                    waypoint = {
                        "lat": cmd.x,
                        "lng": cmd.y, 
                        "alt": cmd.z,
                        "num": waypoint_count
                    }
                    self.waypoints.append(waypoint)
                    
                    # Haritaya waypoint ekle
                    self.bridge.addWaypoint.emit(cmd.x, cmd.y)
                    
                    self.log_message_received.emit(
                        f"Waypoint #{waypoint_count} okundu: {cmd.x:.6f}, {cmd.y:.6f}, {cmd.z:.1f}m"
                    )
            
            if waypoint_count > 0:
                self.status_message_received.emit(f"Rota başarıyla okundu! {waypoint_count} waypoint alındı.")
                self.log_message_received.emit(f"Toplam {waypoint_count} waypoint araçtan okundu.")
            else:
                self.status_message_received.emit("Araçta kayıtlı rota bulunamadı.")
                self.log_message_received.emit("Araçta herhangi bir waypoint bulunamadı.")
                
        except Exception as e:
            self.status_message_received.emit(f"Rota okuma hatası: {e}")
            self.log_message_received.emit(f"HATA: Rota okunamadı - {e}")
    
    def clear_mission(self):
        self.waypoints = []
        self.bridge.clearMap.emit()
        self.log_message_received.emit("Rota ve harita temizlendi.")

    @pyqtSlot(str)
    def on_connection_type_changed(self, text):
        if text == "UDP (Kablosuz)":
            self.ip_input.setVisible(True)
            self.udp_port_input.setVisible(True)
        elif text == "TCP (WiFi)":
            self.ip_input.setVisible(True)
            self.udp_port_input.setVisible(False)
        else:
            self.ip_input.setVisible(False)
            self.udp_port_input.setVisible(False)

    # Rota Kontrol Fonksiyonları
    def read_current_heading(self):
        """Aracın mevcut rotasını okur"""
        if not self.is_connected or not self.vehicle:
            self.log_message_received.emit("Rota okumak için araç bağlantısı gerekli")
            return
        
        try:
            current_heading = getattr(self.vehicle, 'heading', None)
            if current_heading is not None:
                self.current_heading_label.setText(f"Mevcut Rota: {current_heading}°")
                self.target_heading_input.setText(str(int(current_heading)))
                self.log_message_received.emit(f"Mevcut rota okundu: {current_heading}°")
            else:
                self.log_message_received.emit("Rota bilgisi henüz mevcut değil")
        except Exception as e:
            self.log_message_received.emit(f"Rota okuma hatası: {e}")

    def send_heading_command(self):
        """Girilen hedef rotayı araca gönderir"""
        if not self.is_connected or not self.vehicle:
            self.log_message_received.emit("Rota göndermek için araç bağlantısı gerekli")
            return
        
        try:
            target_heading_text = self.target_heading_input.text().strip()
            if not target_heading_text:
                self.log_message_received.emit("Hedef rota değeri giriniz (0-359°)")
                return
            
            target_heading = float(target_heading_text)
            if target_heading < 0 or target_heading > 359:
                self.log_message_received.emit("Rota değeri 0-359° arasında olmalıdır")
                return
            
            # GUIDED modda hedef rota gönder
            if self.vehicle.mode.name != "GUIDED":
                self.log_message_received.emit("Rota göndermek için GUIDED moda geçiliyor...")
                self.vehicle.mode = VehicleMode("GUIDED")
                time.sleep(2)  # Mod değişimini bekle
            
            # Rota komutunu gönder - ArduPilot SUB için
            msg = self.vehicle.message_factory.set_position_target_global_int_encode(
                0,  # time_boot_ms
                self.vehicle._master.target_system,
                self.vehicle._master.target_component,
                8,  # coordinate_frame (MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
                0b110111111000,  # type_mask (sadece yaw açısını kullan)
                0, 0, 0,  # lat, lon, alt (kullanılmıyor)
                0, 0, 0,  # vx, vy, vz (kullanılmıyor) 
                0, 0, math.radians(target_heading)  # afx, afy, yaw
            )
            self.vehicle.send_mavlink(msg)
            
            self.log_message_received.emit(f"Hedef rota gönderildi: {target_heading}°")
            
        except ValueError:
            self.log_message_received.emit("Geçersiz rota değeri. Sayı giriniz.")
        except Exception as e:
            self.log_message_received.emit(f"Rota gönderme hatası: {e}")

    def clear_heading_command(self):
        """Rota komutlarını temizler"""
        if not self.is_connected or not self.vehicle:
            self.log_message_received.emit("Rota temizlemek için araç bağlantısı gerekli")
            return
        
        try:
            # MANUAL moda geçerek otomatik rota kontrolünü durdur
            self.vehicle.mode = VehicleMode("MANUAL")
            self.current_heading_label.setText("Mevcut Rota: MANUAL")
            self.target_heading_input.clear()
            self.log_message_received.emit("Rota komutları temizlendi - MANUAL moda geçildi")
            
        except Exception as e:
            self.log_message_received.emit(f"Rota temizleme hatası: {e}")

if __name__ == '__main__':
    # qputenv("QTWEBENGINE_REMOTE_DEBUGGING", "9223") # Debug için
    app = QApplication(sys.argv)
    gcs_app = GCSApp()
    gcs_app.show()
    sys.exit(app.exec_())