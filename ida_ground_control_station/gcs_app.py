import sys
import os
import time
import threading
import math
from PyQt5.QtCore import QUrl, Qt, QTimer, pyqtSignal, QObject, pyqtSlot, QDateTime
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QPushButton, QLabel,
                             QTextEdit, QHBoxLayout, QMessageBox, QGridLayout,
                             QProgressBar, QGroupBox, QComboBox, QDoubleSpinBox, QDialog, QFormLayout, QFrame, QScrollArea)
from PyQt5.QtGui import QPixmap, QIcon, QTransform, QTextCursor, QFont
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage
from PyQt5.QtWebChannel import QWebChannel

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
        
        # Thruster durumu için basit gösterim (Deniz aracı - 2 motor)
        row += 1
        thruster_layout = QHBoxLayout()
        self.thruster_labels = []
        for i in range(2):  # Deniz aracı için 2 thruster
            thruster_label = QLabel(f"T{i+1}: 0%")
            thruster_label.setStyleSheet("border: 1px solid gray; padding: 3px; font-size: 9px;")
            thruster_label.setMinimumWidth(120)  # Minimum genişlik ekle
            thruster_label.setMaximumWidth(120)  # Sabit genişlik için
            thruster_layout.addWidget(thruster_label)
            self.thruster_labels.append(thruster_label)
        
        telemetry_layout.addWidget(QLabel("Thruster'lar:"), row, 0)
        telemetry_layout.addLayout(thruster_layout, row, 1, 1, 2)

        # Attitude Indicator (Gyro) ekle
        self.attitude_indicator = AttitudeIndicator()
        self.attitude_indicator.setFixedSize(120, 120)
        telemetry_layout.addWidget(self.attitude_indicator, 1, 2, 4, 2)
        
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
            # Bağlandıktan hemen sonra bir kez telemetri güncelle
            QTimer.singleShot(100, self.update_telemetry)  # 100ms sonra
            for btn in self.mode_buttons:
                btn.setEnabled(True)
        else:
            self.connect_button.setText("BAĞLAN")
            self.connect_button.setStyleSheet("")
            self.telemetry_timer.stop()
            self.attitude_timer.stop()
            for btn in self.mode_buttons:
                btn.setEnabled(False)
            self.vehicle = None

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
            # Gerçek target bearing (NAV_CONTROLLER_OUTPUT mesajından)
            target_bearing = getattr(self.vehicle, 'target_bearing', None)
            nav_bearing = getattr(self.vehicle, 'nav_bearing', None)
            
            # Gerçek groundspeed setpoint  
            groundspeed_setpoint = getattr(self.vehicle, 'groundspeed_setpoint', None)
            
            # Mission durumu kontrol et
            current_waypoint = getattr(self.vehicle, 'commands', None)
            
            if mode in ["AUTO", "GUIDED", "RTL"]:
                # Autopilot modlarında gerçek setpoint'ler
                if target_bearing is not None:
                    self.telemetry_values["Heading Setpoint:"].setText(f"{target_bearing:.0f}°")
                elif nav_bearing is not None:
                    self.telemetry_values["Heading Setpoint:"].setText(f"{nav_bearing:.0f}°")
                else:
                    self.telemetry_values["Heading Setpoint:"].setText("Hesaplanıyor...")
                
                if groundspeed_setpoint is not None:
                    self.telemetry_values["Hız Setpoint:"].setText(f"{groundspeed_setpoint:.1f} m/s")
                else:
                    # Alternatif: vehicle.parameters'dan WPNAV_SPEED al
                    wpnav_speed = getattr(self.vehicle.parameters, 'WPNAV_SPEED', None)
                    if wpnav_speed:
                        speed_ms = wpnav_speed / 100.0  # cm/s → m/s
                        self.telemetry_values["Hız Setpoint:"].setText(f"{speed_ms:.1f} m/s")
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
            

        
        except Exception as e:
            self.log_message_received.emit(f"Telemetri okuma hatası: {e}")

    def update_motor_simulation(self):
        """Thruster güçleri - gerçek bağlantıda Pixhawk'tan PWM değerleri alınır"""
        if not hasattr(self, 'thruster_labels'):
            return
        
        if self.is_connected and self.vehicle:
            # GERÇEK VERİ: Pixhawk'tan servo çıkışları (PWM 1000-2000)
            # self.vehicle.channels['1'] = Sol thruster PWM
            # self.vehicle.channels['3'] = Sağ thruster PWM
            try:
                # ArduPilot SERVO_OUTPUT_RAW mesajından alınacak
                if hasattr(self.vehicle, 'channels') and self.vehicle.channels is not None:
                    # PWM değerlerini güvenli şekilde al
                    left_pwm = self.vehicle.channels.get('1')
                    right_pwm = self.vehicle.channels.get('3')
                    
                    # None kontrolü yap
                    if left_pwm is not None and right_pwm is not None:
                        # Debug: Tüm PWM kanallarını kontrol et
                        all_channels_debug = []
                        for ch in range(1, 9):  # Kanal 1-8 arası kontrol et
                            pwm_val = self.vehicle.channels.get(str(ch))
                            if pwm_val is not None:
                                all_channels_debug.append(f"CH{ch}:{pwm_val}")
                        
                        debug_msg = f"Aktif PWM Kanalları: {', '.join(all_channels_debug)}"
                        self.log_message_received.emit(debug_msg)
                        
                        # PWM'i yüzdeye çevir (1000-2000 → 0-100%) - DOĞRU FORMÜL
                        left_power = max(0, min(100, (left_pwm - 1000) / 10))  # 1000-2000 → 0-100
                        right_power = max(0, min(100, (right_pwm - 1000) / 10))  # 1000-2000 → 0-100
                        
                        motor_powers = [left_power, right_power]
                        pwm_values = [left_pwm, right_pwm]  # Gerçek PWM değerleri
                        sides = ["Sol", "Sağ"]
                        
                        for i, power in enumerate(motor_powers):
                            color = "green" if power < 70 else "orange" if power < 90 else "red"
                            real_pwm = pwm_values[i]  # Gerçek PWM değerini kullan
                            self.thruster_labels[i].setText(f"{sides[i]}: {power:.0f}% {real_pwm}μs")
                            self.thruster_labels[i].setStyleSheet(f"border: 1px solid {color}; padding: 3px; font-size: 9px; color: {color};")
                        return
                    else:
                        # PWM değerleri henüz None
                        self.log_message_received.emit("Thruster PWM verileri henüz yüklenmedi")
                else:
                    # Servo çıkışları henüz yüklenmemişse simülasyon yap
                    pass
            except Exception as e:
                self.log_message_received.emit(f"Thruster verisi okunamadı: {e}")
        
        # SADECE GERÇEK VERİ - SİMÜLASYON YOK
        if not self.is_connected or not self.vehicle:
            # Bağlantı yoksa: hiçbir şey gösterme
            for i in range(2):
                self.thruster_labels[i].setText(f"T{i+1}: NO CONNECTION")
                self.thruster_labels[i].setStyleSheet("border: 1px solid gray; padding: 3px; font-size: 9px; color: gray;")
            return
            
        # PWM değerleri yoksa: bekle
        if not hasattr(self.vehicle, 'channels') or self.vehicle.channels is None:
            for i in range(2):
                self.thruster_labels[i].setText(f"T{i+1}: PWM YOK")
                self.thruster_labels[i].setStyleSheet("border: 1px solid orange; padding: 3px; font-size: 9px; color: orange;")
            return
            
        # PWM verisi yoksa: gösterme
        left_pwm = self.vehicle.channels.get('1')
        right_pwm = self.vehicle.channels.get('3')
        
        if left_pwm is None or right_pwm is None:
            for i in range(2):
                side = "Sol" if i == 0 else "Sağ"
                self.thruster_labels[i].setText(f"{side}: SERVO VERİSİ YOK")
                self.thruster_labels[i].setStyleSheet("border: 1px solid orange; padding: 3px; font-size: 9px; color: orange;")
            return

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

if __name__ == '__main__':
    # qputenv("QTWEBENGINE_REMOTE_DEBUGGING", "9223") # Debug için
    app = QApplication(sys.argv)
    gcs_app = GCSApp()
    gcs_app.show()
    sys.exit(app.exec_())