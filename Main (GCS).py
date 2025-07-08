# -*- coding: utf-8 -*-

import serial
import time
import math
import requests
from threading import Thread

# =======================================================================
#                           KONFIGURASI UTAMA
# =======================================================================

# --- Konfigurasi Serial ---
SERIAL_PORT = '/dev/ttyUSB0'  
BAUD_RATE = 115200

# --- Konfigurasi API Telemetri ---

TELEMETRY_API_URL = 'http://127.0.0.1:5000/telemetry'

# --- Konfigurasi Koordinat Stasiun Bumi (Ground Station) ---
GROUND_LAT = -7.7713    # Latitude Stasiun Bumi (derajat)
GROUND_LON = 110.3774   # Longitude Stasiun Bumi (derajat)
GROUND_ALT = 120.0      # Ketinggian Stasiun Bumi (meter di atas permukaan laut)

EARTH_RADIUS = 6371000  # Radius bumi dalam meter

# =======================================================================
#                      KELAS PELACAK DRONE (ANTENNA TRACKER)
# =======================================================================

class DroneTracker:
    def __init__(self, port=SERIAL_PORT, baud_rate=BAUD_RATE, api_url=TELEMETRY_API_URL):
        self.port = port
        self.baud_rate = baud_rate
        self.api_url = api_url
        self.serial_connected = False
        self.api_connected = False
        self.ser = None
        self.running = False
        
        # Data drone saat ini (diperoleh dari API)
        self.drone_lat = 0.0
        self.drone_lon = 0.0
        self.drone_alt = 0.0  
        
        # Data tambahan yang dibutuhkan oleh mikrokontroler (diberi nilai default)
        self.drone_spd = 0.0  # Kecepatan drone
        self.drone_rssi = 0   # Kekuatan sinyal (raw)
        self.drone_sig = 0    # Kekuatan sinyal (persentase)
        
        # Data pelacakan yang dihitung
        self.azimuth = 0.0
        self.elevation = 0.0
        self.distance = 0.0
        
        # Mencoba koneksi serial saat inisialisasi
        self.connect_serial()
    
    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            time.sleep(2) 
            self.serial_connected = True
            print(f"✓ SUKSES: Terhubung ke mikrokontroler di {self.port} pada {self.baud_rate} baud")
            return True
        except Exception as e:
            print(f"✗ GAGAL: Koneksi serial gagal: {e}")
            self.serial_connected = False
            return False
    
    def calculate_azimuth(self, lat1, lon1, lat2, lon2):
    
        # Konversi ke radian
        lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
        lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)
        
        # Rumus bearing
        y = math.sin(lon2_rad - lon1_rad) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
            math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(lon2_rad - lon1_rad)
        bearing_rad = math.atan2(y, x)
        
        # Konversi ke derajat dan normalisasi (0-360°)
        azimuth = (math.degrees(bearing_rad) + 360) % 360
        return azimuth
    
    def calculate_haversine_distance(self, lat1, lon1, lat2, lon2):

        # Konversi ke radian
        lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
        lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)
        
        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = EARTH_RADIUS * c  # Jarak dalam meter
        return distance
    
    def calculate_elevation(self, distance, alt_diff):
        if distance == 0:
            return 90.0 if alt_diff >= 0 else -90.0
        
        # Hitung sudut elevasi dalam radian dan konversi ke derajat
        elevation_deg = math.degrees(math.atan2(alt_diff, distance))
        return elevation_deg
    
    def fetch_telemetry_data(self):

        try:
            response = requests.get(self.api_url, timeout=2.5)
            
            if response.status_code == 200:
                data = response.json()
                
                # Ekstrak data posisi wajib
                if 'lat' in data and 'lon' in data and 'alt' in data:
                    self.drone_lat = data['lat']
                    self.drone_lon = data['lon']
                    self.drone_alt = data['alt'] 
                    
                    self.drone_spd = data.get('spd', self.drone_spd)
                    self.drone_rssi = data.get('rssi', self.drone_rssi)
                    self.drone_sig = data.get('sig', self.drone_sig)
                    
                    if not self.api_connected:
                        print("✓ SUKSES: Terhubung ke API Telemetri.")
                        self.api_connected = True
                    return True
                else:
                    print("✗ GAGAL: Data 'lat', 'lon', atau 'alt' tidak ditemukan di respon API.")
                    return False
            else:
                print(f"✗ GAGAL: Error API dengan status {response.status_code}")
                self.api_connected = False
                return False
                
        except requests.exceptions.RequestException as e:
            if self.api_connected:
                print(f"✗ GAGAL: Gagal mengambil data dari API: {e}")
            self.api_connected = False
            return False
    
    def update_tracking_data(self):
       
        # Hitung jarak horizontal menggunakan Haversine
        self.distance = self.calculate_haversine_distance(
            GROUND_LAT, GROUND_LON, self.drone_lat, self.drone_lon
        )
        
        # Hitung azimuth (arah kompas)
        self.azimuth = self.calculate_azimuth(
            GROUND_LAT, GROUND_LON, self.drone_lat, self.drone_lon
        )
        
        # Hitung perbedaan ketinggian (drone_alt dianggap relatif terhadap ground)
        alt_diff = self.drone_alt # 
        
        # Hitung sudut elevasi
        self.elevation = self.calculate_elevation(self.distance, alt_diff)
        
        # Log data ke konsol
        print(f"Target -> Azm: {self.azimuth:.1f}°, Elv: {self.elevation:.1f}°, Dist: {self.distance:.1f}m | "
              f"Posisi: Lat={self.drone_lat:.5f}, Lon={self.drone_lon:.5f}, Alt={self.drone_alt:.1f}m")
    
    def send_to_controller(self):
       
        if not self.serial_connected:
            return False
        
        try:
            # Membuat command string 
            command = (f"AZM:{self.azimuth:.1f},ELV:{self.elevation:.1f},"
                       f"LAT:{self.drone_lat:.6f},LON:{self.drone_lon:.6f},"
                       f"ALT:{self.drone_alt:.1f},SPD:{self.drone_spd:.1f},"
                       f"RSSI:{self.drone_rssi},SIG:{self.drone_sig}\n")
            
            self.ser.write(command.encode('utf-8'))
            # print(f"Sent: {command.strip()}") 
            
            return True
        except Exception as e:
            print(f"✗ GAGAL: Error saat mengirim data serial: {e}. Koneksi terputus.")
            self.serial_connected = False
            # Mencoba menutup port jika masih terbuka
            if self.ser:
                self.ser.close()
            return False
    
    def run(self):
        self.running = True
        
        while self.running:
            # 1. Coba ambil data dari API
            if self.fetch_telemetry_data():
                # 2. Jika berhasil, hitung data pelacakan
                self.update_tracking_data()
                
                # 3. Kirim data ke mikrokontroler
                if self.serial_connected:
                    self.send_to_controller()
                else:
                    # Jika serial tidak terhubung, coba sambungkan kembali
                    self.connect_serial()
            else:
                # Jika API gagal, cetak pesan tunggu
                if not self.api_connected:
                    print("Menunggu data dari API telemetri...")
            
            # Tunggu sebelum iterasi berikutnya
            time.sleep(0.01)  # Update setiap 100ms (10Hz)
    
    def start(self):
        thread = Thread(target=self.run, daemon=True)
        thread.start()
        return thread
    
    def stop(self):
        print("Menghentikan thread pelacak...")
        self.running = False
        time.sleep(0.5) # Beri waktu agar loop berhenti
        if self.serial_connected and self.ser:
            self.ser.close()
            print("Koneksi serial ditutup.")


if __name__ == "__main__":
    print("="*50)
    print("   Sistem Pelacak Antena via Telemetri API")
    print("="*50)
    print(f"Stasiun Bumi: Lat={GROUND_LAT}, Lon={GROUND_LON}, Alt={GROUND_ALT}m")
    print(f"Port Serial  : {SERIAL_PORT}")
    print(f"URL API      : {TELEMETRY_API_URL}")
    print("="*50)
    
    tracker = DroneTracker()
    
    try:
        tracker.start()
        print("Pelacak dimulai. Tekan Ctrl+C untuk keluar.")
        
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nInterupsi diterima. Menghentikan program...")
        tracker.stop()
        print("Program berhenti.")

