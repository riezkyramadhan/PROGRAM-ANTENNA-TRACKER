# -*- coding: utf-8 -*-

# --- Import Libraries ---
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

# =======================================================================
#                           KONFIGURASI UTAMA
# =======================================================================

# --- Konfigurasi Koneksi Drone ---
CONNECTION_STRING = 'udp:127.0.0.1:14550'

# --- Konfigurasi Kecepatan Waypoint ---
# PASTIKAN JUMLAH ELEMEN DI SINI SAMA DENGAN JUMLAH WAYPOINT DI MISI DRONE ANDA
WAYPOINT_SPEEDS = [
    1,      # Kecepatan untuk WP 1
    1,      # Kecepatan untuk WP 2
    1,      # Kecepatan untuk WP 3
    1,      # Kecepatan untuk WP 4
    1.5,    # Kecepatan untuk WP 5
    1.5,    # Kecepatan untuk WP 6
    1.5,    # Kecepatan untuk WP 7
    1.5,    # Kecepatan untuk WP 8
    2,      # Kecepatan untuk WP 9
    2,      # Kecepatan untuk WP 10
    # Tambahkan atau kurangi baris sesuai jumlah waypoint di misi Anda
]

# =======================================================================
#                      KELAS NAVIGASI DRONE
# =======================================================================
class DroneWaypointNavigator:
    def __init__(self, connection_string):
        self.connection_string = connection_string
        self.vehicle = None

    def connect_vehicle(self):
        """Menghubungkan ke drone dan menunggu hingga siap."""
        print(f"INFO [Nav]: Menghubungkan ke drone di {self.connection_string}...")
        try:
            self.vehicle = connect(self.connection_string, wait_ready=True, timeout=60)
            print("✓ SUKSES [Nav]: Berhasil terhubung ke drone")
            # Tunggu hingga home location diketahui
            while not self.vehicle.home_location:
                cmds = self.vehicle.commands
                cmds.download()
                cmds.wait_ready()
                if not self.vehicle.home_location:
                    print(" [Nav] Menunggu lokasi home dari drone...")
                    time.sleep(1)
            print(f"✓ SUKSES [Nav]: Lokasi home drone diketahui: {self.vehicle.home_location}")
            return self.vehicle
        except Exception as e:
            print(f"✗ GAGAL [Nav]: Gagal terhubung ke drone: {e}")
            return None

    def download_mission(self):
        """Mengunduh daftar perintah (misi) dari flight controller."""
        print("INFO [Nav]: Mengunduh misi dari drone...")
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()  # Tunggu hingga proses download selesai
        print(f"✓ SUKSES [Nav]: Misi berhasil diunduh, ditemukan total {len(cmds)} perintah.")
        return cmds

    def get_distance_metres(self, aLocation1, aLocation2):
        """Menghitung jarak dalam meter antara dua titik LocationGlobal."""
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    def takeoff(self, target_altitude):
        """Proses arming dan takeoff drone ke ketinggian yang ditentukan."""
        print(f"\n==> TAHAP: TAKEOFF ke {target_altitude} meter <==")
        self.vehicle.mode = VehicleMode("GUIDED")
        while not self.vehicle.is_armable:
            print(" [Nav] Menunggu drone untuk bisa di-arm...")
            time.sleep(1)
        print(" [Nav] Arming drone...")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print(" [Nav] Menunggu konfirmasi arming...")
            time.sleep(1)
        print(f" [Nav] Memulai takeoff, target {target_altitude}m...")
        self.vehicle.simple_takeoff(target_altitude)
        while True:
            current_alt = self.vehicle.location.global_relative_frame.alt
            print(f" [Nav] Ketinggian aktual: {current_alt:.1f}m")
            if current_alt >= target_altitude * 0.95:
                print("✓ [Nav] Ketinggian takeoff tercapai")
                break
            time.sleep(1)

    def goto_waypoint(self, target_location, speed, waypoint_name):
        """Navigasi ke lokasi target dengan kecepatan yang ditentukan."""
        print(f"\n==> TAHAP: Menuju {waypoint_name} <==")
        
        print(f" [Nav] Mengatur kecepatan target ke: {speed} m/s")
        self.vehicle.groundspeed = speed
        
        print(f" [Nav] Target: Lat={target_location.lat:.7f}, Lon={target_location.lon:.7f}, Alt={target_location.alt}m")
        self.vehicle.simple_goto(target_location, groundspeed=speed)
        
        while self.vehicle.mode.name == "GUIDED":
            current_location = self.vehicle.location.global_relative_frame
            distance = self.get_distance_metres(current_location, target_location)
            
            print(f" [Nav] Ke {waypoint_name} -> Jarak: {distance:.1f}m | "
                  f"Kecepatan: {self.vehicle.groundspeed:.1f}m/s (Target: {speed}m/s) | "
                  f"Ketinggian: {current_location.alt:.1f}m")

            # Toleransi jarak sampai ke waypoint (2 meter)
            acceptance_radius = 2.0
            if distance <= acceptance_radius:
                print(f"✓ [Nav] {waypoint_name} tercapai!")
                break
            time.sleep(1)

    def land_at_last_waypoint(self):
        """Mengubah mode drone ke LAND untuk mendarat di posisi saat ini."""
        print("\n==> TAHAP: Landing di Waypoint Terakhir <==")
        print(" [Nav] Mengubah mode ke LAND...")
        self.vehicle.mode = VehicleMode("LAND")
        
        # Tunggu hingga drone mendarat dan disarm
        while self.vehicle.armed:
            current_alt = self.vehicle.location.global_relative_frame.alt
            print(f" [Nav] Proses landing... Ketinggian: {current_alt:.1f}m")
            time.sleep(2)
        
        print("✓ [Nav] Drone telah mendarat dan disarmed.")

    def run_mission(self):
        """
        Alur kerja utama: unduh misi, takeoff, jalankan waypoint, lalu mendarat.
        """
        try:
            # 1. Unduh misi dari drone
            mission_commands = self.download_mission()
            if not mission_commands:
                print("✗ GAGAL [Nav]: Tidak ada misi di drone untuk dijalankan. Program berhenti.")
                return

            # 2. Filter hanya perintah MAV_CMD_NAV_WAYPOINT
            waypoints = [cmd for cmd in mission_commands if cmd.command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT]
            
            if not waypoints:
                print("✗ GAGAL [Nav]: Misi ada, tetapi tidak ditemukan perintah WAYPOINT. Program berhenti.")
                return
            
            print(f"INFO [Nav]: Ditemukan {len(waypoints)} waypoint dalam misi untuk dieksekusi.")

            # 3. Validasi jumlah waypoint dengan daftar kecepatan
            if len(waypoints) != len(WAYPOINT_SPEEDS):
                print("\n" + "!"*60)
                print(f"!!! KESALAHAN KONFIGURASI !!!")
                print(f"Jumlah waypoint di drone ({len(waypoints)}) TIDAK SAMA dengan jumlah kecepatan di skrip ({len(WAYPOINT_SPEEDS)}).")
                print("Pastikan daftar 'WAYPOINT_SPEEDS' di skrip memiliki jumlah elemen yang sama.")
                print("!"*60 + "\n")
                return

            # 4. Takeoff ke ketinggian waypoint pertama
            takeoff_altitude = waypoints[0].z
            self.takeoff(takeoff_altitude)
            
            # 5. Loop melalui semua waypoint
            for i, wp in enumerate(waypoints):
                waypoint_name = f"WAYPOINT {i + 1}"
                target_location = LocationGlobalRelative(wp.x, wp.y, wp.z)
                target_speed = WAYPOINT_SPEEDS[i]
                
                self.goto_waypoint(target_location, target_speed, waypoint_name)
                
                # Jeda singkat setelah mencapai waypoint (kecuali yang terakhir)
                if i < len(waypoints) - 1:
                    print(f" [Nav] Tiba di {waypoint_name}. Jeda 3 detik...")
                    time.sleep(3)
            
            # 6. Mendarat di lokasi waypoint terakhir
            self.land_at_last_waypoint()
            
            print("\n" + "★"*20 + " MISI SELESAI " + "★"*20)
            
        except Exception as e:
            print(f"\n✗ ERROR DALAM MISI: {e}")
            print("INFO: Mengaktifkan RTL karena terjadi error.")
            if self.vehicle and self.vehicle.mode.name != "RTL":
                self.vehicle.mode = VehicleMode("RTL")

# =======================================================================
#                      FUNGSI UTAMA (MAIN)
# =======================================================================
if __name__ == "__main__":
    print("="*60)
    print("         PROGRAM NAVIGASI DRONE OTOMATIS")
    print("      (Versi 2.1b - Load Misi & Land at End)")
    print("="*60)

    navigator = DroneWaypointNavigator(CONNECTION_STRING)
    vehicle = navigator.connect_vehicle()

    if vehicle:
        try:
            # Langsung jalankan misi setelah terhubung
            navigator.run_mission()
            
        except KeyboardInterrupt:
            print("\n! INFO: Interupsi keyboard (Ctrl+C) terdeteksi. Mengaktifkan RTL.")
            if vehicle.armed and vehicle.mode.name != "RTL":
                vehicle.mode = VehicleMode("RTL")
        
        finally:
            print("\nINFO: Memulai proses shutdown...")
            if vehicle and vehicle.is_connected:
                vehicle.close()
                print("✓ INFO: Koneksi drone ditutup.")
            print("="*60)
            print("                 PROGRAM BERHENTI")
            print("="*60)
    else:
        print("\n✗ GAGAL: Tidak bisa melanjutkan karena koneksi ke drone gagal.")