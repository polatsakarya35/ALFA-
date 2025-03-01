from pymavlink import mavutil
import time
from threading import Thread
import math

class Drone:
    def __init__(self, connection_string):
        self.connection = mavutil.mavlink_connection(connection_string)
        print("Bağlantı bekleniyor...")
        self.connection.wait_heartbeat()
        print("Drone bağlantısı kuruldu.")

    def get_drone_location(self):
        try:
            msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)        
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0  # İrtifa: mm cinsinden, metreye çevrilir
            return lat, lon, alt
        except Exception as e:
            print(f"GPS koordinatları alınırken hata oluştu: {e}")
            return None, None, None
        
    def get_drone_speed(self):
        try:
            msg = self.connection.recv_match(type='VFR_HUD', blocking=True)
            airspeed = msg.airspeed
            groundspeed = msg.groundspeed
            return airspeed, groundspeed
        except Exception as e:
            print(f"Hız verileri alınırken hata oluştu: {e}")
            return None, None

    def get_drone_mode(self):
        try:
            msg = self.connection.recv_match(type='HEARTBEAT', blocking=True)
            mode = mavutil.mode_string_v10(msg)
            return mode
        except Exception as e:
            print(f"Mod verisi alınırken hata oluştu: {e}")
            return None
        
    def get_battery_level(self):
        try:
            msg = self.connection.recv_match(type='SYS_STATUS', blocking=True)
            battery_level = msg.battery_remaining
            return battery_level
        except Exception as e:
            print(f"Batarya seviyesi alınırken hata oluştu: {e}")
            return None
        
    def get_yaw_pitch_roll(self):
        try:
            msg = self.connection.recv_match(type='ATTITUDE', blocking=True)
            yaw = msg.yaw
            pitch = msg.pitch
            roll = msg.roll
            return yaw, pitch, roll
        except Exception as e:
            print(f"Yaw, pitch , roll verileri alınırken hata oluştu: {e}")
            return None, None, None
        
    def set_mode(self, mode):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mode,
            0, 0, 0, 0, 0, 0
        )

    def arm_drone(self):
        try:
            print("Drone GUIDED moda alınıyor...")
            self.set_mode(mavutil.mavlink.MAV_MODE_GUIDED)
            time.sleep(2)

            print("Drone ARM ediliyor...")
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,    # Onay
                1,    # ARM
                0, 0, 0, 0, 0, 0
            )
            while True:
                msg = self.connection.recv_match(type='HEARTBEAT', blocking=True)
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    print("Drone ARM edildi (uçuşa hazır).")
                    return True
                time.sleep(1)
        except Exception as e:
            print(f"Drone ARM edilirken hata oluştu: {e}")
            return False

    def takeoff_drone(self, target_altitude):
        try:
            print(f"Drone {target_altitude} metreye kalkış yapıyor...")
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,    # Onay
                0, 0, 0, 0,          # Kullanılmayan parametreler
                0, 0, target_altitude  # Hedef irtifa (metre)
            )
            while True:
                msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                if msg.relative_alt >= target_altitude * 1000:  # relative_alt: mm cinsinden
                    print(f"Drone {target_altitude} metreye ulaştı.")
                    print("Drone AUTO moda alınıyor...")
                    self.set_mode(mavutil.mavlink.MAV_MODE_AUTO)
                    return True
                time.sleep(1)
        except Exception as e:
            print(f"Kalkış komutu gönderilirken hata oluştu: {e}")
            return False  

    def navigate_to_waypoint(self, target_lat, target_lon, altitude):
        try:
            print(f"Drone {target_lat}, {target_lon} konumuna {altitude} metre yükseklikte uçuyor...")
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,    # Onay
                0, 0, 0, 0,          # Kullanılmayan parametreler
                target_lat, target_lon, altitude   # Hedef koordinatlar ve irtifa
            )
            while True:
                msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                current_lat = msg.lat / 1e7
                current_lon = msg.lon / 1e7
                if abs(current_lat - target_lat) < 0.0001 and abs(current_lon - target_lon) < 0.0001:
                    print(f"Drone {target_lat}, {target_lon} konumuna ulaştı.")
                    return True
                time.sleep(1)
        except Exception as e:
            print(f"Konuma uçma komutu gönderilirken hata oluştu: {e}")
            return False  

    def land_drone(self):
        try:
            print("Drone iniş yapıyor...")
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,    # Onay
                0, 0, 0, 0,          # Kullanılmayan parametreler
                0, 0, 0              # Hedef irtifa kullanılmaz (yer seviyesi)
            )
            while True:
                msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                if msg.relative_alt <= 100:  # 100 mm (0.1 m) altı kabul edelim
                    print("Drone iniş yaptı.")
                    return True
                time.sleep(1)
        except Exception as e:
            print(f"İniş komutu gönderilirken hata oluştu: {e}")
            return False             

    def disarm_drone(self):
        try:
            print("Drone DISARM ediliyor...")
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,    # Onay
                0,    # DISARM (motorları kapat)
                0, 0, 0, 0, 0, 0
            )
            print("Drone DISARM edildi (güvenli duruma geçti).")
            return True
        except Exception as e:
            print(f"Drone DISARM edilirken hata oluştu: {e}")
            return False
                
    def check_battery_level(self):
        try:
            msg = self.connection.recv_match(type='SYS_STATUS', blocking=True)
            battery_level = msg.battery_remaining
            if battery_level < 20:
                print("Dikkat! Batarya seviyesi kritik, iniş yapılıyor...")
                self.land_drone()
                self.disarm_drone()
                return False
            return True
        except Exception as e:
            print(f"Batarya seviyesi kontrol edilirken hata oluştu: {e}")
            return False

    def check_gps_signal(self):
        try:
            lat, lon, alt = self.get_drone_location()
            if lat is None or lon is None:
                print("GPS sinyali kayboldu, iniş yapılıyor...")
                self.land_drone()
                self.disarm_drone()
                return False
            return True
        except Exception as e:
            print(f"GPS sinyali kontrol edilirken hata oluştu: {e}")
            return False
                    
    def send_telemetry(self):
        """
        Ter istasyonuna telemetri verileri gönderir.
        """
        while True:
            lat, lon, alt = self.get_drone_location()
            airspeed, groundspeed = self.get_drone_speed()
            battery_level = self.get_battery_level()
            yaw, pitch, roll = self.get_yaw_pitch_roll()
            mode = self.get_drone_mode()

            print(f"Konum: {lat}, {lon}, {alt} m")
            print(f"Hız: {airspeed} m/s (hava), {groundspeed} m/s (yer)")
            print(f"Batarya: {battery_level}%")
            print(f"Yön: {yaw}, {pitch}, {roll}")
            print(f"Mod: {mode}")

            time.sleep(2)

    def main(self):
        lat, lon, alt = self.get_drone_location()
        if lat is None or lon is None:
            print("GPS koordinatları alınamadı.")
            return
        
        if self.arm_drone()
            if self.takeoff_drone(10): # Örnek olarak 10 metre 
                print("Uçuş başladı!")
                airspeed, groundspeed = self.get_drone_speed()
                if airspeed is not None and groundspeed is not None:
                    print(f"Hava hızı: {airspeed} m/s, Yer hızı: {groundspeed} m/s")
                
                telemetry_thread = Thread(target=self.send_telemetry)
                telemetry_thread.start()

                battery_thread = Thread(target=self.check_battery_level)
                gps_thread = Thread(target=self.check_gps_signal)
                battery_thread.start()
                gps_thread.start()

                if self.navigate_to_waypoint(41.0, 29.0, 10):  # İlk konum
                    if self.navigate_to_waypoint(40.0, 28.0, 10): #İkinci konum
                        print("Uçuş sona erdi.") # İki farklı konumada sırasıyla gider 
                        
                        if self.land_drone():
                            self.disarm_drone()
                            print("Drone güvenli bir şekilde durduruldu.")
            else:
                print("Kalkış işlemi başarısız.")
        else:
            print("Drone ARM edilemedi.")

        battery_thread.join()
        gps_thread.join()
        telemetry_thread.join()

if __name__ == "__main__":
    drone = Drone('udp:127.0.0.1:14550')
    drone.main()                        
            
            