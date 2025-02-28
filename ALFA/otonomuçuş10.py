from pymavlink import mavutil
import time
from threading import Thread
import math

# MAVLink bağlantısını kur (UDP bağlantısı)
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

print("Bağlantı bekleniyor...")
connection.wait_heartbeat()
print("Drone bağlantısı kuruldu.")

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Dünya yarıçapı (metre)
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def get_drone_location():
    try:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.relative_alt / 1000.0  # İrtifa: mm cinsinden, metreye çevrilir
        return lat, lon, alt
    except Exception as e:
        print(f"GPS koordinatları alınırken hata oluştu: {e}")
        return None, None, None

def get_drone_speed():
    try:
        msg = connection.recv_match(type='VFR_HUD', blocking=True)
        airspeed = msg.airspeed
        groundspeed = msg.groundspeed
        return airspeed, groundspeed
    except Exception as e:
        print(f"Hız verileri alınırken hata oluştu: {e}")
        return None, None

def set_mode(mode):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mode,
        0, 0, 0, 0, 0, 0
    )

def arm_drone():
    try:
        print("Drone GUIDED moda alınıyor...")
        set_mode(mavutil.mavlink.MAV_MODE_GUIDED)
        time.sleep(2)

        print("Drone ARM ediliyor...")
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,    # Onay
            1,    # ARM
            0, 0, 0, 0, 0, 0
        )
        while True:
            msg = connection.recv_match(type='HEARTBEAT', blocking=True)
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("Drone ARM edildi (uçuşa hazır).")
                return True
            time.sleep(1)
    except Exception as e:
        print(f"Drone ARM edilirken hata oluştu: {e}")
        return False

def takeoff_drone(target_altitude):
    try:
        print(f"Drone {target_altitude} metreye kalkış yapıyor...")
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,    # Onay
            0, 0, 0, 0,          # Kullanılmayan parametreler
            0, 0, target_altitude  # Hedef irtifa (metre)
        )
        while True:
            msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg.relative_alt >= target_altitude * 1000:  # relative_alt: mm cinsinden
                print(f"Drone {target_altitude} metreye ulaştı.")
                print("Drone AUTO moda alınıyor...")
                set_mode(mavutil.mavlink.MAV_MODE_AUTO)
                return True
            time.sleep(1)
    except Exception as e:
        print(f"Kalkış komutu gönderilirken hata oluştu: {e}")
        return False

def navigate_to_waypoint(target_lat, target_lon, altitude):
    try:
        print(f"Drone {target_lat}, {target_lon} konumuna {altitude} metre yükseklikte uçuyor...")
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,    # Onay
            0, 0, 0, 0,          # Kullanılmayan parametreler
            target_lat, target_lon, altitude   # Hedef koordinatlar ve irtifa
        )
        while True:
            msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            if abs(current_lat - target_lat) < 0.0001 and abs(current_lon - target_lon) < 0.0001:
                print(f"Drone {target_lat}, {target_lon} konumuna ulaştı.")
                return True
            time.sleep(1)
    except Exception as e:
        print(f"Konuma uçma komutu gönderilirken hata oluştu: {e}")
        return False

def land_drone():
    try:
        print("Drone iniş yapıyor...")
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,    # Onay
            0, 0, 0, 0,          # Kullanılmayan parametreler
            0, 0, 0              # Hedef irtifa kullanılmaz (yer seviyesi)
        )
        while True:
            msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg.relative_alt <= 100:  # 100 mm (0.1 m) altı kabul edelim
                print("Drone iniş yaptı.")
                return True
            time.sleep(1)
    except Exception as e:
        print(f"İniş komutu gönderilirken hata oluştu: {e}")
        return False

def disarm_drone():
    try:
        print("Drone DISARM ediliyor...")
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
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

def check_battery_level():
    try:
        msg = connection.recv_match(type='SYS_STATUS', blocking=True)
        battery_level = msg.battery_remaining
        if battery_level < 20:  # Batarya seviyesi %20'nin altına düşerse
            print("Batarya seviyesi kritik, iniş yapılıyor...")
            land_drone()
            disarm_drone()
            return False
        return True
    except Exception as e:
        print(f"Batarya seviyesi kontrol edilirken hata oluştu: {e}")
        return False

def check_gps_signal():
    try:
        lat, lon, alt = get_drone_location()
        if lat is None or lon is None:
            print("GPS sinyali kayboldu, iniş yapılıyor...")
            land_drone()
            disarm_drone()
            return False
        return True
    except Exception as e:
        print(f"GPS sinyali kontrol edilirken hata oluştu: {e}")
        return False

def main():
    lat, lon, alt = get_drone_location()
    if lat is None or lon is None:
        print("GPS koordinatları alınamadı.")
        return

    if arm_drone():
        if takeoff_drone(10):  # Hedef 10 metre
            print("Uçuş başladı!")
            airspeed, groundspeed = get_drone_speed()
            if airspeed is not None and groundspeed is not None:
                print(f"Hava hızı: {airspeed} m/s, Yer hızı: {groundspeed} m/s")
            
            # Batarya seviyesi ve GPS sinyalini kontrol etmek için ayrı thread'ler başlat
            battery_thread = Thread(target=check_battery_level)
            gps_thread = Thread(target=check_gps_signal)
            battery_thread.start()
            gps_thread.start()

            # Örnek: 300 metre ileri uçuş için iki waypoint (örnek koordinatlar)
            if navigate_to_waypoint(41.0, 29.0, 10):
                if navigate_to_waypoint(40.0, 28.0, 10):
                    print("Uçuş sona erdi.")

                    if land_drone():
                        disarm_drone()
        else:
            print("Kalkış işlemi başarısız.")
    else:
        print("ARM işlemi başarısız.")

    # Thread'leri durdur
    battery_thread.join()
    gps_thread.join()

if __name__ == "__main__":
    main()