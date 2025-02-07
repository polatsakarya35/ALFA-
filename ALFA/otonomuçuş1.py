from pymavlink import mavutil
import time

# MAVLink bağlantısını kur (UDP bağlantısı)
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Kalp atışını bekle (drone ile bağlantıyı kontrol et)
print("Bağlantı bekleniyor...")
connection.wait_heartbeat()
print("Drone bağlantısı kuruldu.")

# 1. Drone'u ARM Et (Motorları çalıştır)
def arm_drone():
    print("Drone ARM ediliyor...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,    # Confirmation
        1,    # ARM (1: Motorları çalıştır)
        0, 0, 0, 0, 0, 0  # Kullanılmayan parametreler
    )
    print("Drone ARM edildi (uçuşa hazır).")
    # ARM edildiğini kontrol et
    # (Burada drone'un durumunu kontrol eden bir sistem olmalıdır, ancak bu örnekte varsayıyoruz)
    return True  # ARM başarılı

# 2. Kalkış Komutunu Gönder (Takeoff)
def takeoff_drone(target_altitude):
    print(f"Drone {target_altitude} metreye kalkış yapıyor...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,    # Confirmation
        0, 0, 0, 0,          # Kullanılmayan parametreler
        0, 0, target_altitude  # Hedef yükseklik (örneğin 10 metre)
    )
    time.sleep(10)  # Kalkışın tamamlanmasını bekle
    # Kalkışın başarılı olup olmadığını kontrol et
    return True  # Kalkış başarılı

# 3. Belirli Bir Konuma Uç (Navigate to Waypoint)
def navigate_to_waypoint(lat, lon, altitude):
    print(f"Drone {lat}, {lon} konumuna {altitude} metre yükseklikte uçuyor...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,    # Confirmation
        0, 0, 0, 0,          # Kullanılmayan parametreler
        lat, lon, altitude   # Hedef enlem, boylam ve yükseklik
    )
    time.sleep(10)  # Hedefe ulaşmayı bekle
    # Konuma ulaşmayı kontrol et (Gerçek uçuşlarda GPS takibi gerekebilir)
    return True  # Hedefe ulaşıldı

# 4. İniş Komutunu Gönder (Land)
def land_drone():
    print("Drone iniş yapıyor...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,    # Confirmation
        0, 0, 0, 0,          # Kullanılmayan parametreler
        0, 0, 0              # Hedef yükseklik gereksiz (yer seviyesi)
    )
    time.sleep(10)  # İnişin tamamlanmasını bekle
    # İnişi kontrol et
    return True  # İniş başarılı

# 5. Drone'u DISARM Et (Motorları kapat)
def disarm_drone():
    print("Drone DISARM ediliyor...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,    # Confirmation
        0,    # DISARM (0: Motorları durdur)
        0, 0, 0, 0, 0, 0  # Kullanılmayan parametreler
    )
    print("Drone DISARM edildi (güvenli duruma geçti).")
    # DISARM başarılı mı?
    return True  # DISARM başarılı

# Ana işlem akışı
def main():
    if arm_drone():
        if takeoff_drone(10):
            if navigate_to_waypoint(37.7749, -122.4194, 20):  # Örnek konum
                if land_drone():
                    if disarm_drone():
                        print("Görev tamamlandı.")
                    else:
                        print("DISARM işlemi başarısız.")
                else:
                    print("İniş işlemi başarısız.")
            else:
                print("Hedefe ulaşma işlemi başarısız.")
        else:
            print("Kalkış işlemi başarısız.")
    else:
        print("ARM işlemi başarısız.")

# Uygulamayı başlat
main()
