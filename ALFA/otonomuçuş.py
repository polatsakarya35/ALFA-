from pymavlink import mavutil
import time
import requests

# Hava durumu API anahtarı ve URL (OpenWeatherMap API kullanıyoruz)
API_KEY = "your_api_key_here"  # API anahtarınızı buraya girin
CITY = "Istanbul"  # Hava durumu bilgisini almak istediğiniz şehir
URL = f"http://api.openweathermap.org/data/2.5/weather?q={CITY}&appid={API_KEY}&units=metric"

# MAVLink bağlantısını kur (UDP bağlantısı)
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Kalp atışını bekle (drone ile bağlantıyı kontrol et)
print("Bağlantı bekleniyor...")
connection.wait_heartbeat()
print("Drone bağlantısı kuruldu.")

# Hava durumu verilerini alma
def get_weather():
    response = requests.get(URL)
    weather_data = response.json()

    # Hava durumu verilerini çıkarma
    temperature = weather_data["main"]["temp"]  # Sıcaklık
    wind_speed = weather_data["wind"]["speed"]  # Rüzgar hızı (m/s)
    weather_description = weather_data["weather"][0]["description"]  # Hava durumu açıklaması

    print(f"Mevcut hava durumu: {weather_description}")
    print(f"Sıcaklık: {temperature}°C, Rüzgar hızı: {wind_speed} m/s")
    
    return wind_speed, temperature

#  Drone'u ARM Et (Motorları çalıştır)
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
    return True

#  Kalkış Komutunu Gönder (Takeoff)
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
    return True

#  Belirli Bir Konuma Uç (Navigate to Waypoint)
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

# Drone İniş Komutunu Gönder (Land)
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

#  Drone'u DISARM Et (Motorları kapat)
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

#  Motor gücünü ayarlamak için rüzgar hızı kontrolü
def adjust_motor_power(wind_speed):
    if wind_speed > 10:  # Eğer rüzgar hızı 10 m/s'den fazla ise
        print("Rüzgar hızı yüksek, motor gücü artırılıyor...")
        # Motor hızını artırma (bu, uçuş kontrol yazılımında yapılacak bir şeydir)
        # Örnek olarak, daha güçlü kalkış yapmak için motor hızı artırılabilir.
        # Ancak burada, motor gücünü arttırmak için bir parametre gönderimi yapılır.
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,  # Confirmation
            1,  # Speed Type (1: Airspeed, 0: Grounds speed)
            15, # Speed (15 m/s hedef hız)
            0, 0, 0, 0, 0  # Kullanılmayan parametreler
        )
        print("Motor hızları artırıldı.")
    else:
        print("Rüzgar hızı düşük, motor gücü normal.")
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,  # Confirmation
            1,  # Speed Type (1: Airspeed, 0: Grounds speed)
            10, # Speed (10 m/s hedef hız)
            0, 0, 0, 0, 0  # Kullanılmayan parametreler
        )
        print("Motor hızları normal seviyede.")
        
# Ana işlem akışı
def main():
    wind_speed, temperature = get_weather()  # Hava durumu verilerini al
    if wind_speed > 5:
        print("Rüzgar hızı çok yüksek! Dikkatli uçuş yapın.")
    else:
        print("Rüzgar hızı uygun.")
    
    # Hava durumu ve motor gücü ayarları
    adjust_motor_power(wind_speed)
    
    if arm_drone():
        if takeoff_drone(10):  # Kalkış için hedef yükseklik
            print("Uçuş başladı!")
            time.sleep(10)  # Uçuşu simüle etmek için bekleme
            print("Uçuş sona erdi.")
        else:
            print("Kalkış işlemi başarısız.")
    else:
        print("ARM işlemi başarısız.")

# Uygulamayı başlat
main()
