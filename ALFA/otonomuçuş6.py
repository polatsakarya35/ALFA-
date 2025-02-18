from pymavlink import mavutil
import time
import requests
from threading import Thread

# Hava durumu API anahtarı ve URL (OpenWeatherMap API kullanıyoruz)
API_KEY = "your_api_key_here"  # API anahtarınızı buraya girin
URL = "http://api.openweathermap.org/data/2.5/weather"

# MAVLink bağlantısını kur (UDP bağlantısı)
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Kalp atışını bekle (drone ile bağlantıyı kontrol et)
print("Bağlantı bekleniyor...")
connection.wait_heartbeat()
print("Drone bağlantısı kuruldu.")

# Drone'un GPS koordinatlarını alma
def get_drone_location():
    try:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        return lat, lon
    except Exception as e:
        print(f"GPS koordinatları alınırken hata oluştu: {e}")
        return None, None

# Hava durumu verilerini alma
def get_weather(lat, lon):
    try:
        params = {
            'lat': lat,
            'lon': lon,
            'appid': API_KEY,
            'units': 'metric'
        }
        response = requests.get(URL, params=params)
        response.raise_for_status()
        weather_data = response.json()

        # Hava durumu verilerini çıkarma
        temperature = weather_data["main"]["temp"]  # Sıcaklık
        wind_speed = weather_data["wind"]["speed"]  # Rüzgar hızı (m/s)
        wind_deg = weather_data["wind"]["deg"]  # Rüzgar yönü (derece)
        weather_description = weather_data["weather"][0]["description"]  # Hava durumu açıklaması

        print(f"Mevcut hava durumu: {weather_description}")
        print(f"Sıcaklık: {temperature}°C, Rüzgar hızı: {wind_speed} m/s, Rüzgar yönü: {wind_deg}°")
        
        return wind_speed, wind_deg, temperature
    except requests.RequestException as e:
        print(f"Hava durumu verileri alınırken hata oluştu: {e}")
        return None, None, None

#  Drone'u ARM Et (Motorları çalıştır)
def arm_drone():
    try:
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
    except Exception as e:
        print(f"Drone ARM edilirken hata oluştu: {e}")
        return False

#  Kalkış Komutunu Gönder (Takeoff)
def takeoff_drone(target_altitude):
    try:
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
    except Exception as e:
        print(f"Kalkış komutu gönderilirken hata oluştu: {e}")
        return False

#  Belirli Bir Konuma Uç (Navigate to Waypoint)
def navigate_to_waypoint(lat, lon, altitude):
    try:
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
        return True  # Hedefe ulaşıldı
    except Exception as e:
        print(f"Konuma uçma komutu gönderilirken hata oluştu: {e}")
        return False

# Drone İniş Komutunu Gönder (Land)
def land_drone():
    try:
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
        return True  # İniş başarılı
    except Exception as e:
        print(f"İniş komutu gönderilirken hata oluştu: {e}")
        return False

#  Drone'u DISARM Et (Motorları kapat)
def disarm_drone():
    try:
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
        return True  # DISARM başarılı
    except Exception as e:
        print(f"Drone DISARM edilirken hata oluştu: {e}")
        return False

#  Motor gücünü ayarlamak için rüzgar hızı ve yönü kontrolü
def adjust_motor_power(wind_speed, wind_deg):
    base_speed = 10  # Temel motor hızı

    # Rüzgar hızına göre motor hızını ayarlama
    if wind_speed <= 5:
        speed_adjustment = 0
    elif wind_speed <= 10:
        speed_adjustment = 2
    elif wind_speed <= 15:
        speed_adjustment = 4
    elif wind_speed <= 20:
        speed_adjustment = 6
    elif wind_speed <= 25:
        speed_adjustment = 8
    else:
        speed_adjustment = 10

    # Rüzgar yönüne göre motor hızlarını ayarlama
    motor_speeds = {
        "motor_1": base_speed,
        "motor_2": base_speed,
        "motor_3": base_speed,
        "motor_4": base_speed
    }

    if 45 <= wind_deg < 135:  # Doğudan gelen rüzgar
        motor_speeds["motor_1"] += speed_adjustment
        motor_speeds["motor_2"] -= speed_adjustment
        motor_speeds["motor_3"] += speed_adjustment
        motor_speeds["motor_4"] -= speed_adjustment
    elif 135 <= wind_deg < 225:  # Güneyden gelen rüzgar
        motor_speeds["motor_1"] -= speed_adjustment
        motor_speeds["motor_2"] -= speed_adjustment
        motor_speeds["motor_3"] += speed_adjustment
        motor_speeds["motor_4"] += speed_adjustment
    elif 225 <= wind_deg < 315:  # Batıdan gelen rüzgar
        motor_speeds["motor_1"] -= speed_adjustment
        motor_speeds["motor_2"] += speed_adjustment
        motor_speeds["motor_3"] -= speed_adjustment
        motor_speeds["motor_4"] += speed_adjustment
    else:  # Kuzeyden gelen rüzgar
        motor_speeds["motor_1"] += speed_adjustment
        motor_speeds["motor_2"] += speed_adjustment
        motor_speeds["motor_3"] -= speed_adjustment
        motor_speeds["motor_4"] -= speed_adjustment

    print(f"Rüzgar hızı {wind_speed} m/s, motor hızları {motor_speeds} olarak ayarlanıyor...")
    for motor, speed in motor_speeds.items():
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,  # Confirmation
            1,  # Speed Type (1: Airspeed, 0: Grounds speed)
            speed,  # Speed
            0, 0, 0, 0, 0  # Kullanılmayan parametreler
        )
    print(f"Motor hızları {motor_speeds} olarak ayarlandı.")

# Rüzgar hızına göre motor gücünü sürekli ayarlamak için bir fonksiyon
def maintain_hover(stop_event):
    while not stop_event.is_set():
        lat, lon = get_drone_location()
        if lat is None or lon is None:
            continue
        wind_speed, wind_deg, _ = get_weather(lat, lon)
        if wind_speed is None or wind_deg is None:
            continue
        adjust_motor_power(wind_speed, wind_deg)
        time.sleep(5)  # Her 5 saniyede bir kontrol et

# Ana işlem akışı
def main():
    lat, lon = get_drone_location()
    if lat is None or lon is None:
        print("GPS koordinatları alınamadı.")
        return
    wind_speed, wind_deg, temperature = get_weather(lat, lon)  # Hava durumu verilerini al
    if wind_speed is None or wind_deg is None:
        print("Hava durumu verileri alınamadı.")
        return
    
    # Hava durumu ve motor gücü ayarları
    adjust_motor_power(wind_speed, wind_deg)
    
    if arm_drone():
        if takeoff_drone(10):  # Kalkış için hedef yükseklik
            print("Uçuş başladı!")
            
            # Rüzgar hızına göre motor gücünü sürekli ayarlamak için maintain_hover fonksiyonunu ayrı bir thread olarak çalıştır
            stop_event = Thread.Event()
            hover_thread = Thread(target=maintain_hover, args=(stop_event,))
            hover_thread.start()
            
            # 300 metre ileriye uç
            if navigate_to_waypoint(41.0, 29.0, 10):  # Örnek koordinatlar
                # Geri dön
                if navigate_to_waypoint(40.0, 28.0, 10):  # Başlangıç koordinatlarına geri dön
                    print("Uçuş sona erdi.")
                    land_drone()
                    disarm_drone()
            
            # maintain_hover thread'ini durdur
            stop_event.set()
            hover_thread.join()
        else:
            print("Kalkış işlemi başarısız.")
    else:
        print("ARM işlemi başarısız.")

# Uygulamayı başlat
main()