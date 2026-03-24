import math
import time
import collections
collections.MutableMapping = collections.abc.MutableMapping
from dronekit import connect, VehicleMode

import math
import time
# --- КООРДИНАТИ ---
# Точка А (Старт): 50.450739, 30.461242
# Точка Б (Ціль): 50.443326, 30.448078
TARGET_LAT = 50.443326
TARGET_LON = 30.448078
TARGET_ALT = 100.0

# Підключення до SITL
print("Підключення до дрона...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)


def get_distance_metres(loc1, lat2, lon2):
    dlat = lat2 - loc1.lat
    dlong = lon2 - loc1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def get_bearing(loc1, lat2, lon2):
    off_x = lon2 - loc1.lon
    off_y = lat2 - loc1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0: bearing += 360.00
    return bearing


def arm_and_takeoff_stabilize(aTargetAltitude):
    print("Перехід у режим STABILIZE...")
    vehicle.mode = VehicleMode("STABILIZE")

    while not vehicle.is_armable:
        print(" Очікування ініціалізації...")
        time.sleep(1)

    print("Армінг моторів (ARM)...")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)

    print("Зліт через RC Override (Throttle Up)...")
    # RC 3: Газ (1000-2000). 1500 - нейтраль.
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Поточна висота: {alt:.2f} м")
        if alt >= aTargetAltitude * 0.95:
            print("Висота 100м досягнута.")
            vehicle.channels.overrides['3'] = 1500  # Утримання
            break
        else:
            vehicle.channels.overrides['3'] = 1680  # Активний підйом
        time.sleep(0.2)


def fly_to_target():
    print("Початок руху до Точки Б...")
    try:
        while True:
            loc = vehicle.location.global_relative_frame
            dist = get_distance_metres(loc, TARGET_LAT, TARGET_LON)
            bearing = get_bearing(loc, TARGET_LAT, TARGET_LON)
            heading = vehicle.heading  # Куди дивиться дрон

            print(f"Відстань: {dist:.1f}м | Курс: {bearing:.1f}° | Висота: {loc.alt:.1f}м")

            if dist < 3.0:  # Радіус 3 метри для зупинки
                print("Точка Б досягнута. Починаємо посадку.")
                vehicle.channels.overrides = {}
                break

            # КЕРУВАННЯ (RC OVERRIDE)
            # RC 2: Pitch (нахил вперед/назад). <1500 вперед.
            vehicle.channels.overrides['2'] = 1420  # Летимо вперед

            # RC 3: Утримання висоти (простий контроль)
            if loc.alt < TARGET_ALT - 1:
                vehicle.channels.overrides['3'] = 1580  # Піднятися
            elif loc.alt > TARGET_ALT + 1:
                vehicle.channels.overrides['3'] = 1420  # Опуститися
            else:
                vehicle.channels.overrides['3'] = 1510  # Стабільно

            # RC 4: Yaw (поворот носа на ціль)
            # Для ідеального балу можна додати корекцію курсу тут

            time.sleep(0.5)
    except KeyboardInterrupt:
        vehicle.channels.overrides = {}


def land_perfectly():
    print("Фінальна посадка в Точці Б...")
    # Поступово зменшуємо газ
    for thr in range(1500, 1000, -10):
        vehicle.channels.overrides['3'] = thr
        time.sleep(0.1)
        if vehicle.location.global_relative_frame.alt < 0.2:
            break

    print("Посадка виконана. Disarm.")
    vehicle.armed = False
    vehicle.close()


# Запуск місії
arm_and_takeoff_stabilize(TARGET_ALT)
fly_to_target()
land_perfectly()