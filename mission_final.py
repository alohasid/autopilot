import collections
import math
import time

try:
    collections.MutableMapping = collections.abc.MutableMapping
    collections.Iterable = collections.abc.Iterable
except AttributeError:
    pass

from dronekit import connect, VehicleMode

TARGET_LAT = 50.443326
TARGET_LON = 30.448078
TARGET_ALT = 100.0

vehicle = connect('127.0.0.1:14550', wait_ready=True)


def get_distance_metres(loc, target_lat, target_lon):
    dlat = target_lat - loc.lat
    dlong = target_lon - loc.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 111319.5


def arm_and_takeoff(target_alt):
    vehicle.mode = VehicleMode("STABILIZE")
    while not vehicle.is_armable: time.sleep(1)
    vehicle.armed = True
    while not vehicle.armed: time.sleep(0.5)
    while vehicle.location.global_relative_frame.alt < target_alt * 0.95:
        vehicle.channels.overrides['3'] = 1720
        time.sleep(0.1)
    vehicle.channels.overrides['3'] = 1515


def fly_to_target_pid():
    kp = 220000.0
    ki = 2500.0
    kd = 900000.0

    i_lat, i_lon = 0.0, 0.0
    last_err_lat, last_err_lon = 0.0, 0.0
    last_time = time.time()

    print(">>> Політ до Точки Б: Агресивний PID активовано")

    while True:
        curr = vehicle.location.global_relative_frame
        dist = get_distance_metres(curr, TARGET_LAT, TARGET_LON)
        now = time.time()
        dt = now - last_time
        if dt <= 0: dt = 0.02
        last_time = now

        if dist < 0.6:
            print(f"DONE! Target B reached. Final Dist: {dist:.2f}m")
            vehicle.channels.overrides.update({'1': 1500, '2': 1500})
            break

        err_lat = TARGET_LAT - curr.lat
        err_lon = TARGET_LON - curr.lon

        i_lat += err_lat * dt
        i_lon += err_lon * dt
        i_lat = max(-1.2, min(1.2, i_lat))
        i_lon = max(-1.2, min(1.2, i_lon))

        d_lat = (err_lat - last_err_lat) / dt
        d_lon = (err_lon - last_err_lon) / dt
        last_err_lat, last_err_lon = err_lat, err_lon

        pitch_out = (err_lat * kp) + (i_lat * ki) + (d_lat * kd)
        roll_out = (err_lon * kp) + (i_lon * ki) + (d_lon * kd)

        vehicle.channels.overrides['2'] = int(max(1100, min(1900, 1500 - pitch_out)))
        vehicle.channels.overrides['1'] = int(max(1100, min(1900, 1500 + roll_out)))

        alt_err = TARGET_ALT - curr.alt
        vehicle.channels.overrides['3'] = 1500 + int(alt_err * 70)

        print(f"Dist: {dist:.2f}m | RC_P: {vehicle.channels.overrides['2']} | Alt: {curr.alt:.1f}m")
        time.sleep(0.02)


def precision_landing():
    print(">>> Фінальна посадка точно в нуль (п. 5)...")
    kp_l = 400000.0
    ki_l = 3500.0
    i_lat, i_lon = 0.0, 0.0
    last_time = time.time()

    while True:
        curr = vehicle.location.global_relative_frame
        if curr.alt < 0.1:
            vehicle.channels.overrides = {'3': 1000}
            break

        now = time.time()
        dt = now - last_time
        if dt <= 0: dt = 0.05
        last_time = now

        err_lat = TARGET_LAT - curr.lat
        err_lon = TARGET_LON - curr.lon
        i_lat += err_lat * dt
        i_lon += err_lon * dt

        vehicle.channels.overrides['2'] = int(max(1200, min(1800, 1500 - (err_lat * kp_l + i_lat * ki_l))))
        vehicle.channels.overrides['1'] = int(max(1200, min(1800, 1500 + (err_lon * kp_l + i_lon * ki_l))))

        vehicle.channels.overrides['3'] = 1415
        time.sleep(0.05)

    vehicle.armed = False
    vehicle.close()


try:
    arm_and_takeoff(TARGET_ALT)
    fly_to_target_pid()
    precision_landing()
except KeyboardInterrupt:
    vehicle.channels.overrides = {}
    vehicle.close()