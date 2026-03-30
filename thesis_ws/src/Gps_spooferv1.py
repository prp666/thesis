from __future__ import annotations

import math
import time

from gz.transport13 import Node
from gz.msgs10.navsat_pb2 import NavSat


GPS_TOPIC = "/world/default/model/x500_0/link/base_link/sensor/navsat_sensor/navsat"


def meters_to_latlon_delta(lat_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    delta_lat_deg = north_m / 111111.0
    cos_lat = math.cos(math.radians(lat_deg))
    if abs(cos_lat) < 1e-8:
        raise ValueError("Latitude too close to pole for longitude conversion.")
    delta_lon_deg = east_m / (111111.0 * cos_lat)
    return delta_lat_deg, delta_lon_deg


def main() -> None:
    node = Node()

    # 在同一个 GPS topic 上再发布一份 spoofed NavSat
    pub = node.advertise(GPS_TOPIC, NavSat)

    # 你刚刚从 Gazebo topic 里读到的真实 baseline
    real_lat = 47.397971057728981
    real_lon = 8.5461637398001447
    real_alt = 0.22699981648474932

    # 先做固定偏移：北偏 5 米
    north_offset_m = 5.0
    east_offset_m = 0.0
    up_offset_m = 0.0

    dlat, dlon = meters_to_latlon_delta(real_lat, north_offset_m, east_offset_m)

    fake_lat = real_lat + dlat
    fake_lon = real_lon + dlon
    fake_alt = real_alt + up_offset_m

    print("[INFO] Gazebo GPS spoofer started")
    print(f"[INFO] Topic: {GPS_TOPIC}")
    print(f"[INFO] Real GPS : lat={real_lat}, lon={real_lon}, alt={real_alt}")
    print(f"[INFO] Fake GPS : lat={fake_lat}, lon={fake_lon}, alt={fake_alt}")
    print("[INFO] Publishing at 50 Hz")
    print("[INFO] Press Ctrl+C to stop.\n")

    seq = 0
    try:
        while True:
            msg = NavSat()
            msg.latitude_deg = fake_lat
            msg.longitude_deg = fake_lon
            msg.altitude = fake_alt
            msg.velocity_east = 0.0
            msg.velocity_north = 0.0
            msg.velocity_up = 0.0
            msg.frame_id = "base_link"

            msg.header.data.add(key="seq", value=str(seq))
            seq += 1

            ok = pub.publish(msg)
            if not ok:
                print("[WARN] publish() returned False")

            time.sleep(1.0 / 50.0)   # 50 Hz，高于原始 30 Hz

    except KeyboardInterrupt:
        print("\n[INFO] Stopped.")


if __name__ == "__main__":
    main()