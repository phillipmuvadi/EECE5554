#!/usr/bin/env python3
import argparse
import serial
import utm

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from gps_interfaces.msg import GpsMsg


def nmea_to_decimal(coord_str: str, hemi: str) -> float:
    """
    Convert NMEA coordinate to signed decimal degrees.
    lat: ddmm.mmmm (N/S)
    lon: dddmm.mmmm (E/W)
    """
    if not coord_str or not hemi:
        raise ValueError("Missing coordinate field")

    raw = float(coord_str)
    deg = int(raw // 100)
    minutes = raw - deg * 100
    dec = deg + minutes / 60.0

    if hemi in ("S", "W"):
        dec *= -1.0
    return dec


def parse_gpgga(sentence: str):
    if not sentence.startswith("$GPGGA"):
        return None

    parts = sentence.strip().split(",")
    if len(parts) < 15:
        return None

    utc = parts[1]          # hhmmss.ss
    lat_s = parts[2]
    lat_h = parts[3]
    lon_s = parts[4]
    lon_h = parts[5]
    fix = parts[6]          # 0 = invalid
    hdop_s = parts[8]
    alt_s = parts[9]

    if fix == "0":
        return None

    lat = nmea_to_decimal(lat_s, lat_h)
    lon = nmea_to_decimal(lon_s, lon_h)
    alt = float(alt_s) if alt_s else float("nan")
    hdop = float(hdop_s) if hdop_s else float("nan")

    e, n, zone, letter = utm.from_latlon(lat, lon)
    return utc, lat, lon, alt, hdop, float(e), float(n), int(zone), str(letter)


def utc_to_stamp(utc: str):
    """
    Convert hhmmss.ss to (seconds_since_midnight, nanoseconds).
    GPGGA has no date; this satisfies the lab requirement to use GPS time (not system time).
    """
    if not utc or len(utc) < 6:
        return 0, 0

    hh = int(utc[0:2])
    mm = int(utc[2:4])
    ss = float(utc[4:])

    sec_int = int(ss)
    frac = ss - sec_int

    sec = hh * 3600 + mm * 60 + sec_int
    nsec = int(frac * 1e9)
    return sec, nsec


class GPSDriver(Node):
    def __init__(self, port: str, baud: int):
        super().__init__("gps_driver")
        self.pub = self.create_publisher(GpsMsg, "/gps", 10)

        self.get_logger().info(f"Opening {port} at {baud} baud")
        self.ser = serial.Serial(port, baudrate=baud, timeout=1.0)

        self.timer = self.create_timer(0.2, self.loop)  # ~5 Hz

    def loop(self):
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
        except Exception as e:
            self.get_logger().warn(f"Serial error: {e}")
            return

        if not line:
            return

        parsed = parse_gpgga(line)
        if parsed is None:
            return

        utc, lat, lon, alt, hdop, e, n, zone, letter = parsed

        m = GpsMsg()

        h = Header()
        h.frame_id = "GPS1_Frame"
        sec, nsec = utc_to_stamp(utc)
        h.stamp.sec = sec
        h.stamp.nanosec = nsec
        m.header = h

        m.latitude = lat
        m.longitude = lon
        m.altitude = alt
        m.hdop = hdop
        m.utm_easting = e
        m.utm_northing = n
        m.utc = utc
        m.zone = zone
        m.letter = letter

        self.pub.publish(m)

    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyUSB2 or /dev/pts/6)")
    parser.add_argument("--baud", type=int, default=4800, help="Baud rate (default 4800)")
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = GPSDriver(args.port, args.baud)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
