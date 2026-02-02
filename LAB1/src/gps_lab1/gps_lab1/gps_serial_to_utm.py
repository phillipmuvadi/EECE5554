#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from gps_interfaces.msg import GpsUtm

import serial
import pynmea2
import utm


class GpsSerialToUtm(Node):
    def __init__(self):
        super().__init__('gps_serial_to_utm')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 4800)
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('topic', '/gps_utm')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value

        self.pub = self.create_publisher(GpsUtm, self.topic, 10)

        # Open serial
        self.get_logger().info(f"Opening serial port {self.port} @ {self.baud}...")
        self.ser = serial.Serial(self.port, self.baud, timeout=1.0)

        # Timer to poll serial
        self.timer = self.create_timer(0.02, self.poll_serial)  # 50 Hz poll loop

        self.get_logger().info(f"Publishing {self.topic} (type gps_interfaces/msg/GpsUtm)")

    def poll_serial(self):
        try:
            line = self.ser.readline()
            if not line:
                return

            try:
                s = line.decode('ascii', errors='ignore').strip()
            except Exception:
                return

            # Ignore junk/partial lines
            if not s.startswith('$'):
                return

            # Parse NMEA
            try:
                msg = pynmea2.parse(s, check=True)
            except Exception:
                return

            # Prefer GGA for lat/lon/alt
            if msg.sentence_type != 'GGA':
                return

            if msg.latitude is None or msg.longitude is None:
                return

            lat = float(msg.latitude)
            lon = float(msg.longitude)

            # Altitude may be empty sometimes
            alt = 0.0
            try:
                if msg.altitude is not None and msg.altitude != '':
                    alt = float(msg.altitude)
            except Exception:
                alt = 0.0

            easting, northing, zone_num, zone_letter = utm.from_latlon(lat, lon)
            zone = f"{zone_num}{zone_letter}"

            out = GpsUtm()
            out.header = Header()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.frame_id

            out.latitude = lat
            out.longitude = lon
            out.altitude = alt
            out.easting = float(easting)
            out.northing = float(northing)
            out.zone = zone

            self.pub.publish(out)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
        except Exception as e:
            self.get_logger().warn(f"Unexpected error: {e}")


def main():
    rclpy.init()
    node = GpsSerialToUtm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
 
