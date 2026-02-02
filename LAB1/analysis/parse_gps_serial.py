#!/usr/bin/env python3
import time
import argparse
import csv
from pathlib import Path
import serial


def nmea_checksum_ok(sentence: str) -> bool:
    s = sentence.strip()
    if not s.startswith("$") or "*" not in s:
        return False
    data, star = s[1:].split("*", 1)
    try:
        sent_ck = int(star[:2], 16)
    except ValueError:
        return False
    calc = 0
    for ch in data:
        calc ^= ord(ch)
    return calc == sent_ck


def dm_to_deg(dm: str, hemi: str) -> float:
    v = float(dm)
    deg = int(v // 100)
    minutes = v - 100 * deg
    dec = deg + minutes / 60.0
    if hemi in ("S", "W"):
        dec = -dec
    return dec


def parse_gga(line: str):
    fields = line.strip().split(",")
    if len(fields) < 10:
        return None
    if fields[0] not in ("$GPGGA", "$GNGGA"):
        return None

    lat_dm = fields[2]
    lat_hemi = fields[3]
    lon_dm = fields[4]
    lon_hemi = fields[5]

    if not lat_dm or not lon_dm or not lat_hemi or not lon_hemi:
        return None

    fix_quality = int(fields[6]) if fields[6] else 0
    hdop = float(fields[8]) if fields[8] else float("nan")
    alt_m = float(fields[9]) if fields[9] else float("nan")

    lat = dm_to_deg(lat_dm, lat_hemi)
    lon = dm_to_deg(lon_dm, lon_hemi)
    return lat, lon, alt_m, fix_quality, hdop


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=4800)
    ap.add_argument("--csv", default=str(Path.home() / "EECE5554/LAB1/analysis/gps_serial_log.csv"))
    args = ap.parse_args()

    print(f"Opening {args.port} @ {args.baud} baud...")

    out_path = Path(args.csv)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with open(out_path, "w", newline="", buffering=1) as csv_f:
        writer = csv.writer(csv_f)
        writer.writerow(["unix_time", "latitude_deg", "longitude_deg", "altitude_m", "fix_quality", "hdop"])

        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )

        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("ascii", errors="ignore").strip()
                if not line.startswith("$"):
                    continue

                if "*" in line and not nmea_checksum_ok(line):
                    continue

                gga = parse_gga(line)
                if gga is None:
                    continue

                lat, lon, alt_m, fix_q, hdop = gga
                t = time.time()

                print(f"LAT={lat:.8f}  LON={lon:.8f}  ALT={alt_m:.2f} m  FIX={fix_q}  HDOP={hdop:.2f}")
                writer.writerow([t, lat, lon, alt_m, fix_q, hdop])

        except KeyboardInterrupt:
            print("\nStopping.")
        finally:
            ser.close()


if __name__ == "__main__":
    main()
