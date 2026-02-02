#!/usr/bin/env python3
import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
import utm

# pip3 install --user rosbags

def load_known_position(path: str):
    """
    File format:
      lat: 42.12345
      lon: -71.12345
    """
    with open(path, "r") as f:
        lines = [ln.strip() for ln in f.readlines() if ln.strip()]
    lat = float(lines[0].split(":")[1].strip())
    lon = float(lines[1].split(":")[1].strip())
    return lat, lon

def read_ros2_bag_gps(bag_dir: str, topic: str = "/gps"):
    \"\"
    Reads ROS2 bag folder (metadata.yaml + *.db3) and extracts:
    utm_easting, utm_northing, altitude, hdop, and header.stamp time.

    Uses ROS2-native rosbag2_py + rclpy.serialization (no rosbags dependency).
    \"\"
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    topics_and_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics_and_types}

    if topic not in type_map:
        topics = sorted(type_map.keys())
        raise RuntimeError(f"Topic {topic} not found. Bag topics: {topics}")

    msg_type = get_message(type_map[topic])

    e_list, n_list, alt_list, hdop_list, t_list = [], [], [], [], []

    while reader.has_next():
        tpc, rawdata, _t = reader.read_next()
        if tpc != topic:
            continue
        msg = deserialize_message(rawdata, msg_type)

        e_list.append(float(msg.utm_easting))
        n_list.append(float(msg.utm_northing))
        alt_list.append(float(msg.altitude))
        hdop_list.append(float(msg.hdop))

        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        t_list.append(t)

    return {
        "e": np.array(e_list),
        "n": np.array(n_list),
        "alt": np.array(alt_list),
        "hdop": np.array(hdop_list),
        "t": np.array(t_list),
    }

def save_stationary_plots(out_dir, label, e, n, alt, t, e_known, n_known):
    plt.figure()
    plt.scatter(e - e[0], n - n[0], s=6)
    plt.xlabel("Easting (m) (relative)")
    plt.ylabel("Northing (m) (relative)")
    plt.title(f"{label}: Northing vs Easting (relative)")
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, f"{label}_scatter.png"))
    plt.close()

    err = np.sqrt((e - e_known)**2 + (n - n_known)**2)
    plt.figure()
    plt.hist(err, bins=30)
    plt.xlabel("Position error (m)")
    plt.ylabel("Count")
    plt.title(f"{label}: Position Error Histogram")
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, f"{label}_hist.png"))
    plt.close()

    tt = t - t[0]
    plt.figure()
    plt.plot(tt, alt)
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.title(f"{label}: Altitude vs Time")
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, f"{label}_alt.png"))
    plt.close()

    return err

def save_walking_plots(out_dir, label, e, n, alt, t):
    plt.figure()
    plt.scatter(e, n, s=6)
    plt.xlabel("Easting (m)")
    plt.ylabel("Northing (m)")
    plt.title(f"{label}: Northing vs Easting")
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, f"{label}_path.png"))
    plt.close()

    a, b = np.polyfit(e, n, 1)  # n = a*e + b
    dist = np.abs(a*e - n + b) / np.sqrt(a*a + 1.0)

    plt.figure()
    plt.hist(dist, bins=30)
    plt.xlabel("Perpendicular error to best-fit line (m)")
    plt.ylabel("Count")
    plt.title(f"{label}: Line Fit Error Histogram")
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, f"{label}_linefit_hist.png"))
    plt.close()

    tt = t - t[0]
    plt.figure()
    plt.plot(tt, alt)
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.title(f"{label}: Altitude vs Time")
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, f"{label}_alt.png"))
    plt.close()

    return a, b, dist

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_dir", required=True)
    parser.add_argument("--out_dir", required=True)
    parser.add_argument("--topic", default="/gps")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    open_lat, open_lon = load_known_position(os.path.join(args.data_dir, "open_sky_known.txt"))
    occ_lat, occ_lon   = load_known_position(os.path.join(args.data_dir, "occluded_known.txt"))
    open_e, open_n, _, _ = utm.from_latlon(open_lat, open_lon)
    occ_e,  occ_n,  _, _ = utm.from_latlon(occ_lat, occ_lon)

    open_bag = os.path.join(args.data_dir, "open_sky")
    occ_bag  = os.path.join(args.data_dir, "occluded")
    walk_bag = os.path.join(args.data_dir, "walking")

    for p in [open_bag, occ_bag, walk_bag]:
        if not os.path.isdir(p):
            raise RuntimeError(f"Missing bag folder: {p} (must contain metadata.yaml + *.db3)")

    open_data = read_ros2_bag_gps(open_bag, args.topic)
    occ_data  = read_ros2_bag_gps(occ_bag, args.topic)
    walk_data = read_ros2_bag_gps(walk_bag, args.topic)

    err_open = save_stationary_plots(args.out_dir, "open_sky",
                                     open_data["e"], open_data["n"], open_data["alt"], open_data["t"],
                                     open_e, open_n)
    err_occ  = save_stationary_plots(args.out_dir, "occluded",
                                     occ_data["e"], occ_data["n"], occ_data["alt"], occ_data["t"],
                                     occ_e, occ_n)

    a, b, dist = save_walking_plots(args.out_dir, "walking",
                                    walk_data["e"], walk_data["n"], walk_data["alt"], walk_data["t"])

    with open(os.path.join(args.out_dir, "summary.txt"), "w") as f:
        f.write(f"Open sky mean error (m): {np.mean(err_open):.3f}\n")
        f.write(f"Open sky std  error (m): {np.std(err_open):.3f}\n\n")
        f.write(f"Occluded mean error (m): {np.mean(err_occ):.3f}\n")
        f.write(f"Occluded std  error (m): {np.std(err_occ):.3f}\n\n")
        f.write(f"Walking best-fit line: northing = {a:.6f}*easting + {b:.6f}\n")
        f.write(f"Walking mean perp error to line (m): {np.mean(dist):.3f}\n")
        f.write(f"Walking std  perp error to line (m): {np.std(dist):.3f}\n")

    print("DONE. Outputs saved to:", args.out_dir)

if __name__ == "__main__":
    main()
def main():
    parser = argparse.ArgumentParser(description="EECE 5554 Lab 1 GPS Analysis")
    parser.add_argument("--data_dir", required=True, help="Directory with rosbag folders")
    parser.add_argument("--out_dir", required=True, help="Output directory for plots")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    # Known positions
    open_lat, open_lon = load_known_position(os.path.join(args.data_dir, "open_sky_known.txt"))
    occ_lat, occ_lon   = load_known_position(os.path.join(args.data_dir, "occluded_known.txt"))

    open_e0, open_n0, _, _ = utm.from_latlon(open_lat, open_lon)
    occ_e0, occ_n0, _, _   = utm.from_latlon(occ_lat, occ_lon)

    datasets = {
        "open_sky": ("open_sky", open_e0, open_n0),
        "occluded": ("occluded", occ_e0, occ_n0),
        "walking":  ("walking", None, None),
    }

    for name, (bag, e0, n0) in datasets.items():
        data = read_ros2_bag_gps(os.path.join(args.data_dir, bag))

        e = data["e"]
        n = data["n"]

        if e0 is not None:
            e = e - e0
            n = n - n0

        # Scatter plot
        plt.figure()
        plt.scatter(e, n, s=5)
        plt.xlabel("Easting (m)")
        plt.ylabel("Northing (m)")
        plt.title(f"{name.replace('_',' ').title()} Northing vs Easting")
        plt.axis("equal")
        plt.grid(True)
        plt.savefig(os.path.join(args.out_dir, f"{name}_scatter.png"))
        plt.close()

        # Altitude vs time
        plt.figure()
        plt.plot(data["t"] - data["t"][0], data["alt"])
        plt.xlabel("Time (s)")
        plt.ylabel("Altitude (m)")
        plt.title(f"{name.replace('_',' ').title()} Altitude vs Time")
        plt.grid(True)
        plt.savefig(os.path.join(args.out_dir, f"{name}_altitude.png"))
        plt.close()

    print(f"Analysis complete. Outputs saved to {args.out_dir}")


if __name__ == "__main__":
    main ()
