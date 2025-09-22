#!/usr/bin/env python3
import argparse
import os
from .dump import dump_rosbag_to_csv

def main():
    parser = argparse.ArgumentParser(
        description='Dump a ROS 2 bag directory to per-topic CSV files.'
    )
    parser.add_argument('bag', help='Path to ROS 2 bag directory (the folder containing metadata.yaml)')
    parser.add_argument('-o', '--output', default='csv_output', help='Output directory')
    parser.add_argument('--stamp', choices=['ns', 'sec', 'ros'], default='ns',
                        help="Timestamp format: 'ns' (raw int), 'sec' (float seconds), or 'ros' (sec/nanosec columns)")
    parser.add_argument('--include', action='append', default=[],
                        help="Regex for topics to include (can be given multiple times)")
    parser.add_argument('--exclude', action='append', default=[r'^/clock$'],
                        help="Regex for topics to exclude (can be given multiple times). Defaults to '^/clock$'.")

    args = parser.parse_args()
    bag = os.path.abspath(os.path.expanduser(args.bag))
    out = os.path.abspath(os.path.expanduser(args.output))

    dump_rosbag_to_csv(
        bag_path=bag,
        output_dir=out,
        stamp_mode=args.stamp,
        include_patterns=args.include,
        exclude_patterns=args.exclude,
    )
    print(f"[rosbag_to_csv] Wrote CSVs to: {out}")
