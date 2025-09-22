import os
import re
import csv
import yaml
from typing import Dict, Iterable, Optional, Tuple, List

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from .flatten import flatten_message

def _read_metadata(bag_path: str) -> Tuple[str, str]:
    """
    Returns (storage_id, serialization_format) by reading metadata.yaml.
    Falls back to ('sqlite3', 'cdr') if not found.
    """
    meta = os.path.join(bag_path, "metadata.yaml")
    storage_id, ser = "sqlite3", "cdr"
    if os.path.exists(meta):
        with open(meta, "r") as f:
            d = yaml.safe_load(f)
        try:
            storage_id = d["rosbag2_bagfile_information"]["storage_identifier"]
        except Exception:
            pass
        try:
            ser = d["rosbag2_bagfile_information"]["message_definition_files"]["serialization_format"]
        except Exception:
            # older metadata places it in 'serialization_format'
            ser = d.get("rosbag2_bagfile_information", {}).get("serialization_format", ser)
    return storage_id, ser

def _match(topic: str, includes: Iterable[re.Pattern], excludes: Iterable[re.Pattern]) -> bool:
    if includes and not any(p.search(topic) for p in includes):
        return False
    if excludes and any(p.search(topic) for p in excludes):
        return False
    return True

def dump_rosbag_to_csv(
    bag_path: str,
    output_dir: str,
    stamp_mode: str = "ns",               # 'ns' (raw), 'sec' (float seconds), or 'ros' (sec/nanosec cols)
    include_patterns: Optional[Iterable[str]] = None,
    exclude_patterns: Optional[Iterable[str]] = (r"^/clock$",),  # exclude /clock by default
) -> None:
    os.makedirs(output_dir, exist_ok=True)

    storage_id, ser = _read_metadata(bag_path)
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format=ser,
        output_serialization_format=ser
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map: Dict[str, str] = {t.name: t.type for t in topic_types}

    includes = [re.compile(p) for p in (include_patterns or [])]
    excludes = [re.compile(p) for p in (exclude_patterns or [])]

    csv_files: Dict[str, csv.DictWriter] = {}
    file_handles: Dict[str, any] = {}
    fieldnames_map: Dict[str, list] = {}

    try:
        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic not in type_map:
                # can happen if topic was closed/renamed—skip
                continue
            if not _match(topic, includes, excludes):
                continue

            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)

            row = flatten_message(msg)

            # Attach timestamp(s)
            if stamp_mode == "ns":
                row["timestamp_ns"] = int(t)
            elif stamp_mode == "sec":
                row["timestamp_sec"] = float(t) * 1e-9
            elif stamp_mode == "ros":
                row["stamp.sec"] = int(t // 1_000_000_000)
                row["stamp.nanosec"] = int(t % 1_000_000_000)
            else:
                row["timestamp_ns"] = int(t)

            # Lazy open per-topic CSV
            if topic not in csv_files:
                filename = topic.strip("/").replace("/", "_") + ".csv"
                filepath = os.path.join(output_dir, filename)
                f = open(filepath, "w", newline="")
                file_handles[topic] = f
                writer = csv.DictWriter(f, fieldnames=list(row.keys()))
                csv_files[topic] = writer
                fieldnames_map[topic] = list(row.keys())
                writer.writeheader()

            # If new fields appear later (rare), expand columns
            writer = csv_files[topic]
            if set(row.keys()) - set(fieldnames_map[topic]):
                # Expand header: close and rewrite with new header including old rows is heavy.
                # Simple approach: add missing keys as empty for this row only.
                for k in set(row.keys()) - set(fieldnames_map[topic]):
                    fieldnames_map[topic].append(k)
                writer.fieldnames = fieldnames_map[topic]

            # Ensure all keys present
            for k in fieldnames_map[topic]:
                if k not in row:
                    row[k] = ""
            writer.writerow(row)
    finally:
        for f in file_handles.values():
            f.close()


def dump_rosbag_to_single_csv(
    bag_path: str,
    output_dir: str,
    output_filename: Optional[str] = None,
    stamp_mode: str = "ns",
    include_patterns: Optional[Iterable[str]] = None,
    exclude_patterns: Optional[Iterable[str]] = (r"^/clock$",),
    drop_time_like_fields: bool = True,
) -> str:
    """
    Write a single CSV that merges selected topics and synchronizes rows by timestamp.

    Strategy:
    - First pass: read all matching messages, collect per-topic flattened rows keyed by timestamp (nanoseconds),
      and build the global set of CSV columns.
    - Second step: iterate over the sorted union of timestamps; for each timestamp, write a row containing
      the latest known value (forward-fill) for each field from each topic. Timestamp columns are written
      according to stamp_mode.

    If output_filename is None, it will default to '<bag_dir_name>.csv'.

    Returns: absolute path to the written merged CSV file.
    """
    os.makedirs(output_dir, exist_ok=True)

    storage_id, ser = _read_metadata(bag_path)
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format=ser,
        output_serialization_format=ser
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map: Dict[str, str] = {t.name: t.type for t in topic_types}

    includes = [re.compile(p) for p in (include_patterns or [])]
    excludes = [re.compile(p) for p in (exclude_patterns or [])]

    # First pass: collect per-topic data and global columns
    per_topic_data: Dict[str, Dict[int, Dict[str, any]]] = {}
    global_fields: List[str] = []

    def _is_time_field_name(name: str) -> bool:
        # Flattened time-like fields become '<base>.sec' and '<base>.nanosec'
        return name.endswith('.sec') or name.endswith('.nanosec')

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in type_map:
            continue
        if not _match(topic, includes, excludes):
            continue

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        row = flatten_message(msg)

        # Optionally drop time-like fields from flattened messages (e.g., header.stamp)
        if drop_time_like_fields:
            row = {k: v for k, v in row.items() if not _is_time_field_name(k)}

        # Row fields are prefixed with topic name to avoid collisions
        prefixed_row = {f"{topic}:{k}": v for k, v in row.items()}

        if topic not in per_topic_data:
            per_topic_data[topic] = {}

        ts_ns = int(t)
        per_topic_data[topic][ts_ns] = prefixed_row

        # Track global fields (maintain insertion order)
        for k in prefixed_row.keys():
            if k not in global_fields:
                global_fields.append(k)

    # Prepare output
    if not output_filename:
        # Name the CSV after the bag directory name
        output_filename = os.path.basename(os.path.normpath(bag_path)) + ".csv"
    out_path = os.path.join(output_dir, output_filename)

    # Compute sorted union of timestamps across all topics
    all_ts = sorted({ts for topic_map in per_topic_data.values() for ts in topic_map.keys()})

    # Build final header: timestamp columns first, then global_fields
    # Enforce a single timestamp column regardless of mode
    if stamp_mode == "sec":
        ts_headers = ["timestamp_sec"]
    else:
        # Treat 'ns' and 'ros' (or anything else) as ns
        ts_headers = ["timestamp_ns"]

    header = ts_headers + global_fields

    # Forward-fill state per field
    last_values: Dict[str, any] = {k: "" for k in global_fields}

    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()

        for ts in all_ts:
            row_out = {}
            # Timestamp columns
            if stamp_mode == "sec":
                row_out["timestamp_sec"] = ts * 1e-9
            else:
                row_out["timestamp_ns"] = ts

            # Update last_values for any topic that has data at this timestamp
            for topic, topic_map in per_topic_data.items():
                if ts in topic_map:
                    for k, v in topic_map[ts].items():
                        last_values[k] = v

            # Compose row with forward-filled values
            for k in global_fields:
                row_out[k] = last_values.get(k, "")

            writer.writerow(row_out)

    return out_path
