import json
from typing import Any, Dict

def _is_ros_msg(x: Any) -> bool:
    return hasattr(x, "get_fields_and_field_types") and callable(x.get_fields_and_field_types)

def _is_time_like(v: Any) -> bool:
    # builtin_interfaces/Time or Duration (sec/nanosec fields)
    return hasattr(v, "sec") and hasattr(v, "nanosec")

def _primitive_or_str(x: Any) -> bool:
    return isinstance(x, (int, float, bool, str)) or x is None

def flatten_message(msg: Any, prefix: str = "", out: Dict[str, Any] = None) -> Dict[str, Any]:
    """
    Recursively flatten a ROS 2 message into a flat dict.
    - Nested messages get dotted names: 'pose.position.x'
    - Sequences of primitives become JSON strings to keep CSV columns stable
    - Sequences of messages are indexed: 'points[0].x', 'points[1].x', ...
    - builtin_interfaces/Time or Duration become '<name>.sec' and '<name>.nanosec'
    """
    if out is None:
        out = {}
    if _is_time_like(msg):
        base = prefix[:-1] if prefix.endswith('.') else prefix
        out[f"{base}.sec"] = int(getattr(msg, "sec"))
        out[f"{base}.nanosec"] = int(getattr(msg, "nanosec"))
        return out

    if _is_ros_msg(msg):
        for field_name, _ in msg.get_fields_and_field_types().items():
            value = getattr(msg, field_name)
            key = f"{prefix}{field_name}"
            if _is_ros_msg(value) or _is_time_like(value):
                flatten_message(value, key + ".", out)
            elif isinstance(value, (list, tuple)):
                if all(_primitive_or_str(v) for v in value):
                    # keep arrays of primitives in one column as JSON
                    out[key] = json.dumps(value)
                else:
                    for i, v in enumerate(value):
                        if _is_ros_msg(v) or _is_time_like(v):
                            flatten_message(v, f"{key}[{i}].", out)
                        else:
                            out[f"{key}[{i}]"] = v
            else:
                out[key] = value
    else:
        # Fallback for primitives handed directly
        out[prefix] = msg
    return out
