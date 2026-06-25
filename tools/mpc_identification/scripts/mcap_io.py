#!/usr/bin/env python3
"""Dependency-light MCAP + ROS 2 CDR reader for uncompressed rosbag2 MCAP files.

The provided bag stores uncompressed MCAP chunks. This reader intentionally avoids
ROS 2 runtime dependencies so the identification pipeline remains reproducible on a
non-ROS workstation. It supports the message types required by this project.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterator, Optional, Tuple, Any
import struct
import math
import mmap
import numpy as np

MCAP_MAGIC = b"\x89MCAP0\r\n"
OP_SCHEMA = 0x03
OP_CHANNEL = 0x04
OP_MESSAGE = 0x05
OP_CHUNK = 0x06


@dataclass(frozen=True)
class Message:
    topic: str
    type_name: str
    log_time_ns: int
    publish_time_ns: int
    data: bytes | memoryview


def _read_u32(buf: bytes | memoryview, offset: int) -> Tuple[int, int]:
    return struct.unpack_from("<I", buf, offset)[0], offset + 4


def _read_u64(buf: bytes | memoryview, offset: int) -> Tuple[int, int]:
    return struct.unpack_from("<Q", buf, offset)[0], offset + 8


def _read_string(buf: bytes | memoryview, offset: int) -> Tuple[str, int]:
    n, offset = _read_u32(buf, offset)
    end = offset + n
    if end > len(buf):
        raise ValueError("Truncated MCAP string")
    return bytes(buf[offset:end]).decode("utf-8"), end


def _iter_records(buf: bytes | memoryview) -> Iterator[Tuple[int, memoryview]]:
    offset = 0
    nbuf = len(buf)
    while offset + 9 <= nbuf:
        opcode = buf[offset]
        length = struct.unpack_from("<Q", buf, offset + 1)[0]
        start = offset + 9
        end = start + length
        if end > nbuf:
            raise ValueError(f"Truncated MCAP record: opcode={opcode}, length={length}")
        yield opcode, buf[start:end]
        offset = end


class McapReader:
    """Iterate CDR messages from an uncompressed ROS 2 MCAP bag.

    The reader memory-maps the file and keeps every record as a view instead of
    copying each 0.8 MB MCAP chunk into Python bytes. That matters for the
    supplied 729 MB bag, which contains 905 chunks and more than 300,000
    messages. It supports uncompressed MCAP chunks only by design.
    """

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.schemas: Dict[int, str] = {}
        self.channels: Dict[int, Tuple[str, int, str]] = {}

    def _consume_definition(self, opcode: int, body: memoryview) -> None:
        if opcode == OP_SCHEMA:
            schema_id = struct.unpack_from("<H", body, 0)[0]
            offset = 2
            name, offset = _read_string(body, offset)
            _encoding, offset = _read_string(body, offset)
            self.schemas[schema_id] = name
        elif opcode == OP_CHANNEL:
            channel_id, schema_id = struct.unpack_from("<HH", body, 0)
            offset = 4
            topic, offset = _read_string(body, offset)
            encoding, offset = _read_string(body, offset)
            self.channels[channel_id] = (topic, schema_id, encoding)

    def _iter_chunk(self, body: memoryview) -> Iterator[Message]:
        if len(body) < 40:
            raise ValueError("Truncated MCAP chunk")
        # MCAP Chunk fields: start/end, uncompressed size/CRC, compression
        # string, then a uint64 records-byte length followed by record bytes.
        offset = 28
        compression, offset = _read_string(body, offset)
        records_len, offset = _read_u64(body, offset)
        records_end = offset + records_len
        if records_end > len(body):
            raise ValueError("Truncated MCAP chunk record bytes")
        if compression:
            raise RuntimeError(
                f"Compressed MCAP chunk ({compression!r}) is not supported by this dependency-light reader. "
                "Install mcap + mcap-ros2-support or record without chunk compression."
            )
        for opcode, record in _iter_records(body[offset:records_end]):
            if opcode in (OP_SCHEMA, OP_CHANNEL):
                self._consume_definition(opcode, record)
                continue
            if opcode != OP_MESSAGE or len(record) < 22:
                continue
            channel_id, _sequence = struct.unpack_from("<HI", record, 0)
            log_time_ns, publish_time_ns = struct.unpack_from("<QQ", record, 6)
            channel = self.channels.get(channel_id)
            if channel is None:
                continue
            topic, schema_id, encoding = channel
            if encoding != "cdr":
                continue
            yield Message(
                topic=topic,
                type_name=self.schemas.get(schema_id, f"unknown/schema/{schema_id}"),
                log_time_ns=log_time_ns,
                publish_time_ns=publish_time_ns,
                data=record[22:],
            )

    def messages(self) -> Iterator[Message]:
        # Do not use mmap as a context manager here. The final yielded Message
        # can still be bound by the caller when this generator is exhausted;
        # closing a mmap with that exported memoryview raises BufferError.
        with self.path.open("rb") as f:
            mapped = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
            buf = memoryview(mapped)
            try:
                if len(buf) < 8 or bytes(buf[:8]) != MCAP_MAGIC:
                    raise ValueError(f"Not an MCAP file: {self.path}")
                offset = 8
                nbuf = len(buf)
                while offset + 9 <= nbuf:
                    opcode = int(buf[offset])
                    length = struct.unpack_from("<Q", buf, offset + 1)[0]
                    start = offset + 9
                    end = start + length
                    if end > nbuf:
                        # Final trailing 8-byte magic has no MCAP record header.
                        break
                    body = buf[start:end]
                    if opcode in (OP_SCHEMA, OP_CHANNEL):
                        self._consume_definition(opcode, body)
                    elif opcode == OP_CHUNK:
                        yield from self._iter_chunk(body)
                    offset = end
            finally:
                buf.release()
                try:
                    mapped.close()
                except BufferError:
                    # A caller may still retain only the final message's CDR
                    # view. Its memoryview keeps the mapping alive and Python
                    # releases it once that view is dropped.
                    pass


class CdrReader:
    """Minimal little-endian XCDR1 reader for generated ROS 2 message layouts."""

    def __init__(self, data: bytes):
        if len(data) < 4:
            raise ValueError("CDR payload shorter than encapsulation header")
        # ROS 2 CDR messages recorded here use 00 01 00 00: little-endian XCDR1.
        if data[1] != 0x01:
            raise ValueError(f"Unsupported CDR encapsulation: {data[:4].hex()}")
        self.data = memoryview(data)
        self.start = 4
        self.pos = 4

    def _align(self, alignment: int) -> None:
        rel = self.pos - self.start
        self.pos += (-rel) % alignment
        if self.pos > len(self.data):
            raise ValueError("CDR alignment beyond payload")

    def _unpack(self, fmt: str, alignment: int) -> Any:
        self._align(alignment)
        n = struct.calcsize(fmt)
        if self.pos + n > len(self.data):
            raise ValueError("Truncated CDR primitive")
        value = struct.unpack_from(fmt, self.data, self.pos)[0]
        self.pos += n
        return value

    def i8(self) -> int: return self._unpack("<b", 1)
    def u8(self) -> int: return self._unpack("<B", 1)
    def i16(self) -> int: return self._unpack("<h", 2)
    def u16(self) -> int: return self._unpack("<H", 2)
    def i32(self) -> int: return self._unpack("<i", 4)
    def u32(self) -> int: return self._unpack("<I", 4)
    def i64(self) -> int: return self._unpack("<q", 8)
    def u64(self) -> int: return self._unpack("<Q", 8)
    def f32(self) -> float: return self._unpack("<f", 4)
    def f64(self) -> float: return self._unpack("<d", 8)

    def string(self) -> str:
        n = self.u32()
        if n == 0:
            return ""
        end = self.pos + n
        if end > len(self.data):
            raise ValueError("Truncated CDR string")
        raw = self.data[self.pos:end].tobytes()
        self.pos = end
        if raw.endswith(b"\0"):
            raw = raw[:-1]
        return raw.decode("utf-8", errors="replace")

    def array_f32(self) -> np.ndarray:
        n = self.u32()
        self._align(4)
        size = 4 * n
        if self.pos + size > len(self.data):
            raise ValueError("Truncated CDR float32 sequence")
        out = np.frombuffer(self.data[self.pos:self.pos + size], dtype="<f4").copy()
        self.pos += size
        return out

    def array_f64(self, n: int) -> np.ndarray:
        self._align(8)
        size = 8 * n
        if self.pos + size > len(self.data):
            raise ValueError("Truncated CDR float64 array")
        out = np.frombuffer(self.data[self.pos:self.pos + size], dtype="<f8").copy()
        self.pos += size
        return out

    def array_i8(self) -> np.ndarray:
        n = self.u32()
        size = n
        if self.pos + size > len(self.data):
            raise ValueError("Truncated CDR int8 sequence")
        out = np.frombuffer(self.data[self.pos:self.pos + size], dtype=np.int8).copy()
        self.pos += size
        return out


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def read_header(c: CdrReader) -> Dict[str, Any]:
    sec = c.i32()
    nsec = c.u32()
    return {"stamp_ns": sec * 1_000_000_000 + nsec, "frame_id": c.string()}


def read_quaternion(c: CdrReader) -> Dict[str, float]:
    return {"qx": c.f64(), "qy": c.f64(), "qz": c.f64(), "qw": c.f64()}


def read_pose(c: CdrReader) -> Dict[str, float]:
    pose = {"x": c.f64(), "y": c.f64(), "z": c.f64()}
    pose.update(read_quaternion(c))
    pose["yaw"] = quat_to_yaw(pose["qx"], pose["qy"], pose["qz"], pose["qw"])
    return pose


def read_pose_covariance(c: CdrReader) -> Dict[str, Any]:
    pose = read_pose(c)
    pose["covariance"] = c.array_f64(36)
    return pose


def read_twist(c: CdrReader) -> Dict[str, float]:
    return {
        "vx": c.f64(), "vy": c.f64(), "vz": c.f64(),
        "wx": c.f64(), "wy": c.f64(), "wz": c.f64(),
    }


def read_twist_covariance(c: CdrReader) -> Dict[str, Any]:
    twist = read_twist(c)
    twist["covariance"] = c.array_f64(36)
    return twist


def decode_pose_with_covariance_stamped(data: bytes) -> Dict[str, Any]:
    c = CdrReader(data)
    out = read_header(c)
    out.update(read_pose_covariance(c))
    return out


def decode_odometry(data: bytes) -> Dict[str, Any]:
    c = CdrReader(data)
    out = read_header(c)
    out["child_frame_id"] = c.string()
    pose = read_pose_covariance(c)
    twist = read_twist_covariance(c)
    out.update({f"pose_{k}": v for k, v in pose.items() if k != "covariance"})
    out.update({f"twist_{k}": v for k, v in twist.items() if k != "covariance"})
    out["pose_covariance"] = pose["covariance"]
    out["twist_covariance"] = twist["covariance"]
    return out


def decode_ackermann_drive_stamped(data: bytes) -> Dict[str, Any]:
    c = CdrReader(data)
    out = read_header(c)
    out.update({
        "steering_angle": c.f32(),
        "steering_angle_velocity": c.f32(),
        "speed": c.f32(),
        "acceleration": c.f32(),
        "jerk": c.f32(),
    })
    return out


def decode_float64(data: bytes) -> Dict[str, float]:
    return {"value": CdrReader(data).f64()}


def decode_int32(data: bytes) -> Dict[str, int]:
    return {"value": CdrReader(data).i32()}


def decode_laserscan(data: bytes) -> Dict[str, Any]:
    c = CdrReader(data)
    out = read_header(c)
    out.update({
        "angle_min": c.f32(), "angle_max": c.f32(), "angle_increment": c.f32(),
        "time_increment": c.f32(), "scan_time": c.f32(),
        "range_min": c.f32(), "range_max": c.f32(),
    })
    out["ranges"] = c.array_f32()
    out["intensities"] = c.array_f32()
    return out


def decode_occupancy_grid(data: bytes) -> Dict[str, Any]:
    c = CdrReader(data)
    out = read_header(c)
    load_sec = c.i32()
    load_nsec = c.u32()
    resolution = c.f32()
    width = c.u32()
    height = c.u32()
    origin = read_pose(c)
    grid = c.array_i8()
    if grid.size != width * height:
        raise ValueError(f"Occupancy grid length {grid.size} != {width}*{height}")
    out.update({
        "load_time_ns": load_sec * 1_000_000_000 + load_nsec,
        "resolution": resolution, "width": width, "height": height,
        "origin": origin, "grid": grid.reshape((height, width)),
    })
    return out


def decode_tf_message(data: bytes) -> list[Dict[str, Any]]:
    c = CdrReader(data)
    n = c.u32()
    out: list[Dict[str, Any]] = []
    for _ in range(n):
        header = read_header(c)
        child_frame_id = c.string()
        tx, ty, tz = c.f64(), c.f64(), c.f64()
        quat = read_quaternion(c)
        out.append({
            **header, "child_frame_id": child_frame_id,
            "tx": tx, "ty": ty, "tz": tz, **quat,
            "yaw": quat_to_yaw(quat["qx"], quat["qy"], quat["qz"], quat["qw"]),
        })
    return out


def decode_imu(data: bytes) -> Dict[str, Any]:
    c = CdrReader(data)
    out = read_header(c)
    orientation = read_quaternion(c)
    orientation_cov = c.array_f64(9)
    angular = {"av_x": c.f64(), "av_y": c.f64(), "av_z": c.f64()}
    angular_cov = c.array_f64(9)
    linear = {"la_x": c.f64(), "la_y": c.f64(), "la_z": c.f64()}
    linear_cov = c.array_f64(9)
    out.update(orientation)
    out["yaw"] = quat_to_yaw(orientation["qx"], orientation["qy"], orientation["qz"], orientation["qw"])
    out.update(angular)
    out.update(linear)
    out["orientation_covariance"] = orientation_cov
    out["angular_velocity_covariance"] = angular_cov
    out["linear_acceleration_covariance"] = linear_cov
    return out


def decode_path(data: bytes) -> Dict[str, Any]:
    c = CdrReader(data)
    out = read_header(c)
    n = c.u32()
    poses = []
    for _ in range(n):
        h = read_header(c)
        p = read_pose(c)
        poses.append({**h, **p})
    out["poses"] = poses
    return out


def decode_message(type_name: str, data: bytes) -> Optional[Any]:
    """Deserialize the project-relevant ROS 2 message types."""
    if type_name == "std_msgs/msg/Float64":
        return decode_float64(data)
    if type_name == "std_msgs/msg/Int32":
        return decode_int32(data)
    if type_name == "geometry_msgs/msg/PoseWithCovarianceStamped":
        return decode_pose_with_covariance_stamped(data)
    if type_name == "nav_msgs/msg/Odometry":
        return decode_odometry(data)
    if type_name == "ackermann_msgs/msg/AckermannDriveStamped":
        return decode_ackermann_drive_stamped(data)
    if type_name == "sensor_msgs/msg/LaserScan":
        return decode_laserscan(data)
    if type_name == "nav_msgs/msg/OccupancyGrid":
        return decode_occupancy_grid(data)
    if type_name == "tf2_msgs/msg/TFMessage":
        return decode_tf_message(data)
    if type_name == "sensor_msgs/msg/Imu":
        return decode_imu(data)
    if type_name == "nav_msgs/msg/Path":
        return decode_path(data)
    return None
