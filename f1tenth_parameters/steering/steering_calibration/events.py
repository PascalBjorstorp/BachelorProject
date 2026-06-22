"""Structured event markers recorded alongside raw ROS messages."""
from __future__ import annotations

import json
from typing import Any

from std_msgs.msg import String


class EventPublisher:
    def __init__(self, node, topic: str) -> None:
        self._node = node
        self._pub = node.create_publisher(String, topic, 50)

    def emit(self, name: str, **payload: Any) -> None:
        record = {
            "event": name,
            "node_time_ns": int(self._node.get_clock().now().nanoseconds),
            **payload,
        }
        message = String()
        message.data = json.dumps(record, sort_keys=True, default=str)
        self._pub.publish(message)
