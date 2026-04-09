#!/usr/bin/env python3
"""
live_map_bridge.py — Subprocess bridge for OccupancyGrid → PNG file.

Subscribes to /agv/live_map (OccupancyGrid from scan_grid_mapper),
converts to RGBA PNG, and writes to /tmp/agv_live_map.png + .json metadata.

The TypeScript backend reads these files periodically to serve the live map
to the dashboard. This bridges around the rclnodejs DDS discovery bug that
prevents reliable subscription to C++ publisher topics.

Usage (launched as subprocess from TS backend):
    python3 live_map_bridge.py --namespace agv
"""

import argparse
import json
import os
import struct
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid

PNG_PATH = '/tmp/agv_live_map.png'
META_PATH = '/tmp/agv_live_map.json'

# Minimal PNG encoder (no PIL/cv2 dependency on Jetson)
def write_png_rgba(path: str, width: int, height: int, pixels: bytes):
    """Write raw RGBA pixels as PNG using zlib + minimal PNG structure."""
    import zlib

    def chunk(chunk_type: bytes, data: bytes) -> bytes:
        c = chunk_type + data
        crc = struct.pack('>I', zlib.crc32(c) & 0xFFFFFFFF)
        return struct.pack('>I', len(data)) + c + crc

    # IHDR
    ihdr = struct.pack('>IIBBBBB', width, height, 8, 6, 0, 0, 0)  # 8-bit RGBA
    # IDAT: filter byte 0 (None) per row, then deflate
    raw_rows = b''
    stride = width * 4
    for y in range(height):
        raw_rows += b'\x00' + pixels[y * stride:(y + 1) * stride]
    compressed = zlib.compress(raw_rows, 6)

    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n')
        f.write(chunk(b'IHDR', ihdr))
        f.write(chunk(b'IDAT', compressed))
        f.write(chunk(b'IEND', b''))


class LiveMapBridge(Node):
    def __init__(self, namespace: str):
        super().__init__('live_map_bridge')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.sub = self.create_subscription(
            OccupancyGrid,
            f'/{namespace}/live_map',
            self.on_map,
            qos,
        )
        self.get_logger().info(f'Subscribed to /{namespace}/live_map → {PNG_PATH}')

    def on_map(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        data = msg.data

        # Build RGBA pixel buffer
        pixels = bytearray(w * h * 4)
        for i in range(w * h):
            v = data[i]
            p = i * 4
            if v == -1 or v == 255:
                # Unknown — transparent
                pixels[p] = 0; pixels[p+1] = 0; pixels[p+2] = 0; pixels[p+3] = 0
            elif v == 0:
                # Free — white, semi-transparent
                pixels[p] = 240; pixels[p+1] = 245; pixels[p+2] = 240; pixels[p+3] = 180
            elif v >= 80:
                # Occupied — dark
                pixels[p] = 20; pixels[p+1] = 20; pixels[p+2] = 25; pixels[p+3] = 240
            else:
                # Partial
                a = min(220, 80 + int(v * 1.5))
                g = max(20, 240 - int(v * 2.2))
                pixels[p] = g; pixels[p+1] = g; pixels[p+2] = g + 5; pixels[p+3] = a

        # Flip vertically (ROS origin is bottom-left, PNG is top-left)
        stride = w * 4
        flipped = bytearray(w * h * 4)
        for row in range(h):
            src_start = row * stride
            dst_start = (h - 1 - row) * stride
            flipped[dst_start:dst_start + stride] = pixels[src_start:src_start + stride]

        # Write PNG atomically (write to tmp, then rename)
        tmp_png = PNG_PATH + '.tmp'
        tmp_meta = META_PATH + '.tmp'
        try:
            write_png_rgba(tmp_png, w, h, bytes(flipped))
            os.replace(tmp_png, PNG_PATH)

            meta = {
                'resolution': msg.info.resolution,
                'origin_x': msg.info.origin.position.x,
                'origin_y': msg.info.origin.position.y,
                'width': w,
                'height': h,
                'timestamp': time.time(),
            }
            with open(tmp_meta, 'w') as f:
                json.dump(meta, f)
            os.replace(tmp_meta, META_PATH)
        except Exception as e:
            self.get_logger().warn(f'Failed to write PNG: {e}')
            return

        self.get_logger().info(
            f'Map updated: {w}x{h} @ {msg.info.resolution}m '
            f'({os.path.getsize(PNG_PATH) // 1024}KB)',
            throttle_duration_sec=10.0,
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', default='agv')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = LiveMapBridge(args.namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
