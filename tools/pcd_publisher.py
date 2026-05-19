#!/usr/bin/env python3
"""
PCD -> ROS2 PointCloud2 发布器

用途:
  使用 odin1 建图得到 pcd 后, 把地图发布到 RViz, 方便人工对比 odin1
  的实时定位/重定位结果是否与建图一致。

依赖:
  - rclpy
  - sensor_msgs / std_msgs
  (不依赖 open3d/pcl, 内置 PCD 解析器, 支持 ASCII 与 binary 两种 DATA 格式,
   FIELDS 形如 'x y z' 或 'x y z rgb'。)

用法:
  python3 tools/pcd_publisher.py <pcd_path> [--topic /pcd_map] \\
      [--frame map] [--rate 1.0] [--latch]

示例:
  python3 tools/pcd_publisher.py /tmp/odin_slam_xxx.pcd
  python3 tools/pcd_publisher.py map.pcd --topic /pcd_map --frame map --latch

RViz:
  Fixed Frame  = map (与 --frame 一致)
  添加 PointCloud2, Topic = /pcd_map
  若 PCD 带颜色, Color Transformer 选 RGB8 即可看到原色。
"""

import argparse
import struct
import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from rclpy.utilities import remove_ros_args

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField


# PCD TYPE/SIZE -> numpy dtype 字符
_PCD_TYPE_MAP = {
    ("F", 4): "<f4",
    ("F", 8): "<f8",
    ("U", 1): "<u1",
    ("U", 2): "<u2",
    ("U", 4): "<u4",
    ("U", 8): "<u8",
    ("I", 1): "<i1",
    ("I", 2): "<i2",
    ("I", 4): "<i4",
    ("I", 8): "<i8",
}


def _parse_pcd_header(f):
    """从二进制文件流读取 PCD 头, 返回 header_dict; 文件指针定位到 DATA 行之后。"""
    header = {}
    while True:
        line_bytes = f.readline()
        if not line_bytes:
            raise RuntimeError("PCD 头解析失败: 到达文件末尾")
        line = line_bytes.decode("ascii", errors="ignore").strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        key = parts[0].upper()
        vals = parts[1:]
        if key == "DATA":
            header["DATA"] = vals[0].lower() if vals else "ascii"
            return header
        header[key] = vals


def _lzf_decompress(src: bytes, out_size: int) -> bytes:
    """Pure-python LZF decompression, compatible with PCL binary_compressed.

    优先尝试 python-lzf 包以提高速度;失败时回落到本地实现。
    """
    try:
        import lzf  # type: ignore
        out = lzf.decompress(src, out_size)
        if out is None or len(out) != out_size:
            raise RuntimeError("python-lzf decompress 失败")
        return out
    except Exception:
        pass

    out = bytearray(out_size)
    in_len = len(src)
    ip = 0
    op = 0
    while ip < in_len:
        ctrl = src[ip]
        ip += 1
        if ctrl < 32:
            length = ctrl + 1
            if op + length > out_size or ip + length > in_len:
                raise RuntimeError("LZF: 越界 (literal)")
            out[op:op + length] = src[ip:ip + length]
            ip += length
            op += length
        else:
            length = ctrl >> 5
            if length == 7:
                if ip >= in_len:
                    raise RuntimeError("LZF: 越界 (length ext)")
                length += src[ip]
                ip += 1
            length += 2
            if ip >= in_len:
                raise RuntimeError("LZF: 越界 (ref)")
            ref = op - ((ctrl & 0x1f) << 8) - src[ip] - 1
            ip += 1
            if ref < 0 or op + length > out_size:
                raise RuntimeError("LZF: 反向引用越界")
            # 字节级拷贝, 源和目标可能重叠 (经典 LZ 行为)
            for i in range(length):
                out[op + i] = out[ref + i]
            op += length
    if op != out_size:
        raise RuntimeError(f"LZF: 解压字节数 {op} != 预期 {out_size}")
    return bytes(out)


def load_pcd(path: str):
    """加载 PCD, 返回 (xyz: float32 N x 3, rgb_packed_or_None)。
    rgb_packed: uint32 numpy 数组(若 PCD 含 rgb 字段), 元素为 0xRRGGBB。"""
    with open(path, "rb") as f:
        header = _parse_pcd_header(f)

        fields = [s.lower() for s in header.get("FIELDS", [])]
        sizes = [int(s) for s in header.get("SIZE", [])]
        types = [s.upper() for s in header.get("TYPE", [])]
        counts = [int(s) for s in header.get("COUNT", [])] or [1] * len(fields)
        n = int(header.get("POINTS", [0])[0])
        data_fmt = header.get("DATA", "ascii")

        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise RuntimeError(f"PCD 头字段不一致: {header}")
        for fname in ("x", "y", "z"):
            if fname not in fields:
                raise RuntimeError(f"PCD 缺少必要字段 {fname}: fields={fields}")

        dtype_fields = []
        for fname, sz, tp, cnt in zip(fields, sizes, types, counts):
            base = _PCD_TYPE_MAP.get((tp, sz))
            if base is None:
                raise RuntimeError(
                    f"不支持的 PCD 字段类型: {fname} TYPE={tp} SIZE={sz}")
            if cnt == 1:
                dtype_fields.append((fname, base))
            else:
                dtype_fields.append((fname, base, (cnt,)))
        dtype = np.dtype(dtype_fields)

        if data_fmt == "binary":
            buf = f.read(dtype.itemsize * n)
            if len(buf) < dtype.itemsize * n:
                raise RuntimeError(
                    f"PCD 二进制段大小不足: expect={dtype.itemsize*n} got={len(buf)}")
            arr = np.frombuffer(buf, dtype=dtype, count=n)
        elif data_fmt == "ascii":
            arr_raw = np.loadtxt(f, dtype=np.float64, max_rows=n)
            if arr_raw.ndim == 1:
                arr_raw = arr_raw.reshape(1, -1)
            arr = np.zeros(n, dtype=dtype)
            col = 0
            for fname, sz, tp, cnt in zip(fields, sizes, types, counts):
                if cnt == 1:
                    arr[fname] = arr_raw[:, col].astype(_PCD_TYPE_MAP[(tp, sz)])
                    col += 1
                else:
                    arr[fname] = arr_raw[:, col:col + cnt].astype(
                        _PCD_TYPE_MAP[(tp, sz)])
                    col += cnt
        elif data_fmt == "binary_compressed":
            # PCL 格式: 4B LE compressed_size, 4B LE uncompressed_size,
            # 接着是 LZF 压缩数据, 解压后是 SoA 布局 (按字段, 每字段 N 个值连续)
            hdr_bin = f.read(8)
            if len(hdr_bin) < 8:
                raise RuntimeError("binary_compressed: 头部不足 8 字节")
            comp_size = struct.unpack("<I", hdr_bin[:4])[0]
            uncomp_size = struct.unpack("<I", hdr_bin[4:8])[0]
            comp_buf = f.read(comp_size)
            if len(comp_buf) < comp_size:
                raise RuntimeError(
                    f"binary_compressed: 数据段长度不足 {len(comp_buf)}/{comp_size}")
            raw = _lzf_decompress(comp_buf, uncomp_size)
            arr = np.zeros(n, dtype=dtype)
            off = 0
            for fname, sz, tp, cnt in zip(fields, sizes, types, counts):
                base = _PCD_TYPE_MAP[(tp, sz)]
                field_bytes = sz * cnt * n
                col = np.frombuffer(raw, dtype=base,
                                    count=cnt * n, offset=off)
                if cnt > 1:
                    col = col.reshape(n, cnt)
                arr[fname] = col
                off += field_bytes
        else:
            raise RuntimeError(f"未知 PCD DATA 类型: {data_fmt}")

        xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=-1).astype(np.float32)

        rgb_packed = None
        if "rgb" in fields:
            raw = arr["rgb"]
            if raw.dtype.kind == "f":
                rgb_packed = raw.view(np.uint32).copy()
            else:
                rgb_packed = raw.astype(np.uint32)
        elif "rgba" in fields:
            raw = arr["rgba"]
            rgb_packed = (raw.astype(np.uint32) & 0x00FFFFFF)

        return xyz, rgb_packed


class PCDPublisher(Node):
    def __init__(self, pcd_path: str, topic: str, frame_id: str,
                 rate: float, latch: bool):
        super().__init__("pcd_map_publisher")

        self.frame_id = frame_id

        if latch:
            qos = QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
            )
        else:
            qos = 10

        self.pub = self.create_publisher(PointCloud2, topic, qos)
        self.cloud_msg = self._build_cloud_msg(pcd_path)

        self.get_logger().info(
            f"Loaded PCD: {pcd_path} -> topic={topic} frame={frame_id} "
            f"rate={rate}Hz latch={latch}"
        )

        self.publish_once()
        self.timer = self.create_timer(1.0 / max(rate, 1e-3), self.publish_once)

    def _build_cloud_msg(self, path: str) -> PointCloud2:
        xyz, rgb_packed = load_pcd(path)
        n = xyz.shape[0]
        if n == 0:
            raise RuntimeError(f"PCD 为空或读取失败: {path}")

        msg = PointCloud2()
        msg.header = Header()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = n
        msg.is_bigendian = False
        msg.is_dense = True

        if rgb_packed is not None:
            msg.fields = [
                PointField(name="x",   offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name="y",   offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name="z",   offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            msg.point_step = 16
            block = np.empty((n, 4), dtype=np.float32)
            block[:, :3] = xyz
            block[:, 3] = rgb_packed.astype(np.uint32).view(np.float32)
            msg.data = block.tobytes()
            self.get_logger().info(f"点数={n} 含 RGB")
        else:
            msg.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            msg.point_step = 12
            msg.data = xyz.astype(np.float32).tobytes()
            self.get_logger().info(f"点数={n} 无 RGB")

        msg.row_step = msg.point_step * n
        return msg

    def publish_once(self):
        self.cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.cloud_msg)


def parse_args(argv):
    p = argparse.ArgumentParser(description="Publish a .pcd file as ROS2 PointCloud2")
    p.add_argument("pcd", help="PCD 文件路径")
    p.add_argument("--topic", default="/pcd_map", help="发布话题, 默认 /pcd_map")
    p.add_argument("--frame", default="map", help="frame_id, 默认 map")
    p.add_argument("--rate", type=float, default=1.0, help="发布频率 Hz, 默认 1.0")
    p.add_argument("--latch", action="store_true",
                   help="使用 TRANSIENT_LOCAL (latched) QoS, 后开的订阅者也能收到最近一帧")
    return p.parse_args(argv)


def main():
    # Node(...) 启动会追加 '--ros-args ...', 这里先剥掉再交给 argparse
    cli_args = remove_ros_args(args=sys.argv)[1:]
    args = parse_args(cli_args)
    rclpy.init()
    node = PCDPublisher(args.pcd, args.topic, args.frame, args.rate, args.latch)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
