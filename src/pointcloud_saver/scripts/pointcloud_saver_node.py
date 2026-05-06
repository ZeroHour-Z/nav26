#!/usr/bin/env python3
"""
pointcloud_saver_node  –  ROS 2 节点
订阅 SLAM 点云话题，累积并保存为 PCD 文件。
"""

import os
import struct
import threading
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from pointcloud_saver.srv import SaveMap

_F32 = np.float32
_U8 = np.uint8


class PointCloudSaver(Node):
    def __init__(self):
        super().__init__("pointcloud_saver")

        self.declare_parameter("cloud_topic", "/odin1/cloud_slam")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("save_dir",
                               "/home/xjturm/xjtu_nav26/src/odin_ros_driver/pcd")
        self.declare_parameter("max_points", 10_000_000)
        self.declare_parameter("auto_publish", True)
        self.declare_parameter("publish_interval", 2.0)
        self.declare_parameter("merge_interval", 5.0)
        self.declare_parameter("merge_voxel_size", 0.10)
        self.declare_parameter("save_rgb", True)
        self.declare_parameter("max_stat_points", 200_000)
        self.declare_parameter("pcd_binary", True)
        self.declare_parameter("skip_tf", False)
        self.declare_parameter("auto_start", False)
        # 若为非空字符串：退出时写入该绝对路径（单文件，与 Mid360 LIO 的 pcd_save.file_path 语义一致）
        self.declare_parameter("session_pcd_path", "")
        self.declare_parameter("auto_save_interval", -1.0)
        self.declare_parameter("auto_save_voxel_size", 0.05)
        self.declare_parameter("process_every_n", 3)

        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.save_dir = self.get_parameter("save_dir").value
        self.max_points = self.get_parameter("max_points").value
        self.auto_publish = self.get_parameter("auto_publish").value
        self.publish_interval = self.get_parameter("publish_interval").value
        self.merge_interval = self.get_parameter("merge_interval").value
        self.merge_voxel_size = self.get_parameter("merge_voxel_size").value
        self.save_rgb = self.get_parameter("save_rgb").value
        self.max_stat_points = self.get_parameter("max_stat_points").value
        self.pcd_binary = self.get_parameter("pcd_binary").value
        self.skip_tf = self.get_parameter("skip_tf").value
        auto_start = self.get_parameter("auto_start").value
        self.session_pcd_path = (
            self.get_parameter("session_pcd_path").value or ""
        ).strip()
        self.auto_save_interval = self.get_parameter("auto_save_interval").value
        self.auto_save_voxel_size = self.get_parameter("auto_save_voxel_size").value
        self.process_every_n = max(1, self.get_parameter("process_every_n").value)

        self.is_recording = bool(auto_start)
        self._recv_count = 0
        self._frame_count = 0
        self._save_seq = 0
        self._first_msg_logged = False

        self._pending = deque()
        self._pending_rgb = deque()
        self._lock = threading.Lock()
        self._merged_xyz = np.empty((0, 3), dtype=_F32)
        self._merged_rgb = np.empty((0, 3), dtype=_U8) if self.save_rgb else None

        if not self.skip_tf:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
            self.tf_listener = None

        os.makedirs(self.save_dir, exist_ok=True)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(PointCloud2, self.cloud_topic, self._cloud_cb, qos)

        self.create_timer(self.merge_interval, self._merge_cb)

        if self.auto_publish:
            self._pub = self.create_publisher(PointCloud2, "/accumulated_map", 1)
            self.create_timer(self.publish_interval, self._publish_cb)

        if self.auto_save_interval > 0:
            self.create_timer(self.auto_save_interval, self._auto_save_cb)

        self.create_service(Trigger, "~/start_recording", self._srv_start)
        self.create_service(Trigger, "~/stop_recording", self._srv_stop)
        self.create_service(SaveMap, "~/save_map", self._srv_save)
        self.create_service(Trigger, "~/clear_map", self._srv_clear)

        self.get_logger().info("=" * 60)
        self.get_logger().info("点云保存节点已启动")
        self.get_logger().info(f"  话题: {self.cloud_topic}  帧率: 1/{self.process_every_n}")
        self.get_logger().info(f"  坐标系: {self.target_frame}  skip_tf: {self.skip_tf}")
        self.get_logger().info(f"  保存: {self.save_dir}")
        self.get_logger().info(f"  merge: 每{self.merge_interval}s  voxel={self.merge_voxel_size}m")
        self.get_logger().info(f"  自动录制: {'是' if auto_start else '否'}  "
                               f"自动保存: {self.auto_save_interval}s")
        self.get_logger().info(f"  PCD 二进制: {self.pcd_binary}")
        if self.session_pcd_path:
            self.get_logger().info(f"  会话地图(退出写入): {self.session_pcd_path}")
        self.get_logger().info("=" * 60)

    # ================================================================
    #  回调
    # ================================================================
    def _cloud_cb(self, msg: PointCloud2):
        if not self.is_recording:
            return

        self._recv_count += 1
        if self._recv_count % self.process_every_n != 0:
            return

        try:
            xyz, rgb = self._parse_cloud(msg)
        except Exception as e:
            if not self._first_msg_logged:
                self.get_logger().error(f"解析点云失败: {e}")
            return

        if xyz is None or xyz.shape[0] == 0:
            if not self._first_msg_logged:
                self.get_logger().warn("首帧点云为空!")
            return

        if not self._first_msg_logged:
            self._first_msg_logged = True
            self._log_first_msg(msg, xyz, rgb)

        if not self.skip_tf and self.tf_buffer is not None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame, msg.header.frame_id, rclpy.time.Time()
                )
            except TransformException:
                return
            tr = tf.transform.translation
            ro = tf.transform.rotation
            R = _quat_to_rot(ro.x, ro.y, ro.z, ro.w)
            t = np.array([tr.x, tr.y, tr.z])
            xyz = (xyz.astype(np.float64) @ R.T + t).astype(_F32)

        self._pending.append(xyz)
        if self.save_rgb and rgb is not None:
            self._pending_rgb.append(rgb)

        self._frame_count += 1

    def _log_first_msg(self, msg, xyz, rgb):
        """首帧诊断日志"""
        self.get_logger().info("=== 首帧点云诊断 ===")
        self.get_logger().info(f"  frame_id: {msg.header.frame_id}")
        self.get_logger().info(f"  width={msg.width} height={msg.height} "
                               f"point_step={msg.point_step} row_step={msg.row_step}")
        for f in msg.fields:
            self.get_logger().info(f"  field: {f.name}  offset={f.offset}  "
                                   f"datatype={f.datatype}  count={f.count}")
        self.get_logger().info(f"  解析后: {xyz.shape[0]} 点")
        self.get_logger().info(f"  X范围: [{xyz[:, 0].min():.3f}, {xyz[:, 0].max():.3f}]")
        self.get_logger().info(f"  Y范围: [{xyz[:, 1].min():.3f}, {xyz[:, 1].max():.3f}]")
        self.get_logger().info(f"  Z范围: [{xyz[:, 2].min():.3f}, {xyz[:, 2].max():.3f}]")
        if rgb is not None:
            self.get_logger().info(f"  RGB: 有 ({rgb.shape})")
        else:
            self.get_logger().info(f"  RGB: 无")
        self.get_logger().info("=== 诊断结束 ===")

    # ================================================================
    #  Merge 定时器
    # ================================================================
    def _merge_cb(self):
        n_pending = len(self._pending)
        if n_pending == 0:
            return

        chunks = []
        chunks_rgb = []
        while self._pending:
            chunks.append(self._pending.popleft())
        while self._pending_rgb:
            chunks_rgb.append(self._pending_rgb.popleft())

        new_xyz = np.concatenate(chunks, axis=0)
        new_rgb = np.concatenate(chunks_rgb, axis=0) if chunks_rgb else None

        with self._lock:
            if self._merged_xyz.shape[0] > 0:
                self._merged_xyz = np.concatenate(
                    [self._merged_xyz, new_xyz], axis=0)
                if self.save_rgb and self._merged_rgb is not None and new_rgb is not None:
                    self._merged_rgb = np.concatenate(
                        [self._merged_rgb, new_rgb], axis=0)
            else:
                self._merged_xyz = new_xyz
                if self.save_rgb and new_rgb is not None:
                    self._merged_rgb = new_rgb

            if self.merge_voxel_size > 0 and self._merged_xyz.shape[0] > 50_000:
                self._merged_xyz, self._merged_rgb = _voxel_downsample(
                    self._merged_xyz, self._merged_rgb, self.merge_voxel_size
                )

            total = self._merged_xyz.shape[0]

        self.get_logger().info(
            f"[merge] +{n_pending}帧 -> 累积 {total} 点  (已处理 {self._frame_count} 帧)"
        )

    # ================================================================
    #  RViz 发布
    # ================================================================
    def _publish_cb(self):
        with self._lock:
            if self._merged_xyz.shape[0] == 0:
                return
            pts = self._merged_xyz.copy()

        if pts.shape[0] > 200_000:
            pts = pts[:: max(1, pts.shape[0] // 200_000)]

        self._pub.publish(_xyz_to_pc2(pts, self.target_frame, self.get_clock().now()))

    # ================================================================
    #  自动保存
    # ================================================================
    def _auto_save_cb(self):
        self._merge_cb()
        self._do_save("auto")

    def _do_save(self, prefix, fixed_path=None):
        """执行一次保存，返回是否成功。fixed_path 非空时写入该路径（单文件，覆盖）。"""
        with self._lock:
            if self._merged_xyz.shape[0] == 0:
                return False
            xyz = self._merged_xyz.copy()
            rgb = self._merged_rgb.copy() if self._merged_rgb is not None else None

        if fixed_path:
            fpath = fixed_path
            parent = os.path.dirname(fpath)
            if parent:
                os.makedirs(parent, exist_ok=True)
        else:
            self._save_seq += 1
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            fname = f"{prefix}_{ts}_seq{self._save_seq:03d}.pcd"
            fpath = os.path.join(self.save_dir, fname)

        try:
            xyz_f, rgb_f = _voxel_downsample(xyz, rgb, self.auto_save_voxel_size)
            _write_pcd(xyz_f, rgb_f, fpath, binary=self.pcd_binary)
            self.get_logger().info(
                f"[保存] #{self._save_seq}  {xyz.shape[0]}->{xyz_f.shape[0]} 点  {fpath}"
            )
            return True
        except Exception as e:
            self.get_logger().error(f"[保存] 失败: {e}")
            return False

    def save_on_shutdown(self):
        """节点关闭时保存（若配置了 session_pcd_path 则与 Mid360 一样写入单一路径）"""
        self._merge_cb()
        with self._lock:
            n = self._merged_xyz.shape[0]
        if n > 0:
            self.get_logger().info(f"[shutdown] 保存 {n} 点...")
            out = self.session_pcd_path if self.session_pcd_path else None
            self._do_save("shutdown", fixed_path=out)
        else:
            self.get_logger().info("[shutdown] 无累积点云，跳过保存")

    # ================================================================
    #  服务
    # ================================================================
    def _srv_start(self, req, resp):
        self.is_recording = True
        resp.success = True
        resp.message = "开始记录"
        self.get_logger().info("开始记录点云")
        return resp

    def _srv_stop(self, req, resp):
        self.is_recording = False
        with self._lock:
            cnt = self._merged_xyz.shape[0]
        resp.success = True
        resp.message = f"停止记录，累积 {cnt} 点"
        self.get_logger().info(resp.message)
        return resp

    def _srv_clear(self, req, resp):
        self._pending.clear()
        self._pending_rgb.clear()
        with self._lock:
            old = self._merged_xyz.shape[0]
            self._merged_xyz = np.empty((0, 3), dtype=_F32)
            if self.save_rgb:
                self._merged_rgb = np.empty((0, 3), dtype=_U8)
        self._frame_count = 0
        resp.success = True
        resp.message = f"已清空 {old} 点"
        self.get_logger().info(resp.message)
        return resp

    def _srv_save(self, req, resp):
        self._merge_cb()

        with self._lock:
            if self._merged_xyz.shape[0] == 0:
                resp.success = False
                resp.message = "没有累积的点云数据"
                resp.saved_path = ""
                resp.total_points = 0
                resp.filtered_points = 0
                return resp
            xyz = self._merged_xyz.copy()
            rgb = self._merged_rgb.copy() if self._merged_rgb is not None else None

        total = xyz.shape[0]
        vs = req.voxel_size if req.voxel_size > 0 else 0.05
        xyz_f, rgb_f = _voxel_downsample(xyz, rgb, vs)

        if req.apply_statistical_filter and xyz_f.shape[0] > 100:
            try:
                mask = _stat_filter_mask(xyz_f, self.max_stat_points)
                xyz_f = xyz_f[mask]
                if rgb_f is not None:
                    rgb_f = rgb_f[mask]
            except Exception as e:
                self.get_logger().warn(f"统计滤波失败: {e}")

        if req.filename:
            fname = req.filename if req.filename.endswith(".pcd") else req.filename + ".pcd"
        else:
            fname = f"map_{datetime.now().strftime('%Y%m%d_%H%M%S')}.pcd"
        fpath = os.path.join(self.save_dir, fname)

        try:
            _write_pcd(xyz_f, rgb_f, fpath, binary=self.pcd_binary)
            self.get_logger().info(f"保存成功: {fpath}  {total}->{xyz_f.shape[0]} 点")
            resp.success = True
            resp.message = "保存成功"
            resp.saved_path = fpath
            resp.total_points = total
            resp.filtered_points = xyz_f.shape[0]
        except Exception as e:
            self.get_logger().error(f"保存失败: {e}")
            resp.success = False
            resp.message = str(e)
            resp.saved_path = ""
            resp.total_points = 0
            resp.filtered_points = 0
        return resp

    # ================================================================
    #  点云解析（使用 structured dtype，安全可靠）
    # ================================================================
    @staticmethod
    def _parse_cloud(msg: PointCloud2):
        n = msg.width * msg.height
        if n == 0:
            return None, None

        field_map = {f.name: f for f in msg.fields}
        ps = msg.point_step

        if "x" not in field_map or "y" not in field_map or "z" not in field_map:
            return None, None

        ox = field_map["x"].offset
        oy = field_map["y"].offset
        oz = field_map["z"].offset

        dt_fields = {
            "names": ["x", "y", "z"],
            "formats": [_F32, _F32, _F32],
            "offsets": [ox, oy, oz],
            "itemsize": ps,
        }

        has_rgb = False
        rf = field_map.get("rgb") or field_map.get("rgba")
        if rf is not None:
            dt_fields["names"].append("rgb_packed")
            dt_fields["formats"].append(np.uint32)
            dt_fields["offsets"].append(rf.offset)
            has_rgb = True

        dt = np.dtype(dt_fields)

        raw = bytes(msg.data)
        expected = n * ps
        if len(raw) < expected:
            return None, None

        pts = np.frombuffer(raw, dtype=dt, count=n)

        x = pts["x"]
        y = pts["y"]
        z = pts["z"]

        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        xyz = np.column_stack([x[valid], y[valid], z[valid]])

        rgb = None
        if has_rgb:
            packed = pts["rgb_packed"][valid]
            r = ((packed >> 16) & 0xFF).astype(_U8)
            g = ((packed >> 8) & 0xFF).astype(_U8)
            b = (packed & 0xFF).astype(_U8)
            rgb = np.column_stack([r, g, b])

        return xyz, rgb


# ====================================================================
#  纯函数
# ====================================================================

def _quat_to_rot(qx, qy, qz, qw):
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ], dtype=np.float64)


def _voxel_downsample(xyz, rgb, voxel_size):
    if voxel_size <= 0 or xyz.shape[0] == 0:
        return xyz, rgb
    vox = np.floor(xyz / float(voxel_size)).astype(np.int64)
    _, inv, counts = np.unique(vox, axis=0, return_inverse=True, return_counts=True)
    cx = np.bincount(inv, weights=xyz[:, 0]) / counts
    cy = np.bincount(inv, weights=xyz[:, 1]) / counts
    cz = np.bincount(inv, weights=xyz[:, 2]) / counts
    xyz_out = np.column_stack([cx, cy, cz]).astype(_F32)

    rgb_out = None
    if rgb is not None and rgb.shape[0] == xyz.shape[0]:
        cr = np.bincount(inv, weights=rgb[:, 0].astype(np.float64)) / counts
        cg = np.bincount(inv, weights=rgb[:, 1].astype(np.float64)) / counts
        cb = np.bincount(inv, weights=rgb[:, 2].astype(np.float64)) / counts
        rgb_out = np.column_stack([cr, cg, cb]).astype(_U8)
    return xyz_out, rgb_out


def _stat_filter_mask(xyz, max_pts, k=50, std_ratio=1.0):
    n = xyz.shape[0]
    if n > max_pts:
        return np.ones(n, dtype=bool)
    from scipy.spatial import cKDTree
    tree = cKDTree(xyz)
    dists, _ = tree.query(xyz, k=min(k+1, n))
    md = np.mean(dists[:, 1:], axis=1)
    return md <= (np.mean(md) + std_ratio * np.std(md))


def _xyz_to_pc2(xyz, frame_id, stamp=None):
    n = xyz.shape[0]
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp.to_msg()
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * n
    msg.data = xyz.astype(_F32).tobytes()
    msg.is_dense = True
    return msg


def _write_pcd(xyz, rgb, filepath, binary=True):
    has_rgb = rgb is not None and rgb.shape[0] == xyz.shape[0]
    n = xyz.shape[0]

    if binary:
        with open(filepath, "wb") as f:
            hdr = "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
            if has_rgb:
                hdr += "FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n"
            else:
                hdr += "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            hdr += f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
            hdr += f"POINTS {n}\nDATA binary\n"
            f.write(hdr.encode("ascii"))
            if has_rgb:
                packed = (rgb[:, 0].astype(np.uint32) << 16) | \
                         (rgb[:, 1].astype(np.uint32) << 8) | \
                         rgb[:, 2].astype(np.uint32)
                block = np.empty((n, 4), dtype=_F32)
                block[:, :3] = xyz
                block[:, 3] = packed.view(_F32)
                f.write(block.tobytes())
            else:
                f.write(xyz.astype(_F32).tobytes())
    else:
        with open(filepath, "w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n")
            if has_rgb:
                f.write("FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n")
            else:
                f.write("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
            f.write(f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {n}\nDATA ascii\n")
            if has_rgb:
                packed = (rgb[:, 0].astype(np.uint32) << 16) | \
                         (rgb[:, 1].astype(np.uint32) << 8) | \
                         rgb[:, 2].astype(np.uint32)
                lines = [f"{xyz[i,0]:.6f} {xyz[i,1]:.6f} {xyz[i,2]:.6f} {packed[i]}\n"
                         for i in range(n)]
                f.writelines(lines)
            else:
                lines = [f"{xyz[i,0]:.6f} {xyz[i,1]:.6f} {xyz[i,2]:.6f}\n"
                         for i in range(n)]
                f.writelines(lines)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_on_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
