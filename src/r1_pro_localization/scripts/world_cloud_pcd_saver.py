#!/usr/bin/env python3
"""world_cloud PCD 地图累积器.

订阅 fastlio2 的 /fastlio2/world_cloud (PointXYZRGBA/XYZI格式), 累积到体素滤波地图,
save_map 服务(std_srvs/SetBool) 或 SIGINT 时保存 PCD (binary, XYZINormal 兼容 indoor_location).

用法:
  ros2 run r1_pro_localization world_cloud_pcd_saver.py
  # 推着机器人走完一圈后:
  ros2 service call /save_map std_srvs/srv/SetBool '{data: true}'
"""
import os
import signal
import sys
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_srvs.srv import SetBool

from r1_pro_localization.version import __version__  # noqa: F401


class WorldCloudPcdSaver(Node):
    def __init__(self):
        super().__init__('world_cloud_pcd_saver')
        self.declare_parameter('cloud_topic', '/fastlio2/world_cloud')
        self.declare_parameter('map_path', os.path.expanduser(
            '~/galaxea/maps/pcd/YYYYmmdd_HHMMSS.pcd'))
        # map_path 含 'YYYYmmdd_HHMMSS' 时替换为启动时间戳(utc+8)
        raw_path = self.get_parameter('map_path').value
        if 'YYYYmmdd_HHMMSS' in raw_path:
            raw_path = raw_path.replace(
                'YYYYmmdd_HHMMSS',
                datetime.now().strftime('%Y%m%d_%H%M%S'))
        self.map_path = raw_path
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('save_every_sec', 0.0)  # 0=仅手动

        self.map_path = self.get_parameter('map_path').value
        self.voxel = float(self.get_parameter('voxel_size').value)
        save_every = float(self.get_parameter('save_every_sec').value)

        # 用 numpy 哈希体素, 不依赖 python-pcl
        self.voxel_map = {}          # voxel key -> np.array([x,y,z,i,nx,ny,nz])
        self.count = 0

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2,
                                 self.get_parameter('cloud_topic').value,
                                 self.cb, qos)
        self.create_service(SetBool, 'save_map', self.save_cb)
        if save_every > 0:
            self.create_timer(save_every, lambda: self._save('periodic'))
        self.get_logger().info(
            f'accumulating {self.get_parameter("cloud_topic").value} '
            f'-> {self.map_path} (voxel={self.voxel})')

    def cb(self, msg: PointCloud2):
        pts = pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'),
                              skip_nans=True)
        arr = np.fromiter((p for p in pts),
                          dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
                                 ('intensity', 'f4')])
        if arr.size == 0:
            return
        vmap = self.voxel_map
        inv = 1.0 / self.voxel
        for x, y, z, inten in arr:
            key = (int(np.floor(x * inv)), int(np.floor(y * inv)),
                   int(np.floor(z * inv)))
            cur = vmap.get(key)
            if cur is None:
                vmap[key] = np.array([x, y, z, inten, 0, 0, 0], dtype='f4')
            else:  # 均值融合
                cur[:4] = (cur[:4] * 127 + np.array([x, y, z, inten],
                                                    dtype='f4')) / 128
        self.count += 1
        if self.count % 50 == 0:
            self.get_logger().info(
                f'frames={self.count} voxels={len(vmap)}')

    def _save(self, why: str):
        if not self.voxel_map:
            self.get_logger().warning('empty map, skip save')
            return False
        os.makedirs(os.path.dirname(self.map_path), exist_ok=True)
        data = np.asarray(list(self.voxel_map.values()), dtype='f4')
        data = data[np.isfinite(data).all(axis=1)]
        # PCD binary: XYZI4f + normal4f (PointXYZINormal 内存布局)
        n = data.shape[0]
        pts = np.zeros((n, 8), dtype='f4')
        pts[:, :4] = data[:, :4]
        pts[:, 5:8] = 0.0  # nx,ny,nz=0; curv 位留0
        # XYZINormal 布局: x y z intensity normal_x normal_y normal_z curvature
        header = (
            '# .PCD v0.7 - Point Cloud Data file format\n'
            'VERSION 0.7\nFIELDS x y z intensity normal_x normal_y normal_z '
            'curvature\nSIZE 4 4 4 4 4 4 4 4\nTYPE F F F F F F F F\nCOUNT '
            '1 1 1 1 1 1 1 1\n'
            f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\n'
            'DATA binary\n')
        tmp = self.map_path + '.tmp'
        with open(tmp, 'wb') as f:
            f.write(header.encode())
            f.write(pts.tobytes())
        os.replace(tmp, self.map_path)
        self.get_logger().info(
            f'saved ({why}): {self.map_path} points={n}')
        return True

    def save_cb(self, req, resp):
        resp.success = self._save('service')
        resp.message = f'{len(self.voxel_map)} voxels'
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = WorldCloudPcdSaver()
    # Ctrl-C / tmux kill 时也保存
    def _sig(signum, frame):
        node._save('signal')
        rclpy.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)
    try:
        rclpy.spin(node)
    finally:
        node._save('shutdown')


if __name__ == '__main__':
    main()
