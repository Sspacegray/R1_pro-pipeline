#!/usr/bin/env python3
"""Deterministic random-initial-pose global relocalization benchmark."""
import argparse, csv, math, random, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from interface.srv import Relocalize


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
def angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi

class Eval(Node):
    def __init__(self, args):
        super().__init__('map_eval')
        self.args = args
        self.latest = None
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 10)
        self.client = self.create_client(Relocalize, args.service)
        self.reference = self.load_reference(args.reference)
    def odom_cb(self, msg):
        self.latest = msg
    @staticmethod
    def load_reference(path):
        rows = []
        if not path: return rows
        with open(path) as f:
            for r in csv.DictReader(f):
                rows.append((float(r['t']), float(r['x']), float(r['y']), float(r['z']),
                             float(r['qw']), float(r['qx']), float(r['qy']), float(r['qz'])))
        return rows
    def nearest_ref(self, stamp):
        if not self.reference: return None
        t = stamp.sec + stamp.nanosec * 1e-9
        return min(self.reference, key=lambda r: abs(r[0] - t))

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--pcd', required=True); p.add_argument('--reference', default='')
    p.add_argument('--odom-topic', default='/fastlio2/lio_odom'); p.add_argument('--service', default='/localizer/relocalize')
    p.add_argument('--trials', type=int, default=100); p.add_argument('--seed', type=int, default=42)
    p.add_argument('--num-candidates', type=int, default=128)
    p.add_argument('--xmin', type=float, default=-27.3); p.add_argument('--xmax', type=float, default=21.8)
    p.add_argument('--ymin', type=float, default=-40.2); p.add_argument('--ymax', type=float, default=18.9)
    p.add_argument('--zmin', type=float, default=-1.7); p.add_argument('--zmax', type=float, default=5.4)
    p.add_argument('--xy-thresh', type=float, default=0.5); p.add_argument('--yaw-thresh-deg', type=float, default=10.0)
    p.add_argument('--max-reference-gap', type=float, default=0.5)
    p.add_argument('--timeout', type=float, default=120.0)
    p.add_argument('--out', default='map_eval.csv')
    args = p.parse_args()
    rclpy.init(); node = Eval(args)
    if not node.client.wait_for_service(timeout_sec=30.0):
        raise RuntimeError('relocalization service not available')
    deadline = time.time() + 60
    while node.latest is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.latest is None: raise RuntimeError('no synchronized odometry received')

    rng = random.Random(args.seed)
    rows = []
    for i in range(args.trials):
        rclpy.spin_once(node, timeout_sec=0.05)
        msg = node.latest
        req = Relocalize.Request()
        req.pcd_path = args.pcd
        req.x = rng.uniform(args.xmin, args.xmax); req.y = rng.uniform(args.ymin, args.ymax)
        req.z = rng.uniform(args.zmin, args.zmax); req.yaw = rng.uniform(-math.pi, math.pi)
        req.pitch = 0.0; req.roll = 0.0
        req.global_search = True; req.num_candidates = args.num_candidates; req.seed = args.seed + i
        t0 = time.monotonic(); fut = node.client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=args.timeout)
        elapsed = time.monotonic() - t0
        res = fut.result() if fut.done() else None
        ref = node.nearest_ref(type('Stamp', (), {'sec': int(res.stamp), 'nanosec': int((res.stamp-int(res.stamp))*1e9)})()) if res and res.success else None
        xy = yaw = gap = float('nan')
        if res and ref:
            gap = abs(res.stamp - ref[0])
            xy = math.hypot(res.pose_x-ref[1], res.pose_y-ref[2])
            rq = type('Q', (), {'w':ref[4], 'x':ref[5], 'y':ref[6], 'z':ref[7]})()
            yaw = abs(math.degrees(angle_diff(res.pose_yaw, yaw_from_quat(rq))))
        ok = bool(res and res.success and res.method != 'kiss_matcher_coarse_unverified' and math.isfinite(res.score) and ref and gap <= args.max_reference_gap and xy <= args.xy_thresh and yaw <= args.yaw_thresh_deg)
        rows.append((i, req.x, req.y, req.z, req.yaw, bool(res and res.success), res.method if res else '', res.score if res else float('nan'), xy, yaw, gap, elapsed, ok))
        print(f'{i:03d} success={rows[-1][5]} method={rows[-1][6]} xy={xy:.3f} yaw={yaw:.2f} time={elapsed:.2f}s')
    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['trial','initial_x','initial_y','initial_z','initial_yaw','success','method','score','xy_error','yaw_error_deg','reference_gap_s','time_s','accepted']); w.writerows(rows)
    accepted = sum(r[-1] for r in rows)
    print(f'summary trials={len(rows)} accepted={accepted} success_rate={accepted/max(1,len(rows)):.4f}')
    node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
