"""Interactive, read-only calibration of passive-hook effort thresholds.

The program never commands the robot.  The operator manually performs the
small test lift while this node records JointState effort feedback from both
arms.  It writes a YAML report containing the raw trial statistics and a
recommended ``hook_min_effort_delta`` for the passive-hook workflow.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np


CONDITIONS: tuple[tuple[str, str], ...] = (
    ("no_load", "不挂料盆，只保留左右钩子"),
    ("empty_tray", "挂空料盆"),
    ("half_load", "挂半载料盆"),
    ("full_load", "挂满载料盆"),
)


def _finite_vector(values: Any) -> list[float]:
    vector = [float(value) for value in values]
    if not vector or not all(math.isfinite(value) for value in vector):
        raise ValueError("effort 必须是非空有限数数组")
    return vector


def median_effort(samples: list[list[float]]) -> list[float]:
    """Return a per-joint median, using the common joint count."""

    if not samples:
        raise ValueError("没有采集到 effort 样本")
    count = min(len(sample) for sample in samples)
    if count <= 0:
        raise ValueError("effort 样本没有有效关节")
    matrix = np.asarray([sample[:count] for sample in samples], dtype=np.float64)
    result = np.median(matrix, axis=0).tolist()
    return [float(value) for value in result]


def effort_delta(before: dict[str, list[float]], after: dict[str, list[float]]) -> dict[str, Any]:
    """Calculate per-arm and combined max absolute effort changes."""

    per_arm: dict[str, float] = {}
    all_deltas: list[float] = []
    for arm in ("left", "right"):
        previous = before.get(arm) or []
        current = after.get(arm) or []
        if not previous or not current:
            raise ValueError(f"{arm} 臂缺少 before/after effort")
        count = min(len(previous), len(current))
        deltas = [abs(float(current[i]) - float(previous[i])) for i in range(count)]
        if not deltas or not all(math.isfinite(value) for value in deltas):
            raise ValueError(f"{arm} 臂 effort 差值无效")
        per_arm[arm] = float(max(deltas))
        all_deltas.extend(deltas)
    return {
        "left": round(per_arm["left"], 6),
        "right": round(per_arm["right"], 6),
        "combined_max": round(max(all_deltas), 6),
    }


def recommend_threshold(no_load: list[float], loaded: list[float]) -> dict[str, float]:
    """Select a conservative threshold between unloaded and loaded trials."""

    if len(no_load) < 2 or len(loaded) < 2:
        raise ValueError("未挂载和挂载样本都至少需要 2 次")
    no_p95 = float(np.percentile(np.asarray(no_load, dtype=np.float64), 95))
    loaded_p05 = float(np.percentile(np.asarray(loaded, dtype=np.float64), 5))
    if loaded_p05 <= no_p95:
        raise ValueError(
            "未挂载与挂载 effort 差值没有形成安全分离："
            f" no_load_p95={no_p95:.6f}, loaded_p05={loaded_p05:.6f}。"
            "请检查姿态、试抬升高度、负载或重新采样。"
        )
    # Keep a margin on both sides rather than using either observed extreme.
    threshold = no_p95 + 0.5 * (loaded_p05 - no_p95)
    return {
        "no_load_p95": round(no_p95, 6),
        "loaded_p05": round(loaded_p05, 6),
        "recommended_hook_min_effort_delta": round(float(threshold), 6),
    }


class EffortRecorder:
    """ROS subscriptions plus a thread-safe short history for both arms."""

    def __init__(self, node: Any, left_topic: str, right_topic: str, history_s: float) -> None:
        from sensor_msgs.msg import JointState
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        self._lock = threading.Lock()
        self._history_s = max(10.0, float(history_s))
        self._samples: dict[str, deque[tuple[float, list[float]]]] = {
            "left": deque(),
            "right": deque(),
        }
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self._subscriptions = [
            node.create_subscription(
                JointState,
                left_topic,
                lambda msg: self._on_message("left", msg),
                qos,
            ),
            node.create_subscription(
                JointState,
                right_topic,
                lambda msg: self._on_message("right", msg),
                qos,
            ),
        ]

    def _on_message(self, arm: str, msg: Any) -> None:
        try:
            effort = _finite_vector(getattr(msg, "effort", []))
        except (TypeError, ValueError):
            return
        now = time.monotonic()
        with self._lock:
            history = self._samples[arm]
            history.append((now, effort))
            cutoff = now - self._history_s
            while history and history[0][0] < cutoff:
                history.popleft()

    def wait_for_feedback(self, timeout_s: float) -> None:
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        while time.monotonic() < deadline:
            with self._lock:
                if self._samples["left"] and self._samples["right"]:
                    return
            time.sleep(0.05)
        raise RuntimeError("未同时收到左右臂 effort 反馈，请检查 ROS topic 和 DDS")

    def capture(self, window_s: float) -> tuple[dict[str, list[float]], dict[str, int]]:
        self.wait_for_feedback(max(2.0, float(window_s) + 1.0))
        start = time.monotonic()
        deadline = start + max(0.5, float(window_s))
        while time.monotonic() < deadline:
            time.sleep(0.02)
        with self._lock:
            selected: dict[str, list[list[float]]] = {"left": [], "right": []}
            for arm in ("left", "right"):
                selected[arm] = [effort for stamp, effort in self._samples[arm] if stamp >= start]
        medians = {arm: median_effort(samples) for arm, samples in selected.items()}
        counts = {arm: len(samples) for arm, samples in selected.items()}
        return medians, counts


def _prompt(message: str) -> None:
    print("\n" + message)
    input("准备好后按 Enter 继续，输入 Ctrl+C 可随时退出：")


def _yaml_dump(payload: dict[str, Any]) -> str:
    try:
        import yaml
    except ImportError:
        # JSON is valid YAML 1.2 and keeps the program usable in minimal ROS images.
        return json.dumps(payload, ensure_ascii=False, indent=2) + "\n"
    return yaml.safe_dump(payload, allow_unicode=True, sort_keys=False)


def run_calibration(args: argparse.Namespace) -> int:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    output = Path(args.output).expanduser()
    if output.exists() and not args.overwrite:
        answer = input(f"输出文件已存在：{output}，覆盖请输入 yes：").strip().lower()
        if answer != "yes":
            print("已取消，不覆盖原文件。")
            return 2
    output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node("r1pro_hook_effort_calibration")
    recorder = EffortRecorder(node, args.left_topic, args.right_topic, args.history_s)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("钩子 effort 阈值标定（只读模式）")
    print("本程序不会发送机械臂、夹爪、底盘或导航指令。")
    print(f"左臂 topic：{args.left_topic}")
    print(f"右臂 topic：{args.right_topic}")
    print(f"每次采样窗口：{args.window_s:.2f} s；每种工况重复：{args.repeats} 次")
    print("执行试抬升时请低速、保持 10～20 mm，并准备好急停。")

    trials: dict[str, list[dict[str, Any]]] = {key: [] for key, _ in CONDITIONS}
    try:
        recorder.wait_for_feedback(args.feedback_timeout_s)
        for condition_key, condition_label in CONDITIONS:
            _prompt(
                f"工况：{condition_label}\n"
                "请把机器人放到统一的预抬升姿态，保持机械臂和底盘静止。"
            )
            for repeat in range(1, args.repeats + 1):
                print(f"\n[{condition_key}] 第 {repeat}/{args.repeats} 次")
                print("即将采集抬升前 effort，请保持未抬升状态。")
                before, before_counts = recorder.capture(args.window_s)
                print("抬升前采样完成。现在手动执行低速试抬升 10～20 mm 并保持。")
                input("保持在抬升后位置后按 Enter，程序开始采集抬升后 effort：")
                after, after_counts = recorder.capture(args.window_s)
                delta = effort_delta(before, after)
                trial = {
                    "repeat": repeat,
                    "before": before,
                    "after": after,
                    "delta": delta,
                    "sample_count_before": before_counts,
                    "sample_count_after": after_counts,
                }
                trials[condition_key].append(trial)
                print(
                    "本次差值："
                    f" left={delta['left']:.6f}, right={delta['right']:.6f}, "
                    f"combined_max={delta['combined_max']:.6f}"
                )
                if repeat < args.repeats:
                    input("请将机器人恢复到下一次试验的预抬升姿态后按 Enter：")

        no_load = [float(item["delta"]["combined_max"]) for item in trials["no_load"]]
        loaded = [
            float(item["delta"]["combined_max"])
            for key in ("empty_tray", "half_load", "full_load")
            for item in trials[key]
        ]
        recommendation = recommend_threshold(no_load, loaded)
        payload: dict[str, Any] = {
            "hook_effort_calibration": {
                "created_at": datetime.now(timezone.utc).isoformat(),
                "left_topic": args.left_topic,
                "right_topic": args.right_topic,
                "window_s": float(args.window_s),
                "repeats_per_condition": int(args.repeats),
                "conditions": dict(CONDITIONS),
                "trials": trials,
                "statistics": recommendation,
                "workflow_params": {
                    "hook_feedback_mode": "effort_delta",
                    "hook_min_effort_delta": recommendation[
                        "recommended_hook_min_effort_delta"
                    ],
                    "hook_feedback_timeout_s": max(1.0, float(args.window_s)),
                },
            }
        }
        output.write_text(_yaml_dump(payload), encoding="utf-8")
        print("\n标定完成。")
        print(f"报告已写入：{output}")
        print(
            "推荐写入 workflow 的参数：\n"
            f"  hook_feedback_mode: effort_delta\n"
            f"  hook_min_effort_delta: {recommendation['recommended_hook_min_effort_delta']}\n"
            f"  hook_feedback_timeout_s: {max(1.0, float(args.window_s))}"
        )
        return 0
    except KeyboardInterrupt:
        print("\n用户中断，未写入不完整标定结果。")
        return 130
    except (RuntimeError, ValueError) as exc:
        print(f"\n标定失败：{exc}")
        return 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="只读采集双臂钩子 effort 阈值")
    parser.add_argument(
        "--left-topic",
        default="/hdas/feedback_arm_left",
        help="左臂 sensor_msgs/JointState topic",
    )
    parser.add_argument(
        "--right-topic",
        default="/hdas/feedback_arm_right",
        help="右臂 sensor_msgs/JointState topic",
    )
    parser.add_argument("--repeats", type=int, default=5, help="每种工况重复次数")
    parser.add_argument("--window-s", type=float, default=2.0, help="每次 effort 采样窗口")
    parser.add_argument(
        "--feedback-timeout-s", type=float, default=10.0, help="等待左右臂反馈的超时时间"
    )
    parser.add_argument("--history-s", type=float, default=30.0, help="内存反馈历史保留时间")
    parser.add_argument(
        "--output",
        default="runtime_data/r1pro_web_control/hook_effort_calibration.yaml",
        help="标定报告输出路径",
    )
    parser.add_argument("--overwrite", action="store_true", help="允许覆盖已有输出文件")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.repeats < 2:
        raise SystemExit("--repeats 至少为 2")
    if args.window_s <= 0.0:
        raise SystemExit("--window-s 必须大于 0")
    raise SystemExit(run_calibration(args))


if __name__ == "__main__":
    main()

