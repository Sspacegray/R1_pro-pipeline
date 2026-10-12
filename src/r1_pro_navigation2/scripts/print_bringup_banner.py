#!/usr/bin/env python3
"""
Print the R1 Pro navigation bringup banner.

启动 bringup 时打印一段简洁的提示：

  - 关键诊断节点（保留在终端）
  - 详细日志路径（Livox / nav2 等大户被重定向到这里）
  - 常用排查命令

避免 nav2 + livox 同时刷屏让用户看不到前面的报错。
"""

import argparse
import os
import sys


BANNER = """\
================================================================================
  R1-Pro Nav2 bringup
--------------------------------------------------------------------------------
  详细日志目录   : {log_dir}
  终端只显示     : wait_for_odom / odom_watchdog / localization 桥 / lifecycle / 错误
  详细输出（含 nav2 server / livox / chassis）已重定向到上面目录
--------------------------------------------------------------------------------
  想看实时输出   : tail -F {log_dir}/launch.log
  想看某个节点   : ls {log_dir} && tail -F {log_dir}/<node>-stdout.log
  /odom 没出     : 看 odom_watchdog 的告警，保持静止重启或检查 localization 链路
================================================================================
"""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--log-dir', required=True)
    args = ap.parse_args()
    log_dir = os.path.abspath(args.log_dir)
    sys.stdout.write(BANNER.format(log_dir=log_dir))
    sys.stdout.flush()
    return 0


if __name__ == '__main__':
    sys.exit(main())
