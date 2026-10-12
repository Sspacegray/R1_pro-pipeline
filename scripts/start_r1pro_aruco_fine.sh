#!/usr/bin/env bash
set -eo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
CONTAINER_NAME="${R1PRO_CONTAINER:-ros2_humble_gpu}"
if [[ "${1:-}" != "--inside-container" ]]; then
  docker start "$CONTAINER_NAME" >/dev/null
  docker_tty=()
  if [[ -t 0 && -t 1 ]]; then
    docker_tty=(-it)
  fi
  exec docker exec "${docker_tty[@]}" "$CONTAINER_NAME" bash "$SCRIPT_PATH" --inside-container "$@"
fi
shift

python3 - "$@" <<'PY'
import argparse
import fcntl
import json
from pathlib import Path
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

parser = argparse.ArgumentParser(description='通过 Web 后端运行 demo 的 ArUco 离散微调')
parser.add_argument('--dry-run', action='store_true', help='读取位姿并计算动作，不移动底盘')
parser.add_argument('--stop', action='store_true', help='停止当前 ArUco 离散微调')
args = parser.parse_args()
base = 'http://127.0.0.1:8080'
workflow_id = 'aruco_id1_chassis_fine_adjust'
token_path = Path('/root/.ros/r1pro_web_control/token')
lock = Path('/tmp/r1pro_aruco_discrete.lock').open('w')
if not args.stop:
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('已有命令行 ArUco 离散微调正在运行')

def request(path, payload=None):
    headers = {}
    data = None
    if payload is not None:
        if not token_path.is_file():
            raise RuntimeError(f'找不到 Web 控制令牌：{token_path}')
        headers = {'Content-Type': 'application/json', 'X-Control-Token': token_path.read_text().strip()}
        data = json.dumps(payload).encode()
    req = Request(base + path, data=data, headers=headers)
    try:
        with urlopen(req, timeout=20) as response:
            result = json.load(response)
    except HTTPError as exc:
        try:
            detail = json.load(exc).get('message', str(exc))
        except (ValueError, AttributeError):
            detail = str(exc)
        raise RuntimeError(detail) from exc
    except URLError as exc:
        raise RuntimeError('Web 后端不可用；请先启动 scripts/start_r1pro_web_real.sh') from exc
    if result.get('ok') is False:
        raise RuntimeError(result.get('message', 'Web 请求失败'))
    return result

def stop_owned_workflow():
    runtime = request('/api/state').get('code_workflow_runtime', {})
    if runtime.get('running') and runtime.get('action_name') == workflow_id:
        print(request('/api/code_workflows/stop', {})['message'], flush=True)

started = False
try:
    state = request('/api/state')
    if args.stop:
        stop_owned_workflow()
        sys.exit(0)
    if state.get('active_control_profile') != 'real':
        raise RuntimeError('当前 Web 不是 real 模式，请先选择真机控制模式')
    for key in ('sequence_runtime', 'action_runtime', 'code_workflow_runtime'):
        if state.get(key, {}).get('running'):
            raise RuntimeError('Web 正在执行其他动作；请等待完成后再启动离散微调')
    print('运行 Web demo 的 ArUco 离散微调：目标 x=0.60 m、y=0、yaw=-90°', flush=True)
    print('只计算动作' if args.dry_run else '将交接底盘控制权、确认停稳后执行真实微调；Ctrl+C 停止', flush=True)
    result = request('/api/code_workflows/run', {
        'id': workflow_id,
        'params': {
            'dry_run': args.dry_run,
            'motion_confirmed': not args.dry_run,
            'disable_continuous_fine': True,
            'start_aruco_pose': True,
        },
    })
    started = True
    print(result['message'], flush=True)
    seen = 0
    deadline = time.monotonic() + 300
    while time.monotonic() < deadline:
        runtime = request('/api/state').get('code_workflow_runtime', {})
        if runtime.get('action_name') != workflow_id:
            raise RuntimeError('Web 当前工作流发生变化，请检查 Web 执行状态')
        lines = runtime.get('log', [])
        for line in lines[seen:]:
            print(line, flush=True)
        seen = len(lines)
        if not runtime.get('running'):
            message = runtime.get('message', '')
            if message != 'complete':
                raise RuntimeError(message or '离散微调未完成')
            started = False
            print('离散微调完成' if not args.dry_run else '动作计算完成，底盘未移动', flush=True)
            break
        time.sleep(0.5)
    else:
        stop_owned_workflow()
        raise RuntimeError('离散微调等待超时，已发送停止请求')
except KeyboardInterrupt:
    stop_owned_workflow()
    sys.exit(130)
except Exception as exc:
    if started:
        try:
            stop_owned_workflow()
        except Exception as stop_error:
            print(f'停止请求失败：{stop_error}', file=sys.stderr)
    print(f'错误：{exc}', file=sys.stderr)
    sys.exit(1)
PY
