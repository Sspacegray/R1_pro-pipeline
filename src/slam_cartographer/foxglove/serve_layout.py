#!/usr/bin/env python3
"""
启动一个超轻量 HTTP 服务，把 foxglove/*.json 通过 HTTP 暴露出去，
这样 Foxglove 浏览器/桌面客户端可以通过 URL 参数自动加载布局。

默认：
  - Foxglove WebSocket: ws://<机器人IP>:8766
  - 布局文件 URL:      http://<机器人IP>:8767/mapping.json
"""

import argparse
import http.server
import os
import socket
import socketserver
import sys

PKG_SHARE = '/home/nvidia/galaxea/galaxea_overlay/install/slam_cartographer/share/slam_cartographer/foxglove'
SRC_SHARE = '/home/nvidia/galaxea/galaxea_overlay/src/slam_cartographer/foxglove'

# 优先用 install 里的；如果没编译，回退到 src
SERVE_DIR = PKG_SHARE if os.path.isdir(PKG_SHARE) else SRC_SHARE


class CORSHandler(http.server.SimpleHTTPRequestHandler):
    """加 CORS 头，让 Foxglove 能跨域拉布局文件"""

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', '*')
        super().end_headers()

    def log_message(self, fmt, *args):
        # 简化日志
        sys.stdout.write(f'[layout-server] {self.address_string()} - {fmt % args}\n')
        sys.stdout.flush()


def get_lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'
    finally:
        s.close()


def parse_args():
    parser = argparse.ArgumentParser(description='Serve Foxglove layouts over HTTP.')
    parser.add_argument('--port', type=int, default=8767, help='HTTP port for layout files.')
    parser.add_argument(
        '--bridge-port',
        type=int,
        default=8766,
        help='Foxglove bridge websocket port shown in helper URLs.',
    )
    parser.add_argument(
        '--default-layout',
        default='mapping.json',
        help='Layout JSON used in the helper URL printed at startup.',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    os.chdir(SERVE_DIR)
    default_layout = args.default_layout
    if not default_layout.endswith('.json') or not os.path.exists(default_layout):
        print(f"[layout-server] default layout '{default_layout}' not found, using mapping.json")
        default_layout = 'mapping.json'
    ip = get_lan_ip()
    handler = CORSHandler
    with socketserver.TCPServer(('0.0.0.0', args.port), handler) as httpd:
        print('=' * 60)
        print(f'Foxglove 布局服务运行在: http://{ip}:{args.port}/')
        print('')
        print('可用布局:')
        for f in sorted(os.listdir(SERVE_DIR)):
            if f.endswith('.json'):
                print(f'  http://{ip}:{args.port}/{f}')
        print('')
        print('在 Foxglove 里:')
        print(f'  Layout → Import from URL → http://{ip}:{args.port}/{default_layout}')
        print('')
        print('或者直接打开下面这条 URL，自动连数据+加载布局:')
        print(f'  https://app.foxglove.dev/?ds=foxglove-websocket'
              f'&ds.url=ws://{ip}:{args.bridge_port}'
              f'&layoutUrl=http://{ip}:{args.port}/{default_layout}')
        print('=' * 60)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print('\n关闭')


if __name__ == '__main__':
    main()
