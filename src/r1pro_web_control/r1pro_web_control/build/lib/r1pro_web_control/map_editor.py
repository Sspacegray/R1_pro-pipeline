#!/usr/bin/env python3

"""Standalone browser editor for ROS map_server image/YAML pairs."""

from __future__ import annotations

import argparse
import json
import math
import re
import threading
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import cv2
import yaml


OUTPUT_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")


def load_map(yaml_path: Path) -> tuple[dict[str, Any], Path, Any]:
    yaml_path = yaml_path.expanduser().resolve()
    if not yaml_path.is_file():
        raise FileNotFoundError(f"地图 YAML 不存在: {yaml_path}")
    with yaml_path.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream) or {}
    if not isinstance(metadata, dict):
        raise ValueError("地图 YAML 顶层必须是对象")
    required = ("image", "resolution", "origin")
    missing = [key for key in required if key not in metadata]
    if missing:
        raise ValueError(f"地图 YAML 缺少字段: {', '.join(missing)}")

    image_path = Path(str(metadata["image"])).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve()
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"无法读取地图图像: {image_path}")

    resolution = float(metadata["resolution"])
    origin = metadata["origin"]
    if resolution <= 0 or not isinstance(origin, list) or len(origin) < 3:
        raise ValueError("resolution 必须大于 0，origin 必须包含 [x, y, yaw]")
    metadata["resolution"] = resolution
    metadata["origin"] = [float(origin[0]), float(origin[1]), float(origin[2])]
    return metadata, image_path, image


def cropped_origin(
    origin: list[float], resolution: float, image_height: int, crop: dict[str, int]
) -> tuple[float, float]:
    """Return the world position of a top-left-referenced crop's lower-left cell."""
    local_x = crop["x"] * resolution
    local_y = (image_height - crop["y"] - crop["height"]) * resolution
    yaw = origin[2]
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        origin[0] + cos_yaw * local_x - sin_yaw * local_y,
        origin[1] + sin_yaw * local_x + cos_yaw * local_y,
    )


def validate_crop(value: Any, width: int, height: int) -> dict[str, int]:
    if not isinstance(value, dict):
        raise ValueError("crop 必须是对象")
    try:
        crop = {key: int(value[key]) for key in ("x", "y", "width", "height")}
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("裁剪参数必须是整数") from exc
    if crop["x"] < 0 or crop["y"] < 0 or crop["width"] <= 0 or crop["height"] <= 0:
        raise ValueError("裁剪区域超出地图或尺寸为零")
    if crop["x"] + crop["width"] > width or crop["y"] + crop["height"] > height:
        raise ValueError("裁剪区域超出地图")
    return crop


def yaml_text(metadata: dict[str, Any], image_name: str, origin: list[float]) -> str:
    output = dict(metadata)
    output["image"] = image_name
    output["origin"] = [float(value) for value in origin]
    preferred = [
        "image",
        "mode",
        "resolution",
        "origin",
        "negate",
        "occupied_thresh",
        "free_thresh",
    ]
    ordered = {key: output.pop(key) for key in preferred if key in output}
    ordered.update(output)
    return yaml.safe_dump(ordered, sort_keys=False, default_flow_style=None, allow_unicode=True)


class MapEditorApp:
    def __init__(self, yaml_path: Path, web_root: Path, output_dir: Path | None = None):
        self.yaml_path = yaml_path.expanduser().resolve()
        self.metadata, self.image_path, self.image = load_map(self.yaml_path)
        self.web_root = web_root.resolve()
        self.output_dir = (output_dir or self.yaml_path.parent).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)

    @property
    def width(self) -> int:
        return int(self.image.shape[1])

    @property
    def height(self) -> int:
        return int(self.image.shape[0])

    def info(self) -> dict[str, Any]:
        origin = self.metadata["origin"]
        return {
            "yaml_name": self.yaml_path.name,
            "image_name": self.image_path.name,
            "width": self.width,
            "height": self.height,
            "resolution": self.metadata["resolution"],
            "origin": origin,
            "origin_yaw_deg": math.degrees(origin[2]),
            "mode": self.metadata.get("mode", "trinary"),
            "output_dir": str(self.output_dir),
        }

    def image_png(self) -> bytes:
        ok, encoded = cv2.imencode(".png", self.image)
        if not ok:
            raise RuntimeError("地图图像 PNG 编码失败")
        return encoded.tobytes()

    def export(self, payload: Any) -> dict[str, Any]:
        if not isinstance(payload, dict):
            raise ValueError("请求内容必须是对象")
        crop = validate_crop(payload.get("crop"), self.width, self.height)
        name = str(payload.get("name", "")).strip()
        if name.lower().endswith((".yaml", ".yml", ".pgm", ".png")):
            name = name.rsplit(".", 1)[0]
        if not OUTPUT_NAME_RE.fullmatch(name) or name in {".", ".."}:
            raise ValueError("输出名称只能包含字母、数字、点、短横线和下划线")

        auto_x, auto_y = cropped_origin(
            self.metadata["origin"], self.metadata["resolution"], self.height, crop
        )
        try:
            yaw = math.radians(float(payload["yaw_deg"]))
            output_origin = payload.get("origin")
            if output_origin is None:
                origin_x, origin_y = auto_x, auto_y
            else:
                origin_x, origin_y = float(output_origin[0]), float(output_origin[1])
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            raise ValueError("原点和旋转角度必须是有效数字") from exc
        if not all(math.isfinite(value) for value in (origin_x, origin_y, yaw)):
            raise ValueError("原点和旋转角度必须是有限数字")
        origin_x = round(origin_x, 12)
        origin_y = round(origin_y, 12)
        yaw = round(yaw, 12)

        yaml_path = self.output_dir / f"{name}.yaml"
        pgm_path = self.output_dir / f"{name}.pgm"
        png_path = self.output_dir / f"{name}.png"
        overwrite = bool(payload.get("overwrite", False))
        existing = [path.name for path in (yaml_path, pgm_path, png_path) if path.exists()]
        if existing and not overwrite:
            raise FileExistsError(f"文件已存在: {', '.join(existing)}")

        x, y, width, height = (crop[key] for key in ("x", "y", "width", "height"))
        cropped = self.image[y : y + height, x : x + width]
        if not cv2.imwrite(str(pgm_path), cropped):
            raise RuntimeError(f"写入失败: {pgm_path}")
        if not cv2.imwrite(str(png_path), cropped):
            pgm_path.unlink(missing_ok=True)
            raise RuntimeError(f"写入失败: {png_path}")
        origin = [origin_x, origin_y, yaw]
        try:
            yaml_path.write_text(
                yaml_text(self.metadata, pgm_path.name, origin), encoding="utf-8"
            )
        except Exception:
            pgm_path.unlink(missing_ok=True)
            png_path.unlink(missing_ok=True)
            raise
        return {
            "message": "地图导出成功",
            "files": [str(yaml_path), str(pgm_path), str(png_path)],
            "size": [width, height],
            "origin": origin,
        }


def find_web_root() -> Path:
    source_root = Path(__file__).resolve().parent.parent / "web" / "map_editor"
    if source_root.is_dir():
        return source_root
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory("r1pro_web_control")) / "web" / "map_editor"
        if installed.is_dir():
            return installed
    except (ImportError, LookupError):
        pass
    raise FileNotFoundError("找不到 map_editor Web 资源，请重新构建 r1pro_web_control")


def make_handler(app: MapEditorApp) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            path = urlparse(self.path).path
            try:
                if path == "/api/map":
                    return self._send_json(HTTPStatus.OK, app.info())
                if path == "/api/image":
                    return self._send(HTTPStatus.OK, app.image_png(), "image/png", no_cache=True)
                if path == "/":
                    path = "/index.html"
                return self._static(path)
            except Exception as exc:  # pragma: no cover - unexpected I/O failure
                return self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": str(exc)})

        def do_POST(self) -> None:
            if urlparse(self.path).path != "/api/export":
                return self._send_json(HTTPStatus.NOT_FOUND, {"error": "接口不存在"})
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 64 * 1024:
                    raise ValueError("请求内容为空或过大")
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                result = app.export(payload)
                return self._send_json(HTTPStatus.OK, result)
            except FileExistsError as exc:
                return self._send_json(HTTPStatus.CONFLICT, {"error": str(exc)})
            except (ValueError, json.JSONDecodeError) as exc:
                return self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            except Exception as exc:  # pragma: no cover - unexpected I/O failure
                return self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": str(exc)})

        def _static(self, request_path: str) -> None:
            relative = request_path.lstrip("/")
            path = (app.web_root / relative).resolve()
            if app.web_root not in path.parents or not path.is_file():
                return self._send_json(HTTPStatus.NOT_FOUND, {"error": "文件不存在"})
            mime = {
                ".html": "text/html; charset=utf-8",
                ".css": "text/css; charset=utf-8",
                ".js": "text/javascript; charset=utf-8",
                ".svg": "image/svg+xml",
            }.get(path.suffix.lower(), "application/octet-stream")
            return self._send(HTTPStatus.OK, path.read_bytes(), mime)

        def _send_json(self, status: HTTPStatus, value: Any) -> None:
            data = json.dumps(value, ensure_ascii=False).encode("utf-8")
            self._send(status, data, "application/json; charset=utf-8", no_cache=True)

        def _send(
            self, status: HTTPStatus, data: bytes, mime: str, no_cache: bool = False
        ) -> None:
            self.send_response(status)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(data)))
            if no_cache:
                self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, format: str, *args: Any) -> None:
            print(f"[map-editor] {self.address_string()} - {format % args}")

    return Handler


def parse_args() -> argparse.Namespace:
    default_yaml = Path.cwd() / "mx_map.yaml"
    parser = argparse.ArgumentParser(description="ROS 地图可视化裁剪与坐标系旋转工具")
    parser.add_argument("map_yaml", nargs="?", default=str(default_yaml), help="输入地图 YAML")
    parser.add_argument("--output-dir", help="输出目录，默认与输入 YAML 相同")
    parser.add_argument("--host", default="127.0.0.1", help="监听地址，默认 127.0.0.1")
    parser.add_argument("--port", type=int, default=8091, help="监听端口，默认 8091")
    parser.add_argument("--no-browser", action="store_true", help="不自动打开浏览器")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    app = MapEditorApp(
        Path(args.map_yaml),
        find_web_root(),
        Path(args.output_dir) if args.output_dir else None,
    )
    server = ThreadingHTTPServer((args.host, args.port), make_handler(app))
    shown_host = "127.0.0.1" if args.host in {"0.0.0.0", "::"} else args.host
    url = f"http://{shown_host}:{server.server_port}/"
    print(f"地图: {app.yaml_path} ({app.width} x {app.height})")
    print(f"输出目录: {app.output_dir}")
    print(f"打开: {url}  （Ctrl+C 停止）")
    if not args.no_browser:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n地图编辑器已停止")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
