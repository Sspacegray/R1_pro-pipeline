from __future__ import annotations

from typing import Any


class Demo02PreloadHeadBottleYoloWorkflow:
    """Start demo02 perception early without running YOLO inference."""

    id = "demo02_preload_head_bottle_yolo"
    name = "demo02 预加载头部 bottle YOLO"
    description = (
        "demo02 首步专用：自动请求启动头部深度，后台加载头部 bottle YOLO，"
        "并提前启动 ArUco 积累 TF；不执行 CUDA 预热或真实 YOLO 检测。"
    )
    # The normal head-camera gate requires ready depth before a workflow may
    # start. This bootstrap workflow is the intentional exception: it requests
    # depth early so loading overlaps the following robot motion.
    auto_starts_head_depth = True
    resources = ["camera:head_left"]
    produces: list[str] = []
    requires: list[str] = []
    default_params: dict[str, Any] = {
        "prompt": "bottle",
        "confidence_threshold": 0.4,
        "model_path": "",
        "image_size": 640,
        "max_detections": 100,
        "device": "",
    }
    params_schema: dict[str, dict[str, Any]] = {}

    def metadata(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "description": self.description,
            "resources": list(self.resources),
            "produces": list(self.produces),
            "requires": list(self.requires),
            "default_params": dict(self.default_params),
            "params_schema": dict(self.params_schema),
        }

    def resources_for_params(self, _params: dict[str, Any]) -> list[str]:
        return list(self.resources)

    def execute(self, ctx: Any, params: dict[str, Any]) -> dict[str, Any]:
        values = dict(self.default_params)
        values.update(params or {})
        prompt = str(values.get("prompt") or "").strip()
        if prompt != "bottle":
            raise RuntimeError("demo02 预加载仅允许 prompt=bottle")

        head_depth_result = ctx.node.set_head_depth_enabled(True)
        if not head_depth_result.get("ok", False):
            raise RuntimeError(
                str(head_depth_result.get("message") or "头部深度自动启动失败")
            )
        head_depth_status = dict(head_depth_result.get("processor") or {})
        ctx.log(
            "demo02 头部深度自动启动: "
            + str(head_depth_result.get("message") or "启动请求已提交")
            + f"; phase={head_depth_status.get('phase') or 'starting'}"
        )

        aruco_result = ctx.node.start_aruco_box_pose({})
        if not aruco_result.get("ok", False):
            raise RuntimeError(
                str(aruco_result.get("message") or "demo02 ArUco 预加载失败")
            )
        ctx.log(
            "demo02 ArUco 预加载: "
            + str(aruco_result.get("message") or "ArUco pose started")
        )

        config = ctx.node.current_camera_config("head_left")
        config.update(
            {
                "model_path": str(values.get("model_path") or config["model_path"]),
                "prompt_classes": [prompt],
                "device": str(values.get("device") or ""),
                "confidence_threshold": float(values["confidence_threshold"]),
                "image_size": int(values["image_size"]),
                "max_detections": int(values["max_detections"]),
                "asset_id": "",
                "asset_texture_file": "",
                "enable_detection": False,
                "load_model": True,
                "warmup_model": False,
                "preload_only": True,
            }
        )
        result = ctx.node.publish_vision_config(config)
        if not result.get("ok", False):
            raise RuntimeError(
                str(result.get("message") or "demo02 YOLO 预加载提交失败")
            )
        ctx.log(
            "demo02 头部 bottle YOLO 预加载已提交: "
            "真实检测关闭，仅后台加载权重和提示词，不执行 CUDA 推理"
        )
        return {
            "ok": True,
            "status": "preloading",
            "message": "demo02 正在后台加载 YOLO，ArUco 已提前启动并积累 TF",
            "config": result.get("config", config),
            "aruco": aruco_result,
            "head_depth": head_depth_status,
        }
