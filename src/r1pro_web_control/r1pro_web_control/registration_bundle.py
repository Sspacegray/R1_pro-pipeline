from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import secrets
import shutil
import tempfile
from typing import Any

import cv2
import numpy as np


BUNDLE_SCHEMA_VERSION = 1
_CANDIDATE_FILES = ("rgb.png", "depth.npy", "camera_info.json", "raw_mask.png", "detections.json")
_LATCHED_FILES = _CANDIDATE_FILES + ("mask.png",)


class RegistrationBundleError(ValueError):
    pass


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path: Path, payload: Any) -> None:
    data = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    with path.open("w", encoding="utf-8") as stream:
        stream.write(data)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def _safe_component(value: Any, fallback: str) -> str:
    text = "".join(
        character if character.isalnum() or character in ("-", "_") else "_"
        for character in str(value or "")
    ).strip("_")
    return text or fallback


def _stamp_ns(header: Any) -> int:
    stamp = getattr(header, "stamp", None)
    try:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return 0


def camera_info_payload(camera_info: Any, *, source_stamp_ns: int) -> dict[str, Any]:
    return {
        "stamp_ns": int(source_stamp_ns),
        "source_stamp_ns": int(source_stamp_ns),
        "original_stamp_ns": _stamp_ns(getattr(camera_info, "header", None)),
        "frame_id": str(getattr(getattr(camera_info, "header", None), "frame_id", "")),
        "width": int(getattr(camera_info, "width", 0) or 0),
        "height": int(getattr(camera_info, "height", 0) or 0),
        "distortion_model": str(getattr(camera_info, "distortion_model", "")),
        "d": [float(value) for value in getattr(camera_info, "d", [])],
        "k": [float(value) for value in getattr(camera_info, "k", [])],
        "r": [float(value) for value in getattr(camera_info, "r", [])],
        "p": [float(value) for value in getattr(camera_info, "p", [])],
        "binning_x": int(getattr(camera_info, "binning_x", 0) or 0),
        "binning_y": int(getattr(camera_info, "binning_y", 0) or 0),
        "roi": {
            "x_offset": int(getattr(getattr(camera_info, "roi", None), "x_offset", 0) or 0),
            "y_offset": int(getattr(getattr(camera_info, "roi", None), "y_offset", 0) or 0),
            "height": int(getattr(getattr(camera_info, "roi", None), "height", 0) or 0),
            "width": int(getattr(getattr(camera_info, "roi", None), "width", 0) or 0),
            "do_rectify": bool(getattr(getattr(camera_info, "roi", None), "do_rectify", False)),
        },
    }


def _validate_arrays(
    rgb_bgr: np.ndarray, depth_m: np.ndarray, raw_mask: np.ndarray, camera_info: Any
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rgb = np.asarray(rgb_bgr)
    depth = np.asarray(depth_m, dtype=np.float32)
    mask = np.asarray(raw_mask)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise RegistrationBundleError("RGB must be uint8 BGR with three channels")
    if depth.ndim != 2:
        raise RegistrationBundleError("depth must be a 2D 32FC1 array")
    if mask.ndim != 2:
        raise RegistrationBundleError("raw mask must be a 2D array")
    if depth.shape != rgb.shape[:2] or mask.shape != rgb.shape[:2]:
        raise RegistrationBundleError("RGB, depth and mask sizes must be identical")
    if len(getattr(camera_info, "k", [])) != 9:
        raise RegistrationBundleError("CameraInfo K must contain nine values")
    return np.ascontiguousarray(rgb), np.ascontiguousarray(depth), np.where(mask > 0, 255, 0).astype(np.uint8)


def write_candidate_bundle(
    root: str | Path,
    *,
    rgb_bgr: np.ndarray,
    depth_m: np.ndarray,
    camera_info: Any,
    raw_mask: np.ndarray,
    detections: list[dict[str, Any]],
    selected_index: int,
    source_stamp_ns: int,
    depth_stamp_ns: int,
    camera_info_stamp_ns: int,
    frame_id: str,
    camera_key: str,
    config_id: int,
    worker_session_id: int,
) -> str:
    if source_stamp_ns <= 0:
        raise RegistrationBundleError("source timestamp must be positive")
    if depth_stamp_ns != source_stamp_ns:
        raise RegistrationBundleError(
            f"depth timestamp {depth_stamp_ns} does not match RGB {source_stamp_ns}"
        )
    if camera_info_stamp_ns not in (0, source_stamp_ns):
        raise RegistrationBundleError(
            f"CameraInfo timestamp {camera_info_stamp_ns} does not match RGB {source_stamp_ns}"
        )
    if selected_index < 0 or selected_index >= len(detections):
        raise RegistrationBundleError("selected detection index is invalid")
    rgb, depth, mask = _validate_arrays(rgb_bgr, depth_m, raw_mask, camera_info)
    base = Path(root).expanduser().resolve() / "candidates"
    base.mkdir(parents=True, exist_ok=True)
    name = (
        f"candidate_s{int(worker_session_id)}_c{int(config_id)}_"
        f"{_safe_component(camera_key, 'camera')}_{int(source_stamp_ns)}"
    )
    target = base / name
    if target.is_dir():
        return str(target)
    temporary = Path(tempfile.mkdtemp(prefix=f".{name}.", dir=base))
    try:
        if not cv2.imwrite(str(temporary / "rgb.png"), rgb):
            raise RegistrationBundleError("failed to encode RGB PNG")
        with (temporary / "depth.npy").open("wb") as stream:
            np.save(stream, depth, allow_pickle=False)
            stream.flush()
            os.fsync(stream.fileno())
        if not cv2.imwrite(str(temporary / "raw_mask.png"), mask):
            raise RegistrationBundleError("failed to encode raw mask PNG")
        _write_json(
            temporary / "camera_info.json",
            camera_info_payload(camera_info, source_stamp_ns=source_stamp_ns),
        )
        _write_json(
            temporary / "detections.json",
            {"selected_index": int(selected_index), "detections": deepcopy(detections)},
        )
        manifest = {
            "schema_version": BUNDLE_SCHEMA_VERSION,
            "kind": "yolo_registration_candidate",
            "source_stamp_ns": int(source_stamp_ns),
            "rgb_stamp_ns": int(source_stamp_ns),
            "depth_stamp_ns": int(depth_stamp_ns),
            "camera_info_stamp_ns": int(source_stamp_ns),
            "camera_info_original_stamp_ns": int(camera_info_stamp_ns),
            "frame_id": str(frame_id),
            "camera_key": str(camera_key),
            "config_id": int(config_id),
            "worker_session_id": int(worker_session_id),
            "rgb_encoding": "bgr8_png",
            "depth_encoding": "32FC1",
            "depth_unit": "m",
            "mask_encoding": "mono8",
            "image_size": [int(rgb.shape[1]), int(rgb.shape[0])],
            "selected_index": int(selected_index),
            "files": {name: name for name in _CANDIDATE_FILES},
            "sha256": {name: _sha256(temporary / name) for name in _CANDIDATE_FILES},
        }
        _write_json(temporary / "manifest.json", manifest)
        os.replace(temporary, target)
        return str(target)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def _load_manifest(directory: Path, expected_kind: str) -> dict[str, Any]:
    try:
        payload = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RegistrationBundleError(f"invalid registration manifest: {exc}") from exc
    if not isinstance(payload, dict) or payload.get("schema_version") != BUNDLE_SCHEMA_VERSION:
        raise RegistrationBundleError("unsupported registration bundle schema")
    if payload.get("kind") != expected_kind:
        raise RegistrationBundleError(f"unexpected registration bundle kind: {payload.get('kind')}")
    return payload


def validate_candidate_bundle(
    directory_value: str | Path,
    *,
    camera_key: str,
    config_id: int,
    worker_session_id: int,
    source_stamp_ns: int,
) -> dict[str, Any]:
    directory = Path(directory_value).expanduser().resolve()
    if not directory.is_dir():
        raise RegistrationBundleError(f"candidate bundle is missing: {directory}")
    manifest = _load_manifest(directory, "yolo_registration_candidate")
    expected = {
        "camera_key": str(camera_key),
        "config_id": int(config_id),
        "worker_session_id": int(worker_session_id),
        "source_stamp_ns": int(source_stamp_ns),
    }
    for key, value in expected.items():
        if manifest.get(key) != value:
            raise RegistrationBundleError(f"candidate {key} does not match detection")
    if not (
        int(manifest.get("rgb_stamp_ns", 0))
        == int(manifest.get("depth_stamp_ns", 0))
        == int(manifest.get("camera_info_stamp_ns", 0))
        == int(source_stamp_ns)
    ):
        raise RegistrationBundleError("candidate RGB/depth/CameraInfo timestamps differ")
    hashes = manifest.get("sha256")
    if not isinstance(hashes, dict):
        raise RegistrationBundleError("candidate checksums are missing")
    for filename in _CANDIDATE_FILES:
        path = directory / filename
        if not path.is_file() or _sha256(path) != hashes.get(filename):
            raise RegistrationBundleError(f"candidate file is missing or changed: {filename}")
    rgb = cv2.imread(str(directory / "rgb.png"), cv2.IMREAD_COLOR)
    mask = cv2.imread(str(directory / "raw_mask.png"), cv2.IMREAD_GRAYSCALE)
    try:
        depth = np.load(str(directory / "depth.npy"), allow_pickle=False)
        camera_info = json.loads((directory / "camera_info.json").read_text(encoding="utf-8"))
        detections = json.loads((directory / "detections.json").read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise RegistrationBundleError(f"candidate payload cannot be decoded: {exc}") from exc
    if rgb is None or mask is None or depth.dtype != np.float32 or depth.ndim != 2:
        raise RegistrationBundleError("candidate image/depth encoding is invalid")
    if rgb.shape[:2] != depth.shape or mask.shape != depth.shape:
        raise RegistrationBundleError("candidate RGB/depth/mask sizes differ")
    if not isinstance(camera_info, dict) or len(camera_info.get("k", [])) != 9:
        raise RegistrationBundleError("candidate CameraInfo is invalid")
    return {
        "directory": directory,
        "manifest": manifest,
        "rgb": rgb,
        "depth": depth,
        "camera_info": camera_info,
        "raw_mask": mask,
        "detections": detections,
    }


def postprocess_mask(raw_mask: np.ndarray, config: dict[str, Any]) -> np.ndarray:
    mask = np.where(np.asarray(raw_mask) > 0, 255, 0).astype(np.uint8)
    close_px = max(1, int(config.get("close_kernel_px", 1)))
    if close_px > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_px, close_px))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    fill_max = max(0, int(config.get("fill_hole_max_area_px", 0)))
    if fill_max > 0:
        inverse = np.where(mask > 0, 0, 255).astype(np.uint8)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(inverse, 8)
        border = set(np.unique(labels[0, :]).tolist())
        border.update(np.unique(labels[-1, :]).tolist())
        border.update(np.unique(labels[:, 0]).tolist())
        border.update(np.unique(labels[:, -1]).tolist())
        for label in range(1, count):
            if label not in border and int(stats[label, cv2.CC_STAT_AREA]) <= fill_max:
                mask[labels == label] = 255
    expand_px = max(0, int(config.get("edge_expand_px", 0)))
    if expand_px > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (expand_px, expand_px))
        mask = cv2.dilate(mask, kernel, iterations=1)
    return np.ascontiguousarray(mask)


def create_latched_bundle(
    candidate_dir: str | Path,
    destination_root: str | Path,
    *,
    detection: dict[str, Any],
    asset: dict[str, Any],
) -> dict[str, Any]:
    candidate = validate_candidate_bundle(
        candidate_dir,
        camera_key=str(detection.get("camera_key") or ""),
        config_id=int(detection.get("config_id", 0) or 0),
        worker_session_id=int(detection.get("worker_session_id", 0) or 0),
        source_stamp_ns=int(detection.get("source_stamp_ns", 0) or 0),
    )
    root = Path(destination_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    name = (
        f"latched_s{int(detection.get('worker_session_id', 0))}_"
        f"c{int(detection.get('config_id', 0))}_{_safe_component(asset.get('id'), 'asset')}_"
        f"{int(detection.get('source_stamp_ns', 0))}_{secrets.token_hex(4)}"
    )
    target = root / name
    temporary = Path(tempfile.mkdtemp(prefix=f".{name}.", dir=root))
    try:
        for filename in _CANDIDATE_FILES:
            shutil.copyfile(candidate["directory"] / filename, temporary / filename)
        processed = postprocess_mask(candidate["raw_mask"], asset.get("mask") or {})
        if int(np.count_nonzero(processed)) <= 0:
            raise RegistrationBundleError("postprocessed mask is empty")
        if not cv2.imwrite(str(temporary / "mask.png"), processed):
            raise RegistrationBundleError("failed to encode postprocessed mask")
        selected = deepcopy(detection)
        selected["asset_id"] = str(asset["id"])
        _write_json(temporary / "selected_detection.json", selected)
        files = {name: name for name in _LATCHED_FILES}
        files["selected_detection.json"] = "selected_detection.json"
        manifest = {
            **deepcopy(candidate["manifest"]),
            "kind": "yolo_registration_latched",
            "asset_id": str(asset["id"]),
            "asset_manifest_file": str(asset.get("manifest_file") or ""),
            "mesh_file": str(asset.get("mesh_file") or ""),
            "mask_config": deepcopy(asset.get("mask") or {}),
            "raw_mask_area_px": int(np.count_nonzero(candidate["raw_mask"])),
            "mask_area_px": int(np.count_nonzero(processed)),
            "bbox_xyxy": deepcopy(detection.get("bbox_xyxy")),
            "confidence": float(detection.get("confidence", 0.0) or 0.0),
            "class_name": str(detection.get("class_name") or ""),
            "files": files,
            "sha256": {filename: _sha256(temporary / filename) for filename in files},
        }
        _write_json(temporary / "manifest.json", manifest)
        os.replace(temporary, target)
        return {
            "bundle_dir": str(target),
            "bundle_manifest_file": str(target / "manifest.json"),
            "rgb_file": str(target / "rgb.png"),
            "depth_file": str(target / "depth.npy"),
            "camera_info_file": str(target / "camera_info.json"),
            "raw_mask_file": str(target / "raw_mask.png"),
            "postprocessed_mask_file": str(target / "mask.png"),
            "mask_area_px": int(manifest["mask_area_px"]),
            "raw_mask_area_px": int(manifest["raw_mask_area_px"]),
            "mask_config": deepcopy(manifest["mask_config"]),
        }
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def validate_latched_bundle(manifest_value: str | Path) -> dict[str, Any]:
    manifest_path = Path(manifest_value).expanduser().resolve()
    directory = manifest_path.parent
    manifest = _load_manifest(directory, "yolo_registration_latched")
    hashes = manifest.get("sha256")
    files = manifest.get("files")
    if not isinstance(hashes, dict) or not isinstance(files, dict):
        raise RegistrationBundleError("latched bundle file metadata is missing")
    for logical_name, relative_name in files.items():
        if logical_name != relative_name or Path(relative_name).name != relative_name:
            raise RegistrationBundleError("latched bundle contains an unsafe file path")
        path = directory / relative_name
        if not path.is_file() or _sha256(path) != hashes.get(logical_name):
            raise RegistrationBundleError(f"latched bundle file changed: {logical_name}")
    stamp = int(manifest.get("source_stamp_ns", 0) or 0)
    if stamp <= 0 or not (
        int(manifest.get("rgb_stamp_ns", 0))
        == int(manifest.get("depth_stamp_ns", 0))
        == int(manifest.get("camera_info_stamp_ns", 0))
        == stamp
    ):
        raise RegistrationBundleError("latched bundle timestamps are inconsistent")
    return {"directory": directory, "manifest": manifest}
