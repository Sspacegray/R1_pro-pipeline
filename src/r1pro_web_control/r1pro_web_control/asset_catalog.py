from __future__ import annotations

from copy import deepcopy
from difflib import SequenceMatcher
import math
from pathlib import Path
import re
import threading
import time
from typing import Any, Iterable
import unicodedata

import cv2
import yaml


ASSET_SCHEMA_VERSION = 1
ASSET_ID_PATTERN = re.compile(r"^[a-z0-9][a-z0-9_-]*$")
SUPPORTED_MESH_SUFFIXES = {".obj"}
SUPPORTED_TEXTURE_SUFFIXES = {".png", ".jpg", ".jpeg", ".webp"}
AUTO_MATCH_MIN_SCORE = 78
AUTO_MATCH_MIN_MARGIN = 8


class AssetValidationError(ValueError):
    pass


def normalize_asset_text(value: Any) -> str:
    text = unicodedata.normalize("NFKC", str(value or "")).casefold()
    text = re.sub(r"[_\-/\\]+", " ", text)
    text = "".join(
        character if (character.isalnum() or character.isspace()) else " "
        for character in text
    )
    return " ".join(text.split())


def _compact(value: str) -> str:
    return value.replace(" ", "")


def _string_list(value: Any, field: str, *, allow_empty: bool = False) -> list[str]:
    if not isinstance(value, list):
        raise AssetValidationError(f"{field} must be a list")
    result: list[str] = []
    seen: set[str] = set()
    for item in value:
        text = str(item or "").strip()
        normalized = normalize_asset_text(text)
        if not normalized:
            raise AssetValidationError(f"{field} contains an empty value")
        if normalized in seen:
            continue
        seen.add(normalized)
        result.append(text)
    if not allow_empty and not result:
        raise AssetValidationError(f"{field} must not be empty")
    return result


def _finite_triplet(value: Any, field: str, *, positive: bool = False) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise AssetValidationError(f"{field} must contain exactly three numbers")
    result: list[float] = []
    for item in value:
        try:
            number = float(item)
        except (TypeError, ValueError) as exc:
            raise AssetValidationError(f"{field} contains a non-number") from exc
        if not math.isfinite(number):
            raise AssetValidationError(f"{field} contains NaN or Inf")
        if positive and number <= 0.0:
            raise AssetValidationError(f"{field} values must be positive")
        result.append(number)
    return result


def _bounded_int(value: Any, field: str, minimum: int, maximum: int) -> int:
    try:
        number = int(value)
    except (TypeError, ValueError) as exc:
        raise AssetValidationError(f"{field} must be an integer") from exc
    if not minimum <= number <= maximum:
        raise AssetValidationError(
            f"{field} must be within [{minimum}, {maximum}]"
        )
    return number


def _bounded_float(value: Any, field: str, minimum: float, maximum: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise AssetValidationError(f"{field} must be a number") from exc
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise AssetValidationError(
            f"{field} must be finite and within [{minimum}, {maximum}]"
        )
    return number


def _safe_asset_file(
    asset_dir: Path,
    relative_value: Any,
    field: str,
    supported_suffixes: set[str],
) -> tuple[str, Path]:
    relative_text = str(relative_value or "").strip()
    relative_path = Path(relative_text)
    if not relative_text or relative_path.is_absolute():
        raise AssetValidationError(f"{field} must be a non-empty relative path")
    resolved = (asset_dir / relative_path).resolve()
    try:
        resolved.relative_to(asset_dir)
    except ValueError as exc:
        raise AssetValidationError(f"{field} escapes the asset directory") from exc
    if resolved.suffix.lower() not in supported_suffixes:
        raise AssetValidationError(
            f"unsupported {field} suffix: {resolved.suffix or '<none>'}"
        )
    if not resolved.is_file():
        raise AssetValidationError(f"{field} file is missing: {relative_text}")
    return relative_path.as_posix(), resolved


def _obj_geometry(path: Path) -> dict[str, Any]:
    vertices: list[tuple[float, float, float]] = []
    texcoord_count = 0
    face_count = 0
    face_texture_indices: list[int] = []
    try:
        with path.open("r", encoding="utf-8", errors="replace") as stream:
            for line_number, raw_line in enumerate(stream, start=1):
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                fields = line.split()
                if fields[0] == "v" and len(fields) >= 4:
                    try:
                        vertex = tuple(float(value) for value in fields[1:4])
                    except ValueError as exc:
                        raise AssetValidationError(
                            f"mesh has an invalid vertex at line {line_number}"
                        ) from exc
                    if not all(math.isfinite(value) for value in vertex):
                        raise AssetValidationError("mesh vertex contains NaN or Inf")
                    vertices.append(vertex)
                elif fields[0] == "vt" and len(fields) >= 3:
                    texcoord_count += 1
                elif fields[0] == "f" and len(fields) >= 4:
                    face_count += 1
                    for corner in fields[1:]:
                        parts = corner.split("/")
                        if len(parts) < 2 or not parts[1]:
                            raise AssetValidationError(
                                f"mesh face lacks a UV index at line {line_number}"
                            )
                        try:
                            face_texture_indices.append(int(parts[1]))
                        except ValueError as exc:
                            raise AssetValidationError(
                                f"mesh has an invalid UV index at line {line_number}"
                            ) from exc
    except OSError as exc:
        raise AssetValidationError(f"unable to read mesh: {exc}") from exc
    if len(vertices) < 3 or face_count < 1:
        raise AssetValidationError("mesh has no usable geometry")
    if texcoord_count < 1:
        raise AssetValidationError("mesh has no texture coordinates")
    for index in face_texture_indices:
        resolved_index = index if index > 0 else texcoord_count + index + 1
        if resolved_index < 1 or resolved_index > texcoord_count:
            raise AssetValidationError("mesh face references an invalid UV index")
    minimum = [min(vertex[axis] for vertex in vertices) for axis in range(3)]
    maximum = [max(vertex[axis] for vertex in vertices) for axis in range(3)]
    extents = [maximum[axis] - minimum[axis] for axis in range(3)]
    center = [(maximum[axis] + minimum[axis]) * 0.5 for axis in range(3)]
    if any(value <= 0.0 for value in extents):
        raise AssetValidationError("mesh has a zero-sized axis")
    return {
        "vertex_count": len(vertices),
        "face_count": face_count,
        "texture_coordinate_count": texcoord_count,
        "all_face_corners_have_uv": True,
        "actual_extents_m": extents,
        "actual_center_m": center,
    }


def _texture_metadata(path: Path) -> dict[str, int]:
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None or image.size == 0:
        raise AssetValidationError("texture cannot be decoded")
    height, width = image.shape[:2]
    channels = 1 if image.ndim == 2 else int(image.shape[2])
    return {"width": int(width), "height": int(height), "channels": channels}


def _near(actual: Iterable[float], expected: Iterable[float]) -> bool:
    return all(
        abs(float(actual_value) - float(expected_value))
        <= max(1e-6, abs(float(expected_value)) * 1e-4)
        for actual_value, expected_value in zip(actual, expected)
    )


class AssetCatalog:
    def __init__(self, root: str | Path) -> None:
        self.root = Path(root).expanduser().resolve()
        self._lock = threading.RLock()
        self._scan_lock = threading.Lock()
        self._generation = 0
        self._scanned_at_unix_s = 0.0
        self._assets: dict[str, dict[str, Any]] = {}
        self._errors: list[dict[str, Any]] = []
        self._warnings: list[dict[str, Any]] = []

    def scan(self) -> dict[str, Any]:
        assets: dict[str, dict[str, Any]] = {}
        errors: list[dict[str, Any]] = []
        warnings: list[dict[str, Any]] = []
        with self._scan_lock:
            if not self.root.is_dir():
                errors.append(
                    {
                        "code": "asset_root_missing",
                        "path": str(self.root),
                        "message": f"asset root is missing: {self.root}",
                    }
                )
            else:
                manifests = sorted(
                    self.root.glob("*/asset.yaml"),
                    key=lambda path: path.parent.name.casefold(),
                )
                for manifest_path in manifests:
                    if manifest_path.parent.is_symlink():
                        continue
                    try:
                        asset = self._load_asset(manifest_path)
                        asset_id = str(asset["id"])
                        if asset_id in assets:
                            raise AssetValidationError(
                                f"duplicate asset id: {asset_id}"
                            )
                        assets[asset_id] = asset
                    except Exception as exc:
                        errors.append(
                            {
                                "code": "invalid_asset",
                                "asset_dir": manifest_path.parent.name,
                                "manifest": str(manifest_path),
                                "message": str(exc),
                            }
                        )
                exact_owners: dict[str, list[str]] = {}
                for asset in assets.values():
                    for term in self._identity_terms(asset):
                        normalized = normalize_asset_text(term[1])
                        if normalized:
                            exact_owners.setdefault(normalized, []).append(asset["id"])
                for term, owners in exact_owners.items():
                    unique_owners = sorted(set(owners))
                    if len(unique_owners) > 1:
                        warnings.append(
                            {
                                "code": "ambiguous_exact_identity",
                                "term": term,
                                "asset_ids": unique_owners,
                                "message": "name or alias belongs to multiple assets",
                            }
                        )
            with self._lock:
                self._generation += 1
                self._scanned_at_unix_s = time.time()
                self._assets = assets
                self._errors = errors
                self._warnings = warnings
                return self.list_payload()

    def _load_asset(self, manifest_path: Path) -> dict[str, Any]:
        try:
            payload = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
        except (OSError, yaml.YAMLError) as exc:
            raise AssetValidationError(f"unable to parse asset.yaml: {exc}") from exc
        if not isinstance(payload, dict):
            raise AssetValidationError("asset.yaml must contain a mapping")
        if payload.get("schema_version") != ASSET_SCHEMA_VERSION:
            raise AssetValidationError(
                f"unsupported schema_version: {payload.get('schema_version')!r}"
            )
        asset_id = str(payload.get("id") or "").strip().casefold()
        if not ASSET_ID_PATTERN.fullmatch(asset_id):
            raise AssetValidationError(
                "id must use lowercase letters, digits, underscores or hyphens"
            )
        name = str(payload.get("name") or "").strip()
        category = str(payload.get("category") or "").strip()
        if not name or not category:
            raise AssetValidationError("name and category are required")
        if not isinstance(payload.get("enabled"), bool):
            raise AssetValidationError("enabled must be true or false")
        aliases = _string_list(payload.get("aliases"), "aliases")
        detection_prompts = _string_list(
            payload.get("detection_prompts"), "detection_prompts"
        )
        if str(payload.get("units") or "").strip() != "m":
            raise AssetValidationError("units must be m")
        asset_dir = manifest_path.parent.resolve()
        mesh_relative, mesh_path = _safe_asset_file(
            asset_dir,
            payload.get("mesh"),
            "mesh",
            SUPPORTED_MESH_SUFFIXES,
        )
        texture_relative, texture_path = _safe_asset_file(
            asset_dir,
            payload.get("texture"),
            "texture",
            SUPPORTED_TEXTURE_SUFFIXES,
        )
        expected_extents = _finite_triplet(
            payload.get("expected_extents_m"),
            "expected_extents_m",
            positive=True,
        )
        origin = payload.get("origin")
        if not isinstance(origin, dict):
            raise AssetValidationError("origin must be a mapping")
        origin_semantics = str(origin.get("semantics") or "").strip()
        if origin_semantics not in {"geometric_center", "bottom_center"}:
            raise AssetValidationError(
                "origin.semantics must be geometric_center or bottom_center"
            )
        expected_center = _finite_triplet(
            origin.get("expected_center_m"), "origin.expected_center_m"
        )
        axes = payload.get("axes")
        if not isinstance(axes, dict) or any(
            not str(axes.get(axis) or "").strip() for axis in ("x", "y", "z")
        ):
            raise AssetValidationError("axes.x, axes.y and axes.z are required")
        foundation = payload.get("foundation_pose")
        if not isinstance(foundation, dict):
            raise AssetValidationError("foundation_pose must be a mapping")
        foundation_config = {
            "input_scale": _bounded_float(
                foundation.get("input_scale"),
                "foundation_pose.input_scale",
                0.25,
                1.0,
            ),
            "est_refine_iter": _bounded_int(
                foundation.get("est_refine_iter"),
                "foundation_pose.est_refine_iter",
                1,
                20,
            ),
            "track_refine_iter": _bounded_int(
                foundation.get("track_refine_iter"),
                "foundation_pose.track_refine_iter",
                1,
                20,
            ),
            "z_axis_symmetry": bool(foundation.get("z_axis_symmetry", False)),
            "z_axis_symmetry_steps": _bounded_int(
                foundation.get("z_axis_symmetry_steps", 36),
                "foundation_pose.z_axis_symmetry_steps",
                1,
                360,
            ),
            "allow_bbox_mask_fallback": bool(
                foundation.get("allow_bbox_mask_fallback", False)
            ),
        }
        if not isinstance(foundation.get("z_axis_symmetry"), bool):
            raise AssetValidationError(
                "foundation_pose.z_axis_symmetry must be true or false"
            )
        if not isinstance(foundation.get("allow_bbox_mask_fallback"), bool):
            raise AssetValidationError(
                "foundation_pose.allow_bbox_mask_fallback must be true or false"
            )
        mask = payload.get("mask")
        if not isinstance(mask, dict):
            raise AssetValidationError("mask must be a mapping")
        mask_config = {
            "edge_expand_px": _bounded_int(
                mask.get("edge_expand_px"), "mask.edge_expand_px", 0, 101
            ),
            "close_kernel_px": _bounded_int(
                mask.get("close_kernel_px"), "mask.close_kernel_px", 1, 101
            ),
            "fill_hole_max_area_px": _bounded_int(
                mask.get("fill_hole_max_area_px"),
                "mask.fill_hole_max_area_px",
                0,
                10_000_000,
            ),
        }
        appearance = payload.get("appearance", {})
        if not isinstance(appearance, dict):
            raise AssetValidationError("appearance must be a mapping")
        appearance_config = {
            "min_score": _bounded_float(
                appearance.get("min_score", 0.28),
                "appearance.min_score",
                0.0,
                1.0,
            ),
            "min_margin": _bounded_float(
                appearance.get("min_margin", 0.04),
                "appearance.min_margin",
                0.0,
                1.0,
            ),
        }
        geometry = _obj_geometry(mesh_path)
        if not _near(geometry["actual_extents_m"], expected_extents):
            raise AssetValidationError(
                "mesh extents do not match expected_extents_m: "
                f"actual={geometry['actual_extents_m']} expected={expected_extents}"
            )
        if not _near(geometry["actual_center_m"], expected_center):
            raise AssetValidationError(
                "mesh center does not match origin.expected_center_m: "
                f"actual={geometry['actual_center_m']} expected={expected_center}"
            )
        texture_metadata = _texture_metadata(texture_path)
        modified_ns = max(
            manifest_path.stat().st_mtime_ns,
            mesh_path.stat().st_mtime_ns,
            texture_path.stat().st_mtime_ns,
        )
        return {
            "id": asset_id,
            "name": name,
            "enabled": bool(payload["enabled"]),
            "category": category,
            "aliases": aliases,
            "detection_prompts": detection_prompts,
            "asset_dir": str(asset_dir),
            "manifest_file": str(manifest_path.resolve()),
            "mesh": mesh_relative,
            "mesh_file": str(mesh_path),
            "texture": texture_relative,
            "texture_file": str(texture_path),
            "units": "m",
            "expected_extents_m": expected_extents,
            "origin": {
                "semantics": origin_semantics,
                "expected_center_m": expected_center,
            },
            "axes": {axis: str(axes[axis]).strip() for axis in ("x", "y", "z")},
            "foundation_pose": foundation_config,
            "mask": mask_config,
            "appearance": appearance_config,
            "mesh_metadata": geometry,
            "texture_metadata": texture_metadata,
            "modified_ns": int(modified_ns),
        }

    @staticmethod
    def _identity_terms(asset: dict[str, Any]) -> list[tuple[str, str, int]]:
        terms: list[tuple[str, str, int]] = [
            ("id", str(asset["id"]), 100),
            ("name", str(asset["name"]), 98),
        ]
        terms.extend(("alias", str(value), 96) for value in asset["aliases"])
        return terms

    @classmethod
    def _all_match_terms(cls, asset: dict[str, Any]) -> list[tuple[str, str, int]]:
        terms = cls._identity_terms(asset)
        terms.extend(
            ("detection_prompt", str(value), 60)
            for value in asset["detection_prompts"]
        )
        terms.append(("category", str(asset["category"]), 48))
        return terms

    @staticmethod
    def _term_score(
        query: str, term: str, kind: str, base_score: int
    ) -> tuple[int, str]:
        normalized_term = normalize_asset_text(term)
        if not normalized_term:
            return 0, ""
        query_compact = _compact(query)
        term_compact = _compact(normalized_term)
        if query == normalized_term or query_compact == term_compact:
            return base_score, f"exact_{kind}"
        minimum_length = 2
        if len(term_compact) >= minimum_length and term_compact in query_compact:
            penalty = 8 if kind in {"id", "name", "alias"} else 10
            return max(1, base_score - penalty), f"query_contains_{kind}"
        if len(query_compact) >= minimum_length and query_compact in term_compact:
            penalty = 15 if kind in {"id", "name", "alias"} else 12
            return max(1, base_score - penalty), f"{kind}_contains_query"
        if kind in {"id", "name", "alias"} and min(
            len(query_compact), len(term_compact)
        ) >= 3:
            ratio = SequenceMatcher(None, query_compact, term_compact).ratio()
            if ratio >= 0.62:
                return min(90, int(round(45.0 + ratio * 45.0))), f"fuzzy_{kind}"
        return 0, ""

    def match(
        self,
        prompt: Any,
        *,
        limit: int = 8,
        minimum_score: int = AUTO_MATCH_MIN_SCORE,
        minimum_margin: int = AUTO_MATCH_MIN_MARGIN,
    ) -> dict[str, Any]:
        prompt_text = str(prompt or "").strip()
        query = normalize_asset_text(prompt_text)
        if not query:
            return {
                "ok": False,
                "status": "invalid_prompt",
                "message": "prompt must not be empty",
                "prompt": prompt_text,
                "normalized_prompt": query,
                "matched": False,
                "ambiguous": False,
                "asset": None,
                "candidates": [],
            }
        limit = max(1, min(50, int(limit)))
        minimum_score = max(1, min(100, int(minimum_score)))
        minimum_margin = max(0, min(100, int(minimum_margin)))
        with self._lock:
            assets = deepcopy(list(self._assets.values()))
            generation = self._generation
        candidates: list[dict[str, Any]] = []
        for asset in assets:
            if not asset.get("enabled", False):
                continue
            best_score = 0
            best_reason = ""
            best_term = ""
            for kind, term, base_score in self._all_match_terms(asset):
                score, reason = self._term_score(query, term, kind, base_score)
                if score > best_score:
                    best_score = score
                    best_reason = reason
                    best_term = term
            if best_score > 0:
                candidates.append(
                    {
                        "score": int(best_score),
                        "confidence": round(best_score / 100.0, 3),
                        "reason": best_reason,
                        "matched_term": best_term,
                        "asset": self._public_asset(asset),
                    }
                )
        candidates.sort(
            key=lambda item: (
                -int(item["score"]),
                str(item["asset"]["id"]),
            )
        )
        top = candidates[0] if candidates else None
        second_score = int(candidates[1]["score"]) if len(candidates) > 1 else 0
        top_score = int(top["score"]) if top else 0
        margin = top_score - second_score
        auto_selected = bool(
            top
            and top_score >= minimum_score
            and (len(candidates) == 1 or margin >= minimum_margin)
        )
        ambiguous = bool(top and not auto_selected and len(candidates) > 1)
        selected_asset = deepcopy(top["asset"]) if auto_selected and top else None
        if auto_selected:
            status = "matched"
            message = f"matched asset {selected_asset['id']}"
        elif ambiguous:
            status = "ambiguous"
            message = "multiple assets match; select one candidate explicitly"
        elif top:
            status = "low_confidence"
            message = "no asset reached the automatic match threshold"
        else:
            status = "not_found"
            message = "no asset matches the prompt"
        return {
            "ok": True,
            "status": status,
            "message": message,
            "prompt": prompt_text,
            "normalized_prompt": query,
            "generation": generation,
            "matched": auto_selected,
            "ambiguous": ambiguous,
            "score": top_score,
            "confidence": round(top_score / 100.0, 3),
            "margin": margin,
            "asset": selected_asset,
            "candidates": deepcopy(candidates[:limit]),
        }

    @staticmethod
    def _public_asset(asset: dict[str, Any]) -> dict[str, Any]:
        return deepcopy(asset)

    def get(self, asset_id: Any) -> dict[str, Any] | None:
        normalized_id = str(asset_id or "").strip().casefold()
        with self._lock:
            asset = self._assets.get(normalized_id)
            return None if asset is None else self._public_asset(asset)

    def summary(self) -> dict[str, Any]:
        with self._lock:
            return {
                "root": str(self.root),
                "generation": self._generation,
                "scanned_at_unix_s": self._scanned_at_unix_s,
                "asset_count": len(self._assets),
                "enabled_count": sum(
                    1 for asset in self._assets.values() if asset.get("enabled")
                ),
                "error_count": len(self._errors),
                "warning_count": len(self._warnings),
            }

    def list_payload(self) -> dict[str, Any]:
        with self._lock:
            assets = [
                self._public_asset(asset)
                for asset in sorted(
                    self._assets.values(), key=lambda item: str(item["id"])
                )
            ]
            return {
                "ok": True,
                **self.summary(),
                "errors": deepcopy(self._errors),
                "warnings": deepcopy(self._warnings),
                "assets": assets,
            }
