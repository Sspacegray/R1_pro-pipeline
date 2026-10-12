from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import yaml

from r1pro_web_control.asset_catalog import AssetCatalog, normalize_asset_text
from r1pro_web_control.server_node import R1ProWebNode


SOURCE_ROOT = Path(__file__).resolve().parents[2]
REAL_ASSET_ROOT = SOURCE_ROOT / "FoundationPose" / "objects"


def _write_asset(
    root: Path,
    directory: str,
    asset_id: str,
    name: str,
    aliases: list[str],
    *,
    mesh_value: str = "assets/model.obj",
    include_uv: bool = True,
) -> Path:
    asset_dir = root / directory
    assets_dir = asset_dir / "assets"
    assets_dir.mkdir(parents=True)
    corners = [
        "v 0 0 0",
        "v 1 0 0",
        "v 0 1 0",
        "v 0 0 1",
    ]
    texture_coordinates = [
        "vt 0 0",
        "vt 1 0",
        "vt 0 1",
        "vt 1 1",
    ]
    if include_uv:
        faces = [
            "f 1/1 2/2 3/3",
            "f 1/1 2/2 4/4",
            "f 1/1 3/3 4/4",
            "f 2/2 3/3 4/4",
        ]
    else:
        faces = ["f 1 2 3", "f 1 2 4", "f 1 3 4", "f 2 3 4"]
    (assets_dir / "model.obj").write_text(
        "\n".join(corners + texture_coordinates + faces) + "\n",
        encoding="utf-8",
    )
    assert cv2.imwrite(
        str(assets_dir / "texture.png"), np.full((4, 4, 3), 127, dtype=np.uint8)
    )
    payload = {
        "schema_version": 1,
        "id": asset_id,
        "name": name,
        "enabled": True,
        "category": "test_object",
        "aliases": aliases,
        "detection_prompts": ["object"],
        "mesh": mesh_value,
        "texture": "assets/texture.png",
        "units": "m",
        "expected_extents_m": [1.0, 1.0, 1.0],
        "origin": {
            "semantics": "geometric_center",
            "expected_center_m": [0.5, 0.5, 0.5],
        },
        "axes": {"x": "horizontal", "y": "horizontal", "z": "up"},
        "foundation_pose": {
            "input_scale": 0.5,
            "est_refine_iter": 5,
            "track_refine_iter": 2,
            "z_axis_symmetry": False,
            "allow_bbox_mask_fallback": False,
        },
        "mask": {
            "edge_expand_px": 3,
            "close_kernel_px": 5,
            "fill_hole_max_area_px": 200,
        },
    }
    (asset_dir / "asset.yaml").write_text(
        yaml.safe_dump(payload, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    return asset_dir


def test_real_catalog_scans_valid_assets_and_skips_legacy_links() -> None:
    catalog = AssetCatalog(REAL_ASSET_ROOT)
    result = catalog.scan()
    assert result["asset_count"] >= 38
    assert result["enabled_count"] == result["asset_count"]
    assert result["error_count"] == 0
    assert result["warning_count"] == 0
    asset_ids = {asset["id"] for asset in result["assets"]}
    assert {"bluelz", "drink_tea_red", "drink_wahaha"} <= asset_ids
    assert len(asset_ids) == result["asset_count"]


def test_specific_chinese_and_sentence_prompts_match_unique_assets() -> None:
    catalog = AssetCatalog(REAL_ASSET_ROOT)
    catalog.scan()
    tea = catalog.match("冰红茶")
    assert tea["matched"] is True
    assert tea["ambiguous"] is False
    assert tea["asset"]["id"] == "drink_tea_red"
    assert tea["asset"]["detection_prompts"] == ["bottle"]

    box = catalog.match("请识别蓝色料箱")
    assert box["matched"] is True
    assert box["asset"]["id"] == "bluelz"
    assert box["asset"]["mesh_file"].endswith("/BlueLZ/assets/V_BlueLZ.obj")
    assert box["asset"]["appearance"] == {
        "min_score": 0.20,
        "min_margin": 0.04,
    }

    # Assets without an explicit override retain the conservative defaults.
    assert tea["asset"]["appearance"] == {
        "min_score": 0.28,
        "min_margin": 0.04,
    }


def test_generic_bottle_prompt_is_ambiguous_and_never_auto_selects() -> None:
    catalog = AssetCatalog(REAL_ASSET_ROOT)
    catalog.scan()
    result = catalog.match("bottle", limit=20)
    assert result["matched"] is False
    assert result["ambiguous"] is True
    assert result["asset"] is None
    bottle_ids = {
        candidate["asset"]["id"] for candidate in result["candidates"]
    }
    assert {
        "drink_jingtian",
        "drink_pepsi",
        "drink_sprite",
        "drink_tea_green",
        "drink_tea_red",
        "drink_wahaha",
    } <= bottle_ids


def test_normalization_handles_case_separators_and_full_width_text() -> None:
    assert normalize_asset_text("  DRINK_TEA-RED ") == "drink tea red"
    assert normalize_asset_text("Ｗａｈａｈａ") == "wahaha"
    catalog = AssetCatalog(REAL_ASSET_ROOT)
    catalog.scan()
    result = catalog.match("DRINK_TEA_RED")
    assert result["matched"] is True
    assert result["asset"]["id"] == "drink_tea_red"


def test_invalid_escape_and_missing_face_uv_are_fail_closed(tmp_path: Path) -> None:
    outside = tmp_path / "outside.obj"
    outside.write_text("v 0 0 0\n", encoding="utf-8")
    _write_asset(
        tmp_path,
        "escape",
        "escape",
        "Escape",
        ["escape asset"],
        mesh_value="../outside.obj",
    )
    _write_asset(
        tmp_path,
        "no_uv",
        "no_uv",
        "No UV",
        ["no uv asset"],
        include_uv=False,
    )
    result = AssetCatalog(tmp_path).scan()
    assert result["asset_count"] == 0
    assert result["error_count"] == 2
    messages = " ".join(item["message"] for item in result["errors"])
    assert "escapes the asset directory" in messages
    assert "lacks a UV index" in messages


def test_duplicate_id_is_rejected_and_symlink_directory_is_not_reindexed(
    tmp_path: Path,
) -> None:
    first = _write_asset(tmp_path, "first", "same_id", "First", ["first asset"])
    _write_asset(tmp_path, "second", "same_id", "Second", ["second asset"])
    (tmp_path / "linked").symlink_to(first, target_is_directory=True)
    result = AssetCatalog(tmp_path).scan()
    assert result["asset_count"] == 1
    assert result["error_count"] == 1
    assert "duplicate asset id" in result["errors"][0]["message"]


def test_server_asset_handler_lists_rescans_matches_and_validates_payload() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node.asset_catalog = AssetCatalog(REAL_ASSET_ROOT)
    node.asset_catalog.scan()
    listed = node.handle_assets("list", {})
    assert listed["ok"] is True
    assert listed["asset_count"] >= 38
    matched = node.handle_assets("match", {"prompt": "娃哈哈"})
    assert matched["asset"]["id"] == "drink_wahaha"
    missing = node.handle_assets("match", {})
    assert missing["ok"] is False
    invalid = node.handle_assets("match", {"prompt": "冰红茶", "limit": "bad"})
    assert invalid["status"] == "invalid_match_config"
    rescanned = node.handle_assets("rescan", {})
    assert rescanned["generation"] == 2
