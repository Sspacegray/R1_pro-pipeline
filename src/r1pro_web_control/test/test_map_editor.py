from __future__ import annotations

import math
from pathlib import Path

import cv2
import numpy as np
import yaml

from r1pro_web_control.map_editor import MapEditorApp, cropped_origin, validate_crop


def test_cropped_origin_without_rotation() -> None:
    crop = {"x": 10, "y": 20, "width": 40, "height": 30}
    x, y = cropped_origin([-9.0, -7.0, 0.0], 0.05, 100, crop)
    assert x == -8.5
    assert y == -4.5


def test_cropped_origin_with_rotation() -> None:
    crop = {"x": 2, "y": 5, "width": 3, "height": 5}
    x, y = cropped_origin([1.0, 2.0, math.pi / 2], 1.0, 20, crop)
    assert math.isclose(x, -9.0, abs_tol=1e-9)
    assert math.isclose(y, 4.0, abs_tol=1e-9)


def test_validate_crop_rejects_out_of_bounds() -> None:
    try:
        validate_crop({"x": 9, "y": 0, "width": 2, "height": 2}, 10, 10)
    except ValueError as exc:
        assert "超出" in str(exc)
    else:
        raise AssertionError("out-of-bounds crop was accepted")


def test_export_writes_cropped_map_and_yaml(tmp_path: Path) -> None:
    image = np.arange(48, dtype=np.uint8).reshape(6, 8)
    assert cv2.imwrite(str(tmp_path / "source.pgm"), image)
    (tmp_path / "source.yaml").write_text(
        "image: source.pgm\n"
        "mode: trinary\n"
        "resolution: 0.5\n"
        "origin: [1.0, 2.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n",
        encoding="utf-8",
    )
    web_root = tmp_path / "web"
    web_root.mkdir()
    app = MapEditorApp(tmp_path / "source.yaml", web_root)

    result = app.export(
        {
            "crop": {"x": 2, "y": 1, "width": 4, "height": 3},
            "name": "cropped",
            "yaw_deg": 90,
        }
    )

    output = cv2.imread(str(tmp_path / "cropped.pgm"), cv2.IMREAD_GRAYSCALE)
    assert output.tolist() == image[1:4, 2:6].tolist()
    metadata = yaml.safe_load((tmp_path / "cropped.yaml").read_text(encoding="utf-8"))
    assert metadata["image"] == "cropped.pgm"
    assert metadata["origin"][:2] == [2.0, 3.0]
    assert math.isclose(metadata["origin"][2], math.pi / 2, abs_tol=1e-12)
    assert result["size"] == [4, 3]
