import importlib.util
import math
from pathlib import Path

import numpy as np
from geometry_msgs.msg import TransformStamped


SCRIPT = Path(__file__).parents[1] / "scripts" / "lidar_self_filter.py"
SPEC = importlib.util.spec_from_file_location("lidar_self_filter", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _transform(translation=(0.0, 0.0, 0.0), quaternion=(0.0, 0.0, 0.0, 1.0)):
    message = TransformStamped()
    message.transform.translation.x = translation[0]
    message.transform.translation.y = translation[1]
    message.transform.translation.z = translation[2]
    message.transform.rotation.x = quaternion[0]
    message.transform.rotation.y = quaternion[1]
    message.transform.rotation.z = quaternion[2]
    message.transform.rotation.w = quaternion[3]
    return message


def _points(xyz, xyz_dtype=np.float32):
    dtype = np.dtype(
        [
            ("x", xyz_dtype),
            ("y", xyz_dtype),
            ("z", xyz_dtype),
            ("intensity", np.float32),
            ("tag", np.uint8),
            ("line", np.uint8),
            ("timestamp", np.float64),
        ]
    )
    result = np.zeros(len(xyz), dtype=dtype)
    for index, value in enumerate(xyz):
        result[index]["x"] = value[0]
        result[index]["y"] = value[1]
        result[index]["z"] = value[2]
        result[index]["intensity"] = index + 0.25
        result[index]["tag"] = index + 1
        result[index]["line"] = index + 2
        result[index]["timestamp"] = 1000.0 + index
    return result


def test_inside_box_is_inclusive_and_preserves_non_finite_points():
    values = [
        (0.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, -2.0, 0.0),
        (0.0, 2.0, 0.0),
        (0.0, 0.0, -3.0),
        (0.0, 0.0, 3.0),
        (1.001, 0.0, 0.0),
        (float("nan"), 0.0, 0.0),
        (float("inf"), 0.0, 0.0),
        (float("-inf"), 0.0, 0.0),
    ]
    mask = MODULE._inside_body_box(
        _points(values),
        _transform(),
        np.asarray((-1.0, -2.0, -3.0)),
        np.asarray((1.0, 2.0, 3.0)),
    )

    assert mask.tolist() == [True] * 7 + [False] * 4


def test_rotation_and_translation_match_double_precision_reference():
    rng = np.random.default_rng(7)
    xyz = rng.uniform(-4.0, 4.0, size=(5000, 3)).astype(np.float32)
    points = _points(xyz)
    half = math.sqrt(0.5)
    transform = _transform((0.3, -0.2, 0.7), (0.0, 0.0, half, half))
    lower = np.asarray((-0.8, -0.9, -0.4))
    upper = np.asarray((0.9, 0.7, 1.6))

    actual = MODULE._inside_body_box(points, transform, lower, upper)
    rotation, translation = MODULE._transform_components(transform, np.float64)
    reference_body = xyz.astype(np.float64) @ rotation.T + translation
    reference = ((reference_body >= lower) & (reference_body <= upper)).all(axis=1)

    assert np.array_equal(actual, reference)


def test_float64_xyz_keeps_float64_geometry_path():
    points = _points(
        [(1.0 + 1.0e-10, 0.0, 0.0), (1.0 - 1.0e-10, 0.0, 0.0)],
        xyz_dtype=np.float64,
    )
    mask = MODULE._inside_body_box(
        points,
        _transform(),
        np.asarray((-1.0, -1.0, -1.0)),
        np.asarray((1.0, 1.0, 1.0)),
    )

    assert mask.tolist() == [False, True]


def test_boolean_filter_keeps_point_order_and_all_non_xyz_bytes():
    points = _points([(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (3.0, 0.0, 0.0)])
    original = points.copy()
    inside = MODULE._inside_body_box(
        points,
        _transform(),
        np.asarray((-1.0, -1.0, -1.0)),
        np.asarray((1.0, 1.0, 1.0)),
    )

    kept = points[~inside]
    assert kept[["intensity", "tag", "line", "timestamp"]].tobytes() == (
        original[1:][["intensity", "tag", "line", "timestamp"]].tobytes()
    )
    assert kept["x"].tolist() == [2.0, 3.0]


def test_missing_or_vector_xyz_field_is_rejected():
    missing = np.zeros(1, dtype=[("x", np.float32), ("y", np.float32)])
    vector = np.zeros(
        1,
        dtype=[("x", np.float32, (2,)), ("y", np.float32), ("z", np.float32)],
    )

    for points in (missing, vector):
        try:
            MODULE._inside_body_box(
                points,
                _transform(),
                np.zeros(3),
                np.ones(3),
            )
        except ValueError:
            pass
        else:
            raise AssertionError("invalid xyz layout was accepted")
