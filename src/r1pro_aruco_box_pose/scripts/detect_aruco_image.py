#!/usr/bin/env python3

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np


DEFAULT_IMAGE = "/home/nvidia/galaxea/galaxea_overlay/images/5x5_0.jpg"


@dataclass
class Candidate:
    quad: np.ndarray
    area: float
    threshold: int


@dataclass
class Detection:
    marker_id: int
    corners: np.ndarray
    source: str
    score: float


@dataclass
class CenterPose:
    rvec: np.ndarray
    tvec: np.ndarray
    rpy_rad: Tuple[float, float, float]


def make_dictionary(name: str):
    dictionary_name = name if name.startswith("DICT_") else f"DICT_{name}"
    if not hasattr(cv2.aruco, dictionary_name):
        known = ", ".join(sorted(item for item in dir(cv2.aruco) if item.startswith("DICT_")))
        raise ValueError(f"unknown ArUco dictionary '{name}'. Known dictionaries: {known}")
    dictionary_id = getattr(cv2.aruco, dictionary_name)
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dictionary_id)
    return cv2.aruco.Dictionary_get(dictionary_id)


def detector_parameters(relaxed: bool):
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 30
    if hasattr(params, "detectInvertedMarker"):
        params.detectInvertedMarker = True
    if relaxed:
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 101
        params.adaptiveThreshWinSizeStep = 10
        params.errorCorrectionRate = 1.0
        params.minOtsuStdDev = 0.0
        params.minMarkerPerimeterRate = 0.01
        params.minCornerDistanceRate = 0.005
        params.minDistanceToBorder = 1
    return params


def detect_markers(image, dictionary, relaxed: bool):
    params = detector_parameters(relaxed)
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return detector.detectMarkers(image)
    return cv2.aruco.detectMarkers(image, dictionary, parameters=params)


def order_points(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    total = pts.sum(axis=1)
    diff = np.diff(pts, axis=1).reshape(-1)
    ordered = np.zeros((4, 2), dtype=np.float32)
    ordered[0] = pts[np.argmin(total)]
    ordered[2] = pts[np.argmax(total)]
    ordered[1] = pts[np.argmin(diff)]
    ordered[3] = pts[np.argmax(diff)]
    return ordered


def expand_quad(quad: np.ndarray, scale: float) -> np.ndarray:
    center = quad.mean(axis=0)
    return (quad - center) * float(scale) + center


def quad_aspect_ratio(quad: np.ndarray) -> float:
    top = np.linalg.norm(quad[1] - quad[0])
    bottom = np.linalg.norm(quad[2] - quad[3])
    left = np.linalg.norm(quad[3] - quad[0])
    right = np.linalg.norm(quad[2] - quad[1])
    width = 0.5 * (top + bottom)
    height = max(1e-6, 0.5 * (left + right))
    return float(width / height)


def deduplicate_candidates(candidates: Iterable[Candidate], center_tolerance_px: float) -> List[Candidate]:
    unique: List[Candidate] = []
    for candidate in sorted(candidates, key=lambda item: item.area, reverse=True):
        center = candidate.quad.mean(axis=0)
        duplicate = False
        for kept in unique:
            if np.linalg.norm(center - kept.quad.mean(axis=0)) < center_tolerance_px:
                duplicate = True
                break
        if not duplicate:
            unique.append(candidate)
    return unique


def find_square_candidates(
    image_bgr: np.ndarray,
    thresholds: Sequence[int],
    min_area: float,
    max_area: float,
    aspect_min: float,
    aspect_max: float,
) -> List[Candidate]:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    candidates: List[Candidate] = []
    for threshold in thresholds:
        mask = np.where(gray < int(threshold), 255, 0).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area or area > max_area:
                continue
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            if len(approx) != 4:
                continue
            quad = order_points(approx.reshape(-1, 2))
            aspect = quad_aspect_ratio(quad)
            if aspect < aspect_min or aspect > aspect_max:
                continue
            if not cv2.isContourConvex(quad.astype(np.float32)):
                continue
            candidates.append(Candidate(quad=quad, area=area, threshold=int(threshold)))
    return deduplicate_candidates(candidates, center_tolerance_px=20.0)


def image_variants(image_bgr: np.ndarray):
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    yield "bgr", image_bgr
    yield "gray", cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    yield "clahe", cv2.cvtColor(clahe, cv2.COLOR_GRAY2BGR)
    _, otsu = cv2.threshold(clahe, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    yield "otsu", cv2.cvtColor(otsu, cv2.COLOR_GRAY2BGR)
    for block_size in (31, 51, 71, 91, 121):
        adaptive = cv2.adaptiveThreshold(
            clahe,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            block_size,
            3,
        )
        yield f"adaptive{block_size}", cv2.cvtColor(adaptive, cv2.COLOR_GRAY2BGR)


def basic_image_variants(image_bgr: np.ndarray):
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    return [
        ("bgr", image_bgr),
        ("gray", cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)),
    ]


def target_matches(marker_id: int, target_id: int) -> bool:
    return target_id < 0 or int(marker_id) == int(target_id)


def marker_area(corners: np.ndarray) -> float:
    return abs(float(cv2.contourArea(corners.reshape(4, 2).astype(np.float32))))


def marker_center(corners: np.ndarray) -> np.ndarray:
    return corners.reshape(4, 2).mean(axis=0)


def rpy_from_rotation_matrix(rotation: np.ndarray) -> Tuple[float, float, float]:
    sy = math.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def estimate_center_pose(
    detection: Detection,
    marker_length_m: float,
    camera_matrix: Optional[np.ndarray],
    dist_coeffs: Optional[np.ndarray],
) -> Optional[CenterPose]:
    if camera_matrix is None:
        return None
    if dist_coeffs is None:
        dist_coeffs = np.zeros((5, 1), dtype=np.float64)
    try:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [detection.corners.astype(np.float32)],
            marker_length_m,
            camera_matrix,
            dist_coeffs,
        )
    except cv2.error:
        return None
    rvec = rvecs[0].reshape(3, 1)
    tvec = tvecs[0].reshape(3, 1)
    rotation, _ = cv2.Rodrigues(rvec)
    return CenterPose(rvec=rvec, tvec=tvec, rpy_rad=rpy_from_rotation_matrix(rotation))


def detect_direct(
    image_bgr: np.ndarray,
    dictionary,
    target_id: int,
    min_area: float,
    use_adaptive_variants: bool,
) -> List[Detection]:
    detections: List[Detection] = []
    variants = image_variants(image_bgr) if use_adaptive_variants else basic_image_variants(image_bgr)
    for variant_name, variant in variants:
        corners, ids, _ = detect_markers(variant, dictionary, relaxed=False)
        if ids is None:
            continue
        for index, marker_id in enumerate(ids.flatten().astype(int)):
            corners_px = corners[index].reshape(4, 2).astype(np.float32)
            area = marker_area(corners_px)
            if area < min_area or not target_matches(marker_id, target_id):
                continue
            detections.append(
                Detection(
                    marker_id=int(marker_id),
                    corners=corners_px,
                    source=f"direct:standard:{variant_name}",
                    score=area,
                )
            )
    return detections


def detect_from_candidate(
    image_bgr: np.ndarray,
    candidate: Candidate,
    dictionary,
    target_id: int,
    min_decoded_area: float,
    warp_size: int,
    pad: int,
    expansion_scales: Sequence[float],
) -> List[Detection]:
    detections: List[Detection] = []
    dst = np.float32(
        [
            [0.0, 0.0],
            [warp_size - 1.0, 0.0],
            [warp_size - 1.0, warp_size - 1.0],
            [0.0, warp_size - 1.0],
        ]
    )
    for scale in expansion_scales:
        quad = expand_quad(candidate.quad, scale).astype(np.float32)
        transform = cv2.getPerspectiveTransform(quad, dst)
        inverse_transform = np.linalg.inv(transform)
        warp = cv2.warpPerspective(image_bgr, transform, (warp_size, warp_size))
        padded = cv2.copyMakeBorder(warp, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=(255, 255, 255))
        for variant_name, variant in image_variants(padded):
            corners, ids, _ = detect_markers(variant, dictionary, relaxed=True)
            if ids is None:
                continue
            for index, marker_id in enumerate(ids.flatten().astype(int)):
                if not target_matches(marker_id, target_id):
                    continue
                padded_corners = corners[index].reshape(4, 2).astype(np.float32)
                warp_corners = padded_corners - np.float32([pad, pad])
                original_corners = cv2.perspectiveTransform(
                    warp_corners.reshape(1, 4, 2), inverse_transform
                ).reshape(4, 2)
                decoded_area = marker_area(original_corners)
                if decoded_area < min_decoded_area:
                    continue
                detections.append(
                    Detection(
                        marker_id=int(marker_id),
                        corners=order_points(original_corners),
                        source=f"candidate:t{candidate.threshold}:scale{scale:.2f}:{variant_name}",
                        score=decoded_area,
                    )
                )
    return detections


def choose_detection(detections: Sequence[Detection], target_id: int) -> Optional[Detection]:
    if not detections:
        return None
    matches = [item for item in detections if target_matches(item.marker_id, target_id)]
    if not matches:
        return None
    return max(matches, key=lambda item: (detection_priority(item), item.score))


def detection_priority(detection: Detection) -> int:
    if detection.source in ("direct:standard:bgr", "direct:standard:gray"):
        return 2
    if detection.source.startswith("direct:"):
        return 1
    return 0


def annotate(
    image_bgr: np.ndarray,
    detection: Optional[Detection],
    candidates: Sequence[Candidate],
    draw_candidates: bool,
    pose: Optional[CenterPose],
    camera_matrix: Optional[np.ndarray],
    dist_coeffs: Optional[np.ndarray],
    axis_length_m: float,
) -> np.ndarray:
    annotated = image_bgr.copy()
    if draw_candidates:
        for candidate in candidates:
            quad = candidate.quad.astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(annotated, [quad], True, (0, 220, 255), 1, cv2.LINE_AA)

    if detection is None:
        return annotated

    corners = detection.corners.astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(annotated, [corners], True, (0, 255, 0), 3, cv2.LINE_AA)
    center = marker_center(detection.corners)
    cv2.circle(annotated, (int(center[0]), int(center[1])), 7, (255, 0, 255), -1, cv2.LINE_AA)
    for index, point in enumerate(detection.corners.astype(int)):
        cv2.circle(annotated, tuple(point), 5, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.putText(
            annotated,
            str(index),
            (int(point[0]) + 6, int(point[1]) - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

    top_left = detection.corners[0].astype(int)
    label = f"ArUco id={detection.marker_id} center=({center[0]:.1f},{center[1]:.1f})"
    cv2.putText(
        annotated,
        label,
        (int(top_left[0]), max(24, int(top_left[1]) - 14)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 0, 0),
        4,
        cv2.LINE_AA,
    )
    cv2.putText(
        annotated,
        label,
        (int(top_left[0]), max(24, int(top_left[1]) - 14)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    if pose is not None and camera_matrix is not None:
        if dist_coeffs is None:
            dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        try:
            if hasattr(cv2.aruco, "drawAxis"):
                cv2.aruco.drawAxis(
                    annotated,
                    camera_matrix,
                    dist_coeffs,
                    pose.rvec,
                    pose.tvec,
                    axis_length_m,
                )
            else:
                cv2.drawFrameAxes(
                    annotated,
                    camera_matrix,
                    dist_coeffs,
                    pose.rvec,
                    pose.tvec,
                    axis_length_m,
                )
        except cv2.error:
            pass
        x, y, z = pose.tvec.reshape(3)
        roll, pitch, yaw = pose.rpy_rad
        pose_lines = [
            "center xyz %.3f %.3f %.3f m" % (x, y, z),
            "rpy %.1f %.1f %.1f deg"
            % (math.degrees(roll), math.degrees(pitch), math.degrees(yaw)),
        ]
        for line_index, line in enumerate(pose_lines):
            cv2.putText(
                annotated,
                line,
                (int(top_left[0]), max(52, int(top_left[1]) - 14) + 28 * (line_index + 1)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 0),
                4,
                cv2.LINE_AA,
            )
            cv2.putText(
                annotated,
                line,
                (int(top_left[0]), max(52, int(top_left[1]) - 14) + 28 * (line_index + 1)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
    return annotated


def parse_int_list(text: str) -> List[int]:
    return [int(item.strip()) for item in text.split(",") if item.strip()]


def parse_float_list(text: str) -> List[float]:
    return [float(item) for item in re.findall(r"[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?", text)]


def has_placeholder_letters(text: str) -> bool:
    return re.search(r"[A-DF-Za-df-z]", text) is not None


def parse_camera_matrix(text: str) -> Optional[np.ndarray]:
    if not text:
        return None
    if has_placeholder_letters(text):
        print(
            "warning: --camera-matrix still contains placeholders like fx/cx/fy/cy; "
            "saving detection image without metric 6D pose"
        )
        return None
    values = parse_float_list(text)
    if len(values) != 9:
        print(
            "warning: --camera-matrix must contain 9 numeric values; "
            "saving detection image without metric 6D pose"
        )
        return None
    matrix = np.asarray(values, dtype=np.float64).reshape(3, 3)
    if abs(float(matrix[0, 0])) <= 1e-9 or abs(float(matrix[1, 1])) <= 1e-9:
        print(
            "warning: --camera-matrix has invalid fx/fy; "
            "saving detection image without metric 6D pose"
        )
        return None
    return matrix


def parse_dist_coeffs(text: str) -> Optional[np.ndarray]:
    if not text:
        return None
    if has_placeholder_letters(text):
        print(
            "warning: --dist-coeffs still contains placeholders like k1/p1; "
            "using zero distortion until real calibration values are provided"
        )
        return None
    values = parse_float_list(text)
    if not values:
        return None
    return np.asarray(values, dtype=np.float64).reshape(-1, 1)


def default_output_path(image_path: Path) -> Path:
    return image_path.with_name(f"{image_path.stem}_aruco_detected{image_path.suffix}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Detect the box ArUco marker in a still image.")
    parser.add_argument("image", nargs="?", default=DEFAULT_IMAGE, help="input image path")
    parser.add_argument("--dictionary", default="DICT_5X5_50", help="OpenCV ArUco dictionary")
    parser.add_argument("--target-id", type=int, default=0, help="target marker id, or -1 to accept any")
    parser.add_argument("--marker-length-m", type=float, default=0.10, help="marker side length in meters")
    parser.add_argument(
        "--camera-matrix",
        default="",
        help='9 numbers, for example "fx,0,cx,0,fy,cy,0,0,1"',
    )
    parser.add_argument(
        "--dist-coeffs",
        default="",
        help='distortion coefficients, for example "k1,k2,p1,p2,k3"; omitted means zero distortion',
    )
    parser.add_argument("--axis-length-m", type=float, default=0.10)
    parser.add_argument("--output", default="", help="annotated output image path")
    parser.add_argument("--min-area", type=float, default=1500.0, help="minimum candidate area in pixels")
    parser.add_argument("--max-area", type=float, default=60000.0, help="maximum candidate area in pixels")
    parser.add_argument("--aspect-min", type=float, default=0.65)
    parser.add_argument("--aspect-max", type=float, default=1.55)
    parser.add_argument("--thresholds", default="40,50,60,70,80,90,100")
    parser.add_argument("--expansion-scales", default="0.96,1.0,1.03,1.06,1.10,1.15,1.20")
    parser.add_argument("--warp-size", type=int, default=420)
    parser.add_argument("--pad", type=int, default=90)
    parser.add_argument("--draw-candidates", action="store_true")
    parser.add_argument("--use-adaptive-variants", action="store_true")
    parser.add_argument("--use-candidate-fallback", action="store_true")
    args = parser.parse_args()

    image_path = Path(args.image).expanduser()
    image_bgr = cv2.imread(str(image_path))
    if image_bgr is None:
        raise SystemExit(f"failed to read image: {image_path}")

    dictionary = make_dictionary(args.dictionary)
    thresholds = parse_int_list(args.thresholds)
    expansion_scales = parse_float_list(args.expansion_scales)
    camera_matrix = parse_camera_matrix(args.camera_matrix)
    dist_coeffs = parse_dist_coeffs(args.dist_coeffs)

    detections = detect_direct(
        image_bgr,
        dictionary,
        args.target_id,
        args.min_area,
        args.use_adaptive_variants,
    )
    candidates: List[Candidate] = []
    if args.use_candidate_fallback or args.draw_candidates:
        candidates = find_square_candidates(
            image_bgr,
            thresholds=thresholds,
            min_area=args.min_area,
            max_area=args.max_area,
            aspect_min=args.aspect_min,
            aspect_max=args.aspect_max,
        )
    if args.use_candidate_fallback:
        for candidate in candidates:
            detections.extend(
                detect_from_candidate(
                    image_bgr,
                    candidate,
                    dictionary,
                    args.target_id,
                    min_decoded_area=args.min_area,
                    warp_size=args.warp_size,
                    pad=args.pad,
                    expansion_scales=expansion_scales,
                )
            )

    detection = choose_detection(detections, args.target_id)
    pose = None
    if detection is not None:
        pose = estimate_center_pose(detection, args.marker_length_m, camera_matrix, dist_coeffs)
    output_path = Path(args.output).expanduser() if args.output else default_output_path(image_path)
    annotated = annotate(
        image_bgr,
        detection,
        candidates,
        args.draw_candidates,
        pose,
        camera_matrix,
        dist_coeffs,
        args.axis_length_m,
    )
    cv2.imwrite(str(output_path), annotated)

    print(f"image: {image_path}")
    print(f"dictionary: {args.dictionary}")
    print(f"target_id: {args.target_id}")
    print(f"candidate_count: {len(candidates)}")
    print(f"detection_count: {len(detections)}")
    print(f"annotated: {output_path}")
    if detection is None:
        print("result: not found")
        return 2

    print("result: found")
    print(f"id: {detection.marker_id}")
    print(f"source: {detection.source}")
    center = marker_center(detection.corners)
    print("center_px: %.2f, %.2f" % (center[0], center[1]))
    print("corners_px_opencv_order:")
    for point in detection.corners:
        print("  %.2f, %.2f" % (point[0], point[1]))
    if pose is None:
        print("center_6d_pose_camera: unavailable_without_camera_intrinsics")
        print("hint: pass --camera-matrix and --dist-coeffs to compute metric 6D pose")
    else:
        x, y, z = pose.tvec.reshape(3)
        roll, pitch, yaw = pose.rpy_rad
        print("center_6d_pose_camera:")
        print("  xyz_m: %.6f, %.6f, %.6f" % (x, y, z))
        print(
            "  rpy_rad: %.6f, %.6f, %.6f"
            % (roll, pitch, yaw)
        )
        print(
            "  rpy_deg: %.3f, %.3f, %.3f"
            % (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
        )
        print(
            "  rvec: %.6f, %.6f, %.6f"
            % (pose.rvec[0, 0], pose.rvec[1, 0], pose.rvec[2, 0])
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
