"""Compare a posed CAD rendering with synchronized measured RGB-D."""
import cv2
import numpy as np


def assess_pose_quality(rgb, depth, rendered_rgb, rendered_depth, target_mask=None):
    silhouette = np.isfinite(rendered_depth) & (rendered_depth > 0)
    pixels = int(silhouette.sum())
    result = {'render_pixels': pixels, 'valid': False, 'reasons': []}
    if pixels < 100:
        result['reasons'].append('model has insufficient visible area')
        return result
    # Erode edges where stereo is less reliable; preserve narrow geometry.
    interior = cv2.erode(silhouette.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
    if interior.sum() < pixels * .4:
        interior = silhouette
    valid_depth = interior & np.isfinite(depth) & (depth > 0)
    fraction = float(valid_depth.sum() / max(1, interior.sum()))
    result['valid_depth_fraction'] = round(fraction, 3)
    if fraction < .25:
        result['reasons'].append('insufficient observed depth on model')
    else:
        residual = np.abs(depth[valid_depth] - rendered_depth[valid_depth])
        object_depth = float(np.median(rendered_depth[valid_depth]))
        tolerance = float(np.clip(.025 + .015 * object_depth ** 2, .04, .10))
        median = float(np.median(residual))
        inliers = float(np.mean(residual <= tolerance))
        result.update(depth_median_error_m=round(median, 4), depth_inlier_fraction=round(inliers, 3), depth_tolerance_m=round(tolerance, 4))
        if median > tolerance or inliers < .5:
            result['reasons'].append('rendered model does not match observed depth')
    real_hsv = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2HSV)
    expected_hsv = cv2.cvtColor(np.ascontiguousarray(rendered_rgb), cv2.COLOR_RGB2HSV)
    colored = interior & (expected_hsv[..., 1] > 80) & (expected_hsv[..., 2] > 35)
    color_fraction = float(colored.sum() / max(1, interior.sum()))
    result['colored_model_fraction'] = round(color_fraction, 3)
    if color_fraction >= .25:
        hue_error = np.abs(real_hsv[..., 0].astype(np.int16) - expected_hsv[..., 0].astype(np.int16))
        hue_error = np.minimum(hue_error, 180 - hue_error)
        matches = (hue_error <= 18) & (real_hsv[..., 1] > 50) & (real_hsv[..., 2] > 25)
        agreement = float(matches[colored].mean())
        result['color_agreement'] = round(agreement, 3)
        if agreement < .35:
            result['reasons'].append('model appearance is not on the current target')
        # For a predominantly single-color asset, also compare its current
        # connected silhouette.  Color/depth agreement alone can miss a wrong
        # yaw on a flat box bottom with an otherwise matching blue texture.
        histogram = np.bincount(expected_hsv[..., 0][colored], minlength=180)
        dominant = int(histogram.argmax())
        expected_delta = np.abs(expected_hsv[..., 0].astype(np.int16)-dominant)
        expected_delta = np.minimum(expected_delta, 180-expected_delta)
        dominant_fraction = float((expected_delta[colored] <= 18).mean())
        if color_fraction >= .65 and dominant_fraction >= .65:
            delta = np.abs(real_hsv[..., 0].astype(np.int16)-dominant)
            delta = np.minimum(delta, 180-delta)
            observed_mask = ((delta <= 18) & (real_hsv[..., 1] > 50) & (real_hsv[..., 2] > 25)).astype(np.uint8)
            observed_mask = cv2.morphologyEx(observed_mask, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
            contours, _ = cv2.findContours(observed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best = None
            best_overlap = 0
            for contour in contours:
                if cv2.contourArea(contour) < 100:
                    continue
                component = np.zeros_like(observed_mask)
                cv2.drawContours(component, [contour], -1, 1, cv2.FILLED)
                overlap = int((component.astype(bool) & silhouette).sum())
                if overlap > best_overlap:
                    best, best_overlap = component.astype(bool), overlap
            silhouette_iou = 0.0 if best is None else float(best_overlap / max(1, int((best | silhouette).sum())))
            result['live_silhouette_iou'] = round(silhouette_iou, 3)
            if silhouette_iou < .60:
                result['reasons'].append('model outline does not match current target position/orientation')

    if target_mask is not None:
        target = np.asarray(target_mask, bool)
        intersection = int((silhouette & target).sum())
        coverage = intersection / pixels
        iou = intersection / max(1, int((silhouette | target).sum()))
        result.update(registration_mask_coverage=round(coverage, 3), registration_mask_iou=round(iou, 3))
        if coverage < .6 or iou < .35:
            result['reasons'].append('registered model silhouette does not match YOLO mask')
    result['valid'] = not result['reasons']
    return result
