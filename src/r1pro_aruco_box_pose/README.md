# R1 Pro ArUco Box Pose

Detects the ArUco marker on the box from the R1 Pro head camera and publishes the box 6D pose in the camera frame.

## Default Inputs

- Image: `/hdas/camera_head/left_raw/image_raw_color/compressed`
- Marker dictionaries: `DICT_5X5_50`
- Marker grid: `5x5`
- Marker ID: `0`
- Marker side length: configured by `marker_length_m`
- Live processing and annotated image publishing are throttled to `2Hz` by `process_period_s: 0.5`.

## Change Marker Parameters

Edit `config/head_left_aruco_box_pose.yaml` when changing the printed code:

```yaml
marker_size_bits: 5
aruco_dictionaries: [DICT_5X5_50]
marker_target_ids: [0, 1]
marker_target_lengths_m: [0.08, 0.10]
marker_target_dictionaries: [DICT_5X5_50, DICT_5X5_50]
marker_target_size_bits: [5, 5]
marker_topic_suffixes: ["0", "1"]
```

This example publishes:

```text
/r1pro_aruco_box_pose/box_pose_camera_0
/r1pro_aruco_box_pose/box_pose_camera_1
/r1pro_aruco_box_pose/marker_pose_camera_0
/r1pro_aruco_box_pose/marker_pose_camera_1
```

`aruco_dictionaries` accepts multiple OpenCV ArUco dictionaries, but keeping only the actual printed dictionary gives the lowest false-positive rate.

## Still Image Detection First

Recommended launch command for a saved image:

```bash
cd /home/nvidia/galaxea/galaxea_overlay
source install/setup.bash
ros2 launch r1pro_aruco_box_pose aruco_box_pose.launch.py \
  input_mode:=image \
  image_path:=/home/nvidia/galaxea/galaxea_overlay/images/5x5_0.jpg \
  output_image_path:=/home/nvidia/galaxea/galaxea_overlay/images/5x5_0_launch_result.jpg
```

If `output_image_path` is empty, the result is saved next to the input image as `*_aruco_result.*`.

The image mode uses the same YAML marker settings and calibrated camera intrinsics as the live camera node. It reads one image, draws the detected marker center, 3D axes, xyz/rpy text, saves the result image, and exits.

The older standalone script can also be used for quick debugging:

```bash
python3 /home/nvidia/galaxea/galaxea_overlay/src/r1pro_aruco_box_pose/scripts/detect_aruco_image.py \
  /home/nvidia/galaxea/galaxea_overlay/images/5x5_0.jpg
```

The script prints the detected ID and four image-space corners, and writes:

```text
/home/nvidia/galaxea/galaxea_overlay/images/5x5_0_aruco_detected.jpg
```

It always prints the marker center in image pixels:

```text
center_px: u, v
```

To compute the marker-center 6D pose in the camera frame, pass the calibrated intrinsics:

```bash
python3 /home/nvidia/galaxea/galaxea_overlay/src/r1pro_aruco_box_pose/scripts/detect_aruco_image.py \
  /home/nvidia/galaxea/galaxea_overlay/images/5x5_0.jpg \
  --marker-length-m 0.10 \
  --camera-matrix="fx,0,cx,0,fy,cy,0,0,1" \
  --dist-coeffs="k1,k2,p1,p2,k3"
```

Replace `fx/cx/fy/cy/k1...` with real calibration numbers. If they are still placeholders, the script still saves the 2D detection result but cannot compute a metric 6D pose.

For the calibrated head-left camera in this workspace:

```bash
python3 /home/motrix/galaxea/galaxea_overlay/src/r1pro_aruco_box_pose/scripts/detect_aruco_image.py \
  /home/motrix/galaxea/galaxea_overlay/images/5x5_0.jpg \
  --marker-length-m 0.10 \
  --camera-matrix="1053.86460,0.0,982.42667,0.0,1046.78350,595.89145,0.0,0.0,1.0" \
  --dist-coeffs="-0.327196,0.084977,-0.006612,-0.001204,0.0" \
  --output /home/motrix/galaxea/galaxea_overlay/images/5x5_0_center_pose_left.jpg
```

These numbers are the unrectified head-left raw calibration from
`/home/motrix/galaxea/galaxea_overlay/calibrationdata1080.tar.gz`
(`left.yaml`, image size 1920x1080). The live ArUco node uses the same values in
`config/head_left_aruco_box_pose.yaml`, so Web ArUco fine adjustment receives
pose feedback computed with this calibration.

Then the output includes:

```text
center_6d_pose_camera:
  xyz_m: x, y, z
  rpy_rad: roll, pitch, yaw
  rpy_deg: roll, pitch, yaw
  rvec: rx, ry, rz
```

The annotated image also draws the marker center and projected 3D axes.

To choose the saved image path:

```bash
python3 /home/nvidia/galaxea/galaxea_overlay/src/r1pro_aruco_box_pose/scripts/detect_aruco_image.py \
  /home/nvidia/galaxea/galaxea_overlay/images/5x5_0.jpg \
  --output /home/nvidia/galaxea/galaxea_overlay/images/5x5_0_result.jpg
```

You can also run it after building:

```bash
ros2 run r1pro_aruco_box_pose detect_aruco_image.py /home/nvidia/galaxea/galaxea_overlay/images/5x5_0.jpg
```

## Outputs

- `/r1pro_aruco_box_pose/box_pose_camera_<suffix>` (`geometry_msgs/msg/PoseStamped`)
- `/r1pro_aruco_box_pose/marker_pose_camera_<suffix>` (`geometry_msgs/msg/PoseStamped`)
- `/r1pro_aruco_box_pose/box_pose_base_link_<suffix>` (`geometry_msgs/msg/PoseStamped`)
- `/r1pro_aruco_box_pose/marker_pose_base_link_<suffix>` (`geometry_msgs/msg/PoseStamped`)
- `/r1pro_aruco_box_pose/aruco_0_pose_in_aruco_1` (`geometry_msgs/msg/PoseStamped`)
- `/r1pro_aruco_box_pose/annotated_image` (`sensor_msgs/msg/Image`)
- `/r1pro_aruco_box_pose/markers` (`visualization_msgs/msg/MarkerArray`)

`box_pose_camera.header.frame_id` is the camera frame from the image header, or `camera_head_left_link` if the header is empty.
The `*_base_link_*` topics require a valid TF from `camera_head_left_link` to `base_link`.
If the TF is missing, the node continues publishing camera-frame poses and prints a throttled warning.
The relative topic publishes ArUco id `0` expressed in ArUco id `1` frame when both markers are verified in the same image frame.

## Fill Camera Intrinsics

Edit `config/head_left_aruco_box_pose.yaml` after calibration:

```yaml
camera_matrix: [fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0]
dist_coeffs: [k1, k2, p1, p2, k3]
```

If a valid `CameraInfo` topic is available, set:

```yaml
use_camera_info: true
camera_info_topic: /your/camera_info
```

## Run

Before checking `base_link` output, confirm the TF is available:

```bash
ros2 run tf2_ros tf2_echo base_link camera_head_left_link
```

```bash
cd /home/nvidia/galaxea/galaxea_overlay
colcon build --packages-select r1pro_aruco_box_pose
source install/setup.bash
ros2 launch r1pro_aruco_box_pose aruco_box_pose.launch.py
```

The node subscribes:

```text
/hdas/camera_head/left_raw/image_raw_color/compressed
```

It publishes the box pose in the camera frame, one topic per configured marker:

```bash
ros2 topic echo /r1pro_aruco_box_pose/box_pose_camera_0
ros2 topic echo /r1pro_aruco_box_pose/box_pose_camera_1
ros2 topic echo /r1pro_aruco_box_pose/box_pose_base_link_0
ros2 topic echo /r1pro_aruco_box_pose/box_pose_base_link_1
ros2 topic echo /r1pro_aruco_box_pose/aruco_0_pose_in_aruco_1
```

By default the live node only publishes poses from conservative OpenCV ArUco detection on the original/gray image.
It also runs strict publish gates to reduce false positives: exact marker bit verification, pose bounds, shape checks, and 3 stable consecutive detections.
The wider adaptive/candidate fallback can be enabled for debugging, but it can mis-detect black/white textures as the target ID.
Those unverified detections are drawn on `/r1pro_aruco_box_pose/annotated_image` and are suppressed from the pose topics unless explicitly allowed:

```bash
ros2 launch r1pro_aruco_box_pose aruco_box_pose.launch.py \
  use_adaptive_variants:=true \
  use_candidate_fallback:=true
```

Only use this unsafe override when you deliberately want fallback/adaptive detections to publish pose messages:

```bash
ros2 launch r1pro_aruco_box_pose aruco_box_pose.launch.py \
  use_adaptive_variants:=true \
  use_candidate_fallback:=true \
  publish_unverified_detections:=true
```

For the lowest false-positive rate, keep these defaults in `config/head_left_aruco_box_pose.yaml`:

```yaml
strict_marker_bit_check: true
max_marker_bit_mismatches: 0
required_stable_detections: 3
publish_unverified_detections: false
use_adaptive_variants: false
use_candidate_fallback: false
```

The annotated image is published on:

```bash
ros2 topic echo /r1pro_aruco_box_pose/annotated_image --once
```

## Box Frame Offset

By default the box frame is the same as the marker frame. If the desired box coordinate system is at another point on the box, set:

```yaml
box_translation_in_marker_frame_m: [x, y, z]
box_rpy_in_marker_frame_rad: [roll, pitch, yaw]
```
