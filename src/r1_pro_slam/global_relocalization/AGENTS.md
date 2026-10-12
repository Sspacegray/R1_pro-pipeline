# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 C++ package named `indoor_location`. Core source lives under `src/`, organized by localization subsystem: `cloud_process/` for LiDAR preprocessing, `p2pl_icp/` for point-to-plane ICP, `global_relocalization/` for BBS/global relocalization, `eskf/` for filtering, `pose_persistence/` for saved pose recovery, and `utility/` for shared helpers and tests. Public headers and common parameters are in `include/`. Runtime configuration is in `config/`, launch files are in `launch/`, and custom ROS messages are in `msg/`.

## Build, Test, and Development Commands

- `colcon build --packages-select indoor_location`: build this package in a ROS 2 workspace.
- `colcon test --packages-select indoor_location`: run package tests registered through `ament_cmake_gtest`.
- `colcon test-result --verbose`: inspect test failures after a test run.
- `ros2 launch indoor_location run_rs16.launch.py`: run the RS16 LiDAR localization setup.
- `ros2 launch indoor_location run_qr100_livox_mid360.launch.py`: run the QR100/Livox MID360 setup.

Run these from the workspace root after sourcing the appropriate ROS 2 environment and installing dependencies listed in `package.xml`.

## Coding Style & Naming Conventions

Use C++17 and follow the existing CMake/ROS 2 `ament_cmake` style. Keep indentation consistent with nearby files, generally two spaces in CMake and compact C++ formatting in source files. Use descriptive snake_case for files, directories, parameters, ROS topics, and tests, such as `pose_save_policy.cpp` and `test_global_relocalization.cpp`. Keep ROS parameters aligned with the YAML naming already used in `config/`.

## Testing Guidelines

Tests are C++ GoogleTest targets declared in `CMakeLists.txt` with `ament_add_gtest`. Place focused tests near the module they validate, following the existing `test_<feature>.cpp` pattern. Add or update tests when changing pose persistence, relocalization scoring, ICP behavior, or state transitions. Prefer deterministic unit tests for save policies and bounded relocalization logic before relying on live ROS launch testing.

## Commit & Pull Request Guidelines

The current Git history is minimal and uses a bracketed type style such as `[feat:commits]`. Prefer concise commits with an explicit type, for example `[feat:pose-restore] add stable pose recovery` or `[fix:bbs] bound yaw search window`. Pull requests should describe the behavior change, list tested commands, note affected launch/config files, and include screenshots or RViz observations when localization output changes.

## Security & Configuration Tips

Do not commit machine-specific maps, large PCD files, or private deployment paths. Keep persistence paths configurable through YAML, and avoid `/tmp` for recovery files because reboot behavior may remove saved state. Document any new external dependency in both `package.xml` and `CMakeLists.txt`.
