# Development Guidelines for ATRI Vision

This repository contains the ROS 2 packages for the ATRI vision system. Follow these guidelines for building, testing, and contributing code.

## 1. Build and Test Commands

The project uses the standard ROS 2 `colcon` build system.

### Build
To build the entire workspace:
```bash
colcon build --symlink-install
```
*   `--symlink-install` is recommended to avoid rebuilding when changing Python scripts or config files.
*   **Environment Variables:** Ensure `ONNXRUNTIME_DIR` is set if working with `atri_detector`.

To build a specific package:
```bash
colcon build --packages-select <package_name> --symlink-install
```

### Test & Lint
Tests and linters are integrated via `ament_cmake`.

To run all tests (including linters):
```bash
colcon test
```

To run tests for a specific package:
```bash
colcon test --packages-select <package_name>
```

To see test output on failure:
```bash
colcon test --event-handlers console_direct+
```

### Running a Single Test
If using GTest (C++):
1.  Find the test executable in `build/<package_name>/test_executable_name` (or similar).
2.  Run it directly:
    ```bash
    ./build/<package_name>/test_executable_name --gtest_filter=TestSuiteName.TestName
    ```

Alternatively, use `colcon test` with regex:
```bash
colcon test --packages-select <package_name> --ctest-args -R <regex_for_test_name>
```

## 2. Code Style Guidelines

### C++ (ROS 2 Packages)
*   **Style:** Follows Google C++ Style with modifications.
*   **Formatter:** Use `clang-format`. Configuration is in `.clang-format`.
    *   Run: `clang-format -i path/to/file.cpp`
*   **Naming Conventions:**
    *   **Packages:** `snake_case` (e.g., `atri_detector`)
    *   **Classes:** `PascalCase` (e.g., `DetectorNode`)
    *   **Files:** `snake_case` (e.g., `detector_node.cpp`)
    *   **Variables/Functions:** `snake_case` (e.g., `process_image`, `image_subscriber_`)
    *   **Member Variables:** Trailing underscore `_` (e.g., `node_handle_`)
    *   **Constants:** `ALL_CAPS`
*   **Headers:** Use `#pragma once` or include guards. Include order: standard libs -> 3rd party -> ROS -> local.
*   **Dependencies:** Check `CMakeLists.txt` and `package.xml` before adding new dependencies.
*   **Smart Pointers:** Prefer `std::shared_ptr`, `std::unique_ptr`, or ROS 2 specific aliases (e.g., `rclcpp::Node::SharedPtr`). Avoid raw pointers (`*`) unless absolutely necessary for non-owning references.

### Python (Scripts/Nodes)
*   **Style:** PEP 8.
*   **Linter:** `flake8` (standard for ROS 2).
*   **Formatting:** `black` or `autopep8` is recommended.
*   **Naming:**
    *   Classes: `PascalCase`
    *   Functions/Variables: `snake_case`

### Error Handling
*   **Exceptions:** Use standard C++ exceptions or `rclcpp::exceptions` where appropriate.
*   **Logging:** Use ROS 2 logging macros:
    *   `RCLCPP_INFO(this->get_logger(), "Message")`
    *   `RCLCPP_WARN(...)`, `RCLCPP_ERROR(...)`
    *   Do not use `std::cout` or `printf` in nodes.

## 3. Repository Structure
*   `src/`: Contains ROS 2 packages.
    *   `atri_detector/`: Object detection node (C++, ONNX, OpenCV).
    *   `atri_tracker/`: Tracking logic.
    *   `atri_interfaces/`: Custom ROS messages and services.
*   `yolo8/`: YOLO training scripts and utilities (Python).

## 4. Environment
*   **Source:** Always source the setup file before running nodes:
    ```bash
    source install/setup.bash
    ```
