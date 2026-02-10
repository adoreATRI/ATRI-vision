## [0.1.0] - 2026-01-29
### Refactored
- 重构detector.cpp的图像处理流程，将图像预处理单独封装为processImage函数

### Chore
- 添加.gitignore文件，忽略编译生成的文件

## [0.1.1] - 2026-01-29
### Perfomance
- 优化ColorBlock检测的图像预处理流程，提升检测稳定性

## [0.1.2] - 2026-01-30
### Refactored
- 重构detector.cpp大部分代码

### Perfomance
- 优化颜色差异检测，提高颜色识别准确率

## [0.1.3] - 2026-02-03
### Feature
- 完成tracker的demo，但是功能未能实现

## [0.1.4] - 2026-02-04
### Bug Fixes
- 修复tracker中无法预测的问题，但是仍然无法准确追踪

## [0.1.5] - 2026-02-05
### Perfomance
- 使用ITERATIVE进行PnP解算，同时采用角点精细化，提高位姿解算精度

## [0.1.6] - 2026-02-09
### Bug Fixes
- 修复了观测函数，使其更加准确地反映目标状态

## [0.2.0] - 2026-02-10
### Chore
- 对旋转轴的确定进行改进，优化了tracker的稳定性