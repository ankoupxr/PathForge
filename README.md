# PathForge


## 概述

PathForge 是一个CAM（计算机辅助制造）软件，玩具

## 技术栈

- **C++17** - 现代 C++ 编程语言
- **Qt5** - 跨平台 GUI 框架
- **VTK 9.x** - 3D 可视化引擎
- **OpenCASCADE 7.7.0+** - 几何建模内核
- **CMake 3.15+** - 跨平台构建系统

## 核心功能

### 模型导入与显示
- 支持多种 CAD 格式：STEP、IGES、STL、BRep
- 高质量 3D 可视化渲染
- 交互式旋转、缩放、平移

### 特征识别 - 智能识别加工特征

| 识别器 | 说明 | 适用加工 |
|--------|------|----------|
| **HoleRecognizer** | 孔特征识别 | 钻孔 |
| **SlotRecognizer** | 槽特征识别 | 型腔铣 |
| **PocketRecognizer** | 型腔特征识别 | 型腔铣 |
| **PlaneRecognizer** | 平面识别 | 面铣 |
| **CylinderRecognizer** | 圆柱面识别 | 钻孔 |

### 刀路生成 - 多种加工策略

| 策略 | 说明 | 特点 |
|------|------|------|
| **面铣削 2D** | 二维平面铣削 | 往复切削、单向切削、入退刀 |
| **型腔铣削** | 凹槽粗加工 | 螺旋线、平行线、往复式 |
| **轮廓铣削** | 侧壁精加工 | 补偿控制、多层切削 |
| **钻孔** | 孔加工 | 啄钻、深孔钻、普通钻 |

### 后处理模块
- 通用 G-code 生成
- 支持 G00/G01/G02/G03 指令
- 圆弧插补、螺旋线插补
- 可配置的行号、注释、坐标系
- 支持导出为 .nc、.gcode、.ncc 格式

### 仿真与验证
- 碰撞检测（刀具柄部、切削部、快速移动）
- 过切检查（轻微/中等/严重）
- 材料去除仿真
- 仿真报告导出

### 多轴加工

| 轴类型 | 说明 | 特点 |
|--------|------|------|
| **3+2 轴** | 定位五轴加工 | 三个直线轴 + 两个旋转轴定位 |
| **5轴联动** | 真正的五轴加工 | 五个轴同时运动插补 |

## 目录结构

```
PathForge/
├── Geometry/           # 几何处理模块
│   ├── CAM/            # CAM 几何原语
│   └── Bridge/         # OCC 转换层
├── Topology/           # 拓扑分析模块
│   ├── AdjacencyGraph/ # 邻接图
│   └── FaceCollector/  # 面收集器
├── Feature/           # 特征识别模块
├── Simulation/        # 仿真与验证模块
├── Path/               # 刀路生成模块
│   ├── Core/           # 核心算法
│   ├── Strategies/     # 加工策略
│   ├── Projection/      # 投影器
│   ├── PostProcess/    # 后处理模块
│   └── MultiAxis/     # 多轴加工模块
│       ├── MultiAxisConfig.h/cpp    # 配置结构
│       ├── MultiAxisPoint.h/cpp      # 多轴点数据
│       ├── ToolAxisController.h/cpp  # 刀轴控制
│       ├── Axis3Plus2Positioning.h/cpp # 3+2轴定位
│       ├── Axis5Interpolator.h/cpp   # 5轴插补
│       ├── InterferenceChecker.h/cpp # 干涉检测
│       └── MultiAxisEngine.h/cpp    # 多轴引擎
└── ...
```

## 多轴加工详解

### ToolAxisController - 刀轴控制
- 法向跟随控制（Normal Control）
- 倾斜轴控制（Tilted Control）
- 固定轴控制（Fixed Control）
- 插值控制（Interpolated Control）

### Axis3Plus2Positioning - 3+2轴定位
- 自动识别加工平面
- 计算旋转角度
- 生成定位路径
- 加工平面切换

### Axis5Interpolator - 5轴联动插补
- 线性插补
- 圆弧插补
- 曲线插补
- 轴角度计算

### InterferenceChecker - 干涉检测
- 刀具柄部干涉
- 主轴干涉
- 工件干涉
- 安全间隙设置

### MultiAxisEngine - 多轴引擎
- 5轴路径生成
- 3+2路径生成
- 干涉检查
- 综合验证

## 快捷键

| 快捷键 | 功能 |
|--------|------|
| Ctrl+O | 导入模型 |
| F5 | 识别特征 |
| F9 | 生成刀路 |
| F10 | 导出 G-code |
| F11 | 运行仿真 |

## 开发计划

- [x] 后处理模块 - G-code 生成
- [x] 刀路策略扩展 - 钻孔、型腔铣、轮廓铣
- [x] 高级特征识别 - 自动识别孔、槽、型腔
- [x] 仿真与验证 - 碰撞检测、过切检查
- [x] 多轴加工 - 3+2 轴、5 轴联动
