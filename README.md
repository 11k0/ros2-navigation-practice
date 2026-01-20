# A* + DWA 路径规划仿真

[![Open in MATLAB Online](https://www.mathworks.com/images/responsive/global/open-in-matlab-online.svg)](https://matlab.mathworks.com/open/github/v1?repo=YOUR_USERNAME/YOUR_REPO_NAME)

> 基于改进A*算法与动态窗口法(DWA)的融合路径规划系统
> 
> **核心功能：避障 + 窄路通行**

## 📁 项目结构

```
├── main.m                      # 主入口（一键运行）
├── Main_AStar_DWA_Simulation.m # 主仿真脚本
├── README.md
├── .gitignore
│
├── src/                        # 核心算法 (5个文件)
│   ├── AStar_Planner.m         # A*全局规划器
│   ├── Adaptive_DWA.m          # 自适应DWA局部规划器
│   ├── Path_Smoother.m         # 路径平滑（PCHIP插值）
│   ├── Lookahead_Point.m       # 前瞻点查找
│   └── MAP.m                   # 地图定义
│
└── utils/                      # 辅助函数 (10个文件)
    ├── distance.m              # 距离计算
    ├── insert_open.m           # A* Open表操作
    ├── min_fn.m                # 最小代价查找
    ├── node_index.m            # 节点索引
    ├── expand_array_Obs8.m     # 8方向节点扩展
    ├── Obs_array.m             # 障碍物密度计算
    ├── Line_OPEN_ST.m          # 路径优化
    ├── Line_OPEN_STtwo.m       # 路径优化
    ├── Optimal_index.m         # 最优路径索引
    └── angle6.m                # 角度计算
```

## 🚀 快速开始

```matlab
% 在MATLAB命令窗口运行
main
```

## ✨ 功能特性

- **改进A*算法**：基于障碍物密度的自适应启发函数
- **自适应DWA**：视线检测 + 动态权重调整，解决切角和震荡问题
- **窄路通行**：机器人可通过狭窄通道（宽度仅1m）
- **实时可视化**：显示全局路径、局部规划、机器人轨迹

## 📊 测试地图

| 地图编号 | 描述 |
|---------|------|
| `MAP(0)` | 空地图（20×20） |
| `MAP(1)` | 狭窄通道（默认） |
| `MAP(2)` | U型陷阱 |

## 🔧 参数配置

在 `Main_AStar_DWA_Simulation.m` 中可调整：
- `robot_radius`: 机器人半径 (默认0.3m)
- `max_speed`: 最大速度 (默认0.5m/s)
- `lookahead_dist`: 前瞻距离 (默认2.0m)
- `use_adaptive`: true=改进DWA, false=传统DWA

## 📝 依赖

- MATLAB R2019b+
- Image Processing Toolbox（可选，用于地图膨胀）

## 📄 License

MIT License
