# auto_aim — 长空御风视觉组 2026 入队考核

ROS 2 (Humble) 自瞄节点：订阅模拟器的 `/image_raw`（1152x648），识别装甲板与
炮台，通过串口发送转向角与开火指令击打异色装甲板。

## 编译

在本仓库根目录下执行：

```bash
colcon build
```

## 运行

```bash
./scripts/run_all.sh          # 启动节点
# 然后启动游戏
```

难度选择超大杯；种子留空则随机。参数见 `config/params.yaml`。

## 结构

- `src/auto_aim_node.cpp` — 控制核心：检测→跟踪→拦截→开火时序→串口。
- `src/armor_detector.cpp` — 红/蓝光柱 + 灰色残骸检测，炮台配色识别。
- `src/plate_tracker.cpp` — 最近邻关联 + 常加速度卡尔曼滤波（运动状态）。
- `src/aim_solver.cpp` — 精确拦截求解、离场时间、弹道走廊检查。
- `src/serial_controller.cpp` — pty 自动探测与串口协议。
