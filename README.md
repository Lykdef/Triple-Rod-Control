# Triple

基于 C++17 和 Eigen 的滑块串联倒立摆仿真，支持一杆、二杆、三杆的直立稳定、下垂摆起和局部失稳恢复。界面使用 GLFW、OpenGL 和 Dear ImGui。

## 构建与运行

要求：CMake 3.20+、支持 OpenGL 的 C++17 编译环境。依赖由 CMake `FetchContent` 获取，首次配置需要网络。

```bash
cmake -S . -B build
cmake --build build -j 4
./build/triple.exe
```

运行自动测试：

```bash
ctest --test-dir build --output-on-failure
```

测试覆盖单杆动力学回归、多杆能量守恒、直立稳定、静止下垂摆起和局部扰动恢复。

## 界面操作

- `Rod Count`：切换一杆、二杆或三杆，切换后会重置仿真。
- `Controller Enabled`：启用或关闭自动控制；关闭后可使用手动力滑块。
- `Reset Manual Force`：将手动力清零。
- `Reset Upright`：以较小的直立偏角重新启动。
- `Reset Hanging`：从全部杆静止下垂的状态开始摆起。
- `Control Mode`：显示当前处于摆起阶段还是 LQR 稳定阶段。

## 控制方法

- 所有杆数共用统一的串联多杆动力学和 LQR 直立稳定控制。
- 单杆采用能量摆起，进入接管区域后切换 LQR。
- 二杆、三杆采用预计算的 50 Hz 摆起参考轨迹和时变状态反馈，末端由 LQR 接管。
- 自动控制的力上限为 60 N，手动力滑块范围为 ±30 N。

## 代码结构

- `model`、`integrator`、`collision`：动力学、RK4 积分和墙面碰撞。
- `sensors`：带可选噪声的状态测量。
- `lqr`、`automatic_controller`：稳定、摆起和控制模式切换。
- `swing_up_data`：默认物理参数下的多杆参考轨迹及反馈增益。
- `simulation`：固定 1 ms 物理步长和模块调度。
- `scene`、`window`、`main`：绘图、窗口和界面。

## 适用范围

多杆摆起轨迹对应当前默认杆长、杆质量等参数。LQR 和扰动恢复属于直立附近控制，不保证任意大扰动下的全局恢复；当前恢复测试使用 0.3 rad 杆角扰动、0.3 m 滑块偏移和 0.2 rad/s 初始角速度。
