# Isaac Sim 三相机现实对齐包

本包把 2026-10-17 完成标定的 D435i、Left C922、Right C922 参数整理为 Isaac Sim 可直接写入的配置。仿真输出目标与当前 VLA/LeRobot 输入一致：`640x480`；D435i 使用原厂彩色内参；两台 C922 使用去畸变后的 `image_rect` 投影内参。

## 关键结论

- `front` D435i 是手眼相机，必须挂在仿真机器人的 `fr3_hand` 下，不能作为世界固定相机。
- `left` 和 `right` C922 是世界固定相机，位姿相对真实机器人基座的 ROS `world`。
- ROS optical 相机是 `+X right, +Y down, +Z forward`；USD Camera 是 `+X right, +Y up, -Z forward`。脚本已自动增加绕 X 轴 180 度转换。
- C922 原始畸变参数保留在配置中，但仿真渲染匹配的是现实端已经去畸变的画面，因此 USD Camera 使用零畸变针孔模型和 ROS `P` 矩阵对应的内参。
- `stage_alignment` 当前为单位变换，但状态是 `MUST_VERIFY_IN_SCANNED_STAGE`。只有扫描场景中的仿真 Franka 基座原点和轴向与真实 ROS `world` 完全一致时，单位变换才正确。

## 文件说明

- `config/camera_alignment.json`：Isaac Sim 唯一输入配置，包含三相机内参、位姿、焦距、aperture、principal-point offset、坐标约定和质量指标。
- `scripts/apply_cameras_to_stage.py`：在当前打开的 Isaac Stage 中创建三台 USD Camera。
- `scripts/validate_alignment_package.py`：无需 Isaac Sim，可检查四元数、ROS/USD 轴转换和镜头参数是否可逆还原内参。
- `source/calibration/`：完整求解结果、相机内参导出和 rosbag 元数据。
- `source/ros/`：现实 ROS 启动时实际加载的内外参文件。
- `source/reproducibility/`：外参求解脚本和现实相机启动文件。
- `evidence/`：标定质量摘要和角点检测预览。
- `reference/`：现实三相机现场画面，用于仿真视角人工复核。

## 使用步骤

1. 在 Isaac Sim 打开扫描生成的场景 USD，并确认 Stage 为 Z-up。
2. 找到仿真 Franka 的 `fr3_hand` prim。脚本会按名称自动搜索，必须且只能找到一个；也可在 JSON 中填绝对 prim path。
3. 确认扫描场景的机器人基座坐标与真实 ROS `world` 一致。若不一致，测量并填写 `config/camera_alignment.json` 的 `stage_alignment`，其含义是 `T_stage_ros_world`。
4. 在 Isaac Sim Script Editor 中打开并运行 `scripts/apply_cameras_to_stage.py`。若编辑器没有保留 `__file__`，把脚本顶部 `CONFIG_PATH` 设置为 JSON 的绝对路径。
5. 脚本创建：
   - `/World/AlignedCameras/camera_left`
   - `/World/AlignedCameras/camera_right`
   - `<fr3_hand prim>/camera_front_color`
6. 为每台 Camera 创建 `640x480` Render Product。分辨率属于 Render Product，不属于 Camera prim；不要使用其他分辨率后直接与现实图像比较。
7. 依次切换三台 Camera 视图，与 `reference/cameras_3in1_20261017_1803.png` 比较，再用场景中至少 4 个静态特征点做像素重投影检查。
8. 确认后使用 `File > Save As` 保存新场景；脚本不会自动覆盖原扫描场景。

## 验证命令

在普通终端运行：

```bash
python3 scripts/validate_alignment_package.py
sha256sum -c SHA256SUMS
```

在 Isaac Sim 中创建 Render Product 的最小示例：

```python
import omni.replicator.core as rep

rep.create.render_product("/World/AlignedCameras/camera_left", (640, 480))
rep.create.render_product("/World/AlignedCameras/camera_right", (640, 480))
# 将下面路径替换成脚本打印的实际 fr3_hand 路径
rep.create.render_product("/World/Franka/fr3_hand/camera_front_color", (640, 480))
```

## 尚需现场确认的一项参数

外参与内参已经完成，不需要重新标定相机。仍必须确认的是扫描场景坐标到真实机器人 `world` 的刚体变换。如果扫描建图时已经用 Franka 基座作为原点且轴向一致，保持单位变换即可；否则三台相机会一起出现相同的整体平移或旋转偏差，这不是相机标定误差，应只修改 `stage_alignment`。

## 标定质量

- Front 重投影 RMS：`0.141710 px`
- 手眼平移一致性 RMS：`0.785181 mm`
- 手眼旋转一致性 RMS：`0.270401 deg`
- Left 重投影 RMS：`0.147314 px`
- Right 重投影 RMS：`0.121271 px`

原始 rosbag 位于 `/media/mac/Elements/calibration/camera_extrinsics_20261017_163011`，约 11.2 GB，仅在需要重新求解外参时使用，因此没有放入本交付包。
