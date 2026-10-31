# MBRA Lite3 Starter — ROS 2 Jazzy / Ubuntu 24.04 / RTX 4050

**仅做视觉推理与 RViz 轨迹预览。绝不向 `/cmd_vel` 发送控制指令。**

本包为独立编写的部署/测试脚本（不是 UCLA 原仓库的代码副本），调用 UCLA-VAIL 官方 `MBRA_PG_Official/inference.py` 的 `MBRAPGNavigator.inference_trajectory()`。没有捆绑官方 254 MB ONNX 权重；请自行通过 Hugging Face 下载。

对应官方文件：
- https://huggingface.co/UCLA-VAIL/Navigation-Model-Zoo-Public/tree/main/MBRA_PG_Official
- https://huggingface.co/UCLA-VAIL/Navigation-Model-Zoo-Public/blob/main/MBRA_PG_Official/inference.py
- https://huggingface.co/UCLA-VAIL/Navigation-Model-Zoo-Public/blob/main/MBRA_PG_Official/model_info.yaml
- https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html

## 0. 部署范围与模型接口

- 机器狗：绝影 Lite3 + D435i（或任何通过 ROS 2 提供 `sensor_msgs/msg/Image` 的 RGB 相机）
- 计算机：Ubuntu 24.04、系统 Python 3.12、ROS 2 Jazzy、RTX 4050
- 方式：系统 Python 3.12 `venv --system-site-packages`，复用 APT 安装的 `rclpy` / `cv_bridge`
- 官方封装的观测：`obs` shape `(1,6,3,H,W)`，RGB `float32 [0,1]`
- 图像帧：官方 `model_info.yaml` 指定 5 Hz；历史窗口 6 帧（约 1 秒）
- 输入目标：`goal_xy=(x_forward, y_left)`，**相对于机器人机体**，米
- 官方封装输出：`trajectory.shape=(1,1,8,2)`，8 个局部 XY 航点，米
- 输出话题：`/mbra/local_path`，类型 `nav_msgs/msg/Path`
- `frame_id=base_link` 仅在模型局部坐标定义与机器人 `base_link` 的平面 XY 轴一致时使用。相机的 optical frame 不是机器人前 X、左 Y。

注意：**原始 ONNX 网络与官方 Python 封装接口不同。** 原始 ONNX 采用 18 通道归一化图像和四元素的 `goal_pose`；封装负责预处理、单位换算。不要绕过封装直接将 `(x,y)` 传入原始 ONNX。

## 1. 环境准备

在无 Conda 的新终端中：

```bash
# 如果当前有 Conda 环境，先 conda deactivate；无 Conda 可忽略。
/usr/bin/python3 --version
nvidia-smi
ls /opt/ros/jazzy/setup.bash

sudo apt update
sudo apt install -y python3-venv python3-opencv ros-jazzy-cv-bridge unzip

mkdir -p ~/mbra_ws
cd ~/mbra_ws
/usr/bin/python3 -m venv --system-site-packages .venv
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

python --version
python -c "import rclpy; print('rclpy OK')"
python -c "import cv_bridge; print('cv_bridge OK')"
python -c "import cv2; print('OpenCV OK')"
```

> 若 `rclpy` / `cv_bridge` 导入失败，先确认安装 Jazzy 与系统 Python 3.12，并且已 `source /opt/ros/jazzy/setup.bash`。不要在 venv 里用 pip 覆盖 ROS 2 的 `rclpy`/`cv_bridge`。

## 2. Python 与 GPU 依赖

保持上面的 `(.venv)` 终端：

```bash
python -m pip install --upgrade pip
python -m pip install 'numpy>=1.26,<2' pillow pyyaml 'huggingface_hub[cli]'

# 适用于 Python 3.12 + CUDA 12.x 的一组可复现基线
python -m pip install torch==2.5.1 torchvision==0.20.1 \
  --index-url https://download.pytorch.org/whl/cu121
python -m pip install onnxruntime-gpu==1.20.1
python -m pip check

python - <<'PY'
import torch
import onnxruntime as ort
print('torch:', torch.__version__, 'CUDA:', torch.cuda.is_available())
print('ORT:', ort.__version__, 'providers:', ort.get_available_providers())
PY
```

ONNX Runtime 1.20.x CUDA 12 包使用 cuDNN 9；PyTorch 2.5.1 的 CUDA 12.1 构建也是相应的匹配基线。`CUDAExecutionProvider` 出现在 **available providers** 里仍不足以证明实际 session 使用 GPU；两个脚本都检查 session providers，并默认拒绝无 GPU 运行。RTX 4050 实际 GPU 利用率仍需通过 `nvidia-smi` / ORT profiling 验证。

不必仅为推理额外安装完整 CUDA Toolkit；GPU 正常运行仍需要兼容的 NVIDIA 驱动和动态链接库。

如出现 `libcudnn.so.9`、`libcublas.so.12` 等加载错误，先检查相应 PyTorch/CUDA/cuDNN 包与驱动是否正确。**不要随意同时安装不兼容的 ONNX Runtime、PyTorch、cuDNN 版本。**

## 3. 下载模型与放置脚本

```bash
cd ~/mbra_ws
source .venv/bin/activate

hf download UCLA-VAIL/Navigation-Model-Zoo-Public \
  --include 'MBRA_PG_Official/*' \
  --local-dir .

ls -lh ~/mbra_ws/MBRA_PG_Official/
```

需要存在：`inference.py`、`mbra.onnx`（约 254 MB）、`model_info.yaml`。

从本回答下载 `mbra_lite3_starter.zip`。假设 ZIP 位于 `~/Downloads`：

```bash
unzip ~/Downloads/mbra_lite3_starter.zip -d ~/mbra_ws
cp ~/mbra_ws/mbra_lite3_starter/*.py ~/mbra_ws/
```

最终目录：

```text
~/mbra_ws/
├── .venv/
├── MBRA_PG_Official/
│   ├── inference.py
│   ├── mbra.onnx
│   └── model_info.yaml
├── mbra_lite3_starter/
│   ├── README.md
│   ├── test_mbra.py
│   └── mbra_ros_preview.py
├── test_mbra.py
└── mbra_ros_preview.py
```

## 4. A/B：离线推理验收

```bash
cd ~/mbra_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

python test_mbra.py --goal-x 3.0 --goal-y 0.0 --warmup 3 --runs 20
```

脚本会打印：PyTorch CUDA、ORT available providers、**实际模型 session providers**、观测 shape、轨迹 shape、8 个航点、推理延迟 p50/p95。输出 `trajectory.csv`。

- 通过标准：`CUDAExecutionProvider` 在 session providers 中；轨迹 shape `(1,1,8,2)`；所有数值有限。
- 初次使用随机图像只用于检查网络与运行时，**没有导航效果参考意义**。
- 仅诊断 CPU：`python test_mbra.py --allow-cpu`（不是最终 GPU 验收）。

如果已准备六张真实 RGB 帧，以时间先后命名 `001.png ... 006.png`，间隔约 0.2 秒：

```bash
python test_mbra.py --images ~/mbra_ws/frames --goal-x 3.0 --goal-y 0.0 --runs 20
```

六张图片需尺寸一致，**时间从早到晚**。图片内容在移动机器人运行状态下才更贴近训练分布；静止六帧虽能执行推理，但不能代表导航性能。

## 5. C：先确认 ROS 2 真正有 RGB 图像

在终端 A：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep -E 'image|color'
ros2 topic info /camera/camera/color/image_raw
ros2 topic hz /camera/camera/color/image_raw
```

把示例 `/camera/camera/color/image_raw` 替换成你当前 Lite3 实际话题。类型必须是 `sensor_msgs/msg/Image`；如果只有 `CompressedImage`，需要先转成普通 Image 或额外修改节点。确保收到的是 RGB 彩色画面而非 depth 图。

## 6. D：运行只做预览的 ROS 节点

在终端 B：

```bash
source /opt/ros/jazzy/setup.bash
source ~/mbra_ws/.venv/bin/activate
cd ~/mbra_ws

python mbra_ros_preview.py --ros-args \
  -p image_topic:=/camera/camera/color/image_raw \
  -p frame_id:=base_link \
  -p goal_x:=3.0 \
  -p goal_y:=0.0 \
  -p sample_hz:=5.0 \
  -p infer_hz:=2.0
```

脚本会缓冲按采样顺序排列的最近 6 帧，调用 `inference_trajectory()`，发布 8 个 XY 航点。每次发布均使用当前 `base_link` 的坐标解释；没有将轨迹变换到固定 `map` 坐标系。

`(3.0, 0.0)` 表示**机器人当前前方 3 米的局部固定测试目标**。当机器人移动时，该目标会随着机身移动；因此本阶段只适合预览，不是固定世界坐标导航点。

需要在运行中更改测试目标时，可执行：

```bash
ros2 param set /mbra_ros_preview goal_y 1.0
```

暂时只修改 `goal_x`/`goal_y`；`frame_id`/`image_topic`/频率需要重启节点后才会生效。

检查 ROS 输出：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep mbra
ros2 topic info /mbra/local_path
ros2 topic echo /mbra/local_path --once
```

预期类型 `nav_msgs/msg/Path`，`frame_id` 为 `base_link`，含 8 个 `poses`。节点不会发布 `/cmd_vel`。

若 GPU 失败需要**临时诊断 CPU**：在节点启动命令中加 `-p require_gpu:=false`。不建议把 CPU 结果作为 GPU 验收。

## 7. E：RViz 可视化

终端 C：

```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

1. 左侧 `Global Options → Fixed Frame` 设置 `base_link`。
2. `Add → By topic → /mbra/local_path → Path`。
3. 设置显眼颜色、宽度。
4. 不移动机器人，改变相机朝向或通过 `ros2 param set` 改变目标，看局部路径是否更新。

注：RViz 可能看见一些航点在机器人后侧或障碍附近。**这不是通过安全验证的可行驶路径**，现阶段仅可视化，不执行。

## 8. 验收清单

- [ ] A：Python 3.12 / rclpy / cv_bridge / CUDA 依赖正常
- [ ] B：完整下载 254MB ONNX，离线推理得到 8 个有限二维航点，记录 p50/p95
- [ ] C：PC 端 ROS 2 收到 D435i `sensor_msgs/msg/Image` 原始 RGB 图像
- [ ] D：`/mbra/local_path` 话题发布 `nav_msgs/msg/Path`，有 8 个 `poses`
- [ ] E：RViz `base_link` 固定帧下路径可见，修改目标能更新，延迟可接受

以后 Nav2 完成，再实现全局 `Path` 采样、TF 转换为当前 `base_link` 的局部子目标、实时定位有效性检查和有安全保护的轨迹跟踪。当前预览节点不包含这些功能。

## 9. 常见错误

| 现象 | 检查点 |
|---|---|
| `No module named rclpy` | `--system-site-packages`、系统 Python 3.12、Jazzy 环境已 source |
| `No module named cv_bridge` | APT 是否装 `ros-jazzy-cv-bridge`，是否有 Python/NumPy ABI 冲突 |
| 找不到模型 | `~/mbra_ws/MBRA_PG_Official` 中有上述三个官方文件 |
| Torch CUDA True 但 ORT CPU | ORT 1.20.1 / cuDNN 9 / CUDA 12 依赖是否能装载；**看 Session Providers** |
| ORT CUDA EP 已加载 | 仍不能保证每个运算节点都在 GPU；如要严格验证可做 ORT profiling |
| 没有 Path | 图像话题是否为 `sensor_msgs/msg/Image`、是否正常采集到至少 6 帧、图像是否超时 |
| RViz 不显示 | Fixed Frame / frame_id / 话题名称，以及是否收到 Path |
| 路径反常 | RGB 与 BGR、历史图像顺序、安装朝向、local x 前 y 左 |
| 实机表现不佳 | MBRA 训练域与 Lite3 深度相机画面、机体高度、走速等分布可能不同；并非自动可迁移 |

> 工程提醒：为了防止网络问题导致旧画面继续推理，脚本在最近 RGB 图像接收超过 `max_image_age_sec`（默认 1.5 秒）后暂停发布。它只检测图像数据接收新鲜度，不提供底层防撞、TF 检测或机器人安全控制。

## 10. 验证范围

本包经过 Python 语法编译、离线仿真单元测试及消息接口逻辑检查；没有条件接入你的真机、NVIDIA GPU、ROS 2 Jazzy 现场环境，也没有下载 254MB 权重进行真实 ONNX 推理。现场最终验收以你本机的 A–E 测试为准。
