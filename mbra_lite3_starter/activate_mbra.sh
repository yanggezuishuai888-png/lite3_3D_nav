#!/usr/bin/env bash

# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# MBRA Python 虚拟环境
source "$HOME/venvs/vln_local/bin/activate"

# 添加当前 venv 中的 NVIDIA CUDA/cuDNN 动态库
NVIDIA_LIBS=$(python - <<'PY'
import sysconfig
from pathlib import Path

base = Path(sysconfig.get_paths()["purelib"]) / "nvidia"

libs = [
    str(p)
    for p in sorted(base.glob("*/lib"))
    if p.is_dir()
]

print(":".join(libs))
PY
)

if [ -n "$NVIDIA_LIBS" ]; then
    export LD_LIBRARY_PATH="$NVIDIA_LIBS${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

unset NVIDIA_LIBS

echo "MBRA ROS 2 + CUDA environment ready"
