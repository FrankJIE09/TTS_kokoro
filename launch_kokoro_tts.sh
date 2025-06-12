#!/bin/bash

# Kokoro TTS ROS 服务启动脚本

echo "=== Kokoro TTS ROS 服务启动脚本 ==="

# 激活 conda 环境
conda activate kokoro

# 检查 ROS 环境
if [ -z "$ROS_MASTER_URI" ]; then
    echo "警告: ROS 环境未设置，正在设置默认环境..."
    export ROS_MASTER_URI=http://localhost:11311
    export ROS_IP=localhost
fi

# 设置 Python 路径
export PYTHONPATH=$PYTHONPATH:$(pwd)

echo "环境信息:"
echo "  ROS_MASTER_URI: $ROS_MASTER_URI"
echo "  Python 环境: $(which python3)"
echo "  当前目录: $(pwd)"

# 设置执行权限
chmod +x kokoro_tts_service.py
chmod +x kokoro_tts_client.py

echo ""
echo "启动 Kokoro TTS 服务..."
echo "服务节点: kokoro_tts_service"
echo "监听话题: /kokoro_tts/text_input"
echo "状态话题: /kokoro_tts/status"
echo ""

# 运行服务
python3 kokoro_tts_service.py 