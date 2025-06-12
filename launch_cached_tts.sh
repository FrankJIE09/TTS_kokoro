#!/bin/bash

# Kokoro 缓存 TTS ROS 服务启动脚本

echo "🎤 Kokoro 缓存 TTS 服务启动脚本"
echo "=================================="

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

# 创建缓存目录
cache_dir="$HOME/.sage/kokoro_voice"
if [ ! -d "$cache_dir" ]; then
    mkdir -p "$cache_dir"
    echo "✓ 已创建缓存目录: $cache_dir"
else
    echo "✓ 缓存目录已存在: $cache_dir"
fi

echo ""
echo "环境信息:"
echo "  ROS_MASTER_URI: $ROS_MASTER_URI"
echo "  Python 环境: $(which python3)"
echo "  当前目录: $(pwd)"
echo "  缓存目录: $cache_dir"

# 设置执行权限
chmod +x kokoro_tts_cached_service.py
chmod +x kokoro_cached_client.py

echo ""
echo "启动 Kokoro 缓存 TTS 服务..."
echo "服务节点: kokoro_cached_tts_service"
echo "监听话题: /kokoro_tts/text_input"
echo "状态话题: /kokoro_tts/status"
echo "音频话题: /kokoro_tts/audio_file"
echo ""
echo "✨ 特性:"
echo "  • 智能缓存: 相同文本直接播放缓存音频"
echo "  • 自动清理: 最多保留30个缓存文件"
echo "  • 即时播放: 音频生成后立即播放"
echo ""

# 运行缓存服务
python3 kokoro_tts_cached_service.py 