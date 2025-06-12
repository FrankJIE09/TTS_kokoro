# Kokoro 缓存 TTS ROS 服务

这是一个带有智能缓存功能的中文文字转语音 ROS 服务程序。

## 🚀 主要特性

- **🔄 智能缓存**: 相同文本直接播放缓存音频，无需重复生成
- **📁 统一存储**: 所有音频文件保存在 `~/.sage/kokoro_voice/` 目录
- **🧹 自动清理**: 最多保留30个缓存文件，按时间自动清理旧文件
- **⚡ 即时播放**: 音频生成后立即播放，缓存音频毫秒级响应
- **📊 缓存统计**: 实时查看缓存使用情况

## 📁 文件说明

- `kokoro_tts_cached_service.py` - 带缓存的 TTS 服务端程序
- `kokoro_cached_client.py` - 缓存 TTS 客户端程序
- `launch_cached_tts.sh` - 缓存服务启动脚本
- `test_cache_tts.py` - 缓存功能测试脚本

## 🛠️ 安装依赖

### Python 包
```bash
pip install kokoro
pip install soundfile
pip install numpy
pip install pygame  # 用于音频播放
pip install torch   # 可选，kokoro 可能需要
```

### 系统依赖 (Ubuntu/Debian)
```bash
sudo apt install espeak-ng
sudo apt install alsa-utils      # ALSA 音频播放工具
sudo apt install pulseaudio-utils # PulseAudio 音频播放工具
```

### ROS 依赖
需要安装 ROS (推荐 ROS Noetic)

## 🚀 使用方法

### 1. 启动 ROS Master
```bash
roscore
```

### 2. 启动缓存 TTS 服务
```bash
# 使用启动脚本（推荐）
./launch_cached_tts.sh

# 或直接运行
conda activate kokoro
python3 kokoro_tts_cached_service.py
```

### 3. 使用客户端测试

#### 交互模式（推荐）
```bash
conda activate kokoro
python3 kokoro_cached_client.py
```

#### 命令行模式
```bash
conda activate kokoro
python3 kokoro_cached_client.py "你好世界"
```

### 4. 使用 ROS 话题直接发送

#### 发送文本（现在会有声音输出！）
```bash
rostopic pub /kokoro_tts/text_input std_msgs/String "data: '你好，这是测试文本'"
```

#### 监听状态和音频文件
```bash
# 监听服务状态
rostopic echo /kokoro_tts/status

# 监听生成的音频文件路径
rostopic echo /kokoro_tts/audio_file
```

### 5. 测试缓存功能
```bash
conda activate kokoro
python3 test_cache_tts.py
```

## ⚙️ 配置参数

服务支持以下 ROS 参数：

- `~language_code` - 语言代码 (默认: 'z' 中文普通话)
- `~voice_id` - 语音 ID (默认: 'zf_xiaoyi' 中文女声)
- `~speed` - 语速 (默认: 1.0)
- `~sample_rate` - 采样率 (默认: 24000)
- `~max_cache_files` - 最大缓存文件数 (默认: 30)

### 使用自定义参数启动
```bash
rosrun your_package kokoro_tts_cached_service.py _voice_id:=zf_xiaoyi _speed:=1.2 _max_cache_files:=50
```

## 📊 ROS 话题

### 输入话题
- `/kokoro_tts/text_input` (std_msgs/String) - 接收要转换的文本

### 输出话题
- `/kokoro_tts/status` (std_msgs/String) - 发布服务状态和结果信息
- `/kokoro_tts/audio_file` (std_msgs/String) - 发布生成的音频文件路径

## 💾 缓存机制

### 缓存策略
- **缓存键生成**: 基于文本内容、语音ID和语速的 MD5 哈希
- **文件命名**: `{MD5哈希}.wav`
- **索引管理**: JSON 格式索引文件记录缓存元数据
- **LRU清理**: 基于最后使用时间清理旧缓存

### 缓存目录结构
```
~/.sage/kokoro_voice/
├── cache_index.json          # 缓存索引文件
├── {hash1}.wav              # 音频文件
├── {hash2}.wav
└── ...
```

### 缓存索引格式
```json
{
  "hash_key": {
    "text": "原始文本",
    "voice_id": "zf_xiaoyi",
    "speed": 1.0,
    "audio_file": "/path/to/audio.wav",
    "created_time": "2024-01-01T12:00:00",
    "last_used": "2024-01-01T12:00:00"
  }
}
```

## 🎵 音频播放和缓存

- **缓存目录**: `~/.sage/kokoro_voice/`
- **自动播放**: 服务端生成或找到音频后立即播放
- **支持的播放方式**:
  - 优先使用 pygame 播放（推荐）
  - 备选：aplay (ALSA)
  - 备选：paplay (PulseAudio)

## 📈 性能优势

### 缓存命中时：
- ⚡ **响应时间**: < 100ms（vs 原始 2-3秒）
- 💾 **资源消耗**: 极低（直接读取文件）
- 🔊 **音质**: 与原始生成完全一致

### 缓存管理：
- 🧹 **自动清理**: 超过30个文件时自动删除最旧的
- 📊 **使用统计**: 实时跟踪文件使用情况
- 🔄 **定期维护**: 每小时自动检查和清理

## 🧪 测试示例

```bash
# 启动服务
./launch_cached_tts.sh

# 在另一个终端测试
python3 test_cache_tts.py
```

测试会自动验证：
1. 首次文本生成音频并缓存
2. 相同文本直接使用缓存（速度提升明显）
3. 缓存统计信息显示

## 🔧 故障排除

### 1. 缓存问题
```bash
# 查看缓存目录
ls -la ~/.sage/kokoro_voice/

# 查看缓存索引
cat ~/.sage/kokoro_voice/cache_index.json
```

### 2. 清理所有缓存
```bash
rm -rf ~/.sage/kokoro_voice/
```

### 3. 权限问题
```bash
chmod 755 ~/.sage/kokoro_voice/
```

## 📝 使用示例

### Python 代码示例
```python
import rospy
from std_msgs.msg import String

# 创建发布者
pub = rospy.Publisher('/kokoro_tts/text_input', String, queue_size=10)

# 发送文本（首次会生成并缓存，再次发送会直接播放缓存）
msg = String()
msg.data = "欢迎使用 Kokoro 缓存 TTS 服务"
pub.publish(msg)
```

### 客户端交互示例
```
🗣️  请输入文本: 你好世界
📝 发送文本: '你好世界'
✨ 成功：音频已生成并缓存 '你好世界'
🔊 播放新音频: abc123def.wav

🗣️  请输入文本: 你好世界
📝 发送文本: '你好世界'
🎵 使用缓存音频: '你好世界'
🔊 播放缓存音频: abc123def.wav

🗣️  请输入文本: stats
📊 缓存统计:
  📁 缓存目录: /home/user/.sage/kokoro_voice
  📄 缓存文件数: 3/30
  💾 缓存大小: 1.2 MB
  📝 最新缓存文本:
    1. 你好世界
    2. 欢迎使用缓存系统
    3. 测试音频播放功能
```

---

🎉 **现在您可以享受极速的中文语音合成体验！相同文本只需生成一次，后续使用都是毫秒级响应。** 