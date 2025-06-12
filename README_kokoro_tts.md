# Kokoro TTS ROS 服务

这是一个基于 Kokoro 的中文文字转语音 ROS 服务程序。

## 文件说明

- `TextToSpeech.srv` - ROS 服务消息定义文件
- `kokoro_tts_service.py` - TTS 服务端程序（保存音频到 ./audio 目录）
- `kokoro_tts_client.py` - TTS 客户端程序（自动播放生成的音频）
- `launch_kokoro_tts.sh` - 服务启动脚本
- `test_audio_playback.py` - 音频播放功能测试脚本
- `install_audio_deps.sh` - 音频播放依赖安装脚本
- `generate_audio_kokoro.py` - 原始的批量音频生成程序

## 依赖要求

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

### 快速安装音频播放依赖
```bash
./install_audio_deps.sh  # 自动安装所有音频播放依赖
```

### ROS 依赖
需要安装 ROS (推荐 ROS Noetic)

## 使用方法

### 1. 启动 ROS Master
```bash
roscore
```

### 2. 启动 TTS 服务
```bash
# 方法1: 使用启动脚本
./launch_kokoro_tts.sh

# 方法2: 直接运行
conda activate kokoro
python3 kokoro_tts_service.py
```

### 3. 测试音频播放功能（推荐）
```bash
conda activate kokoro
python3 test_audio_playback.py
```

### 4. 使用客户端测试

#### 交互模式（自动播放音频）
```bash
conda activate kokoro
python3 kokoro_tts_client.py
```
然后在提示符下输入中文文本，生成的音频会自动播放。

#### 命令行模式（自动播放音频）
```bash
conda activate kokoro
python3 kokoro_tts_client.py "你好世界"
```

### 5. 使用 ROS 话题直接发送

#### 发送文本
```bash
roscore
```
```bash
./launch_kokoro_tts.sh
```
```bash
rostopic pub /kokoro_tts/text_input std_msgs/String "data: '大家好！我是小飒，很高兴能在这里和大家见面。我是上海飒智智能科技有限公司大家庭里的最新成员，一款集成了尖端人工智能和精密仿生技术的人形机器人。大家可以把我当成一个既聪明又能干的新朋友。'"
```

#### 监听状态和音频文件
```bash
# 监听服务状态
rostopic echo /kokoro_tts/status

# 监听生成的音频文件路径
rostopic echo /kokoro_tts/audio_file
```

## 配置参数

服务支持以下 ROS 参数：

- `~language_code` - 语言代码 (默认: 'z' 中文普通话)
- `~voice_id` - 语音 ID (默认: 'zf_xiaoyi' 中文女声)
- `~speed` - 语速 (默认: 1.0)
- `~sample_rate` - 采样率 (默认: 24000)
- `~output_dir` - 输出目录 (默认: ./audio)

### 使用自定义参数启动
```bash
rosrun your_package kokoro_tts_service.py _voice_id:=zf_xiaoyi _speed:=1.2 _output_dir:=/tmp/audio
```

## ROS 话题

### 输入话题
- `/kokoro_tts/text_input` (std_msgs/String) - 接收要转换的文本

### 输出话题
- `/kokoro_tts/status` (std_msgs/String) - 发布服务状态和结果信息
- `/kokoro_tts/audio_file` (std_msgs/String) - 发布生成的音频文件路径（客户端自动播放）

## 输出文件和音频播放

- 生成的音频文件保存在当前目录的 `audio/` 文件夹中
- 文件名格式：`tts_YYYYMMDD_HHMMSS_<文本hash>.wav`
- 客户端会自动播放生成的音频文件（支持 pygame 和系统音频工具）
- 支持的音频播放方式：
  - 优先使用 pygame 播放（推荐）
  - 备选：aplay (ALSA)
  - 备选：paplay (PulseAudio)

## 故障排除

### 1. 导入错误
如果出现 kokoro 导入错误，请确保：
- 已激活正确的 conda 环境
- 已安装 kokoro 包及其依赖

### 2. ROS 连接问题
确保 ROS Master 正在运行：
```bash
roscore
```

### 3. 权限问题
确保输出目录有写入权限：
```bash
chmod 755 ~/kokoro_audio
```

## 示例用法

```python
# 在其他 ROS 节点中使用
import rospy
from std_msgs.msg import String

# 创建发布者
pub = rospy.Publisher('/kokoro_tts/text_input', String, queue_size=10)

# 发送文本
msg = String()
msg.data = "欢迎使用 Kokoro TTS 服务"
pub.publish(msg)
``` 

