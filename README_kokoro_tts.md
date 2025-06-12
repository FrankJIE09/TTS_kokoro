# Kokoro TTS ROS 服务

这是一个基于 Kokoro 的中文文字转语音 ROS 服务程序。

## 文件说明

- `TextToSpeech.srv` - ROS 服务消息定义文件
- `kokoro_tts_service.py` - TTS 服务端程序
- `kokoro_tts_client.py` - TTS 客户端测试程序
- `launch_kokoro_tts.sh` - 服务启动脚本
- `generate_audio_kokoro.py` - 原始的批量音频生成程序

## 依赖要求

### Python 包
```bash
pip install kokoro
pip install soundfile
pip install numpy
pip install torch  # 可选，kokoro 可能需要
```

### 系统依赖 (Ubuntu/Debian)
```bash
sudo apt install espeak-ng
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

### 3. 使用客户端测试

#### 交互模式
```bash
conda activate kokoro
python3 kokoro_tts_client.py
```
然后在提示符下输入中文文本。

#### 命令行模式
```bash
conda activate kokoro
python3 kokoro_tts_client.py "你好世界"
```

### 4. 使用 ROS 话题直接发送

#### 发送文本
```bash
rostopic pub /kokoro_tts/text_input std_msgs/String "data: '你好，这是测试文本'"
```

#### 监听状态
```bash
rostopic echo /kokoro_tts/status
```

## 配置参数

服务支持以下 ROS 参数：

- `~language_code` - 语言代码 (默认: 'z' 中文普通话)
- `~voice_id` - 语音 ID (默认: 'zf_xiaoyi' 中文女声)
- `~speed` - 语速 (默认: 1.0)
- `~sample_rate` - 采样率 (默认: 24000)
- `~output_dir` - 输出目录 (默认: ~/kokoro_audio)

### 使用自定义参数启动
```bash
rosrun your_package kokoro_tts_service.py _voice_id:=zf_xiaoyi _speed:=1.2 _output_dir:=/tmp/audio
```

## ROS 话题

### 输入话题
- `/kokoro_tts/text_input` (std_msgs/String) - 接收要转换的文本

### 输出话题
- `/kokoro_tts/status` (std_msgs/String) - 发布服务状态和结果信息

## 输出文件

生成的音频文件保存在指定目录中，文件名格式：
```
tts_YYYYMMDD_HHMMSS_<文本hash>.wav
```

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