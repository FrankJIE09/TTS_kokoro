#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import traceback
import time
import hashlib
from datetime import datetime

# ROS 相关导入
import rospy
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

# 由于没有编译 srv 文件，我们使用标准的 ROS 消息类型
# 如果需要使用自定义 srv，需要先编译
from std_msgs.msg import String
from geometry_msgs.msg import Twist  # 临时使用，实際應該使用自定義服務

# Kokoro 和音频处理相关导入
try:
    from kokoro import KPipeline
except ImportError:
    rospy.logerr("错误：无法导入 'kokoro'。请确保已安装库： pip install kokoro")
    exit(1)

try:
    import soundfile as sf
except ImportError:
    rospy.logerr("错误：无法导入 'soundfile'。请确保已安装库： pip install soundfile")
    exit(1)

try:
    import numpy as np
except ImportError:
    rospy.logerr("错误：无法导入 'numpy'。请确保已安装库： pip install numpy")
    exit(1)


class KokoroTTSService:
    def __init__(self):
        """初始化 Kokoro TTS 服务"""
        rospy.init_node('kokoro_tts_service', anonymous=True)
        
        # 配置参数
        self.language_code = rospy.get_param('~language_code', 'z')  # 中文普通话
        self.default_voice_id = rospy.get_param('~voice_id', 'zf_xiaoyi')  # 默认中文女声
        self.default_speed = rospy.get_param('~speed', 1.0)  # 默认语速
        self.sample_rate = rospy.get_param('~sample_rate', 24000)  # 采样率
        self.output_dir = rospy.get_param('~output_dir', os.path.expanduser('~/kokoro_audio'))  # 输出目录
        
        # 确保输出目录存在
        if not os.path.exists(self.output_dir):
            try:
                os.makedirs(self.output_dir)
                rospy.loginfo(f"已创建输出目录: {self.output_dir}")
            except OSError as e:
                rospy.logerr(f"错误：无法创建输出目录 '{self.output_dir}': {e}")
                return
        
        # 初始化 Kokoro Pipeline
        try:
            rospy.loginfo(f"正在初始化 Kokoro Pipeline (语言: {self.language_code})...")
            self.pipeline = KPipeline(lang_code=self.language_code)
            rospy.loginfo("Kokoro Pipeline 初始化成功")
        except Exception as e:
            rospy.logerr(f"错误：初始化 Kokoro Pipeline 失败: {e}")
            traceback.print_exc()
            return
        
        # 创建服务
        # 由于没有自定义服务消息，我们使用主题来模拟服务
        # 实际使用时应该创建正确的服务定义
        self.text_subscriber = rospy.Subscriber('/kokoro_tts/text_input', String, self.text_callback)
        self.status_publisher = rospy.Publisher('/kokoro_tts/status', String, queue_size=10)
        
        rospy.loginfo("Kokoro TTS 服务已启动，等待文本输入...")
        rospy.loginfo(f"监听话题: /kokoro_tts/text_input")
        rospy.loginfo(f"状态输出话题: /kokoro_tts/status")
    
    def text_callback(self, msg):
        """处理文本输入回调"""
        text = msg.data.strip()
        if not text:
            self.publish_status("错误：输入文本为空")
            return
        
        rospy.loginfo(f"收到文本转语音请求: '{text}'")
        
        # 生成音频
        result = self.generate_speech(text)
        
        if result['success']:
            self.publish_status(f"成功：音频已保存到 {result['audio_file']}")
        else:
            self.publish_status(f"失败：{result['message']}")
    
    def generate_speech(self, text, voice_id=None, speed=None, output_path=None):
        """生成语音音频"""
        # 使用默认参数
        if voice_id is None:
            voice_id = self.default_voice_id
        if speed is None:
            speed = self.default_speed
        
        # 生成输出文件名
        if output_path is None:
            # 使用文本的 hash 值和时间戳生成唯一文件名
            text_hash = hashlib.md5(text.encode('utf-8')).hexdigest()[:8]
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"tts_{timestamp}_{text_hash}.wav"
            output_path = os.path.join(self.output_dir, filename)
        
        try:
            rospy.loginfo(f"开始生成语音: 文本='{text}', 语音={voice_id}, 语速={speed}")
            
            # 调用 pipeline 获取生成器
            generator = self.pipeline(text, voice=voice_id, speed=speed)
            
            # 收集所有音频块
            audio_chunks = []
            chunk_count = 0
            
            for i, (gs, ps, audio_chunk) in enumerate(generator):
                # 确保是 numpy array
                if not isinstance(audio_chunk, np.ndarray):
                    if hasattr(audio_chunk, 'numpy'):  # torch tensor
                        audio_chunk = audio_chunk.cpu().numpy()
                    else:
                        rospy.logerr(f"未知的音频块类型 {type(audio_chunk)}")
                        return {'success': False, 'message': f'未知的音频块类型: {type(audio_chunk)}', 'audio_file': ''}
                
                audio_chunks.append(audio_chunk)
                chunk_count += 1
            
            if not audio_chunks:
                return {'success': False, 'message': '没有生成音频块', 'audio_file': ''}
            
            # 合并所有块
            full_audio_data = np.concatenate(audio_chunks)
            
            # 保存为 WAV 文件
            sf.write(output_path, full_audio_data, self.sample_rate)
            
            # 计算音频时长
            duration = len(full_audio_data) / self.sample_rate
            
            rospy.loginfo(f"语音生成成功: 文件={output_path}, 时长={duration:.2f}秒, 音频块数={chunk_count}")
            
            return {
                'success': True,
                'message': '语音生成成功',
                'audio_file': output_path,
                'duration': duration
            }
            
        except Exception as e:
            error_msg = f"生成语音时发生错误: {str(e)}"
            rospy.logerr(error_msg)
            traceback.print_exc()
            return {
                'success': False,
                'message': error_msg,
                'audio_file': ''
            }
    
    def publish_status(self, message):
        """发布状态消息"""
        status_msg = String()
        status_msg.data = message
        self.status_publisher.publish(status_msg)
        rospy.loginfo(f"状态: {message}")
    
    def run(self):
        """运行服务"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Kokoro TTS 服务停止")


def main():
    """主函数"""
    try:
        # 检查 PyTorch 是否可用
        try:
            import torch
            rospy.loginfo(f"PyTorch version: {torch.__version__}")
        except ImportError:
            rospy.logwarn("警告：未找到 PyTorch。kokoro 库可能需要它。")
        
        # 创建并运行服务
        service = KokoroTTSService()
        service.run()
        
    except Exception as e:
        rospy.logerr(f"服务启动失败: {e}")
        traceback.print_exc()


if __name__ == '__main__':
    main() 