#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import sys
import os
import threading

# 音频播放相关导入
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("警告：pygame 未安装，将尝试使用其他方法播放音频")
    
try:
    import subprocess
    SUBPROCESS_AVAILABLE = True
except ImportError:
    SUBPROCESS_AVAILABLE = False


class KokoroTTSClient:
    def __init__(self):
        """初始化 Kokoro TTS 客户端"""
        rospy.init_node('kokoro_tts_client', anonymous=True)
        
        # 创建发布者和订阅者
        self.text_publisher = rospy.Publisher('/kokoro_tts/text_input', String, queue_size=10)
        self.status_subscriber = rospy.Subscriber('/kokoro_tts/status', String, self.status_callback)
        self.audio_subscriber = rospy.Subscriber('/kokoro_tts/audio_file', String, self.audio_callback)
        
        # 初始化音频播放器
        self.init_audio_player()
        
        # 等待服务准备
        rospy.sleep(1)
        rospy.loginfo("Kokoro TTS 客户端已准备就绪")
    
    def init_audio_player(self):
        """初始化音频播放器"""
        if PYGAME_AVAILABLE:
            try:
                pygame.mixer.init()
                rospy.loginfo("使用 pygame 播放音频")
                self.audio_method = 'pygame'
            except Exception as e:
                rospy.logwarn(f"pygame 初始化失败: {e}")
                self.audio_method = 'system'
        else:
            self.audio_method = 'system'
            rospy.loginfo("使用系统命令播放音频")
    
    def play_audio(self, audio_file):
        """播放音频文件"""
        if not os.path.exists(audio_file):
            rospy.logerr(f"音频文件不存在: {audio_file}")
            return
        
        def play_thread():
            try:
                if self.audio_method == 'pygame' and PYGAME_AVAILABLE:
                    # 使用 pygame 播放
                    pygame.mixer.music.load(audio_file)
                    pygame.mixer.music.play()
                    # 等待播放完成
                    while pygame.mixer.music.get_busy():
                        pygame.time.wait(100)
                    rospy.loginfo(f"播放完成: {os.path.basename(audio_file)}")
                
                elif SUBPROCESS_AVAILABLE:
                    # 使用系统命令播放
                    try:
                        # 尝试使用 aplay (ALSA)
                        result = subprocess.run(['aplay', audio_file], 
                                              capture_output=True, text=True, timeout=30)
                        if result.returncode == 0:
                            rospy.loginfo(f"播放完成: {os.path.basename(audio_file)}")
                        else:
                            raise Exception("aplay 失败")
                    except (subprocess.TimeoutExpired, FileNotFoundError, Exception):
                        try:
                            # 尝试使用 paplay (PulseAudio)
                            result = subprocess.run(['paplay', audio_file], 
                                                  capture_output=True, text=True, timeout=30)
                            if result.returncode == 0:
                                rospy.loginfo(f"播放完成: {os.path.basename(audio_file)}")
                            else:
                                raise Exception("paplay 失败")
                        except (subprocess.TimeoutExpired, FileNotFoundError, Exception):
                            try:
                                # 尝试使用 mpg123
                                result = subprocess.run(['mpg123', audio_file], 
                                                      capture_output=True, text=True, timeout=30)
                                if result.returncode == 0:
                                    rospy.loginfo(f"播放完成: {os.path.basename(audio_file)}")
                                else:
                                    raise Exception("mpg123 失败")
                            except Exception:
                                rospy.logerr(f"无法播放音频文件: {audio_file}")
                else:
                    rospy.logerr("没有可用的音频播放方法")
                    
            except Exception as e:
                rospy.logerr(f"播放音频时出错: {e}")
        
        # 在单独线程中播放音频，避免阻塞
        threading.Thread(target=play_thread, daemon=True).start()
    
    def status_callback(self, msg):
        """接收状态消息回调"""
        rospy.loginfo(f"服务状态: {msg.data}")
    
    def audio_callback(self, msg):
        """接收音频文件路径回调"""
        audio_file = msg.data.strip()
        if audio_file:
            rospy.loginfo(f"收到音频文件: {os.path.basename(audio_file)}")
            self.play_audio(audio_file)
    
    def send_text(self, text):
        """发送文本给 TTS 服务"""
        if not text:
            rospy.logwarn("文本为空")
            return
        
        msg = String()
        msg.data = text
        
        rospy.loginfo(f"发送文本: '{text}'")
        self.text_publisher.publish(msg)
    
    def interactive_mode(self):
        """交互模式"""
        rospy.loginfo("进入交互模式，输入文本进行语音合成（输入 'quit' 退出）:")
        
        while not rospy.is_shutdown():
            try:
                text = input("请输入中文文本: ").strip()
                
                if text.lower() == 'quit':
                    break
                
                if text:
                    self.send_text(text)
                    rospy.sleep(0.5)  # 短暂等待
                else:
                    print("请输入有效的文本")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
        
        rospy.loginfo("客户端退出")


def main():
    """主函数"""
    try:
        client = KokoroTTSClient()
        
        # 检查命令行参数
        if len(sys.argv) > 1:
            # 如果有命令行参数，直接发送
            text = " ".join(sys.argv[1:])
            client.send_text(text)
            rospy.sleep(2)  # 等待处理完成
        else:
            # 否则进入交互模式
            client.interactive_mode()
            
    except Exception as e:
        rospy.logerr(f"客户端错误: {e}")


if __name__ == '__main__':
    main() 