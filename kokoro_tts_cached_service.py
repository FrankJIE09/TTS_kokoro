#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import traceback
import time
import hashlib
from datetime import datetime
import json
import threading
import glob

# ROS 相关导入
import rospy
from std_msgs.msg import String

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

# 音频播放相关导入
try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    rospy.logwarn("警告：pygame 未安装，将尝试使用系统音频播放工具")

try:
    import subprocess
    SUBPROCESS_AVAILABLE = True
except ImportError:
    SUBPROCESS_AVAILABLE = False


class KokoroCachedTTSService:
    def __init__(self):
        """初始化带缓存的 Kokoro TTS 服务"""
        rospy.init_node('kokoro_cached_tts_service', anonymous=True)
        
        # 配置参数
        self.language_code = rospy.get_param('~language_code', 'z')  # 中文普通话
        self.default_voice_id = rospy.get_param('~voice_id', 'zf_xiaoyi')  # 默认中文女声
        self.default_speed = rospy.get_param('~speed', 1.0)  # 默认语速
        self.sample_rate = rospy.get_param('~sample_rate', 24000)  # 采样率
        self.max_cache_files = rospy.get_param('~max_cache_files', 30)  # 最大缓存文件数
        
        # 缓存目录设置
        self.cache_dir = os.path.expanduser('~/.sage/kokoro_voice')
        self.cache_index_file = os.path.join(self.cache_dir, 'cache_index.json')
        
        # 确保缓存目录存在
        if not os.path.exists(self.cache_dir):
            try:
                os.makedirs(self.cache_dir)
                rospy.loginfo(f"已创建缓存目录: {self.cache_dir}")
            except OSError as e:
                rospy.logerr(f"错误：无法创建缓存目录 '{self.cache_dir}': {e}")
                return
        
        # 加载缓存索引
        self.cache_index = self.load_cache_index()
        
        # 初始化 Kokoro Pipeline
        try:
            rospy.loginfo(f"正在初始化 Kokoro Pipeline (语言: {self.language_code})...")
            self.pipeline = KPipeline(lang_code=self.language_code)
            rospy.loginfo("Kokoro Pipeline 初始化成功")
        except Exception as e:
            rospy.logerr(f"错误：初始化 Kokoro Pipeline 失败: {e}")
            traceback.print_exc()
            return
        
        # 初始化音频播放器
        self.init_audio_player()
        
        # 创建服务
        self.text_subscriber = rospy.Subscriber('/kokoro_tts/text_input', String, self.text_callback)
        self.status_publisher = rospy.Publisher('/kokoro_tts/status', String, queue_size=10)
        self.audio_file_publisher = rospy.Publisher('/kokoro_tts/audio_file', String, queue_size=10)
        
        # 启动缓存清理线程
        self.cleanup_thread = threading.Thread(target=self.periodic_cache_cleanup, daemon=True)
        self.cleanup_thread.start()
        
        rospy.loginfo("Kokoro 缓存 TTS 服务已启动，等待文本输入...")
        rospy.loginfo(f"监听话题: /kokoro_tts/text_input")
        rospy.loginfo(f"状态输出话题: /kokoro_tts/status")
        rospy.loginfo(f"音频文件话题: /kokoro_tts/audio_file")
        rospy.loginfo(f"缓存目录: {self.cache_dir}")
        rospy.loginfo(f"最大缓存文件数: {self.max_cache_files}")
    
    def generate_cache_key(self, text, voice_id, speed):
        """生成缓存键（基于文本内容、语音ID和语速）"""
        # 标准化文本（去除多余空白符）
        normalized_text = ' '.join(text.split())
        
        # 创建包含所有参数的字符串
        cache_string = f"{normalized_text}|{voice_id}|{speed}"
        
        # 生成 MD5 哈希
        cache_key = hashlib.md5(cache_string.encode('utf-8')).hexdigest()
        
        return cache_key, normalized_text
    
    def get_cache_filename(self, cache_key):
        """获取缓存文件名"""
        return os.path.join(self.cache_dir, f"{cache_key}.wav")
    
    def load_cache_index(self):
        """加载缓存索引"""
        if os.path.exists(self.cache_index_file):
            try:
                with open(self.cache_index_file, 'r', encoding='utf-8') as f:
                    return json.load(f)
            except Exception as e:
                rospy.logwarn(f"无法加载缓存索引: {e}")
        return {}
    
    def save_cache_index(self):
        """保存缓存索引"""
        try:
            with open(self.cache_index_file, 'w', encoding='utf-8') as f:
                json.dump(self.cache_index, f, ensure_ascii=False, indent=2)
        except Exception as e:
            rospy.logerr(f"无法保存缓存索引: {e}")
    
    def update_cache_entry(self, cache_key, text, voice_id, speed, audio_file):
        """更新缓存条目"""
        self.cache_index[cache_key] = {
            'text': text,
            'voice_id': voice_id,
            'speed': speed,
            'audio_file': audio_file,
            'created_time': datetime.now().isoformat(),
            'last_used': datetime.now().isoformat()
        }
        self.save_cache_index()
    
    def mark_cache_used(self, cache_key):
        """标记缓存为已使用（更新最后使用时间）"""
        if cache_key in self.cache_index:
            self.cache_index[cache_key]['last_used'] = datetime.now().isoformat()
            self.save_cache_index()
    
    def cleanup_old_cache(self):
        """清理旧的缓存文件"""
        if len(self.cache_index) <= self.max_cache_files:
            return
        
        rospy.loginfo(f"缓存文件数量 ({len(self.cache_index)}) 超过限制 ({self.max_cache_files})，开始清理...")
        
        # 按照最后使用时间排序
        sorted_entries = sorted(
            self.cache_index.items(),
            key=lambda x: x[1]['last_used']
        )
        
        # 计算需要删除的文件数
        files_to_delete = len(self.cache_index) - self.max_cache_files
        
        for i in range(files_to_delete):
            cache_key, entry = sorted_entries[i]
            audio_file = entry['audio_file']
            
            # 删除音频文件
            if os.path.exists(audio_file):
                try:
                    os.remove(audio_file)
                    rospy.loginfo(f"已删除旧缓存文件: {os.path.basename(audio_file)}")
                except Exception as e:
                    rospy.logwarn(f"删除缓存文件失败: {e}")
            
            # 从索引中移除
            del self.cache_index[cache_key]
        
        # 保存更新后的索引
        self.save_cache_index()
        rospy.loginfo(f"缓存清理完成，当前缓存文件数: {len(self.cache_index)}")
    
    def periodic_cache_cleanup(self):
        """定期缓存清理（每小时执行一次）"""
        while not rospy.is_shutdown():
            try:
                # 等待1小时
                time.sleep(3600)
                self.cleanup_old_cache()
            except Exception as e:
                rospy.logwarn(f"定期缓存清理出错: {e}")
    
    def find_cached_audio(self, text, voice_id, speed):
        """查找缓存的音频文件"""
        cache_key, normalized_text = self.generate_cache_key(text, voice_id, speed)
        
        if cache_key in self.cache_index:
            entry = self.cache_index[cache_key]
            audio_file = entry['audio_file']
            
            # 检查文件是否存在
            if os.path.exists(audio_file):
                # 标记为已使用
                self.mark_cache_used(cache_key)
                rospy.loginfo(f"找到缓存音频: {os.path.basename(audio_file)} (文本: '{normalized_text}')")
                return audio_file, True
            else:
                # 文件不存在，从索引中移除
                del self.cache_index[cache_key]
                self.save_cache_index()
                rospy.logwarn(f"缓存文件丢失，已从索引移除: {audio_file}")
        
        return None, False
    
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
                        except Exception:
                            rospy.logerr(f"无法播放音频文件: {audio_file}")
                else:
                    rospy.logerr("没有可用的音频播放方法")
                    
            except Exception as e:
                rospy.logerr(f"播放音频时出错: {e}")
        
        # 在单独线程中播放音频，避免阻塞
        threading.Thread(target=play_thread, daemon=True).start()
    
    def text_callback(self, msg):
        """处理文本输入回调"""
        text = msg.data.strip()
        if not text:
            self.publish_status("错误：输入文本为空")
            return
        
        rospy.loginfo(f"收到文本转语音请求: '{text}'")
        
        # 先查找缓存
        cached_audio, is_cached = self.find_cached_audio(text, self.default_voice_id, self.default_speed)
        
        if is_cached:
            # 使用缓存音频
            self.publish_status(f"使用缓存音频: '{text}'")
            
            # 发布音频文件路径
            audio_msg = String()
            audio_msg.data = cached_audio
            self.audio_file_publisher.publish(audio_msg)
            
            # 播放音频
            self.play_audio(cached_audio)
        else:
            # 生成新音频
            result = self.generate_and_cache_speech(text)
            
            if result['success']:
                self.publish_status(f"成功：音频已生成并缓存 '{text}'")
                
                # 发布音频文件路径
                audio_msg = String()
                audio_msg.data = result['audio_file']
                self.audio_file_publisher.publish(audio_msg)
                
                # 播放音频
                self.play_audio(result['audio_file'])
            else:
                self.publish_status(f"失败：{result['message']}")
    
    def generate_and_cache_speech(self, text, voice_id=None, speed=None):
        """生成语音并缓存"""
        # 使用默认参数
        if voice_id is None:
            voice_id = self.default_voice_id
        if speed is None:
            speed = self.default_speed
        
        # 生成缓存键和文件名
        cache_key, normalized_text = self.generate_cache_key(text, voice_id, speed)
        cache_filename = self.get_cache_filename(cache_key)
        
        try:
            rospy.loginfo(f"开始生成语音: 文本='{normalized_text}', 语音={voice_id}, 语速={speed}")
            
            # 调用 pipeline 获取生成器
            generator = self.pipeline(normalized_text, voice=voice_id, speed=speed)
            
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
            
            # 保存为 WAV 文件到缓存目录
            sf.write(cache_filename, full_audio_data, self.sample_rate)
            
            # 计算音频时长
            duration = len(full_audio_data) / self.sample_rate
            
            # 更新缓存索引
            self.update_cache_entry(cache_key, normalized_text, voice_id, speed, cache_filename)
            
            # 检查并清理旧缓存
            self.cleanup_old_cache()
            
            rospy.loginfo(f"语音生成并缓存成功: 文件={os.path.basename(cache_filename)}, 时长={duration:.2f}秒, 音频块数={chunk_count}")
            
            return {
                'success': True,
                'message': '语音生成并缓存成功',
                'audio_file': cache_filename,
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
    
    def get_cache_stats(self):
        """获取缓存统计信息"""
        return {
            'total_files': len(self.cache_index),
            'max_files': self.max_cache_files,
            'cache_dir': self.cache_dir,
            'cache_size_mb': sum(
                os.path.getsize(entry['audio_file']) 
                for entry in self.cache_index.values() 
                if os.path.exists(entry['audio_file'])
            ) / (1024 * 1024)
        }
    
    def run(self):
        """运行服务"""
        try:
            # 打印缓存统计
            stats = self.get_cache_stats()
            rospy.loginfo(f"缓存统计: {stats['total_files']}/{stats['max_files']} 文件, {stats['cache_size_mb']:.2f} MB")
            
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Kokoro 缓存 TTS 服务停止")


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
        service = KokoroCachedTTSService()
        service.run()
        
    except Exception as e:
        rospy.logerr(f"服务启动失败: {e}")
        traceback.print_exc()


if __name__ == '__main__':
    main() 