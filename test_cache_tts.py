#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试 Kokoro 缓存 TTS 功能
"""

import rospy
from std_msgs.msg import String
import time
import os
import json


def test_cache_functionality():
    """测试缓存功能"""
    
    rospy.init_node('test_cache_tts', anonymous=True)
    
    # 创建发布者
    text_publisher = rospy.Publisher('/kokoro_tts/text_input', String, queue_size=10)
    
    # 订阅状态信息
    status_messages = []
    audio_files = []
    
    def status_callback(msg):
        status_messages.append(msg.data)
        print(f"[状态] {msg.data}")
    
    def audio_callback(msg):
        audio_files.append(msg.data)
        print(f"[音频] {msg.data}")
    
    status_subscriber = rospy.Subscriber('/kokoro_tts/status', String, status_callback)
    audio_subscriber = rospy.Subscriber('/kokoro_tts/audio_file', String, audio_callback)
    
    # 等待连接建立
    rospy.sleep(2)
    
    print("🧪 Kokoro 缓存 TTS 功能测试")
    print("=" * 50)
    
    # 测试案例
    test_cases = [
        "你好，世界",           # 第一次生成
        "欢迎使用缓存系统",     # 第一次生成
        "你好，世界",           # 应该使用缓存
        "测试音频播放功能",     # 第一次生成
        "欢迎使用缓存系统",     # 应该使用缓存
        "你好，世界",           # 应该使用缓存
    ]
    
    print(f"📝 测试用例: {len(test_cases)} 个")
    print(f"📂 缓存目录: ~/.sage/kokoro_voice")
    print()
    
    for i, text in enumerate(test_cases, 1):
        print(f"🔄 测试 {i}/{len(test_cases)}: '{text}'")
        
        # 记录开始时间
        start_time = time.time()
        
        # 发布文本消息
        msg = String()
        msg.data = text
        text_publisher.publish(msg)
        
        # 等待处理
        time.sleep(4)  # 给足够时间生成或读取缓存
        
        # 计算耗时
        elapsed = time.time() - start_time
        
        # 分析结果
        if status_messages:
            last_status = status_messages[-1]
            if "使用缓存音频" in last_status:
                print(f"  ✅ 缓存命中 (耗时: {elapsed:.2f}s)")
            elif "音频已生成并缓存" in last_status:
                print(f"  🆕 新生成并缓存 (耗时: {elapsed:.2f}s)")
            else:
                print(f"  ❓ 状态: {last_status}")
        
        print()
    
    # 显示缓存统计
    print("📊 缓存统计信息:")
    cache_dir = os.path.expanduser('~/.sage/kokoro_voice')
    cache_index_file = os.path.join(cache_dir, 'cache_index.json')
    
    if os.path.exists(cache_index_file):
        try:
            with open(cache_index_file, 'r', encoding='utf-8') as f:
                cache_index = json.load(f)
            
            print(f"  📄 缓存文件数: {len(cache_index)}")
            print(f"  📁 缓存目录: {cache_dir}")
            
            # 计算缓存大小
            total_size = 0
            for entry in cache_index.values():
                if os.path.exists(entry['audio_file']):
                    total_size += os.path.getsize(entry['audio_file'])
            
            print(f"  💾 缓存大小: {total_size / (1024 * 1024):.2f} MB")
            
            print(f"  📝 缓存内容:")
            for key, entry in cache_index.items():
                print(f"    • {entry['text']}")
                
        except Exception as e:
            print(f"  ❌ 读取缓存失败: {e}")
    else:
        print("  📂 暂无缓存文件")
    
    print("\n✅ 测试完成!")


def main():
    """主函数"""
    try:
        test_cache_functionality()
    except rospy.ROSInterruptException:
        print("测试被中断")
    except KeyboardInterrupt:
        print("用户中断测试")


if __name__ == '__main__':
    main() 