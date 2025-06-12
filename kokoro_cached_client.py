#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import sys
import os


class KokoroCachedTTSClient:
    def __init__(self):
        """初始化 Kokoro 缓存 TTS 客户端"""
        rospy.init_node('kokoro_cached_tts_client', anonymous=True)
        
        # 创建发布者和订阅者
        self.text_publisher = rospy.Publisher('/kokoro_tts/text_input', String, queue_size=10)
        self.status_subscriber = rospy.Subscriber('/kokoro_tts/status', String, self.status_callback)
        self.audio_subscriber = rospy.Subscriber('/kokoro_tts/audio_file', String, self.audio_callback)
        
        # 等待服务准备
        rospy.sleep(1)
        rospy.loginfo("Kokoro 缓存 TTS 客户端已准备就绪")
        rospy.loginfo("📂 缓存目录: ~/.sage/kokoro_voice")
        rospy.loginfo("🔄 智能缓存: 相同文本会直接播放缓存音频")
    
    def status_callback(self, msg):
        """接收状态消息回调"""
        status = msg.data
        if "使用缓存音频" in status:
            print(f"🎵 {status}")
        elif "成功：音频已生成并缓存" in status:
            print(f"✨ {status}")
        elif "失败" in status:
            print(f"❌ {status}")
        else:
            print(f"ℹ️  {status}")
    
    def audio_callback(self, msg):
        """接收音频文件路径回调"""
        audio_file = msg.data.strip()
        if audio_file:
            filename = os.path.basename(audio_file)
            if audio_file.startswith(os.path.expanduser('~/.sage/kokoro_voice')):
                print(f"🔊 播放缓存音频: {filename}")
            else:
                print(f"🔊 播放新音频: {filename}")
    
    def send_text(self, text):
        """发送文本给 TTS 服务"""
        if not text:
            print("⚠️  文本为空")
            return
        
        msg = String()
        msg.data = text
        
        print(f"📝 发送文本: '{text}'")
        self.text_publisher.publish(msg)
    
    def interactive_mode(self):
        """交互模式"""
        print("\n🎤 Kokoro 缓存 TTS 交互模式")
        print("=" * 50)
        print("✨ 特性:")
        print("  • 智能缓存: 相同文本直接播放缓存音频")
        print("  • 自动清理: 最多保留30个缓存文件")
        print("  • 即时播放: 音频生成后立即播放")
        print()
        print("💡 输入中文文本进行语音合成")
        print("   输入 'quit' 或 'exit' 退出")
        print("   输入 'stats' 查看缓存统计")
        print("=" * 50)
        
        while not rospy.is_shutdown():
            try:
                text = input("\n🗣️  请输入文本: ").strip()
                
                if text.lower() in ['quit', 'exit', '退出']:
                    break
                elif text.lower() == 'stats':
                    self.show_cache_stats()
                elif text:
                    self.send_text(text)
                    rospy.sleep(0.5)  # 短暂等待
                else:
                    print("⚠️  请输入有效的文本")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
        
        print("\n👋 客户端退出")
    
    def show_cache_stats(self):
        """显示缓存统计信息"""
        cache_dir = os.path.expanduser('~/.sage/kokoro_voice')
        cache_index_file = os.path.join(cache_dir, 'cache_index.json')
        
        if os.path.exists(cache_index_file):
            try:
                import json
                with open(cache_index_file, 'r', encoding='utf-8') as f:
                    cache_index = json.load(f)
                
                print(f"\n📊 缓存统计:")
                print(f"  📁 缓存目录: {cache_dir}")
                print(f"  📄 缓存文件数: {len(cache_index)}/30")
                
                # 计算缓存大小
                total_size = 0
                for entry in cache_index.values():
                    if os.path.exists(entry['audio_file']):
                        total_size += os.path.getsize(entry['audio_file'])
                
                print(f"  💾 缓存大小: {total_size / (1024 * 1024):.2f} MB")
                
                if cache_index:
                    print(f"  📝 最新缓存文本:")
                    # 显示最近的5个缓存文本
                    sorted_entries = sorted(
                        cache_index.items(),
                        key=lambda x: x[1]['last_used'],
                        reverse=True
                    )
                    for i, (key, entry) in enumerate(sorted_entries[:5], 1):
                        print(f"    {i}. {entry['text']}")
                
            except Exception as e:
                print(f"❌ 读取缓存统计失败: {e}")
        else:
            print("📊 暂无缓存文件")


def main():
    """主函数"""
    try:
        client = KokoroCachedTTSClient()
        
        # 检查命令行参数
        if len(sys.argv) > 1:
            # 如果有命令行参数，直接发送
            text = " ".join(sys.argv[1:])
            client.send_text(text)
            rospy.sleep(3)  # 等待处理完成
        else:
            # 否则进入交互模式
            client.interactive_mode()
            
    except Exception as e:
        rospy.logerr(f"客户端错误: {e}")


if __name__ == '__main__':
    main() 