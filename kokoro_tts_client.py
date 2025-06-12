#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import sys


class KokoroTTSClient:
    def __init__(self):
        """初始化 Kokoro TTS 客户端"""
        rospy.init_node('kokoro_tts_client', anonymous=True)
        
        # 创建发布者和订阅者
        self.text_publisher = rospy.Publisher('/kokoro_tts/text_input', String, queue_size=10)
        self.status_subscriber = rospy.Subscriber('/kokoro_tts/status', String, self.status_callback)
        
        # 等待服务准备
        rospy.sleep(1)
        rospy.loginfo("Kokoro TTS 客户端已准备就绪")
    
    def status_callback(self, msg):
        """接收状态消息回调"""
        rospy.loginfo(f"服务状态: {msg.data}")
    
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