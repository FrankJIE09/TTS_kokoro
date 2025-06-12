#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Kokoro TTS 快速测试脚本
用于验证 kokoro 库和音频生成功能
"""

import os
import traceback
import numpy as np
import soundfile as sf
from datetime import datetime

def test_kokoro_import():
    """测试 kokoro 库导入"""
    try:
        from kokoro import KPipeline
        print("✓ kokoro 库导入成功")
        return True
    except ImportError as e:
        print(f"✗ kokoro 库导入失败: {e}")
        return False

def test_dependencies():
    """测试依赖库"""
    success = True
    
    try:
        import soundfile as sf
        print("✓ soundfile 库可用")
    except ImportError:
        print("✗ soundfile 库未安装")
        success = False
    
    try:
        import numpy as np
        print("✓ numpy 库可用")
    except ImportError:
        print("✗ numpy 库未安装")
        success = False
    
    try:
        import torch
        print(f"✓ PyTorch 可用 (版本: {torch.__version__})")
    except ImportError:
        print("⚠ PyTorch 未安装 (kokoro 可能需要)")
    
    return success

def test_tts_generation():
    """测试 TTS 音频生成"""
    try:
        from kokoro import KPipeline
        
        print("正在初始化 Kokoro Pipeline...")
        pipeline = KPipeline(lang_code='z')  # 中文
        
        test_text = "你好，这是一个测试"
        print(f"测试文本: '{test_text}'")
        
        print("生成音频中...")
        generator = pipeline(test_text, voice='zf_xiaoyi', speed=1.0)
        
        # 收集音频块
        audio_chunks = []
        for i, (gs, ps, audio_chunk) in enumerate(generator):
            if not isinstance(audio_chunk, np.ndarray):
                if hasattr(audio_chunk, 'numpy'):
                    audio_chunk = audio_chunk.cpu().numpy()
                else:
                    raise TypeError(f"未知的音频块类型: {type(audio_chunk)}")
            audio_chunks.append(audio_chunk)
            print(f"  收集音频块 {i+1}")
        
        if audio_chunks:
            # 合并音频
            full_audio = np.concatenate(audio_chunks)
            
            # 保存测试文件
            output_dir = "test_audio"
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = os.path.join(output_dir, f"test_{timestamp}.wav")
            
            sf.write(output_path, full_audio, 24000)
            
            duration = len(full_audio) / 24000
            print(f"✓ 音频生成成功!")
            print(f"  文件: {output_path}")
            print(f"  时长: {duration:.2f} 秒")
            print(f"  音频块数: {len(audio_chunks)}")
            
            return True
        else:
            print("✗ 没有生成音频块")
            return False
            
    except Exception as e:
        print(f"✗ TTS 生成测试失败: {e}")
        traceback.print_exc()
        return False

def main():
    """主测试函数"""
    print("=== Kokoro TTS 功能测试 ===\n")
    
    # 测试导入
    if not test_kokoro_import():
        print("\n请先安装 kokoro 库: pip install kokoro")
        return
    
    print()
    
    # 测试依赖
    if not test_dependencies():
        print("\n请安装缺失的依赖库")
        return
    
    print()
    
    # 测试 TTS 生成
    if test_tts_generation():
        print("\n🎉 所有测试通过! Kokoro TTS 准备就绪")
        print("\n现在可以启动 ROS 服务:")
        print("  1. roscore")
        print("  2. ./launch_kokoro_tts.sh")
        print("  3. python3 kokoro_tts_client.py")
    else:
        print("\n❌ TTS 生成测试失败，请检查配置")

if __name__ == '__main__':
    main() 