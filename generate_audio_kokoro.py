# -*- coding: utf-8 -*-
import os
import traceback
import time

try:
    from kokoro import KPipeline
except ImportError:
    print("错误：无法导入 'kokoro'。")
    print("请确保已安装库： pip install kokoro")
    print("并且可能需要安装: sudo apt install espeak-ng")
    exit()
try:
    import soundfile as sf
except ImportError:
    print("错误：无法导入 'soundfile'。")
    print("请确保已安装库： pip install soundfile")
    exit()
try:
    import numpy as np
except ImportError:
    print("错误：无法导入 'numpy'。")
    print("请确保已安装库： pip install numpy")
    exit()

# --- 全局配置和初始化 ---
LANGUAGE_CODE = "z"  # 中文普通话
SPEECH_SPEED = 1.0
SAMPLE_RATE = 24000
pipeline = None

def initialize_pipeline():
    """初始化并返回全局的 Kokoro Pipeline 实例。"""
    global pipeline
    if pipeline is None:
        print(f"正在初始化 Kokoro Pipeline (语言: {LANGUAGE_CODE})...")
        try:
            pipeline = KPipeline(lang_code=LANGUAGE_CODE)
            print("Kokoro Pipeline 初始化成功。")
        except Exception as e:
            print(f"错误：初始化 Kokoro Pipeline 失败: {e}")
            traceback.print_exc()
            raise  # 重新引发异常，以便调用者知道失败了
    return pipeline

def generate_audio(text: str, voice: str, output_path: str):
    """
    使用 Kokoro TTS 生成单个音频文件。

    Args:
        text (str): 要转换为语音的文本。
        voice (str): 要使用的声音ID (例如 "zf_xiaoyi")。
        output_path (str): 保存生成的 .wav 文件的完整路径。
    """
    global pipeline
    try:
        # 确保 pipeline 已初始化
        pipeline = initialize_pipeline()

        print(f"  生成音频 -> '{output_path}' (声音: '{voice}', 文本: '{text}')...")

        # 调用 pipeline 获取生成器
        generator = pipeline(text, voice=voice, speed=SPEECH_SPEED)

        # 收集所有音频块
        audio_chunks = []
        for audio_chunk in generator:
            # 处理不同类型的音频数据
            if isinstance(audio_chunk, np.ndarray):
                # 直接是numpy数组
                audio_data = audio_chunk
            elif hasattr(audio_chunk, 'audio'):
                # Kokoro Result对象，提取audio属性
                audio_data = audio_chunk.audio
                if not isinstance(audio_data, np.ndarray):
                    if hasattr(audio_data, 'detach'):
                        audio_data = audio_data.detach().cpu().numpy()
                    else:
                        audio_data = np.array(audio_data)
            elif hasattr(audio_chunk, 'detach'):
                # PyTorch张量
                audio_data = audio_chunk.detach().cpu().numpy()
            elif hasattr(audio_chunk, 'numpy'):
                # 可能是TensorFlow张量
                audio_data = audio_chunk.numpy()
            else:
                # 尝试直接转换为numpy数组
                try:
                    audio_data = np.array(audio_chunk)
                except:
                    raise TypeError(f"未知的音频块类型 {type(audio_chunk)}，无法转换为numpy数组")
            
            # 确保是1维数组
            if audio_data.ndim > 1:
                audio_data = audio_data.flatten()
            audio_chunks.append(audio_data)

        if not audio_chunks:
            raise ValueError("未能生成任何音频数据块。")

        # 合并并保存音频
        full_audio_data = np.concatenate(audio_chunks)
        
        # 确保数据类型正确
        if full_audio_data.dtype != np.float32:
            full_audio_data = full_audio_data.astype(np.float32)
            # 检查并进行归一化（如果需要）
            min_val, max_val = np.min(full_audio_data), np.max(full_audio_data)
            if max_val > 1.0 or min_val < -1.0:
                print(f"    警告: 音频数据范围 [{min_val:.2f}, {max_val:.2f}] 超出 [-1, 1]，进行削波处理。")
                full_audio_data = np.clip(full_audio_data, -1.0, 1.0)

        sf.write(output_path, full_audio_data, SAMPLE_RATE, format='WAV', subtype='FLOAT')
        print(f"    成功保存到: {output_path}")

    except Exception as e:
        print(f"\n    错误：在生成 '{output_path}' 时失败: {e}")
        traceback.print_exc()
        raise # 重新引发异常，以便 web server 可以捕获它

# --- 用于批量生成的旧逻辑 ---
OUTPUT_DIR = "audio"
VOICE_ID_DEFAULT = "zf_xiaoyi"
AUDIO_MAPPING = {
    "system_ready": "系统准备就绪",
    "xyz_mode": "位置控制模式",
    "rpy_mode": "姿态控制模式",
    "reset_mode": "回正模式",
    "vision_enter": "进入视觉模式",
    "vision_exit": "退出视觉模式",
    "left_open": "打开左夹爪",
    "left_close": "关闭左夹爪",
    "right_open": "打开右夹爪",
    "right_close": "关闭右夹爪",
    "gripper_inactive": "夹爪未激活",
    "left_reset_success": "左臂回正成功",
    "left_reset_fail": "左臂回正失败",
    "right_reset_success": "右臂回正成功",
    "right_reset_fail": "右臂回正失败",
}

def generate_files():
    """
    （旧功能）根据 AUDIO_MAPPING 批量生成预设的音频文件。
    """
    print("\n--- Kokoro WAV 文件批量生成器 ---")
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"已创建输出目录: {OUTPUT_DIR}")

    success_count = 0
    fail_count = 0
    for event_name, text in AUDIO_MAPPING.items():
        output_filename = f"{event_name}.wav"
        output_path = os.path.join(OUTPUT_DIR, output_filename)
        try:
            generate_audio(text, VOICE_ID_DEFAULT, output_path)
            success_count += 1
        except Exception:
            fail_count += 1
    
    print("\n--- 批量生成结束 ---")
    print(f"成功: {success_count} 个文件")
    print(f"失败: {fail_count} 个文件")

if __name__ == "__main__":
    # 当作为主脚本运行时，执行批量生成
    generate_files()
