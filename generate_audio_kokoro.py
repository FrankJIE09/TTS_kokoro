import os
import traceback
import time  # 用于可能的延迟

try:
    from kokoro import KPipeline  # 导入 kokoro 库
except ImportError:
    print("错误：无法导入 'kokoro'。")
    print("请确保已安装库： pip install kokoro")
    print("并且可能需要安装: sudo apt install espeak-ng")
    exit()
try:
    import soundfile as sf  # 用于保存 WAV 文件
except ImportError:
    print("错误：无法导入 'soundfile'。")
    print("请确保已安装库： pip install soundfile")
    exit()
try:
    import numpy as np  # 用于合并音频块
except ImportError:
    print("错误：无法导入 'numpy'。")
    print("请确保已安装库： pip install numpy")
    exit()

# --- 配置 ---
OUTPUT_DIR = "audio"  # 输出 WAV 文件的子目录
LANGUAGE_CODE = "z"  # 指定中文普通话 (根据 kokoro-onnx 文档)
VOICE_ID = "zf_xiaoyi"  # 你示例中使用的特定中文女声，推荐使用这个
SPEECH_SPEED = 1  # 语速 (1.0 = 正常)
SAMPLE_RATE = 24000  # Kokoro 模型通常使用 24kHz 采样率

# 需要生成的语音提示及其对应的文本
# (键名需要与你的 dual_arm_config.yaml 中的 audio_files 部分匹配)
AUDIO_MAPPING = {
    "system_ready": "系统准备就绪",
    "xyz_mode": "位置控制模式",
    "rpy_mode": "姿态控制模式",
    "vision_enter": "进入视觉模式",
    "vision_exit": "退出视觉模式",
    "left_open": "打开左夹爪",
    "left_close": "关闭左夹爪",
    "right_open": "打开右夹爪",
    "right_close": "关闭右夹爪",
    "gripper_inactive": "夹爪未激活",
    "speed_up": "加速",
    "speed_down": "减速",
    # "speed_down": "减速",  # 可选
    # "speed_down": "减速",  # 可选

}


# --- 主程序 ---
def generate_files():
    print("--- Kokoro WAV 文件生成器 (使用 KPipeline) ---")

    # 创建输出目录
    if not os.path.exists(OUTPUT_DIR):
        try:
            os.makedirs(OUTPUT_DIR)
            print(f"已创建输出目录: {OUTPUT_DIR}")
        except OSError as e:
            print(f"错误：无法创建输出目录 '{OUTPUT_DIR}': {e}")
            return

    # 初始化 Kokoro Pipeline
    try:
        print(f"正在初始化 Kokoro Pipeline (语言: {LANGUAGE_CODE})...")
        # 注意：lang_code 可能在 KPipeline 初始化时指定，或在调用时指定
        # 参考 kokoro 库的实际用法，这里假设在初始化时指定
        # 如果初始化时不能指定 lang_code，则需要在 pipeline(...) 调用中指定
        pipeline = KPipeline(lang_code=LANGUAGE_CODE)
        print("Kokoro Pipeline 初始化成功。")
    except Exception as e:
        print(f"错误：初始化 Kokoro Pipeline 失败: {e}")
        print("请确保已安装 kokoro 库及其依赖 (可能需要 torch, espeak-ng等)。")
        traceback.print_exc()
        return

    # 生成每个音频文件
    print("\n开始生成音频文件...")
    success_count = 0
    fail_count = 0
    for event_name, text in AUDIO_MAPPING.items():
        output_path = os.path.join(OUTPUT_DIR, f"{event_name}.wav")
        print(f"  生成 '{event_name}' -> '{output_path}' (文本: '{text}')...")

        try:
            # 调用 pipeline 获取生成器
            # 如果初始化时没指定 lang_code，在这里加: lang=LANGUAGE_CODE
            generator = pipeline(text, voice=VOICE_ID, speed=SPEECH_SPEED)

            # 收集所有音频块
            audio_chunks = []
            print("    收集合成块: ", end="")
            for i, (gs, ps, audio_chunk) in enumerate(generator):
                # audio_chunk 通常是 numpy array 或 torch tensor
                # 确保它是 numpy array
                if not isinstance(audio_chunk, np.ndarray):
                    if hasattr(audio_chunk, 'numpy'):  # 检查是否是 tensor
                        audio_chunk = audio_chunk.cpu().numpy()
                    else:
                        print(f"\n    错误：未知的音频块类型 {type(audio_chunk)}，跳过 '{event_name}'。")
                        audio_chunks = None  # 标记失败
                        break
                audio_chunks.append(audio_chunk)
                print(f"{i + 1}", end=" ")  # 打印块编号
            print(" ...完成")

            if audio_chunks:  # 如果成功收集到块
                # 合并所有块
                full_audio_data = np.concatenate(audio_chunks)

                # 保存为 WAV 文件
                sf.write(output_path, full_audio_data, SAMPLE_RATE)
                print(f"    成功保存到: {output_path}")
                success_count += 1
            else:
                fail_count += 1


        except Exception as e:
            print(f"\n    错误：生成或保存 '{event_name}' 时失败: {e}")
            traceback.print_exc()  # 打印详细错误
            fail_count += 1
        # time.sleep(0.5) # 可选：每次生成后停顿一下

    print("\n--- 生成结束 ---")
    print(f"成功: {success_count} 个文件")
    print(f"失败: {fail_count} 个文件")
    print(f"文件已保存在 '{OUTPUT_DIR}' 目录下。")
    print("请确保 dual_arm_config.yaml 中的 audio_files 路径指向这些生成的文件。")


if __name__ == "__main__":
    # 检查 PyTorch 是否可用 (kokoro 可能需要)
    try:
        import torch

        print(f"PyTorch version: {torch.__version__}")
    except ImportError:
        print("警告：未找到 PyTorch。kokoro 库可能需要它。")
        print("可以尝试安装: pip install torch")

    generate_files()
