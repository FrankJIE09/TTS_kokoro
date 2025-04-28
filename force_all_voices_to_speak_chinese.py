# Filename: force_all_voices_to_speak_chinese.py

import os
# Ensure kokoro is installed and imported correctly
try:
    from kokoro import KPipeline
except ImportError:
    print("错误: 未找到 'kokoro' 库。请确保已安装。")
    exit()
# from IPython.display import display, Audio # Optional for notebooks
import soundfile as sf
# import torch # Keep if needed by kokoro library

# --- Configuration ---

# WARNING / 警告:
# This list contains voice IDs designed for VARIOUS languages, not just Chinese.
# 这个列表包含为多种语言设计的语音ID（英语、日语、西班牙语等），不仅仅是中文。
# Forcing them to speak Chinese with lang_code='z' is technically incorrect
# and likely to FAIL or produce GARBLED audio for non-Chinese voices.
# 强制它们使用 lang_code='z' 说中文在技术上是不正确的，并且很可能对非中文语音导致失败或产生混乱的音频。
# Only the zf_* and zm_* voices are expected to work correctly.
# 只有 zf_* 和 zm_* 的语音预计能正常工作。
all_voices_combined = [
    # Chinese Voices (中文语音 - 预计能工作)
    'zf_xiaobei', 'zf_xiaoni', 'zf_xiaoxiao', 'zf_xiaoyi',
    'zm_yunjian', 'zm_yunxi', 'zm_yunxia', 'zm_yunyang',
    # American/General English Voices (美式/通用英语 - 预计会失败/混乱)
    'af_alloy', 'af_aoede', 'af_bella', 'af_korie',
    'af_nova', 'af_river', 'af_shara', 'af_sky',
    'am_adam', 'am_echo', 'am_eric', 'am_fenrir', 'am_liam', 'am_michael',
    'am_onyx', 'am_puck', 'am_santa',
    # British English Voices (英式英语 - 预计会失败/混乱)
    'bf_alice', 'bf_emma', 'bf_isabella', 'bf_lily', 'bm_daniel', 'bm_fable',
    'bm_george', 'bm_lewis',
    # Japanese Voices (日语 - 预计会失败/混乱)
    'jf_alpha', 'jf_gongitsune', 'jf_nezumi', 'jf_tebukuro', 'jm_kumo',
    # Spanish Voices (西班牙语 - 预计会失败/混乱)
    'ef_dora', 'em_alex', 'em_santa',
    # French Voice (法语 - 预计会失败/混乱)
    'ff_siwis',
    # Hindi Voices (印地语 - 预计会失败/混乱)
    'hf_alpha', 'hf_beta', 'hm_omega', 'hm_psi',
    # Note: Italian and Portuguese voices were excluded based on the user's previous list update.
]

# Chinese text to be spoken by ALL voices (所有语音都尝试说的中文文本)
# Using the original text provided by the user (使用用户最初提供的文本)
chinese_text_for_all = '''
朝中社当天援引朝鲜劳动党中央军事委员会27日一份书面声明说，按照朝鲜国家元首命令，朝鲜武装力量部队参加库尔斯克地区收复作战，为收复俄罗斯联邦领土作出重大贡献，“完全符合联合国宪章等国际法和朝俄《全面战略伙伴关系条约》的各条款精神”。

声明说，库尔斯克地区收复作战胜利结束，“是彰显朝俄两国牢固战斗友谊和两国人民的同盟关系、兄弟关系（所达到）的最高层次战略高度的历史新篇章”。

声明就朝方当时决策赴俄参战一事说，朝鲜劳动党总书记、国务委员长金正恩分析并判定当时战况符合行使朝俄《全面战略伙伴关系条约》相关条款，据此决定朝鲜武装力量参战并通报了俄方。

俄罗斯外交部发言人扎哈罗娃26日表示，朝鲜士兵在库尔斯克州作战，为俄军收复该州作出重大贡献。

26日早些时候，俄武装力量总参谋长格拉西莫夫在向俄总统普京汇报已收复库尔斯克州时说，朝鲜军人根据俄朝《全面战略伙伴关系条约》，为击溃库尔斯克州乌军提供了巨大帮助。(新华社
'''

# Base output directory (基础输出目录)
base_output_dir = "forced_chinese_output_for_all_voices"
os.makedirs(base_output_dir, exist_ok=True)
print(f"基础输出目录: '{base_output_dir}'")

# --- Initialization (初始化) ---
pipeline = None
try:
    # Initialize pipeline ONCE for Chinese ('z')
    # 仅为中文 ('z') 初始化一次 Pipeline
    print(f"正在初始化用于中文 (lang_code='z') 的 Kokoro Pipeline...")
    pipeline = KPipeline(lang_code='z')
    print("Pipeline 初始化成功。")
except NameError:
     print(f"错误: 未找到 KPipeline 类 (kokoro 是否已正确导入?).")
     exit()
except Exception as e:
    print(f"初始化 Kokoro Pipeline 时出错: {e}")
    exit() # If pipeline cannot be initialized, exit

# --- Main Processing Loop (主要处理循环) ---
print("\n*** 警告：正在尝试让所有语音说中文。 ***")
print("*** 对于非中文语音 (af_*, am_*, bf_*, jf_* 等)，预计会发生错误或结果很差。 ***\n")

# Loop through the single combined list of all voices
# 循环处理包含所有语音的单一合并列表
for voice in all_voices_combined:
    # Determine if the voice is expected to be Chinese
    # 判断语音是否预期为中文
    is_chinese_voice = voice.startswith('zf_') or voice.startswith('zm_')
    expected_outcome = "工作 (Work)" if is_chinese_voice else "失败/混乱 (FAIL/GARBLE)"

    print(f"\n--- 正在处理语音: {voice} (尝试说中文 | 预期: {expected_outcome}) ---")

    # Define and create output directory for the current voice
    # 为当前语音定义并创建输出目录
    voice_output_dir = os.path.join(base_output_dir, voice)
    try:
        os.makedirs(voice_output_dir, exist_ok=True)
    except OSError as e:
        print(f"  创建目录 '{voice_output_dir}' 时出错: {e}. 跳过语音 '{voice}'.")
        continue # Skip to next voice if directory creation fails

    # Generate audio using lang_code='z' and Chinese text for ALL voices
    # 对所有语音使用 lang_code='z' 和中文文本生成音频
    try:
        print(f"  正在使用中文文本生成音频...")
        generator = pipeline(chinese_text_for_all, voice=voice)
        segment_count = 0
        audio_generated = False # Flag to check if generator yields anything

        for i, (gs, ps, audio) in enumerate(generator):
            audio_generated = True
            # Construct the full path for the output file
            output_filename = os.path.join(voice_output_dir, f'segment_{str(i).zfill(3)}.wav')
            print(f"    片段 {i}: gs={gs}, ps={ps} -> 保存至 '{output_filename}'")
            # Save the audio segment to a WAV file
            try:
                # Assuming 24000 Hz is correct for all Kokoro voices. Adjust if needed.
                sf.write(output_filename, audio, 24000)
                segment_count += 1
            except Exception as write_e:
                print(f"      保存片段 {i} 到 '{output_filename}' 时出错: {write_e}")

        # Report outcome based on generation and saving
        if not audio_generated:
            print(f"  结果: 未生成音频片段。(结果: {'非中文语音的预期结果' if not is_chinese_voice else '中文语音的意外结果'})")
        elif segment_count > 0:
            outcome_note = '可能混乱' if not is_chinese_voice else 'OK'
            print(f"  结果: 处理完成。已保存 {segment_count} 个片段。(结果: {'非中文语音' + outcome_note if not is_chinese_voice else '中文语音' + outcome_note})")
        else:
             # Generated audio but failed to save any files
             print(f"  结果: 处理完成，但未能成功保存任何片段。(结果: 发生问题)")

    except Exception as e:
        # Catch errors during the pipeline call or iteration
        print(f"  为语音 '{voice}' 生成/处理音频时出错: {e}")
        if not is_chinese_voice:
            print(f"  结果: 失败符合非中文语音的预期。")
        else:
            print(f"  结果: 错误 - 中文语音发生意外失败！")
        # Continue with the next voice even if one fails

print(f"\n{'='*10} 完成尝试让所有语音说中文 {'='*10}")
print("*** 请仔细检查输出结果。非中文语音很可能产生了错误或无法使用的音频。 ***")