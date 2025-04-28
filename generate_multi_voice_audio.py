# Filename: generate_multi_voice_audio.py

import os
from kokoro import KPipeline
# from IPython.display import display, Audio # Keep if running in Jupyter/IPython, comment out for standard script
import soundfile as sf
# import torch # Keep if kokoro library requires it implicitly

# --- Configuration ---

# List of voices to use
voices = [
    'zf_xiaobei', 'zf_xiaoni', 'zf_xiaoxiao', 'zf_xiaoyi',
    'zm_yunjian', 'zm_yunxi', 'zm_yunxia', 'zm_yunyang'
]

# Input text (Corrected a potential typo: 格拉西モ夫 -> 格拉西莫夫)
text = '''
朝中社当天援引朝鲜劳动党中央军事委员会27日一份书面声明说，按照朝鲜国家元首命令，朝鲜武装力量部队参加库尔斯克地区收复作战，为收复俄罗斯联邦领土作出重大贡献，“完全符合联合国宪章等国际法和朝俄《全面战略伙伴关系条约》的各条款精神”。

声明说，库尔斯克地区收复作战胜利结束，“是彰显朝俄两国牢固战斗友谊和两国人民的同盟关系、兄弟关系（所达到）的最高层次战略高度的历史新篇章”。

声明就朝方当时决策赴俄参战一事说，朝鲜劳动党总书记、国务委员长金正恩分析并判定当时战况符合行使朝俄《全面战略伙伴关系条约》相关条款，据此决定朝鲜武装力量参战并通报了俄方。

俄罗斯外交部发言人扎哈罗娃26日表示，朝鲜士兵在库尔斯克州作战，为俄军收复该州作出重大贡献。

26日早些时候，俄武装力量总参谋长格拉西莫夫在向俄总统普京汇报已收复库尔斯克州时说，朝鲜军人根据俄朝《全面战略伙伴关系条约》，为击溃库尔斯克州乌军提供了巨大帮助。(新华社
'''

# --- Initialization ---

# Initialize the TTS pipeline
# Assuming lang_code="z" is appropriate for these voices (Mandarin Chinese)
try:
    pipeline = KPipeline(lang_code="z")
    print("Kokoro Pipeline initialized successfully.")
except Exception as e:
    print(f"Error initializing Kokoro Pipeline: {e}")
    exit() # Exit if pipeline cannot be initialized

# --- Main Processing Loop ---

# Loop through each specified voice
for voice in voices:
    print(f"\n--- Processing voice: {voice} ---")

    # Define the output directory for the current voice
    output_dir = voice  # Use the voice name as the directory name

    # Create the directory if it doesn't exist
    try:
        os.makedirs(output_dir, exist_ok=True)
        print(f"Output directory created/verified: '{output_dir}'")
    except OSError as e:
        print(f"Error creating directory '{output_dir}': {e}")
        continue # Skip to the next voice if directory creation fails

    # Generate audio for the current voice
    try:
        print(f"Generating audio for voice '{voice}'...")
        generator = pipeline(text, voice=voice)

        # Process and save each audio segment
        segment_count = 0
        for i, (gs, ps, audio) in enumerate(generator):
            # Construct the full path for the output file
            output_filename = os.path.join(output_dir, f'{i}.wav')

            print(f"  Segment {i}: gs={gs}, ps={ps} -> Saving to '{output_filename}'")

            # --- Optional: Display audio if in a suitable environment ---
            # try:
            #     # Set autoplay=False, especially in a loop
            #     display(Audio(data=audio, rate=24000, autoplay=False))
            # except NameError:
            #     # display/Audio not defined (e.g., running as a standard script)
            #     pass
            # except Exception as display_e:
            #     print(f"    Warning: Could not display audio segment {i}: {display_e}")
            # --- End Optional Display ---

            # Save the audio segment to a WAV file
            try:
                sf.write(output_filename, audio, 24000)
                segment_count += 1
            except Exception as write_e:
                print(f"    Error saving segment {i} to '{output_filename}': {write_e}")

        print(f"Finished processing voice: {voice}. Saved {segment_count} segments.")

    except Exception as e:
        print(f"Error generating or processing audio for voice '{voice}': {e}")
        # Continue with the next voice even if one fails

print("\n--- All specified voices processed ---")