from kokoro import KPipeline
from IPython.display import display, Audio
import soundfile as sf
import torch

pipeline = KPipeline(lang_code="z")
text = '''
朝中社当天援引朝鲜劳动党中央军事委员会27日一份书面声明说，按照朝鲜国家元首命令，朝鲜武装力量部队参加库尔斯克地区收复作战，为收复俄罗斯联邦领土作出重大贡献，“完全符合联合国宪章等国际法和朝俄《全面战略伙伴关系条约》的各条款精神”。

声明说，库尔斯克地区收复作战胜利结束，“是彰显朝俄两国牢固战斗友谊和两国人民的同盟关系、兄弟关系（所达到）的最高层次战略高度的历史新篇章”。

声明就朝方当时决策赴俄参战一事说，朝鲜劳动党总书记、国务委员长金正恩分析并判定当时战况符合行使朝俄《全面战略伙伴关系条约》相关条款，据此决定朝鲜武装力量参战并通报了俄方。

俄罗斯外交部发言人扎哈罗娃26日表示，朝鲜士兵在库尔斯克州作战，为俄军收复该州作出重大贡献。

26日早些时候，俄武装力量总参谋长格拉西莫夫在向俄总统普京汇报已收复库尔斯克州时说，朝鲜军人根据俄朝《全面战略伙伴关系条约》，为击溃库尔斯克州乌军提供了巨大帮助。(新华社
'''
generator = pipeline(text, voice='zf_xiaoni')
for i, (gs, ps, audio) in enumerate(generator):
    print(i, gs, ps)
    display(Audio(data=audio, rate=24000, autoplay=i==0))
    sf.write(f'{i}.wav', audio, 24000)