import gradio as gr
import os
from datetime import datetime
import numpy as np
from scipy.io.wavfile import write as write_wav

# 创建保存目录（带权限验证）
RECORDINGS_DIR = os.path.abspath("recordings")
os.makedirs(RECORDINGS_DIR, exist_ok=True)


def save_audio(audio_input):
    try:
        if audio_input is None:
            return "错误：未接收到音频数据"

        audio_data, sample_rate = audio_input
        if not isinstance(audio_data, np.ndarray) or sample_rate < 1000:
            return "错误：无效的音频输入格式"

        # 统一转换为int16格式
        audio_data = (audio_data * 32767).astype(np.int16) if audio_data.dtype == np.float32 else audio_data.astype(
            np.int16)

        # 生成带绝对路径的文件名
        filename = os.path.join(RECORDINGS_DIR, f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}.wav")
        write_wav(filename, sample_rate, audio_data)

        return f"音频已保存至：{os.path.normpath(filename)}"  # 规范化路径显示

    except Exception as e:
        return f"操作失败：{str(e)}"


# 创建接口（更新参数）
interface = gr.Interface(
    fn=save_audio,
    inputs=gr.Audio(sources=["microphone"], type="numpy"),
    outputs="text",
    title="音频录制器",
    description="点击麦克风开始录音，停止后自动保存",
    flagging_mode="never"  # 替换弃用的allow_flagging
)

# 启动配置（增加服务器设置）
interface.launch(
    server_name="0.0.0.0",
    server_port=7863,
    show_error=True,
    share=False  # 显式关闭公开链接
)