import gradio as gr
import os
import tos_appbk
import airsim_agent
import recognition_module

# 确保目标文件夹存在
output_dir = "recordings"
os.makedirs(output_dir, exist_ok=True)


def save_audio(audio):
    my_agent = airsim_agent.AirSimAgent(knowledge_prompt="prompts/aisim_lession52.txt")
    # audio 是一个元组，包含 (sample_rate, audio_data)
    sample_rate, audio_data = audio

    # 生成唯一的文件名
    file_name = f"recording_{len(os.listdir(output_dir)) + 1}.wav"
    file_path = os.path.join(output_dir, file_name)

    # 使用 scipy 保存音频文件
    from scipy.io.wavfile import write
    write(file_path, sample_rate, audio_data)
    print(f"录音已保存: {file_path}")

#上传文件到OSS
    oss_url = tos_appbk.upload_file(file_path, "mp3语音", file_name)
    print(f"录音已保存: {file_path},链接为:{oss_url}")

    #识别语音
    text = recognition_module.process_mp3(oss_url)
    print(f"识别结果: {text}")
    command = text
    python_code = my_agent.process(command, True)  # 执行代码
    print("python_code: \n", python_code)
    #my_agent.process(text)
    return oss_url




# 创建 Gradio 界面
with gr.Blocks() as demo:
    gr.Markdown("## 录音并保存到本地")
    audio_input = gr.Audio(type="numpy", label="录音")  # 不再需要 source 参数
    output_text = gr.Textbox(label="保存状态")

    # 按钮绑定事件
    record_button = gr.Button("保存录音")
    record_button.click(save_audio, inputs=audio_input, outputs=output_text)

# 启动应用
demo.launch()