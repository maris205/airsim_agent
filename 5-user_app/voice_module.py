#coding=utf-8

'''
requires Python 3.6 or later
pip install requests
语音合成
'''
import base64
import json
import uuid
import requests

# 填写平台申请的appid, access_token以及cluster
def read_config():
    try:
        with open('token.json', 'r') as file:
            config = json.load(file)
            return config.get('appid'), config.get('token')
    except FileNotFoundError:
        print("配置文件 ' token.json' 未找到，请检查。")
        exit(1)
    except json.JSONDecodeError:
        print("配置文件 ' token.json' 格式错误，请检查。")
        exit(1)


def text2mp3(text):
    appid, access_token = read_config()
    cluster = "volcano_tts"
    # voice_type = "BV700_streaming"
    voice_type = "BV700_V2_streaming"
    host = "openspeech.bytedance.com"


    header = {"Authorization": f"Bearer;{access_token}"}
    # 从本地文件log.txt 中读取文字


    request_json = {
        "app": {
            "appid": appid,
            "token": "access_token",
            "cluster": cluster
        },
        "user": {
            "uid": "388808087185088"
        },
        "audio": {
            "voice_type": voice_type,
            "encoding": "mp3",
            "speed_ratio": 1.0,
            "volume_ratio": 1.0,
            "pitch_ratio": 1.0,
        },
        "request": {
            "reqid": str(uuid.uuid4()),
            "text": text,
            "text_type": "plain",
            "operation": "query",
            "with_frontend": 1,
            "frontend_type": "unitTson"

        }
    }
    return request_json, header


def  process_text2mp3(text):
    request_json, header = text2mp3(text)
    api_url = f"https://openspeech.bytedance.com/api/v1/tts"
    try:
        resp = requests.post(api_url, json.dumps(request_json), headers=header)
        print(f"resp body: \n{resp.json()}")
        if "data" in resp.json():
            data = resp.json()["data"]
            file_to_save = open("readlog_txt.mp3", "wb")
            file_to_save.write(base64.b64decode(data))
    except Exception as e:
        e.with_traceback()


if __name__ == '__main__':
    text = "你好，我是小爱同学。"
    process_text2mp3(text)
