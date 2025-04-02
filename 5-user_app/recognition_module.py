import json
import time
import uuid
import requests
"""
火山引擎 语音识别实例
"""
# 读取配置文件
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

def submit_task(file_url):
    submit_url = "https://openspeech.bytedance.com/api/v3/auc/bigmodel/submit"
    task_id = str(uuid.uuid4())
    appid, token = read_config()
    headers = {
        "X-Api-App-Key": appid,
        "X-Api-Access-Key": token,
        "X-Api-Resource-Id": "volc.bigasr.auc",
        "X-Api-Request-Id": task_id,
        "X-Api-Sequence": "-1"
    }
    request = {
        "user": {
            "uid": "fake_uid"
        },
        "audio": {
            "url": file_url,
            "format": "mp3",
            "codec": "raw",
            "rate": 16000,
            "bits": 16,
            "channel": 1
        },
        "request": {
            "model_name": "bigmodel",
            # "enable_itn": True,
            # "enable_punc": True,
            # "enable_ddc": True,
            "show_utterances": True,
            # "enable_channel_split": True,
            # "vad_segment": True,
            # "enable_speaker_info": True,
            "corpus": {
                # "boosting_table_name": "test",
                "correct_table_name": "",
                "context": ""
            }
        }
    }
    print(f'Submit task id: {task_id}')
    response = requests.post(submit_url, data=json.dumps(request), headers=headers)
    if 'X-Api-Status-Code' in response.headers and response.headers["X-Api-Status-Code"] == "20000000":
        print(f'Submit task response header X-Api-Status-Code: {response.headers["X-Api-Status-Code"]}')
        print(f'Submit task response header X-Api-Message: {response.headers["X-Api-Message"]}')
        x_tt_logid = response.headers.get("X-Tt-Logid", "")
        print(f'Submit task response header X-Tt-Logid: {response.headers["X-Tt-Logid"]}\n')
        return task_id, x_tt_logid
    else:
        print(f'Submit task failed and the response headers are: {response.headers}')
        exit(1)
    return task_id


def query_task(task_id, x_tt_logid):
    query_url = "https://openspeech.bytedance.com/api/v3/auc/bigmodel/query"
    appid, token = read_config()
    headers = {
        "X-Api-App-Key": appid,
        "X-Api-Access-Key": token,
        "X-Api-Resource-Id": "volc.bigasr.auc",
        "X-Api-Request-Id": task_id,
        "X-Tt-Logid": x_tt_logid  # 固定传递 x-tt-logid
    }
    response = requests.post(query_url, json.dumps({}), headers=headers)
    if 'X-Api-Status-Code' in response.headers:
        print(f'Query task response header X-Api-Status-Code: {response.headers["X-Api-Status-Code"]}')
        print(f'Query task response header X-Api-Message: {response.headers["X-Api-Message"]}')
        print(f'Query task response header X-Tt-Logid: {response.headers["X-Tt-Logid"]}\n')
    else:
        print(f'Query task failed and the response headers are: {response.headers}')
        exit(1)
    return response


def process_mp3(file_url):

    task_id, x_tt_logid = submit_task(file_url)
    while True:
        query_response = query_task(task_id, x_tt_logid)
        code = query_response.headers.get('X-Api-Status-Code', "")
        if code == '20000000':  # task finished
            data = query_response.json()
            print(data)
            # 输出result 中的 result
            text = data["result"]["text"]
            print("识别到的文字",text)
            print("SUCCESS!")
            return text
        elif code != '20000001' and code != '20000002':  # task failed
            print("FAILED!")
            exit(1)
        time.sleep(1)

# 需要使用在线url，推荐使用TOS


if __name__ == '__main__':
    file_url = "https://yt-shanghai.tos-cn-shanghai.volces.com/mp3%E8%AF%AD%E9%9F%B3/%E7%81%AB%E5%B1%B1%E4%BA%91%E8%AF%AD%E9%9F%B3%E6%B5%8B%E8%AF%95.mp3"
    process_mp3(file_url)