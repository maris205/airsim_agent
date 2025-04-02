# -*- coding: utf-8 -*-
# @Time    : 2025/2/25  14:27
# @Author  : mariswang@rflysim
# @File    : tos_appbk.py
# @Software: PyCharm
# @Describe: 
# -*- encoding:utf-8 -*-

import json
import os
import sys
import time
import json
import requests
import tos

#pip install tos

# 从环境变量获取 AK 和 SK 信息。
ak = ''
sk = ''
# your endpoint 和 your region 填写Bucket 所在区域对应的Endpoint。# 以华北2(北京)为例，your endpoint 填写 tos-cn-beijing.volces.com，your region 填写 cn-beijing。
endpoint = "tos-cn-shanghai.volces.com" #注意ivolces是内网的，volces是外网的
region = "cn-shanghai"
bucket = "yt-" #桶名
# 创建TosClientV2对象
client = tos.TosClientV2(ak, sk, endpoint, region)
base_url = "https://yt-shanghai.tos-cn-shanghai.volces.com/"
#base_url = "tos://{}/".format(bucket)


"""
功能:上传本地文件
输入:local_file, 本地文件名
输入:path, oss上的路径,需要预先设置
输入:filename, oss上文件命名
返回:上的url
"""
def upload_file(local_file, path, filename):
    # 读取文件
    f = open(local_file, 'rb')
    text = f.read()

    # 调用接口请求TOS服务，例如上传对象
    resp = client.put_object(bucket, path + "/" + filename, content=text)

    f.close()
    tos_url = base_url + path + "/" + filename
    return tos_url


"""
功能:上传本地文件到腾讯oss
输入:file_bin, 二进制文件流
输入:path, oss上的路径,需要预先设置
输入:filename, oss上文件命名
返回:腾讯云上的url
"""

def upload_stream(file_bin, path, filename):
    bucket.put_object(path + "/" + filename, file_bin)
    oss_url = base_url + path + "/" + filename
    return oss_url

"""
功能:上传url图片
输入:url, 图片或者文件url,需要下载并上传的
输入:path, oss上的路径,需要预先设置
输入:filename, oss上文件命名
返回:云上的url
"""
def upload_url(url, path, filename):
    # 下载文件
    header = {}
    header["User-Agent"] = "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_14_6) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.114 Safari/537.36"
    res = requests.get(url, headers=header)

    # 如果仍是错误,返回错误
    if 200 != res.status_code:
        print("ERROR:file download error")
        return -1

    # 填写Object完整路径。Object完整路径中不能包含Bucket名称。
    bucket.put_object(path + "/" + filename, res)
    oss_url = base_url + path + "/" + filename
    return oss_url


if __name__ == "__main__":
    local_file = "comand_1.mp3"
    path = "mp3语音"
    filename = "comand_1.mp3"
    tos_url = upload_file(local_file, path, filename)
    print(tos_url)


    # filename = "ebfd506812f0e990a001de9eee984a5c.jpg"
    # url = "http://kkyx-1300721637.cos.ap-beijing.myqcloud.com/user_search/ebfd506812f0e990a001de9eee984a5c.jpg"
    # path = "store"
    # oss_url = upload_url(url, path, filename)
    # print(oss_url)
    # print(upload(url, path, filename))
    # text = "hello world"
    # filename = "1.html"
    # path = "20230311"
    # url = upload_stream(text, path, filename)
    # print(url)
