# 示例：测试API连通性
import requests
response = requests.get("https://api.deepdataspace.com/status", headers={"Authorization": "929cfa1acbe0ef4748bcbb1c4b29703d"})
print(response.status_code)  # 200表示正常