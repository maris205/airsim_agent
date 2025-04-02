# import os
# from bs4 import BeautifulSoup

# def process_html_file(file_path):
#     with open(file_path, 'r', encoding='utf-8') as file:
#         soup = BeautifulSoup(file, 'html.parser')

#         for img in soup.find_all('img'):
#             # 跳过项目 Logo
#             if 'feisi_logo.png' in img['src']:
#                 continue
#             img['class'] = img.get('class', []) + ['thumbnail']
#             img['width'] = "100"
#             img['height'] = "auto"

#             # 包裹 img 标签，添加 lightbox 链接
#             link = soup.new_tag('a', href=img['src'], data_lightbox='image')
#             img.replace_with(link)
#             link.append(img)

#     with open(file_path, 'w', encoding='utf-8') as file:
#         file.write(str(soup))

# def process_directory(directory):
#     for root, _, files in os.walk(directory):
#         for file in files:
#             if file.endswith('.html'):
#                 process_html_file(os.path.join(root, file))

# # 设定Doxygen生成的HTML文件所在目录
# html_directory = 'html'
# process_directory(html_directory)
import os
import chardet
from bs4 import BeautifulSoup

def process_html_file(file_path):
    # 自动检测文件编码
    with open(file_path, 'rb') as file:
        raw_data = file.read()
        result = chardet.detect(raw_data)
        encoding = result['encoding']

    # 以检测到的编码打开文件
    with open(file_path, 'r', encoding=encoding, errors='ignore') as file:
        soup = BeautifulSoup(file, 'html.parser')

        for img in soup.find_all('img'):
            # 跳过项目 Logo 和 SVG 文件
            src = img.get('src', '')
            if 'feisi_logo.png' in src or src.endswith('.svg') or 'open.png' in src or 'closed.png' in src:
                continue

            # 添加缩略图类并调整大小
            img['class'] = img.get('class', []) + ['thumbnail']
            img['width'] = "100"
            img['height'] = "auto"

            # 包裹 img 标签，添加 lightbox 链接
            link = soup.new_tag('a', href=img['src'], data_lightbox='image')
            img.replace_with(link)
            link.append(img)

    with open(file_path, 'w', encoding=encoding, errors='ignore') as file:
        file.write(str(soup))

def process_directory(directory):
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith('.html'):
                process_html_file(os.path.join(root, file))

# 设定Doxygen生成的HTML文件所在目录
html_directory = 'html'
process_directory(html_directory)
