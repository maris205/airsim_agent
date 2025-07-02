import os

from setuptools import setup

description = "The SDK for calling deepdataspace cloud API."
with open("README.md", "r", encoding="utf8") as fp:
    long_description = fp.read()

url = "https://github.com/deepdataspace/dds-cloudapi-sdk"
author = "cvr@idea"

install_requires = [
    "numpy>=1.24.4",
    "pillow>=10.2.0",
    "requests>=2.31.0",
    "sentry_sdk>=2.28.0",
    "opencv-python",
]

classifiers = [
    "Development Status :: 4 - Beta",
    "Topic :: Scientific/Engineering :: Artificial Intelligence",
    "Topic :: Scientific/Engineering :: Image Recognition",
    "Topic :: Scientific/Engineering :: Visualization",
    "Operating System :: MacOS :: MacOS X",
    "Operating System :: POSIX :: Linux",
    "Operating System :: Microsoft :: Windows",
    "Programming Language :: Python :: 3",
    "Programming Language :: Python :: 3.8",
    "Programming Language :: Python :: 3.9",
    "Programming Language :: Python :: 3.10",
]


def find_packages(pkg_dir: str):
    found = []

    for top, dirs, files in os.walk(pkg_dir):
        has_init = False
        has_python = False
        for file in files:
            if file == "__init__.py":
                has_init = True
            if file.endswith(".py"):
                has_python = True

        if has_init and has_python:
            found.append(top)

    return found


setup(name="dds-cloudapi-sdk",
      version="0.5.2",
      description=description,
      long_description=long_description,
      long_description_content_type="text/markdown",
      url=url,
      author=author,
      packages=find_packages("dds_cloudapi_sdk"),
      include_package_data=True,
      install_requires=install_requires,
      classifiers=classifiers,
      )
