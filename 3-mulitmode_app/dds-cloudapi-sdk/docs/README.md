# How to build docs

```shell
pip install -r requirements-dev.txt
cd docs
make html
rsync -azP build/html/* $remote_host:$remote_path 
```
