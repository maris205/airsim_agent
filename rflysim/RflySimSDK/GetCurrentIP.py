import socket
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('10.254.254.254', 1))
    ip = s.getsockname()[0]
except:
    ip=''
finally:
    s.close()
    
if ip == '':
    ip=socket.gethostbyname(socket.gethostname())
    
if ip == '127.0.1.1':
    ip=''
    
print(ip)