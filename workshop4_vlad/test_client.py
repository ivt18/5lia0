import socket
import time

s = socket.socket()
s.connect(("192.168.8.169", 9998))
time.sleep(3)  # ensure connection is kept alive
s.sendall(b"hello")
s.close()
