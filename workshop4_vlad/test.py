import socket
import time

if __name__ == "__main__":
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.bind(('0.0.0.0', 9998))
        server.listen(1)
        print("[OCR Server] Waiting for connection...")

        conn, addr = server.accept()
        time.sleep(1)  # Give the client time to send data
        with conn:
            print(f"[OCR Server] Connected from {addr}")
            print(f"Socket family: {conn.family}, type: {conn.type}")
            try:
                data = conn.recv(1024)
                print(f"Received: {data}")
                print(f"Peer: {conn.getpeername()}")
            except OSError as e:
                print(f"Error receiving data: {e}")
