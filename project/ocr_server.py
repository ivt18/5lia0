#!/usr/bin/env python3
import socket
import struct
import threading
import cv2
import numpy as np
import easyocr
from queue import Queue, Full

OCR_QUEUE_MAX = 10
NUM_WORKERS = 1

def recvall(conn, length):
    data = b''
    while len(data) < length:
        chunk = conn.recv(length - len(data))
        if not chunk:
            raise EOFError("Socket closed early")
        data += chunk
    return data

class OcrServer:
    def __init__(self, host='', port=9999):
        self.a = 0
        self.b = 0
        self.reader = easyocr.Reader(['en'], gpu=True)
        self.sock = socket.socket()
        self.sock.bind((host, port))
        self.sock.listen(1)
        print(f"[OCR] Listening on {host or '0.0.0.0'}:{port}")

        self.conn, addr = self.sock.accept()
        print(f"[OCR] Client connected from {addr}")

        # Bounded queue to smooth out bursts
        self.queue = Queue(maxsize=OCR_QUEUE_MAX)
        self.ocr_queue = Queue(maxsize=OCR_QUEUE_MAX)

        # Thread: receive & enqueue
        t_recv = threading.Thread(target=self.recv_loop, daemon=True)
        t_recv.start()

        # Worker threads: dequeue → OCR → send
        for _ in range(NUM_WORKERS):
            t = threading.Thread(target=self.ocr_worker, daemon=True)
            t.start()

        # Keep main alive
        # t_recv.join()

    def recv_loop(self):
        while True:
            try:
                raw_len = recvall(self.conn, 4)
                img_len = struct.unpack('>I', raw_len)[0]
                img_data = recvall(self.conn, img_len)
                img = cv2.imdecode(np.frombuffer(img_data, np.uint8), cv2.IMREAD_COLOR)
                
                if self.a == 0:
                    self.a = 1
                    print(f"image data: {img}")
                
                if img is None:
                    print("[OCR Server] Failed to decode image")
                    continue

                try:
                    self.queue.put(img, block=True, timeout=1)
                except Full:
                    print("[OCR] Queue full — dropping frame")
            except Exception as e:
                print(f"[OCR] recv_loop error: {e}")
                break

    def ocr_worker(self):
        while True:
            img = self.queue.get()
            if self.b == 0:
                self.b = 1
                print(f"image data from q: {img}")

            try:
                results = self.reader.readtext(img)
                # print(f"[OCR] Found {results} text regions")

                for bbox, text, conf in results:
                    print(f"[OCR] {text} ({conf:.2f})")
                    encoded_text = text.encode('utf-8')
                    text_len = struct.pack('>I', len(encoded_text))
                    self.conn.sendall(text_len + encoded_text)

                    if conf > 0.5:
                        # print(f"[OCR {round(conf,2)}] {text}")
                        top_left = tuple(map(int, bbox[0]))
                        bottom_right = tuple(map(int, bbox[2]))

                        cv2.rectangle(img, top_left, bottom_right, (0, 255, 0), 2)
                        cv2.putText(img, text, (top_left[0], top_left[1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                self.ocr_queue.put(img, block=False)

                # video.write(image)
                # cv2.imshow("Image", img)
                # cv2.waitKey(1)

            except Exception as e:
                print(f"[OCR] ocr_worker error: {e}")
            finally:
                self.queue.task_done()
    
    def run_ui(self):
        cv2.namedWindow("OCR Preview", cv2.WINDOW_NORMAL)
        while True:
            img = self.ocr_queue.get()
            cv2.imshow("OCR Preview", img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        cv2.destroyAllWindows()

if __name__ == '__main__':
    OcrServer(port=9999).run_ui()
