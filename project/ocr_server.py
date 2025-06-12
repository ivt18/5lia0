#!/usr/bin/env python3
import time
import socket
import struct
import threading
import cv2
import numpy as np
import easyocr
from queue import Queue, Full

OCR_QUEUE_MAX = 10
NUM_WORKERS = 3




class OcrServer:
    def __init__(self, host="", port=9999):
        self.a = 0
        self.b = 0
        self.reader = easyocr.Reader(["en"], gpu=True)
        self.sock = socket.socket()
        self.sock.bind((host, port))
        self.sock.listen(1)
        print(f"[OCR] Listening on {host or '0.0.0.0'}:{port}")

        self.conn, addr = self.sock.accept()
        print(f"[OCR] Client connected from {addr}")

        # Bounded queue to smooth out bursts
        self.queue: Queue[cv2.typing.MatLike] = Queue(maxsize=OCR_QUEUE_MAX)
        self.ocr_queue = Queue(maxsize=OCR_QUEUE_MAX)

        self.close_event = threading.Event()
        self.workers = []
        # Thread: receive & enqueue
        t_recv = threading.Thread(target=self.recv_loop, daemon=True)
        t_recv.start()
        self.workers.append(t_recv)

        # Worker threads: dequeue → OCR → send
        for _ in range(NUM_WORKERS):
            t = threading.Thread(target=self.ocr_worker)
            t.start()
            self.workers.append(t)


        # needs to run in the main thread
        self.run_ui()
        
        print("[OCR] Shutting down server")
        # for t in self.workers:
        #     t.join()
        t_recv.join()
        self.close_event.set()

        self.conn.close()
        self.sock.close()

    def recvall(self, length):
        data = b""
        while len(data) < length:
            chunk = self.conn.recv(length - len(data))
            if not chunk:
                self.close_event.set()
                raise EOFError("Socket closed early")
            data += chunk
        return data

    def recv_loop(self):
        while not self.close_event.is_set():
            try:
                raw_len = self.recvall(4)
                img_len = struct.unpack(">I", raw_len)[0]
                img_data = self.recvall(img_len)
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
                self.close_event.set()
                break

    def ocr_worker(self):
        while not self.close_event.is_set():
            img = self.queue.get()
            height, width, _ = img.shape

            # TODO: adjust based on testing, 2/3 seems too small
            crop_y_start = height // 5
            cropped_img = img[crop_y_start:, :]

            # add horizontal like where detection starts
            cv2.line(img, (0, crop_y_start), (width, crop_y_start), (0, 255, 0), 2)

            try:
                results = self.reader.readtext(cropped_img, allowlist="stopSTOP")
                # print(f"[OCR] Found {results} text regions")

                for bbox, text, conf in results:
                    if conf > 0.4:
                        print(f"[OCR] {text} ({conf:.2f})")
                        encoded_text = text.encode("utf-8")
                        text_len = struct.pack(">I", len(encoded_text))
                        self.conn.sendall(text_len + encoded_text)

                        # account for cropping
                        adjusted_bbox = [
                            (int(pt[0]), int(pt[1] + crop_y_start)) for pt in bbox
                        ]
                        top_left = adjusted_bbox[0]
                        bottom_right = adjusted_bbox[2]

                        cv2.rectangle(img, top_left, bottom_right, (0, 255, 0), 2)
                        cv2.putText(
                            img,
                            text,
                            (top_left[0], top_left[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (255, 0, 0),
                            2,
                        )

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
        while not self.close_event.is_set():
            img = self.ocr_queue.get()
            cv2.imshow("OCR Preview", img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
        cv2.destroyAllWindows()


if __name__ == "__main__":
    while True:
        print("Starting OCR server...")
        OcrServer(port=9999)
        print("OCR server stopped. Restarting in 5 seconds...")
        time.sleep(5)
