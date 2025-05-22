import socket
import cv2
import numpy as np
import easyocr
import struct

reader = easyocr.Reader(['en'], gpu=True)



# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# video = cv2.VideoWriter(
#     "./video_undistored.avi",
#     fourcc,
#     15.0,
#     (640, 480)
# )
        
def recvall(conn, length):
    data = b''
    while len(data) < length:
        more = conn.recv(length - len(data))
        if not more:
            raise EOFError("Socket closed before receiving expected data")
        data += more
    return data

if __name__ == "__main__":
    
    with socket.socket() as server:
        server.bind(('0.0.0.0', 9999))
        server.listen(1)
        print("[OCR Server] Waiting for connection...")
        conn, addr = server.accept()
        print(f"[OCR Server] Connected from {addr}")

        try:
            while True:

                raw_len = recvall(conn, 4)
                img_len = struct.unpack('>I', raw_len)[0]

                # Read image data
                img_data = recvall(conn, img_len)

                # Decode JPEG bytes
                np_arr = np.frombuffer(img_data, np.uint8)
                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if image is None:
                    print("[OCR Server] Failed to decode image")
                    continue

                # Run EasyOCR
                results = reader.readtext(image)
                for bbox, text, conf in results:
                    if conf > 0.5:
                        # print(f"[OCR {round(conf,2)}] {text}")
                        top_left = tuple(map(int, bbox[0]))
                        bottom_right = tuple(map(int, bbox[2]))

                        # Draw rectangle
                        cv2.rectangle(image, top_left, bottom_right, (0, 255, 0), 2)

                        # Put label
                        cv2.putText(image, text, (top_left[0], top_left[1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                # video.write(image)
                cv2.imshow("Image", image)
                cv2.waitKey(1)

        except Exception as e:
            print(f"[OCR Server] Error: {e}")
        finally:
            conn.close()
            server.close()
            cv2.destroyAllWindows()
