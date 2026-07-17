#!/usr/bin/env python3
import cv2
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import time

# Глобальный буфер для кадра
frame_buffer = None
buffer_lock = threading.Lock()

class CameraThread(threading.Thread):
    def __init__(self, device_index=0, width=320, height=240):
        super().__init__()
        self.device_index = device_index
        self.width = width
        self.height = height
        self.daemon = True
        self.running = True

    def run(self):
        global frame_buffer
        cap = cv2.VideoCapture(self.device_index)
        if not cap.isOpened():
            print(f"Ошибка: Не удалось открыть камеру {self.device_index}")
            return

        # Установка разрешения
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        print(f"Поток камеры запущен. Разрешение: {self.width}x{self.height}")

        while self.running:
            ret, frame = cap.read()
            if ret:
                # На всякий случай делаем resize, если CAP_PROP не сработал
                if frame.shape[1] != self.width or frame.shape[0] != self.height:
                    frame = cv2.resize(frame, (self.width, self.height))
                
                # Кодируем в JPEG
                ret_encode, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ret_encode:
                    with buffer_lock:
                        frame_buffer = jpeg.tobytes()
            else:
                print("Ошибка захвата кадра")
                time.sleep(1)
            time.sleep(0.01)  # Небольшая пауза для разгрузки процессора
        cap.release()

class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            
            try:
                while True:
                    with buffer_lock:
                        current_frame = frame_buffer
                    
                    if current_frame is not None:
                        # Формируем multipart-ответ
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(current_frame)}\r\n'.encode('utf-8'))
                        self.wfile.write(b'\r\n')
                        self.wfile.write(current_frame)
                        self.wfile.write(b'\r\n')
                    else:
                        time.sleep(0.1)
                        continue
                    
                    time.sleep(0.04)  # ~25 FPS
            except (BrokenPipeError, ConnectionResetError):
                # Клиент отключился, выходим из цикла спокойно
                pass
            except Exception as e:
                print(f"Ошибка трансляции: {e}")
        else:
            self.send_error(404)

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Многопоточный HTTP сервер"""
    daemon_threads = True

def main():
    # Запускаем поток камеры
    cam_thread = CameraThread(device_index=0, width=320, height=240)
    cam_thread.start()
    
    # Ждем первого кадра
    print("Инициализация камеры...")
    timeout = 10
    start_time = time.time()
    while frame_buffer is None:
        if time.time() - start_time > timeout:
            print("Таймаут ожидания кадра. Проверьте подключение камеры.")
            break
        time.sleep(0.5)
    
    server_address = ('', 8080)
    httpd = ThreadedHTTPServer(server_address, StreamingHandler)
    print("Сервер запущен на порту 8080")
    print("Доступ по ссылке: http://<ip_робота>:8080/")
    
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nОстановка сервера...")
        cam_thread.running = False
        cam_thread.join()
        httpd.server_close()

if __name__ == '__main__':
    main()
