#!/usr/bin/env python3

import argparse
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2


latest_jpeg = None


class WebcamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global latest_jpeg

        if self.path == "/" or self.path == "/health":
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"Webcam streamer is running. Open /video\n")
            return

        if self.path != "/video":
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"Use /video\n")
            return

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        while True:
            if latest_jpeg is None:
                time.sleep(0.05)
                continue

            try:
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                self.wfile.write(latest_jpeg)
                self.wfile.write(b"\r\n")
                time.sleep(0.03)
            except Exception:
                break

    def log_message(self, format, *args):
        return


def main():
    global latest_jpeg

    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open laptop camera index {args.camera}")

    server = ThreadingHTTPServer((args.host, args.port), WebcamHandler)

    print("")
    print("Laptop webcam streamer is running.")
    print(f"Local test link: http://127.0.0.1:{args.port}/video")
    print(f"Docker stream URL: http://host.docker.internal:{args.port}/video")
    print("")
    print("Keep this terminal open.")
    print("Press CTRL+C to stop.")
    print("")

    import threading
    threading.Thread(target=server.serve_forever, daemon=True).start()

    frame_delay = 1.0 / max(args.fps, 1)

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Could not read frame from camera.")
                time.sleep(0.2)
                continue

            ok, jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                latest_jpeg = jpeg.tobytes()

            time.sleep(frame_delay)

    except KeyboardInterrupt:
        print("Stopping webcam streamer...")
    finally:
        cap.release()
        server.shutdown()


if __name__ == "__main__":
    main()
