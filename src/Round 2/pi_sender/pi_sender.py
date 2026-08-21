#!/usr/bin/env python3
# WRO 2026 Obstacle round - Pi vision sender (RGB Native)
# Camera -> yolo26n v2 ONNX (green/red/magenta) -> pick actionable block ->
# stream "R"/"G"/"C" lines @115200 to the ESP32 running round2_ino.ino.

import argparse, glob, os, sys, threading, time
import numpy as np
import cv2
import onnxruntime as ort

NAMES  = {0: "green", 1: "red", 2: "magenta"}  # CLASS ORDER LOCKED - never reorder
COLORS = {0: (80, 200, 80), 1: (230, 60, 60), 2: (200, 60, 200)}   # RGB
HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------- detection ----------------

def letterbox(im, s=320):
    h, w = im.shape[:2]
    r = min(s / h, s / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    top, left = (s - nh) // 2, (s - nw) // 2
    out = np.full((s, s, 3), 114, np.uint8)
    out[top:top + nh, left:left + nw] = cv2.resize(im, (nw, nh))
    return out, r, left, top

def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0: return 0.0
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / ua if ua > 0 else 0.0

def nms_per_class(dets, thr=0.5):
    out = []
    for c in set(d[5] for d in dets):
        cl = sorted((d for d in dets if d[5] == c), key=lambda d: -d[4])
        keep = []
        for d in cl:
            if all(iou(d, k) < thr for k in keep):
                keep.append(d)
        out += keep
    return out

class Detector:
    def __init__(self, model_path, conf, threads=2):
        opts = ort.SessionOptions()
        if threads > 0:
            opts.intra_op_num_threads = threads
        self.sess = ort.InferenceSession(model_path, sess_options=opts, providers=["CPUExecutionProvider"])
        self.inp = self.sess.get_inputs()[0].name
        self.conf = conf

    def __call__(self, im_rgb):
        lb, r, dx, dy = letterbox(im_rgb)
        # Array is already RGB, so we bypass the [:, :, ::-1] BGR flip here
        x = lb.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        out = np.squeeze(self.sess.run(None, {self.inp: x})[0])
        dets = []
        for x1, y1, x2, y2, conf, cls in out:
            if conf < self.conf: continue
            dets.append(((x1-dx)/r, (y1-dy)/r, (x2-dx)/r, (y2-dy)/r,
                         float(conf), int(cls)))
        return nms_per_class(dets)

# ---------------- decision ----------------

def decide(dets, fw, fh, min_area_frac, roi_x_frac):
    lo = fw * (1.0 - roi_x_frac) / 2.0
    hi = fw - lo
    best, best_area = None, 0.0
    for d in dets:
        x1, y1, x2, y2, conf, cls = d
        if cls not in (0, 1): continue
        cx = (x1 + x2) / 2.0
        if not (lo <= cx <= hi): continue
        area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        if area < min_area_frac * fw * fh: continue
        if area > best_area:
            best, best_area = d, area
    if best is None: return 'C', None
    return ('R' if best[5] == 1 else 'G'), best

# ---------------- live view ----------------

def annotate(frame, dets, chosen, state, hud, roi_x_frac):
    fh, fw = frame.shape[:2]
    lo, hi = int(fw * (1 - roi_x_frac) / 2), int(fw - fw * (1 - roi_x_frac) / 2)
    cv2.line(frame, (lo, 0), (lo, fh), (120, 120, 120), 1)
    cv2.line(frame, (hi, 0), (hi, fh), (120, 120, 120), 1)
    for d in dets:
        x1, y1, x2, y2, conf, cls = d
        c = COLORS.get(cls, (255, 255, 255))
        thick = 4 if d is chosen else 2
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), c, thick)
        cv2.putText(frame, f"{NAMES.get(cls, cls)} {conf:.2f}",
                    (int(x1), max(14, int(y1) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
    cv2.putText(frame, state, (10, 46), cv2.FONT_HERSHEY_SIMPLEX,
                1.6, (255, 255, 255), 3)
    cv2.putText(frame, hud, (10, fh - 12), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 255), 1)
    return frame

class Streamer:
    def __init__(self, port):
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
        self.lock, self.jpg = threading.Lock(), None
        outer = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a): pass
            def do_GET(self):
                if self.path == "/":
                    body = (b"<html><body style='margin:0;background:#111'>"
                            b"<img src='/stream' style='width:100%'></body></html>")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path != "/stream":
                    self.send_response(404); self.end_headers(); return
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        with outer.lock: buf = outer.jpg
                        if buf is None:
                            time.sleep(0.05); continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         + f"Content-Length: {len(buf)}\r\n\r\n".encode()
                                         + buf + b"\r\n")
                        time.sleep(0.03)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.srv = ThreadingHTTPServer(("0.0.0.0", port), H)
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def update(self, bgr_frame):
        ok, buf = cv2.imencode(".jpg", bgr_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if ok:
            with self.lock: self.jpg = buf.tobytes()

def local_ip():
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80)); ip = s.getsockname()[0]
    except Exception:
        ip = "<pi-ip>"
    finally:
        s.close()
    return ip

# ---------------- telemetry ----------------

def cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return float("nan")

def throttled_flag():
    try:
        import subprocess
        r = subprocess.run(["vcgencmd", "get_throttled"],
                           capture_output=True, text=True, timeout=1)
        return r.stdout.strip().split("=")[-1]
    except Exception:
        return "n/a"

# ---------------- sources ----------------

def frames_picam(w, h, shutter_us):
    from picamera2 import Picamera2
    cam = Picamera2()
    cfg = cam.create_video_configuration(main={"size": (w, h), "format": "RGB888"})
    cam.configure(cfg)
    if shutter_us > 0:
        cam.set_controls({"ExposureTime": int(shutter_us), "AeEnable": False})
    cam.start()
    time.sleep(0.5)
    while True:
        # Convert natively BGR numpy arrays explicitly to RGB at the source
        yield cv2.cvtColor(cam.capture_array(), cv2.COLOR_BGR2RGB)

def frames_video(path):
    cap = cv2.VideoCapture(int(path) if str(path).isdigit() else path)
    while True:
        ok, frame = cap.read()
        if not ok: break
        yield cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    cap.release()

def frames_images(folder):
    for p in sorted(glob.glob(os.path.join(folder, "*"))):
        im = cv2.imread(p)
        if im is not None:
            yield cv2.cvtColor(im, cv2.COLOR_BGR2RGB)

# ---------------- serial ----------------

def open_serial(port, baud):
    import serial
    if port == "auto":
        cands = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        if not cands and os.name == "nt":
            cands = [f"COM{i+1}" for i in range(256)]
        if not cands:
            sys.exit("no serial port found - is the ESP32 plugged in?")
        port = cands[0]
    ser = serial.Serial(port, baud, timeout=0, write_timeout=1)
    try:
        ser.dtr = False
    except Exception:
        pass
    print(f"[serial] {port} @ {baud}")
    return ser

# ---------------- main ----------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=os.path.join(HERE, "best.onnx"))
    ap.add_argument("--source", default="picam", help="picam | video file | image folder | webcam index")
    ap.add_argument("--port", default="auto")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--conf", type=float, default=0.40)
    ap.add_argument("--min-area", type=float, default=0.002)
    ap.add_argument("--roi", type=float, default=0.80)
    ap.add_argument("--confirm", type=int, default=2)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--shutter", type=int, default=0)
    ap.add_argument("--rot180", action="store_true")
    ap.add_argument("--stream", action="store_true", help="serve web view at http://<pi-ip>:PORT")
    ap.add_argument("--stream-port", type=int, default=8080)
    ap.add_argument("--no-show", action="store_true", help="disable local display window")
    ap.add_argument("--threads", type=int, default=2, help="CPU threads for ONNX (prevents brownouts)")
    ap.add_argument("--no-serial", action="store_true")
    ap.add_argument("--echo", action="store_true")
    a = ap.parse_args()

    det = Detector(a.model, a.conf, threads=a.threads)
    print(f"[model] {a.model}  conf>={a.conf}")

    ser = None if a.no_serial else open_serial(a.port, a.baud)
    streamer = Streamer(a.stream_port) if a.stream else None
    if streamer:
        print(f"[stream] live view: http://{local_ip()}:{a.stream_port}")

    if a.source == "picam":
        src = frames_picam(a.width, a.height, a.shutter)
    elif os.path.isdir(a.source):
        src = frames_images(a.source)
    else:
        src = frames_video(a.source)

    show_window = not a.no_show
    if show_window:
        cv2.namedWindow("PISENDER - Local Window", cv2.WINDOW_NORMAL)

    state, cand, cand_n = 'C', 'C', 0
    n, t0, tlast = 0, time.time(), time.time()
    hud = "warming up..."
    try:
        for frame in src:  # 'frame' is now purely RGB
            if a.rot180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            fh, fw = frame.shape[:2]
            dets = det(frame)
            verdict, chosen = decide(dets, fw, fh, a.min_area, a.roi)

            if verdict == state:
                cand, cand_n = state, 0
            elif verdict == cand:
                cand_n += 1
                if cand_n >= a.confirm:
                    state, cand_n = verdict, 0
                    print(f"[state] -> {state}")
            else:
                cand, cand_n = verdict, 1

            if ser:
                ser.write((state + "\n").encode())
                if a.echo:
                    tx = ser.read(4096)
                    if tx: sys.stdout.write(tx.decode(errors="replace"))
                else:
                    ser.read(4096)

            n += 1
            now = time.time()
            if now - tlast >= 1.0:
                fps = n / (now - t0)
                cnt = {"green": 0, "red": 0, "magenta": 0}
                for d in dets: cnt[NAMES[d[5]]] += 1
                hud = (f"fps={fps:.1f} temp={cpu_temp():.1f}C "
                       f"throttle={throttled_flag()} conf>={a.conf}")
                print(f"[tele] fps={fps:5.1f} state={state} g={cnt['green']} "
                      f"r={cnt['red']} m={cnt['magenta']} {hud.split(' ',1)[1]}")
                tlast = now

            if streamer or show_window:
                # Annotate directly on the RGB frame
                annotate(frame, dets, chosen, state, hud, a.roi)
                
                # Convert back to BGR purely for OpenCV's display/encode requirements
                display_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                
                if streamer: 
                    streamer.update(display_frame)
                if show_window:
                    cv2.imshow("PISENDER - Local Window", display_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'): 
                        break
    except KeyboardInterrupt:
        pass
    finally:
        if ser:
            for _ in range(3):
                try: ser.write(b"C\n")
                except Exception: break
            ser.close()
        if show_window: 
            cv2.destroyAllWindows()
        print("\n[exit] sent CLEAR, closed.")

if __name__ == "__main__":
    main()
