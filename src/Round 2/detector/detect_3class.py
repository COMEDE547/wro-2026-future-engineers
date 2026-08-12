# WRO 2026 3-class detector (green/red/magenta) - standalone runner
# Usage:  python detect_3class.py image1.jpg image2.jpg ...
#         python detect_3class.py cam          (webcam, press q to quit)
# Needs:  pip install onnxruntime opencv-python numpy
import sys, os
import cv2, numpy as np, onnxruntime as ort
NAMES  = {0: "green", 1: "red", 2: "magenta"}   # CLASS ORDER LOCKED - never reorder
COLORS = {0: (80,200,80), 1: (60,60,230), 2: (200,60,200)}
IMG, CONF = 320, 0.40
HERE = os.path.dirname(os.path.abspath(__file__))
sess = ort.InferenceSession(os.path.join(HERE, "best.onnx"),
                            providers=["CPUExecutionProvider"])
INP = sess.get_inputs()[0].name
def letterbox(im, s=IMG):
    h, w = im.shape[:2]; r = min(s/h, s/w)
    nh, nw = int(round(h*r)), int(round(w*r))
    top, left = (s-nh)//2, (s-nw)//2
    out = np.full((s, s, 3), 114, np.uint8)
    out[top:top+nh, left:left+nw] = cv2.resize(im, (nw, nh))
    return out, r, left, top
def infer(im):
    lb, r, dx, dy = letterbox(im)
    x = lb[:, :, ::-1].transpose(2,0,1)[None].astype(np.float32) / 255.0
    out = np.squeeze(sess.run(None, {INP: x})[0])
    dets = []
    if out.ndim == 2 and out.shape[1] == 6:      # end2end: x1,y1,x2,y2,conf,cls
        for x1, y1, x2, y2, conf, cls in out:
            if conf < CONF: continue
            dets.append(((x1-dx)/r, (y1-dy)/r, (x2-dx)/r, (y2-dy)/r,
                         float(conf), int(cls)))
    else:
        print("UNEXPECTED output shape", out.shape, "- send this line to Ethan")
    return dets

def annotate(im, dets):
    for x1, y1, x2, y2, conf, cls in dets:
        c = COLORS.get(cls, (255, 255, 255))
        cv2.rectangle(im, (int(x1), int(y1)), (int(x2), int(y2)), c, 2)
        cv2.putText(im, f"{NAMES.get(cls, cls)} {conf:.2f}", (int(x1), max(14, int(y1) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
    return im

if __name__ == "__main__":
    args = sys.argv[1:]
    if not args:
        print("usage: python detect_3class.py img1.jpg [img2.jpg ...]   or:   python detect_3class.py cam")
        sys.exit(0)
    if args[0].lower() == "cam":
        cap = cv2.VideoCapture(0)
        while True:
            ok, frame = cap.read()
            if not ok: break
            cv2.imshow("3class - q quits", annotate(frame, infer(frame)))
            if cv2.waitKey(1) & 0xFF == ord('q'): break
        cap.release(); cv2.destroyAllWindows()
    else:
        for p in args:
            im = cv2.imread(p)
            if im is None:
                print("skip (unreadable):", p); continue
            dets = infer(im)
            out = os.path.splitext(p)[0] + "_det.jpg"
            cv2.imwrite(out, annotate(im, dets))
            names = ", ".join(f"{NAMES[int(c)]} {cf:.2f}" for *_, cf, c in dets) or "none"
            print(f"{os.path.basename(p)}: {len(dets)} detections ({names}) -> {os.path.basename(out)}")
