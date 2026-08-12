WRO 2026 3-class detector - green / red / magenta
Trained 2026-08-12 (yolo26n @ 320, Colab T4). val mAP50-95: green .859 / red .792 / magenta .820

Run:
  pip install onnxruntime opencv-python numpy
  python detect_3class.py photo.jpg          (saves photo_det.jpg next to it)
  python detect_3class.py cam                (webcam live, q quits)

Hard rules:
- Class order is LOCKED: 0=green 1=red 2=magenta. Retraining or re-exporting with
  any other order inverts the robot's steering decisions.
- Input is FIXED 320x320 (the script letterboxes for you). Do not re-export dynamic.
- NOT cleared for the robot: it goes on the Pi only after benchncnn latency AND the
  concurrent-load test pass - the previous model ran 30 fps alone and 0.3 fps in-system.

Files: best.onnx (fixed-320, end2end/NMS-free), detect_3class.py, this README.
