# serial_log.py -- timestamped capture of the ESP32's serial telemetry.
#
# Why this exists: no run has ever been logged. The firmware prints per-loop
# telemetry ([steer], [turn] n/12, L/C/R distances) and every open question we
# have -- true loop rate, telemetry cost, turn-counter behaviour, run-on
# timing -- falls out of ONE captured log with host timestamps.
#
# Usage (Windows laptop at the mat):
#   py -3 -m pip install pyserial          (once)
#   py -3 tools\serial_log.py COM5         (find the port in Device Manager)
#   py -3 tools\serial_log.py COM5 --tag round1-run1
#
# Output: logs/run-YYYYmmdd-HHMMSS[-tag].log
#   Each line: <ms since capture start, host clock>\t<line from ESP32>
#   Host timestamps are ~ms accurate -- enough to measure loop period from
#   deltas between consecutive per-loop prints.
# Ctrl+C stops cleanly and prints the file path + line count.

import argparse, datetime, pathlib, sys, time

def main():
    ap = argparse.ArgumentParser(description="Timestamped ESP32 serial logger")
    ap.add_argument("port", help="e.g. COM5 (Windows) or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--tag", default="", help="suffix for the log filename")
    args = ap.parse_args()

    try:
        import serial  # pyserial
    except ImportError:
        sys.exit("pyserial missing: py -3 -m pip install pyserial")

    logdir = pathlib.Path(__file__).resolve().parent.parent / "logs"
    logdir.mkdir(exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    name = f"run-{stamp}" + (f"-{args.tag}" if args.tag else "") + ".log"
    path = logdir / name

    ser = serial.Serial(args.port, args.baud, timeout=1)
    t0 = time.perf_counter()
    n = 0
    print(f"logging {args.port} @ {args.baud} -> {path}  (Ctrl+C to stop)")
    try:
        with open(path, "w", encoding="utf-8", errors="replace") as f:
            f.write(f"# port={args.port} baud={args.baud} started={stamp}\n")
            f.write("# ms_since_start\tline\n")
            while True:
                line = ser.readline()
                if not line:
                    continue
                ms = (time.perf_counter() - t0) * 1000.0
                text = line.decode("utf-8", errors="replace").rstrip("\r\n")
                f.write(f"{ms:10.1f}\t{text}\n")
                n += 1
                if n % 50 == 0:
                    f.flush()
                print(text)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    print(f"\nsaved {n} lines -> {path}")

if __name__ == "__main__":
    main()
