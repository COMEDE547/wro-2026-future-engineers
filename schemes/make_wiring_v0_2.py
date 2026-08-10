# schemes/make_wiring_v0_2.py
# Generates wiring-v0.2.png / .pdf - signal + power, single page.
# v0.1 (2026-08-05) drew signal only and deliberately left the power tree
# undrawn "pending battery selection". The pack was read off the hardware on
# 2026-08-06 (3S 11.1 V 2600 mAh, DC-jack) and the dual-rail buck was
# identified from the top v-photo on 2026-08-10, so v0.2 draws the tree and
# labels every remaining unknown as TBD instead of guessing.
# Run:  py -3 schemes/make_wiring_v0_2.py   (from repo root; needs matplotlib)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

W, H = 180, 122
fig, ax = plt.subplots(figsize=(18, 12.2))
ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis("off")

C_DARK = "#15152e"; C_BLUE = "#173a5e"; C_RED = "#701b1b"; C_PWR = "#5b3a14"
W_I2C = "#0e7c5b"; W_SIG = "#c06818"; W_USB = "#7040a8"; W_PWR = "#c62828"
TBD = "#ff6b6b"; MONO = "DejaVu Sans Mono"

def box(x, y, w, h, fc, title, lines, tsz=11, lsz=8.6):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.6",
                 fc=fc, ec="none", zorder=2))
    ax.text(x + 1.6, y + h - 2.6, title, color="white", fontsize=tsz,
            fontweight="bold", family=MONO, va="top", zorder=3)
    yy = y + h - 6.4
    for ln, col in lines:
        ax.text(x + 1.6, yy, ln, color=col, fontsize=lsz, family=MONO,
                va="top", zorder=3)
        yy -= 3.4

def wire(pts, color, lw=2.0, ls="-", z=1):
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    ax.plot(xs, ys, color=color, lw=lw, ls=ls, zorder=z,
            solid_capstyle="round")

def lab(x, y, s, color, sz=8.2, ha="left", bold=False):
    ax.text(x, y, s, color=color, fontsize=sz, family=MONO, ha=ha,
            fontweight="bold" if bold else "normal", zorder=4,
            bbox=dict(fc="white", ec="none", pad=0.5))

WH = "white"; GY = "#c9c9d6"

# ---------- title ----------
ax.text(3, 119.5, "WRO Future Engineers 2026 - signal + power wiring v0.2  -  2026-08-10",
        fontsize=15, fontweight="bold", va="top")

# ---------- signal-layer boxes ----------
box(4, 96, 33, 16, C_DARK, "Raspberry Pi 5",
    [("pillar detection", GY), ("pass-side decision", GY),
     ("PWR 5 V/5 A - SOURCE TBD", TBD)])
box(44, 100, 31, 12, C_DARK, "USB UVC webcam",
    [("MJPEG 640x480 @30", GY), ("model under verification", TBD)])
box(4, 50, 33, 40, C_DARK, "ESP32 DevKit-V1",
    [("VIN 5V         GND", GY), ("GPIO21  I2C SDA", GY),
     ("GPIO22  I2C SCL", GY), ("GPIO13  servo sig", GY),
     ("GPIO25  TB AIN1", GY), ("GPIO26  TB AIN2", GY),
     ("GPIO33  TB PWMA", GY), ("GPIO27  TB STBY", GY),
     ("GPIO0   BOOT = R2 start", GY), ("UART0   USB serial", GY)])
box(4, 30, 32, 14, C_DARK, "Steering servo MG90S",
    [("sig <- GPIO13", GY), ("V+ via ESP32 5V chain", TBD),
     ("GND common", GY)])
box(46, 62, 36, 26, C_BLUE, "PCA9548A mux @0x70",
    [("SDA/SCL <- ESP32", GY), ("CH0  TF-Luna L", GY),
     ("CH1  TF-Luna C", GY), ("CH2  TF-Luna R", GY),
     ("CH4  BNO055", GY)])
box(46, 23, 38, 27, C_RED, "TB6612FNG  ch A",
    [("PWMA<-33 AIN1<-25 AIN2<-26", GY), ("STBY <- GPIO27 (driven HIGH)", GY),
     ("VCC <- 3V3      GND", GY), ("VM  <- pack 11.1 V", "#ffb0a0"),
     ("limit 1.2 A cont / 3.2 A pk", GY),
     ("PWM: R1 20kHz/10b - R2 8b ~1kHz", GY)])
box(90, 28, 27, 14, C_RED, "N20 gearmotor",
    [("600 RPM class", GY), ("rated V: TBD", TBD),
     ("stall 0.75A @6V ds", GY)])
box(128, 100, 48, 12, C_BLUE, "TF-Luna LEFT   @0x10",
    [("I2C ranging - side", GY), ("5 V supply / 3V3 logic", GY)])
box(128, 85, 48, 12, C_BLUE, "TF-Luna CENTRE @0x10",
    [("I2C ranging - front", GY), ("5 V supply / 3V3 logic", GY)])
box(128, 70, 48, 12, C_BLUE, "TF-Luna RIGHT  @0x10",
    [("I2C ranging - side", GY), ("5 V supply / 3V3 logic", GY)])
box(128, 52, 48, 14, C_BLUE, "BNO055 IMU @0x28",
    [("absolute yaw (Euler)", GY), ("mux CH4 - 3V3 supply", GY),
     ("IMUPLUS mode (R2)", GY)])

# ---------- power-layer boxes ----------
box(4, 4, 30, 16, C_PWR, "3S Li-Po pack",
    [("11.1 V nom - 2600 mAh", GY), ("Tamiya-style plug", GY),
     ("no switch, no fuse", TBD)])
box(44, 4, 46, 16, C_PWR, "Dual-rail buck (photo-ID)",
    [("barrel-jack IN <- pack", GY),
     ("XL4015/LM2596-class - chip TBD", TBD),
     ("OUT1 5V screw   OUT2 USB-A 5V", GY)])

# ---------- notes ----------
box(128, 2, 48, 46, "#26262e", "NOTES", [
    ("MISSING (rule 9.10): main", TBD),
    ("power switch - MANDATORY.", TBD),
    ("Fuse absent; rating follows", GY),
    ("from stall measurement.", GY),
    ("GND: one common net incl.", GY),
    ("Pi via USB shield.", GY),
    ("R2 dual 5V into ESP32:", GY),
    ("buck VIN + Pi USB VBUS -", GY),
    ("verify DevKit diode-OR.", GY),
    ("TBD: buck rating - N20", TBD),
    ("rated V - Pi supply path -", TBD),
    ("camera model - Luna 5V tap.", TBD)], lsz=8.2)

# ---------- signal wires ----------
wire([(37, 78), (42, 78), (42, 76), (46, 76)], W_I2C, 2.2)
lab(38.5, 79.2, "SDA21/SCL22 @400k", W_I2C, 7.6)
wire([(82, 80), (100, 80), (100, 106), (128, 106)], W_I2C, 2.0)
lab(102, 107.0, "CH0", W_I2C, 7.6)
wire([(82, 77), (97, 77), (97, 91), (128, 91)], W_I2C, 2.0)
lab(102, 92.0, "CH1", W_I2C, 7.6)
wire([(82, 74), (94, 74), (94, 76), (128, 76)], W_I2C, 2.0)
lab(102, 77.0, "CH2", W_I2C, 7.6)
wire([(82, 68), (100, 68), (100, 59), (128, 59)], W_I2C, 2.0)
lab(102, 60.0, "CH4", W_I2C, 7.6)

wire([(37, 71.5), (41, 71.5), (41, 47), (20, 47), (20, 44)], W_SIG, 2.0)
lab(6, 45.6, "GPIO13 - 50 Hz - 500-2400 us", W_SIG, 7.2)
wire([(37, 64.5), (43, 64.5), (43, 44), (46, 44)], W_SIG, 2.0)
wire([(37, 61), (42.2, 61), (42.2, 41), (46, 41)], W_SIG, 2.0)
wire([(37, 57.5), (41.4, 57.5), (41.4, 38), (46, 38)], W_SIG, 2.0)
wire([(37, 54), (40.6, 54), (40.6, 35), (46, 35)], W_SIG, 2.0)
wire([(84, 36), (90, 36)], "#222222", 3.0)
lab(84.5, 37.6, "AO1/AO2", "#222222", 7.4)

# ---------- data links ----------
wire([(20, 96), (20, 90)], W_USB, 2.2, ls="--")
lab(21.5, 93.0, "USB /dev/ttyUSB0 115200 - RED/GREEN/CLEAR/REVERSE/POS,cx,h - 1.5 s dead-man", W_USB, 7.2)
wire([(37, 105), (44, 105)], W_USB, 2.2, ls="--")
lab(37.8, 106.6, "USB", W_USB, 7.4)

# ---------- power wires ----------
wire([(34, 12), (44, 12)], W_PWR, 3.2)
lab(34.6, 13.6, "Tamiya->barrel", W_PWR, 7.2)
wire([(50, 20), (40, 20), (40, 49.2), (12, 49.2), (12, 50)], W_PWR, 3.2)
lab(14, 47.6, "(buck OUT1) 5 V -> VIN", W_PWR, 7.2)
wire([(4, 55), (2, 55), (2, 40), (4, 40)], W_PWR, 2.2)
lab(0.6, 26.8, "servo V+ off the ESP32 5 V chain - named brownout risk; dedicated rail pending", TBD, 7.2)
wire([(26, 20), (26, 21.3), (65, 21.3), (65, 23)], W_PWR, 3.2)
lab(35.0, 18.2, "11.1 V -> VM", W_PWR, 7.4)
wire([(86, 20), (86, 115), (30, 115), (30, 112)], W_PWR, 2.2, ls="--")
lab(88, 100.0, "OUT2 USB-A 5V => Pi 5", W_PWR, 7.6, bold=True)
lab(88, 97.2, "CANDIDATE - TBD - no PD =>", TBD, 7.2)
lab(88, 94.4, "600 mA USB-periph cap risk", TBD, 7.2)

# ---------- legend ----------
ax.text(4, 1.6, "solid = signal   dashed = data link   thick red = power   dashed red = power TBD",
        fontsize=8.6, family=MONO, color="#333344")

fig.savefig("schemes/wiring-v0.2.png", dpi=210, bbox_inches="tight",
            facecolor="white")
fig.savefig("schemes/wiring-v0.2.pdf", bbox_inches="tight", facecolor="white")
print("wrote schemes/wiring-v0.2.png + .pdf")
