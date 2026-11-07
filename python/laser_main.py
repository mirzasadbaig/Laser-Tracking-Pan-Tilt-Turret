"""
Red-laser-follows-green-laser pan/tilt tracker (open-loop + accuracy measurement)
===================================================================================

Hardware:
    - ESP32 DevKit v1 running esp32_pca9685_bridge.ino, connected to this PC via USB
    - PCA9685 PWM driver (I2C, wired to the ESP32) -> 2x SG90 servos
      (pan on channel 0, tilt on channel 1)
    - Red laser diode mounted on the tilt platform, boresighted with the servo axes
    - EXTERNAL USB webcam pointed at the same general scene as the lasers
    - A second, independently-aimed green laser pointer (your "target")

The PC does all the computer vision and math; it just streams "pan,tilt\n"
angle commands to the ESP32 over serial, and the ESP32 turns those into PWM
on the PCA9685. Flash esp32_pca9685_bridge.ino onto the ESP32 first.

Dependencies (on the PC):
    pip install pyserial opencv-python numpy

Usage:
    # 1. find your external webcam's index
    python laser_main.py --list-cams

    # 2. calibrate (redo this whenever the camera, its position, or resolution changes)
    python laser_main.py --calibrate --port /dev/cu.usbserial-0001 --cam 0

    # 3. track + measure accuracy
    python laser_main.py --track --port /dev/cu.usbserial-0001 --cam 0
    python laser_main.py --track --port COM5 --cam 1 --width 1280 --height 720 --mm-per-px 0.8 --log run1.csv

Calibration controls:
    W,S,A,D  : nudge pan/tilt
    SPACE    : record current red-dot pixel position + current servo angles as a sample
    q        : finish calibration and save calibration2.json (needs >= 6 samples,
               spread across the frame -- corners + center + edges works well)

Tracking controls:
    r        : reset accuracy statistics
    q        : quit (prints an accuracy summary)
"""

import argparse
import csv
import json
import math
import sys
import time
from collections import deque

import cv2
import numpy as np
import serial

CALIB_FILE = "calibration.json"


# Servo driver (talks to the ESP32 over USB serial, which drives the PCA9685)

class ServoRig:
    """Sends pan/tilt angle commands to the ESP32 over serial, with clamping."""

    def __init__(self, port, baud=115200,
                 pan_range=(20, 160), tilt_range=(20, 160)):
        self.pan_range = pan_range
        self.tilt_range = tilt_range

        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # let the ESP32 finish its reset-on-connect before sending anything
        self.ser.reset_input_buffer()

        self.pan_angle = 60.0
        self.tilt_angle = 90.0
        self.set_angles(self.pan_angle, self.tilt_angle)

    def set_angles(self, pan, tilt):
        pan = float(np.clip(pan, *self.pan_range))
        tilt = float(np.clip(tilt, *self.tilt_range))
        self.pan_angle, self.tilt_angle = pan, tilt
        cmd = f"{pan:.1f},{tilt:.1f}\n"
        self.ser.write(cmd.encode("ascii"))

    def nudge(self, d_pan=0.0, d_tilt=0.0):
        self.set_angles(self.pan_angle + d_pan, self.tilt_angle + d_tilt)

    def close(self):
        self.ser.close()


# External webcam handling

def _backend():
    """Pick the native capture backend for this OS (more reliable for external cams)."""
    if sys.platform.startswith("win"):
        return cv2.CAP_DSHOW
    if sys.platform == "darwin":
        return cv2.CAP_AVFOUNDATION
    if sys.platform.startswith("linux"):
        return cv2.CAP_V4L2
    return cv2.CAP_ANY


def open_camera(index, width=None, height=None, fps=None,
                exposure=-6.0, auto_exposure=False):
    """Open an external webcam, apply settings, and return (cap, actual_w, actual_h).

    Exits with a helpful message if the camera can't be opened or gives no frames.
    """
    cap = cv2.VideoCapture(index, _backend())
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(index)  # fall back to OpenCV's default backend
    if not cap.isOpened():
        print(f"Could not open camera index {index}. Run with --list-cams to see what's available.")
        sys.exit(1)

    # MJPG lets most USB webcams do 720p/1080p at 30 fps (raw YUYV is bandwidth limited).
    # Must be set BEFORE width/height. Ignored on macOS/AVFoundation.
    if sys.platform != "darwin":
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fps:
        cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # keep latency low so the servo isn't chasing old frames

    if not auto_exposure:
        # Lower exposure helps isolate the laser dot. Values are driver-specific:
        #   Linux/V4L2 : exposure is usually in 1/10000 s units (try 50-300) once manual mode is on
        #   Windows    : log2 seconds (e.g. -6 -> ~1/64 s); some cams need -8 .. -4
        #   macOS      : most webcams ignore this entirely
        if sys.platform.startswith("linux"):
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)      # 1 = manual on V4L2
        elif sys.platform.startswith("win"):
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)   # 0.25 = manual on DirectShow
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)

    # Webcams often return a few dark/garbage frames right after opening
    ok = False
    for _ in range(15):
        ok, _frame = cap.read()
    if not ok:
        print(f"Camera {index} opened but returned no frames.")
        cap.release()
        sys.exit(1)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera {index} opened at {w}x{h}")
    return cap, w, h


def list_cameras(max_index=6):
    """Show a live preview of every camera index so you can see which one is the external webcam.

    Press any key to move to the next camera, 'q' to stop. Note the index in the window title
    for the camera that shows YOUR scene, then pass it as --cam.
    """
    if sys.platform == "darwin":
        print("Cameras macOS knows about (names, in no guaranteed order):")
        try:
            import subprocess
            out = subprocess.run(["system_profiler", "SPCameraDataType"],
                                 capture_output=True, text=True, timeout=10).stdout
            for line in out.splitlines():
                if line.strip().endswith(":") and not line.startswith("Camera"):
                    print("  -", line.strip().rstrip(":"))
        except Exception:
            pass
        print("OpenCV's index order on macOS does NOT always match this list "
              "(an iPhone via Continuity Camera can also take an index), so use the previews below.\n")

    print("Previewing cameras. Press any key for the next one, 'q' to quit.")
    found = False
    for i in range(max_index):
        cap = cv2.VideoCapture(i, _backend())
        if not cap.isOpened():
            cap.release()
            continue
        frame = None
        for _ in range(15):  # skip dark startup frames
            ok, f = cap.read()
            if ok:
                frame = f
        if frame is None:
            cap.release()
            continue
        found = True
        h, w = frame.shape[:2]
        print(f"  index {i}: working, {w}x{h}")
        cv2.putText(frame, f"--cam {i}   ({w}x{h})   any key = next, q = quit",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imshow(f"camera index {i}", frame)
        key = cv2.waitKey(0) & 0xFF
        cv2.destroyAllWindows()
        cap.release()
        if key == ord('q'):
            break
    if not found:
        print("  no cameras found. On macOS, check System Settings > Privacy & Security > Camera "
              "and allow your terminal (Terminal / iTerm / VS Code).")


# Laser dot detection

def find_dot(frame_bgr, color="green"):
    """Return (x, y) pixel centroid of the brightest matching-color blob, or None."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    if color == "green":
        lower = np.array([40, 90, 180])
        upper = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
    elif color == "red":
        # red wraps around hue 0/180 -- combine two ranges
        lower1 = np.array([0, 100, 200])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([170, 100, 200])
        upper2 = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
    else:
        raise ValueError(color)

    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 2:
        return None

    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    return (cx, cy)


# Calibration: build pixel -> angle mapping

def run_calibration(cam_index=0, port=None, width=None, height=None, fps=None,
                    exposure=-6.0, auto_exposure=False):
    rig = ServoRig(port=port)
    cap, w, h = open_camera(cam_index, width, height, fps, exposure, auto_exposure)

    samples = []  # list of (px, py, pan_angle, tilt_angle)
    step = 2.0

    print("Move the red dot around with W/A/S/D keys. Press SPACE to record a sample")
    print("at each position (aim for >= 6 spread-out points: corners, center, edges).")
    print("Press 'q' when done.")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        dot = find_dot(frame, "red")
        disp = frame.copy()
        if dot:
            cv2.circle(disp, (int(dot[0]), int(dot[1])), 8, (0, 255, 255), 2)
        cv2.putText(disp, f"samples: {len(samples)}  pan={rig.pan_angle:.1f} tilt={rig.tilt_angle:.1f}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("calibration", disp)

        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('w'):
            rig.nudge(d_tilt=step)
        elif key == ord('s'):
            rig.nudge(d_tilt=-step)
        elif key == ord('a'):
            rig.nudge(d_pan=-step)
        elif key == ord('d'):
            rig.nudge(d_pan=step)
        elif key == 32:  # space
            if dot is None:
                print("No red dot detected -- can't record this sample.")
            else:
                samples.append((dot[0], dot[1], rig.pan_angle, rig.tilt_angle))
                print(f"recorded sample #{len(samples)}: px={dot[0]:.0f},{dot[1]:.0f} "
                      f"-> pan={rig.pan_angle:.1f}, tilt={rig.tilt_angle:.1f}")

    cap.release()
    cv2.destroyAllWindows()
    rig.close()

    if len(samples) < 6:
        print("Need at least 6 samples for a stable fit. Re-run calibration.")
        sys.exit(1)

    samples = np.array(samples)
    px, py, pan, tilt = samples[:, 0], samples[:, 1], samples[:, 2], samples[:, 3]

    A = np.column_stack([px, py, np.ones_like(px)])
    pan_coef, *_ = np.linalg.lstsq(A, pan, rcond=None)
    tilt_coef, *_ = np.linalg.lstsq(A, tilt, rcond=None)

    # Fit quality in degrees -- a quick sanity check before you start tracking
    pan_rms = float(np.sqrt(np.mean((A @ pan_coef - pan) ** 2)))
    tilt_rms = float(np.sqrt(np.mean((A @ tilt_coef - tilt) ** 2)))
    print(f"Calibration fit residual: pan RMS={pan_rms:.2f} deg, tilt RMS={tilt_rms:.2f} deg")

    calib = {
        "pan_coef": pan_coef.tolist(),
        "tilt_coef": tilt_coef.tolist(),
        "frame_width": w,
        "frame_height": h,
    }
    with open(CALIB_FILE, "w") as f:
        json.dump(calib, f, indent=2)
    print(f"Saved calibration to {CALIB_FILE}: {calib}")


# Tracking helpers

class OneEuroSmoother:
    """Simple exponential smoother to kill jitter without adding much lag."""
    def __init__(self, alpha=0.35):
        self.alpha = alpha
        self.value = None

    def update(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value


def pixel_to_angles(px, py, calib):
    a = calib["pan_coef"]
    b = calib["tilt_coef"]
    pan = a[0] * px + a[1] * py + a[2]
    tilt = b[0] * px + b[1] * py + b[2]
    return pan, tilt


# Accuracy measurement: distance between the red and green dots

def laser_distance(red, green, mm_per_px=None):
    """Distance between the two dot centroids. Returns (px, mm or None)."""
    d_px = math.hypot(green[0] - red[0], green[1] - red[1])
    d_mm = d_px * mm_per_px if mm_per_px else None
    return d_px, d_mm


class AccuracyStats:
    """Collects red-to-green distances (in px) and summarises tracking accuracy."""

    def __init__(self, hit_px=10.0, mm_per_px=None, window=300):
        self.hit_px = hit_px
        self.mm_per_px = mm_per_px
        self.all = []                      # every settled sample, for the final summary
        self.recent = deque(maxlen=window)  # rolling window for the live HUD

    def add(self, d_px):
        self.all.append(d_px)
        self.recent.append(d_px)

    def reset(self):
        self.all.clear()
        self.recent.clear()

    def summary(self, data=None):
        data = np.array(self.all if data is None else data, dtype=float)
        if data.size == 0:
            return None
        s = {
            "n": int(data.size),
            "mean_px": float(np.mean(data)),
            "median_px": float(np.median(data)),
            "rms_px": float(np.sqrt(np.mean(data ** 2))),
            "p95_px": float(np.percentile(data, 95)),
            "max_px": float(np.max(data)),
            "hit_rate": float(np.mean(data <= self.hit_px) * 100.0),
        }
        if self.mm_per_px:
            for k in ("mean", "median", "rms", "p95", "max"):
                s[f"{k}_mm"] = s[f"{k}_px"] * self.mm_per_px
        return s

    def print_summary(self):
        s = self.summary()
        if s is None:
            print("No settled samples were recorded, so there are no accuracy stats.")
            return
        print("\n=== Tracking accuracy (red dot -> green dot) ===")
        print(f"samples        : {s['n']}")
        for label, key in (("mean error", "mean"), ("median error", "median"),
                           ("RMS error", "rms"), ("95th percentile", "p95"),
                           ("worst case", "max")):
            line = f"{label:<15}: {s[key + '_px']:.1f} px"
            if self.mm_per_px:
                line += f"  ({s[key + '_mm']:.2f} mm)"
            print(line)
        print(f"within {self.hit_px:.0f} px   : {s['hit_rate']:.1f}% of samples")


def draw_hud(disp, d_px, d_mm, settled, stats, hit_px):
    """Overlay current distance + rolling accuracy stats on the frame."""
    y = 25
    def put(text, color=(0, 255, 0)):
        nonlocal y
        cv2.putText(disp, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        y += 24

    if d_px is None:
        put("distance: -- (need both dots visible)", (0, 200, 255))
    else:
        txt = f"distance: {d_px:.1f} px"
        if d_mm is not None:
            txt += f" ({d_mm:.2f} mm)"
        put(txt, (0, 255, 0) if d_px <= hit_px else (0, 165, 255))
    put("status: " + ("settled (recording)" if settled else "settling / moving"),
        (0, 255, 0) if settled else (0, 200, 255))

    s = stats.summary(stats.recent)
    if s:
        mean_txt = f"mean: {s['mean_px']:.1f} px"
        rms_txt = f"rms: {s['rms_px']:.1f} px"
        if stats.mm_per_px:
            mean_txt += f" ({s['mean_mm']:.2f} mm)"
            rms_txt += f" ({s['rms_mm']:.2f} mm)"
        put(f"{mean_txt}   {rms_txt}")
        put(f"within {hit_px:.0f} px: {s['hit_rate']:.0f}%   n={len(stats.all)}")


# Tracking (open-loop)

def run_tracking(cam_index=0, port=None, width=None, height=None, fps=None,
                 exposure=-6.0, auto_exposure=False,
                 mm_per_px=None, hit_px=10.0, settle_frames=15, log_path=None):
    with open(CALIB_FILE) as f:
        calib = json.load(f)

    rig = ServoRig(port=port)
    cap, w, h = open_camera(cam_index, width, height, fps, exposure, auto_exposure)

    cal_w, cal_h = calib.get("frame_width"), calib.get("frame_height")
    if cal_w and cal_h and (cal_w, cal_h) != (w, h):
        print(f"Calibration was made at {cal_w}x{cal_h} but the camera is now {w}x{h}. "
              f"Re-run --calibrate (or pass --width {cal_w} --height {cal_h}).")
        cap.release()
        rig.close()
        sys.exit(1)

    pan_smooth = OneEuroSmoother(alpha=0.35)
    tilt_smooth = OneEuroSmoother(alpha=0.35)

    stats = AccuracyStats(hit_px=hit_px, mm_per_px=mm_per_px)
    STABLE_PX = 2.0           # green dot moving less than this per frame counts as "stationary"
    stable_count = 0
    prev_green = None

    log_file, log_writer = None, None
    if log_path:
        log_file = open(log_path, "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow(["time_s", "green_x", "green_y", "red_x", "red_y",
                             "distance_px", "distance_mm"])
    t0 = time.time()

    print("Tracking started. Press 'r' to reset stats, 'q' to quit.")
    print("Accuracy is only recorded once the green dot has been still long enough "
          "for the red dot to catch up.")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        green = find_dot(frame, "green")
        red = find_dot(frame, "red")  # used ONLY for measuring accuracy, never for control
        disp = frame.copy()

        d_px = d_mm = None
        settled = False

        if green is not None:
            gx, gy = green
            cv2.circle(disp, (int(gx), int(gy)), 8, (0, 255, 0), 2)

            # Open-loop: absolute angle from the calibration mapping
            target_pan, target_tilt = pixel_to_angles(gx, gy, calib)
            target_pan = pan_smooth.update(target_pan)
            target_tilt = tilt_smooth.update(target_tilt)
            rig.set_angles(target_pan, target_tilt)

            # How long has the target been still?
            if prev_green is not None and math.hypot(gx - prev_green[0], gy - prev_green[1]) < STABLE_PX:
                stable_count += 1
            else:
                stable_count = 0
            prev_green = green

            if red is not None:
                cv2.circle(disp, (int(red[0]), int(red[1])), 8, (0, 0, 255), 2)
                cv2.line(disp, (int(gx), int(gy)), (int(red[0]), int(red[1])), (255, 255, 0), 1)
                d_px, d_mm = laser_distance(red, green, mm_per_px)

                settled = stable_count >= settle_frames
                if settled:
                    stats.add(d_px)
                    if log_writer:
                        log_writer.writerow([f"{time.time() - t0:.3f}", f"{gx:.1f}", f"{gy:.1f}",
                                             f"{red[0]:.1f}", f"{red[1]:.1f}", f"{d_px:.2f}",
                                             f"{d_mm:.3f}" if d_mm is not None else ""])
        else:
            stable_count = 0
            prev_green = None

        draw_hud(disp, d_px, d_mm, settled, stats, hit_px)
        cv2.imshow("tracking", disp)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            stats.reset()
            print("Accuracy stats reset.")

    cap.release()
    cv2.destroyAllWindows()
    rig.close()
    if log_file:
        log_file.close()
        print(f"Saved samples to {log_path}")
    stats.print_summary()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibrate", action="store_true")
    parser.add_argument("--track", action="store_true")
    parser.add_argument("--list-cams", action="store_true",
                        help="probe camera indices to find your external webcam, then exit")
    parser.add_argument("--port", type=str,
                        help="ESP32 serial port, e.g. COM5 or /dev/ttyUSB0 or /dev/cu.usbserial-XXXX")

    # external webcam options
    parser.add_argument("--cam", type=int, default=1,
                        help="camera index (built-in is usually 0, external usually 1+). Default 1")
    parser.add_argument("--width", type=int, default=None, help="capture width, e.g. 1280")
    parser.add_argument("--height", type=int, default=None, help="capture height, e.g. 720")
    parser.add_argument("--fps", type=int, default=None, help="capture frame rate, e.g. 30")
    parser.add_argument("--exposure", type=float, default=-6.0,
                        help="manual exposure value (driver specific; see open_camera comments)")
    parser.add_argument("--auto-exposure", action="store_true",
                        help="leave the camera's auto exposure on (skips --exposure)")

    # accuracy options
    parser.add_argument("--mm-per-px", type=float, default=None,
                        help="scale at the target plane, so distances are also shown in mm. "
                             "Measure a known length in the camera image: mm_per_px = real_mm / pixels")
    parser.add_argument("--hit-px", type=float, default=10.0,
                        help="a sample counts as a 'hit' if red is within this many px of green")
    parser.add_argument("--settle-frames", type=int, default=15,
                        help="frames the green dot must be still before accuracy is recorded")
    parser.add_argument("--log", type=str, default=None,
                        help="write every settled accuracy sample to this CSV file")
    args = parser.parse_args()

    if args.list_cams:
        list_cameras()
        sys.exit(0)

    if not (args.calibrate or args.track):
        parser.print_help()
        sys.exit(0)

    if not args.port:
        parser.error("--port is required for --calibrate / --track")

    cam_kwargs = dict(width=args.width, height=args.height, fps=args.fps,
                      exposure=args.exposure, auto_exposure=args.auto_exposure)

    if args.calibrate:
        run_calibration(args.cam, port=args.port, **cam_kwargs)
    elif args.track:
        run_tracking(args.cam, port=args.port, mm_per_px=args.mm_per_px,
                     hit_px=args.hit_px, settle_frames=args.settle_frames,
                     log_path=args.log, **cam_kwargs)
