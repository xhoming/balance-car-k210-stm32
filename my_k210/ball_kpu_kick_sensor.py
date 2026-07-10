import gc
import time

import lcd
import sensor
from maix import KPU

try:
    from modules import ybserial, ybkey
except ImportError:
    ybserial = None
    ybkey = None


HEAD = 0xB6
TAIL = 0x6B

MODEL_PATH = "/sd/det.kmodel"

IMG_W = 320
IMG_H = 240
IMG_CENTER_X = IMG_W // 2
CAMERA_VFLIP = False
CAMERA_HMIRROR = False

CMD_PERIOD_MS = 15
STOP_CMD_PERIOD_MS = 100
KEY_DEBOUNCE_MS = 250
LCD_EVERY_N = 2
SEND_REPEAT = 2
KPU_REFRESH_FRAMES = 5

FLAG_RUNNING = 0x01
FLAG_VALID = 0x02
LABELS = ["ball"]

ANCHORS = (
    1.00, 0.97,
    1.97, 1.94,
    3.53, 3.59,
    4.38, 4.69,
    6.31, 5.53,
)

YOLO_THRESHOLD = 0.25
YOLO_NMS = 0.30

ERROR_GAIN = 1.0
TRACK_LOST_FRAMES = 3
TRACK_ROI_PAD = 14
TRACK_MAX_GROWTH = 180
BALL_ASPECT_MIN = 75
BALL_ASPECT_MAX = 133
BLUE_SEED_RATIO = 45
BLUE_A_MARGIN = 16
BLUE_B_MARGIN = 20
BLUE_PIXELS_MIN = 35

WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
YELLOW = (255, 220, 0)
GRAY = (120, 120, 120)


def clamp(v, low, high):
    if v < low:
        return low
    if v > high:
        return high
    return v


def screen_area_x10(pixels):
    return int(clamp((pixels * 1000 + (IMG_W * IMG_H) // 2) /
                     (IMG_W * IMG_H), 0, 255))


class KeyButton:
    def __init__(self):
        self.last_pressed = False
        self.last_event_ms = 0
        if ybkey:
            self.key = ybkey()
            self.mode = "ybkey"
        else:
            from Maix import GPIO
            from fpioa_manager import fm
            from board import board_info
            fm.register(board_info.BOOT_KEY, fm.fpioa.GPIOHS0)
            self.key = GPIO(GPIO.GPIOHS0, GPIO.IN)
            self.mode = "boot"

    def pressed_event(self):
        if self.mode == "ybkey":
            pressed = bool(self.key.is_press())
        else:
            pressed = self.key.value() == 0
        now = time.ticks_ms()
        event = (pressed and not self.last_pressed and
                 time.ticks_diff(now, self.last_event_ms) > KEY_DEBOUNCE_MS)
        if event:
            self.last_event_ms = now
        self.last_pressed = pressed
        return event


class Stm32Link:
    def __init__(self):
        try:
            from fpioa_manager import fm
            from machine import UART
            fm.register(6, fm.fpioa.UART2_RX)
            fm.register(8, fm.fpioa.UART2_TX)
            self.serial = UART(UART.UART2, 460800, 8, 0, 0,
                               timeout=0, read_buf_len=128)
            self.mode = "uart2"
        except Exception:
            if ybserial:
                self.serial = ybserial()
                self.mode = "ybserial"
            else:
                self.serial = None
                self.mode = "none"

    def send_once(self, seq, flags, error, area_x10, confidence):
        seq = int(seq) & 0xFF
        flags = int(flags) & 0xFF
        error = int(clamp(error, -100, 100))
        area_x10 = int(clamp(area_x10, 0, 255))
        confidence = int(clamp(confidence, 0, 100))
        error_u = error & 0xFF
        chk = (seq + flags + error_u + area_x10 + confidence) & 0xFF
        frame = [HEAD, seq, flags, error_u, area_x10,
                 confidence, chk, TAIL]
        try:
            if self.mode == "ybserial":
                self.serial.send_bytearray(frame)
                return True
            elif self.mode == "uart2":
                self.serial.write(bytes(frame))
                return True
        except Exception:
            pass
        return False

    def send(self, seq, flags, error, area_x10, confidence, repeat=1):
        ok = False
        for _ in range(repeat):
            if self.send_once(seq, flags, error, area_x10, confidence):
                ok = True
        return ok


def clamp_lab(v, low, high):
    return int(clamp(v, low, high))


def clip_roi(x, y, w, h):
    x = int(clamp(x, 0, IMG_W - 1))
    y = int(clamp(y, 0, IMG_H - 1))
    w = int(clamp(w, 1, IMG_W - x))
    h = int(clamp(h, 1, IMG_H - y))
    return (x, y, w, h)


def intersect_roi(first, second):
    x1 = max(first[0], second[0])
    y1 = max(first[1], second[1])
    x2 = min(first[0] + first[2], second[0] + second[2])
    y2 = min(first[1] + first[3], second[1] + second[3])
    if x2 <= x1 or y2 <= y1:
        return None
    return (x1, y1, x2 - x1, y2 - y1)


class BlueBallTracker:
    def __init__(self):
        self.threshold = None
        self.last = None
        self.kpu_roi = None
        self.lost_frames = 0

    def active(self):
        return (self.threshold is not None and self.last is not None and
                self.kpu_roi is not None)

    def reset(self):
        self.threshold = None
        self.last = None
        self.kpu_roi = None
        self.lost_frames = 0

    def acquire(self, img, seed):
        seed_w = max(8, int(seed["w"] * BLUE_SEED_RATIO / 100))
        seed_h = max(8, int(seed["h"] * BLUE_SEED_RATIO / 100))
        seed_x = seed["cx"] - seed_w // 2
        seed_y = seed["cy"] - seed_h // 2
        stats = img.get_statistics(roi=clip_roi(seed_x, seed_y,
                                                 seed_w, seed_h))
        a_mean = int(stats.a_mean())
        b_mean = int(stats.b_mean())

        self.threshold = (0, 100,
                          clamp_lab(a_mean - BLUE_A_MARGIN, -128, 127),
                          clamp_lab(a_mean + BLUE_A_MARGIN, -128, 127),
                          clamp_lab(b_mean - BLUE_B_MARGIN, -128, 127),
                          clamp_lab(b_mean + BLUE_B_MARGIN, -128, 127))
        self.kpu_roi = clip_roi(seed["x"], seed["y"],
                                seed["w"], seed["h"])
        self.last = seed.copy()
        self.lost_frames = 0
        return self.update(img)

    def refresh_kpu_roi(self, seed):
        new_roi = clip_roi(seed["x"], seed["y"], seed["w"], seed["h"])
        tracking_roi = clip_roi(self.last["x"] - TRACK_ROI_PAD,
                                self.last["y"] - TRACK_ROI_PAD,
                                self.last["w"] + TRACK_ROI_PAD * 2,
                                self.last["h"] + TRACK_ROI_PAD * 2)
        if intersect_roi(tracking_roi, new_roi) is None:
            self.last = seed.copy()
        self.kpu_roi = new_roi
        self.lost_frames = 0

    def needs_kpu_refresh(self):
        if not self.active():
            return True
        width = self.last["w"]
        height = self.last["h"]
        return (width * 100 < height * BALL_ASPECT_MIN or
                width * 100 > height * BALL_ASPECT_MAX or
                self.lost_frames > 0)

    def update(self, img):
        if not self.active():
            return None

        tracking_roi = clip_roi(self.last["x"] - TRACK_ROI_PAD,
                                self.last["y"] - TRACK_ROI_PAD,
                                self.last["w"] + TRACK_ROI_PAD * 2,
                                self.last["h"] + TRACK_ROI_PAD * 2)
        roi = intersect_roi(tracking_roi, self.kpu_roi)
        if roi is None:
            self.reset()
            return None
        blobs = img.find_blobs([self.threshold], roi=roi,
                               pixels_threshold=BLUE_PIXELS_MIN,
                               area_threshold=BLUE_PIXELS_MIN,
                               merge=True)
        best = None
        best_score = -1000000
        for blob in blobs:
            if (blob.w() * 100 > self.last["w"] * TRACK_MAX_GROWTH or
                    blob.h() * 100 > self.last["h"] * TRACK_MAX_GROWTH):
                continue
            distance = abs(blob.cx() - self.last["cx"]) + \
                       abs(blob.cy() - self.last["cy"])
            score = blob.pixels() * 2 - distance * 12
            if score > best_score:
                best_score = score
                best = blob

        if best is None:
            self.lost_frames += 1
            if self.lost_frames >= TRACK_LOST_FRAMES:
                self.reset()
            return None

        pixels = best.pixels()
        cx = best.cx()
        cy = best.cy()
        error_px = cx - IMG_CENTER_X
        target = {
            "x": best.x(),
            "y": best.y(),
            "w": best.w(),
            "h": best.h(),
            "cx": cx,
            "cy": cy,
            "pixels": pixels,
            "area_x10": screen_area_x10(pixels),
            "confidence": int(clamp(40 + pixels * 60 / 900, 40, 100)),
            "error_px": error_px,
            "error": int(clamp(error_px * 100 / IMG_CENTER_X * ERROR_GAIN,
                                -100, 100)),
        }
        self.last = target
        self.lost_frames = 0
        return target


def init_camera():
    lcd.init()
    lcd.clear(lcd.RED)
    try:
        sensor.reset(dual_buff=True)
    except Exception:
        sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_vflip(CAMERA_VFLIP)
    sensor.set_hmirror(CAMERA_HMIRROR)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.skip_frames(time=900)


def init_kpu():
    kpu = KPU()
    kpu.load_kmodel(MODEL_PATH)
    kpu.init_yolo2(
        ANCHORS,
        anchor_num=int(len(ANCHORS) / 2),
        img_w=IMG_W,
        img_h=IMG_H,
        net_w=IMG_W,
        net_h=IMG_H,
        layer_w=10,
        layer_h=8,
        threshold=YOLO_THRESHOLD,
        nms_value=YOLO_NMS,
        classes=len(LABELS),
    )
    return kpu


def detection_to_target(det):
    x = int(det[0])
    y = int(det[1])
    w = int(det[2])
    h = int(det[3])
    score = float(det[5])
    cx = x + w // 2
    cy = y + h // 2
    error_px = cx - IMG_CENTER_X
    error = int(error_px * 100 / IMG_CENTER_X * ERROR_GAIN)
    return {
        "x": x,
        "y": y,
        "w": w,
        "h": h,
        "cx": cx,
        "cy": cy,
        "score": score,
        "confidence": int(clamp(score * 100, 0, 100)),
        "error_px": error_px,
        "error": int(clamp(error, -100, 100)),
    }


def choose_target(detections):
    if not detections:
        return None
    best = None
    best_score = -1000000
    for det in detections:
        target = detection_to_target(det)
        center_bonus = IMG_CENTER_X - abs(target["cx"] - IMG_CENTER_X)
        if center_bonus < 0:
            center_bonus = 0
        score = target["score"] * 10000 + center_bonus
        if score > best_score:
            best_score = score
            best = target
    return best


def draw_debug(img, target, det_count, tracker, running, fps,
               link_mode, tx_count):
    img.draw_line(IMG_CENTER_X, 0, IMG_CENTER_X, IMG_H - 1,
                  color=WHITE, thickness=1)
    if tracker.kpu_roi is not None:
        img.draw_rectangle(tracker.kpu_roi, color=GRAY, thickness=1)
    if target is not None:
        img.draw_rectangle((target["x"], target["y"],
                            target["w"], target["h"]),
                           color=YELLOW, thickness=2)
        img.draw_cross(target["cx"], target["cy"], color=RED, thickness=2)
        img.draw_line(IMG_CENTER_X, IMG_H - 1,
                      target["cx"], target["cy"],
                      color=RED, thickness=2)
        area_x10 = target["area_x10"]
        info = "%s n:%d l:%d e:%d a:%d.%d%% c:%d" % (
            "BLU" if tracker.active() else "KPU", det_count,
            tracker.lost_frames, target["error"], area_x10 // 10,
            area_x10 % 10, target["confidence"])
    else:
        info = "KPU n:%d e:0 a:0.0%% c:0" % det_count

    img.draw_string(0, 0, "%s fps:%2.1f %s" %
                    ("RUN" if running else "STOP", fps, link_mode),
                    color=GREEN if running else YELLOW, scale=1)
    img.draw_string(0, 14, info, color=WHITE, scale=1)
    img.draw_string(0, 226, "key:start/stop tx:%d" % tx_count,
                    color=GRAY, scale=1)


def main():
    init_camera()
    key = KeyButton()
    link = Stm32Link()
    tracker = BlueBallTracker()
    kpu = init_kpu()
    clock = time.clock()
    running = False
    last_send = time.ticks_ms()
    tx_count = 0
    frame_count = 0
    seq = 0

    while True:
        try:
            clock.tick()
            img = sensor.snapshot()
            now = time.ticks_ms()
            frame_count += 1
            seq = (seq + 1) & 0xFF
            if frame_count % 30 == 0:
                gc.collect()

            if key.pressed_event():
                running = not running
                tracker.reset()
                if not running:
                    link.send(seq, 0, 0, 0, 0, SEND_REPEAT)

            det_count = 0
            target = None
            tracked_this_frame = False
            refresh_kpu = (tracker.needs_kpu_refresh() or
                           frame_count % KPU_REFRESH_FRAMES == 0)

            if refresh_kpu:
                kpu.run_with_output(img)
                detections = kpu.regionlayer_yolo2()
                det_count = len(detections) if detections else 0
                seed = choose_target(detections)
                if seed is not None:
                    if tracker.active():
                        tracker.refresh_kpu_roi(seed)
                    else:
                        target = tracker.acquire(img, seed)
                        tracked_this_frame = True

            if tracker.active() and not tracked_this_frame:
                target = tracker.update(img)

            if target is None:
                error = 0
                area_x10 = 0
                confidence = 0
            else:
                error = target["error"]
                area_x10 = target["area_x10"]
                confidence = target["confidence"]

            flags = FLAG_RUNNING if running else 0
            if target is not None:
                flags |= FLAG_VALID

            if running and time.ticks_diff(now, last_send) >= CMD_PERIOD_MS:
                if link.send(seq, flags, error, area_x10,
                             confidence, SEND_REPEAT):
                    tx_count += 1
                last_send = now
            elif ((not running) and
                  time.ticks_diff(now, last_send) >= STOP_CMD_PERIOD_MS):
                if link.send(seq, flags, error, area_x10,
                             confidence, SEND_REPEAT):
                    tx_count += 1
                last_send = now

            if frame_count % LCD_EVERY_N == 0:
                draw_debug(img, target, det_count, tracker, running,
                           clock.fps(), link.mode, tx_count)
                lcd.display(img)
        except Exception as err:
            try:
                link.send(seq, 0, 0, 0, 0, SEND_REPEAT)
                img.draw_string(0, 40, "ERR:%s" % err, color=RED, scale=1)
                lcd.display(img)
            except Exception:
                pass
            running = False
            tracker.reset()


main()
