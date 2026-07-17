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


# Protocol: [B8, seq, flags, error, area_x10, reserved,
#               confidence, checksum, 8B]
HEAD = 0xB8
TAIL = 0x8B
FLAG_RUNNING = 0x01
FLAG_VALID = 0x02
TTC_INVALID = 255

MODEL_PATH = "/sd/det.kmodel"
IMG_W = 320
IMG_H = 240
IMG_CENTER_X = IMG_W // 2
CAMERA_VFLIP = False
CAMERA_HMIRROR = False

# Communication and display. LCD refresh does not limit control output rate.
CMD_PERIOD_MS = 15
STOP_CMD_PERIOD_MS = 100
KEY_DEBOUNCE_MS = 250
LCD_EVERY_N = 6
KPU_REFRESH_FRAMES = 4

ANCHORS = (
    1.00, 0.97,
    1.97, 1.94,
    3.53, 3.59,
    4.38, 4.69,
    6.31, 5.53,
)
YOLO_THRESHOLD = 0.20
YOLO_NMS = 0.30

# STM32使用同一门槛把画面分为左、中、右三个区域。
LANE_ERROR_LIMIT = 15

# KPU acquisition followed by fast LAB tracking inside a predictive ROI.
TRACK_LOST_FRAMES = 2
TRACK_ROI_PAD = 36
TRACK_MAX_GROWTH = 190
BALL_ASPECT_MIN = 65
BALL_ASPECT_MAX = 150
BALL_FILL_MIN = 25
BLUE_SEED_RATIO = 45
BLUE_SEED_A_MIN = -25
BLUE_SEED_A_MAX = 25
BLUE_SEED_B_MIN = -80
BLUE_SEED_B_MAX = -12
BLUE_A_MARGIN = 16
BLUE_B_MARGIN = 20
BLUE_PIXELS_MIN = 35

WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
YELLOW = (255, 220, 0)
CYAN = (0, 220, 255)
GRAY = (120, 120, 120)


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def clip_roi(x, y, w, h):
    x = int(clamp(x, 0, IMG_W - 1))
    y = int(clamp(y, 0, IMG_H - 1))
    w = int(clamp(w, 1, IMG_W - x))
    h = int(clamp(h, 1, IMG_H - y))
    return (x, y, w, h)


def screen_area_x10(pixels):
    return int(clamp((pixels * 1000 + (IMG_W * IMG_H) // 2) /
                     (IMG_W * IMG_H), 0, 255))


def seed_lab_means(img, seed):
    seed_w = max(8, int(seed["w"] * BLUE_SEED_RATIO / 100))
    seed_h = max(8, int(seed["h"] * BLUE_SEED_RATIO / 100))
    stats = img.get_statistics(
        roi=clip_roi(seed["cx"] - seed_w // 2,
                     seed["cy"] - seed_h // 2,
                     seed_w, seed_h))
    return int(stats.a_mean()), int(stats.b_mean())


def is_blue_seed(a_mean, b_mean):
    return (BLUE_SEED_A_MIN <= a_mean <= BLUE_SEED_A_MAX and
            BLUE_SEED_B_MIN <= b_mean <= BLUE_SEED_B_MAX)


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
            self.serial = UART(UART.UART2, 115200, 8, 0, 0,
                               timeout=0, read_buf_len=128)
            self.mode = "uart2"
        except Exception:
            if ybserial:
                self.serial = ybserial()
                self.mode = "ybserial"
            else:
                self.serial = None
                self.mode = "none"

    def send(self, seq, flags, error, area_x10,
             reserved, confidence):
        seq = int(seq) & 0xFF
        flags = int(flags) & 0xFF
        error_u = int(clamp(error, -100, 100)) & 0xFF
        area_x10 = int(clamp(area_x10, 0, 255))
        reserved = int(clamp(reserved, 0, 255))
        confidence = int(clamp(confidence, 0, 100))
        chk = (seq + flags + error_u + area_x10 +
               reserved + confidence) & 0xFF
        frame = [HEAD, seq, flags, error_u, area_x10,
                 reserved, confidence, chk, TAIL]
        try:
            if self.mode == "ybserial":
                self.serial.send_bytearray(frame)
                return True
            if self.mode == "uart2":
                self.serial.write(bytes(frame))
                return True
        except Exception:
            pass
        return False


class BlueBallTracker:
    def __init__(self):
        self.threshold = None
        self.last = None
        self.lost_frames = 0
        self.kpu_confidence = 0
        self.vx = 0
        self.vy = 0

    def active(self):
        return self.threshold is not None and self.last is not None

    def reset(self):
        self.threshold = None
        self.last = None
        self.lost_frames = 0
        self.kpu_confidence = 0
        self.vx = 0
        self.vy = 0

    def acquire(self, img, seed):
        a_mean, b_mean = seed_lab_means(img, seed)
        if not is_blue_seed(a_mean, b_mean):
            self.reset()
            return None
        self.threshold = (
            0, 100,
            int(clamp(a_mean - BLUE_A_MARGIN, -128, 127)),
            int(clamp(a_mean + BLUE_A_MARGIN, -128, 127)),
            int(clamp(b_mean - BLUE_B_MARGIN, -128, 127)),
            int(clamp(b_mean + BLUE_B_MARGIN, -128, 127)),
        )
        self.last = seed.copy()
        self.kpu_confidence = int(clamp(seed["score"] * 100, 0, 100))
        self.lost_frames = 0
        self.vx = 0
        self.vy = 0
        return self.update(img)

    def refresh_from_kpu(self, img, seed):
        a_mean, b_mean = seed_lab_means(img, seed)
        if not is_blue_seed(a_mean, b_mean):
            return False
        self.kpu_confidence = int(clamp(seed["score"] * 100, 0, 100))
        if not self.active():
            return False
        distance = abs(seed["cx"] - self.last["cx"]) + \
                   abs(seed["cy"] - self.last["cy"])
        max_distance = max(self.last["w"], self.last["h"]) * 2 + 24
        if distance > max_distance:
            self.last = seed.copy()
            self.vx = 0
            self.vy = 0
        return True

    def update(self, img):
        if not self.active():
            return None

        pad_x = TRACK_ROI_PAD + abs(self.vx) * 2
        pad_y = TRACK_ROI_PAD + abs(self.vy) * 2
        pred_cx = self.last["cx"] + self.vx
        pred_cy = self.last["cy"] + self.vy
        roi_w = self.last["w"] + pad_x * 2
        roi_h = self.last["h"] + pad_y * 2
        roi = clip_roi(pred_cx - roi_w // 2,
                       pred_cy - roi_h // 2,
                       roi_w, roi_h)
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
            distance = abs(blob.cx() - pred_cx) + abs(blob.cy() - pred_cy)
            score = blob.pixels() * 2 - distance * 10
            if score > best_score:
                best_score = score
                best = blob

        if best is None:
            self.lost_frames += 1
            if self.lost_frames >= TRACK_LOST_FRAMES:
                self.reset()
            return None

        pixels = best.pixels()
        box_area = best.w() * best.h()
        aspect_x100 = best.w() * 100 // max(1, best.h())
        fill_ratio = pixels * 100 // max(1, box_area)
        if (aspect_x100 < BALL_ASPECT_MIN or
                aspect_x100 > BALL_ASPECT_MAX or
                fill_ratio < BALL_FILL_MIN):
            self.lost_frames += 1
            if self.lost_frames >= TRACK_LOST_FRAMES:
                self.reset()
            return None

        cx = best.cx()
        cy = best.cy()
        dx = cx - self.last["cx"]
        dy = cy - self.last["cy"]
        self.vx = int((self.vx * 2 + dx) / 3)
        self.vy = int((self.vy * 2 + dy) / 3)
        error = int(clamp((cx - IMG_CENTER_X) * 100 /
                          IMG_CENTER_X, -100, 100))
        confidence = int(clamp(
            self.kpu_confidence * 60 / 100 +
            fill_ratio * 25 / 100 +
            min(pixels, 1200) * 15 / 1200, 0, 100))
        target = {
            "x": best.x(), "y": best.y(),
            "w": best.w(), "h": best.h(),
            "cx": cx, "cy": cy,
            "pixels": pixels,
            "area_x10": screen_area_x10(pixels),
            "error": error,
            "confidence": confidence,
            "area_valid": True,
        }
        self.last = target
        self.lost_frames = 0
        return target


def detection_to_target(det):
    x = int(det[0])
    y = int(det[1])
    w = int(det[2])
    h = int(det[3])
    return {
        "x": x, "y": y, "w": w, "h": h,
        "cx": x + w // 2,
        "cy": y + h // 2,
        "score": float(det[5]),
    }


def choose_target(img, detections):
    if not detections:
        return None
    best = None
    best_score = -1.0
    for det in detections:
        target = detection_to_target(det)
        a_mean, b_mean = seed_lab_means(img, target)
        if not is_blue_seed(a_mean, b_mean):
            continue
        if target["score"] > best_score:
            best_score = target["score"]
            best = target
    return best


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
        img_w=IMG_W, img_h=IMG_H,
        net_w=IMG_W, net_h=IMG_H,
        layer_w=10, layer_h=8,
        threshold=YOLO_THRESHOLD,
        nms_value=YOLO_NMS,
        classes=1,
    )
    return kpu


def draw_debug(img, target, running, control_valid, error,
               area_x10, confidence, flags, fps, link_mode):
    left_x = int(IMG_CENTER_X * (100 - LANE_ERROR_LIMIT) / 100)
    right_x = int(IMG_CENTER_X * (100 + LANE_ERROR_LIMIT) / 100)
    img.draw_line(left_x, 0, left_x, IMG_H - 1,
                  color=GRAY, thickness=1)
    img.draw_line(right_x, 0, right_x, IMG_H - 1,
                  color=GRAY, thickness=1)
    img.draw_line(IMG_CENTER_X, 0, IMG_CENTER_X, IMG_H - 1,
                  color=WHITE, thickness=1)
    if target is not None:
        img.draw_rectangle((target["x"], target["y"],
                            target["w"], target["h"]),
                           color=YELLOW, thickness=2)
        img.draw_cross(target["cx"], target["cy"],
                       color=RED, thickness=2)
    if not running:
        state = "STOP"
    elif control_valid:
        state = "BALL"
    else:
        state = "WAIT"
    img.draw_string(0, 0, "%s fps:%2.1f %s" %
                    (state, fps, link_mode),
                    color=GREEN if control_valid else GRAY, scale=1)
    if error <= -LANE_ERROR_LIMIT:
        lane = "L"
    elif error >= LANE_ERROR_LIMIT:
        lane = "R"
    else:
        lane = "C"
    img.draw_string(0, 14, "e:%d lane:%s a:%d.%d" %
                    (error, lane, area_x10 // 10, area_x10 % 10),
                    color=WHITE, scale=1)
    img.draw_string(0, 28, "c:%d f:%d" %
                    (confidence, flags),
                    color=CYAN, scale=1)


def main():
    init_camera()
    key = KeyButton()
    link = Stm32Link()
    tracker = BlueBallTracker()
    kpu = init_kpu()
    clock = time.clock()

    running = False
    frame_count = 0
    seq = 0
    last_send = time.ticks_ms()

    while True:
        try:
            clock.tick()
            img = sensor.snapshot()
            now = time.ticks_ms()
            frame_count += 1
            seq = (seq + 1) & 0xFF
            if frame_count % 90 == 0:
                gc.collect()

            if key.pressed_event():
                running = not running
                tracker.reset()
                if not running:
                    link.send(seq, 0, 0, 0, TTC_INVALID, 0)

            target = None
            tracked_this_frame = False
            refresh_kpu = (not tracker.active() or
                           frame_count % KPU_REFRESH_FRAMES == 0)
            if refresh_kpu:
                kpu.run_with_output(img)
                seed = choose_target(img, kpu.regionlayer_yolo2())
                if seed is not None:
                    if tracker.active():
                        tracker.refresh_from_kpu(img, seed)
                    else:
                        target = tracker.acquire(img, seed)
                        tracked_this_frame = True

            if tracker.active() and not tracked_this_frame:
                target = tracker.update(img)

            control_valid = running and target is not None
            error = target["error"] if target is not None else 0
            area_x10 = target["area_x10"] if target is not None else 0
            confidence = target["confidence"] if target is not None else 0

            flags = FLAG_RUNNING if running else 0
            if control_valid:
                flags |= FLAG_VALID

            if running and time.ticks_diff(now, last_send) >= CMD_PERIOD_MS:
                link.send(seq, flags, error, area_x10,
                          TTC_INVALID, confidence)
                last_send = now
            elif (not running and
                  time.ticks_diff(now, last_send) >= STOP_CMD_PERIOD_MS):
                link.send(seq, 0, 0, 0, TTC_INVALID, 0)
                last_send = now

            if frame_count % LCD_EVERY_N == 0:
                draw_debug(img, target, running, control_valid, error,
                           area_x10, confidence, flags,
                           clock.fps(), link.mode)
                lcd.display(img)
        except Exception as err:
            try:
                link.send(seq, 0, 0, 0, TTC_INVALID, 0)
                img.draw_string(0, 44, "ERR:%s" % err,
                                color=RED, scale=1)
                lcd.display(img)
            except Exception:
                pass
            running = False
            tracker.reset()


main()
