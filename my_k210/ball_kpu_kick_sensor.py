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
TARGET_HOLD_MS = 5000
SEND_REPEAT = 3

STATE_SEARCH = 0
STATE_ALIGN = 1
STATE_APPROACH = 2
STATE_CHARGE = 3
STATE_BRAKE = 4

STATE_NAMES = ("SRCH", "ALGN", "APPR", "KICK", "STOP")
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

CENTER_WINDOW_PX = 28
ERROR_GAIN = 1.0
CLOSE_BOX_AREA = 9000
BOX_H_CLOSE = 100

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


def area_score(w, h):
    return int(clamp(w * h * 100 / CLOSE_BOX_AREA, 0, 100))


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

    def send_once(self, state, error, near, confidence, y_score):
        state = int(clamp(state, 0, 4))
        error = int(clamp(error, -100, 100))
        near = int(clamp(near, 0, 100))
        confidence = int(clamp(confidence, 0, 100))
        y_score = int(clamp(y_score, 0, 100))
        error_u = error & 0xFF
        chk = (state + error_u + near + confidence + y_score) & 0xFF
        frame = [HEAD, state, error_u, near, confidence, y_score, chk, TAIL]
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

    def send(self, state, error, near, confidence, y_score, repeat=1):
        ok = False
        for _ in range(repeat):
            if self.send_once(state, error, near, confidence, y_score):
                ok = True
        return ok


class TargetMemory:
    def __init__(self):
        self.target = None
        self.last_seen_ms = 0

    def reset(self):
        self.target = None
        self.last_seen_ms = 0

    def update(self, target):
        now = time.ticks_ms()
        if target is not None:
            self.target = target
            self.last_seen_ms = now
            target["fresh"] = True
            return target, True, 0

        if self.target is not None:
            lost_ms = time.ticks_diff(now, self.last_seen_ms)
            if lost_ms <= TARGET_HOLD_MS:
                hold_count = int(lost_ms / 100)
                held = self.target.copy()
                held["fresh"] = False
                return held, False, hold_count

        self.reset()
        return None, False, int(TARGET_HOLD_MS / 100)


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
    box_area_score = area_score(w, h)
    h_score = int(clamp(h * 100 / BOX_H_CLOSE, 0, 100))
    y_score = int(clamp(cy * 100 / (IMG_H - 1), 0, 100))
    near = box_area_score
    if h_score > near:
        near = h_score
    return {
        "x": x,
        "y": y,
        "w": w,
        "h": h,
        "cx": cx,
        "cy": cy,
        "score": score,
        "confidence": int(clamp(score * 100, 0, 100)),
        "area": box_area_score,
        "near": near,
        "y_score": y_score,
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
        score = target["score"] * 10000 + target["area"] * 20 + center_bonus
        if score > best_score:
            best_score = score
            best = target
    return best


def measure_state(target):
    if target is None:
        return STATE_SEARCH
    if abs(target["error_px"]) > CENTER_WINDOW_PX:
        return STATE_ALIGN
    return STATE_APPROACH


def draw_debug(img, target, det_count, hold_count, state, running, fps,
               link_mode, tx_count):
    img.draw_line(IMG_CENTER_X, 0, IMG_CENTER_X, IMG_H - 1,
                  color=WHITE, thickness=1)
    if target is not None:
        img.draw_rectangle((target["x"], target["y"],
                            target["w"], target["h"]),
                           color=YELLOW, thickness=2)
        img.draw_cross(target["cx"], target["cy"], color=RED, thickness=2)
        img.draw_line(IMG_CENTER_X, IMG_H - 1,
                      target["cx"], target["cy"],
                      color=RED, thickness=2)
        info = "%s ball n:%d h:%d e:%d near:%d y:%d c:%d" % (
            STATE_NAMES[state], det_count, hold_count, target["error"],
            target["near"], target["y_score"], target["confidence"])
    else:
        info = "%s none n:%d h:%d e:0 near:0 y:0 c:0" % (
            STATE_NAMES[state], det_count, hold_count)

    img.draw_string(0, 0, "%s kpu:%2.1f %s" %
                    ("RUN" if running else "STOP", fps, link_mode),
                    color=GREEN if running else YELLOW, scale=1)
    img.draw_string(0, 14, info, color=WHITE, scale=1)
    img.draw_string(0, 226, "key:start/stop tx:%d" % tx_count,
                    color=GRAY, scale=1)


def main():
    init_camera()
    key = KeyButton()
    link = Stm32Link()
    memory = TargetMemory()
    kpu = init_kpu()
    clock = time.clock()
    running = False
    last_send = time.ticks_ms()
    tx_count = 0
    frame_count = 0

    while True:
        try:
            gc.collect()
            clock.tick()
            img = sensor.snapshot()
            now = time.ticks_ms()
            frame_count += 1

            if key.pressed_event():
                running = not running
                memory.reset()
                if not running:
                    link.send(STATE_BRAKE, 0, 0, 0, 0, SEND_REPEAT)

            kpu.run_with_output(img)
            detections = kpu.regionlayer_yolo2()
            det_count = len(detections) if detections else 0
            raw_target = choose_target(detections)
            target, fresh, hold_count = memory.update(raw_target)
            state = measure_state(target)

            if target is None:
                error = 0
                near = 0
                confidence = 0
                y_score = 0
            else:
                error = target["error"]
                near = target["near"]
                confidence = target["confidence"]
                y_score = target["y_score"]

            if running and time.ticks_diff(now, last_send) >= CMD_PERIOD_MS:
                if link.send(state, error, near, confidence, y_score, SEND_REPEAT):
                    tx_count += 1
                last_send = now
            elif ((not running) and
                  time.ticks_diff(now, last_send) >= STOP_CMD_PERIOD_MS):
                if link.send(STATE_BRAKE, 0, 0, 0, 0, SEND_REPEAT):
                    tx_count += 1
                last_send = now

            if frame_count % LCD_EVERY_N == 0:
                draw_debug(img, target, det_count, hold_count, state, running,
                           clock.fps(), link.mode, tx_count)
                lcd.display(img)
        except Exception as err:
            try:
                link.send(STATE_BRAKE, 0, 0, 0, 0, SEND_REPEAT)
                img.draw_string(0, 40, "ERR:%s" % err, color=RED, scale=1)
                lcd.display(img)
            except Exception:
                pass
            running = False
            memory.reset()


main()
