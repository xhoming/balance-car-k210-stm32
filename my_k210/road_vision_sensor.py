import sensor
import time
import lcd

try:
    import touchscreen as ts
except ImportError:
    ts = None

try:
    from modules import ybserial, ybkey
except ImportError:
    ybserial = None
    ybkey = None


HEAD = 0xA5
TAIL = 0x5A

IMG_W = 320
IMG_H = 240
IMG_CENTER_X = IMG_W // 2

DEBUG_MODE = True
DISPLAY_WHILE_RUNNING = True
DISPLAY_EVERY_N = 2

# Camera is mounted almost parallel to the ground: use layered strip ROIs
# from near to very far, instead of a single full-screen blob search.
BLACK_AB_LIMIT = 42
ROAD_A_MIN = -6
LOCAL_L_OFFSET = 15
LOCAL_L_MIN = 20
LOCAL_L_MAX = 88
MIN_PIXELS = 60
MIN_AREA = 60
MAX_AREA = 60000
MIN_DENSITY = 0.02
MIN_ROAD_W = 24
MAX_ROAD_GAP = 58
GREEN_PIXELS_THRESHOLD = 90
GREEN_EDGE_SLOPE = 60

CENTER_DEAD_ZONE = 0
SLOPE_DEAD_ZONE = 0
ERROR_GAIN = 0.85
SLOPE_GAIN = 1.10
FAR_SLOPE_GAIN = 1.85
PATH_MAX_JUMP = 185
SEED_CENTER_WINDOW = 92
EDGE_GUARD_MARGIN = 68
EDGE_GUARD_SLOPE = 55

LOST_SEARCH_MS = 800
LOST_SEARCH_ERROR = 55
LOST_SEARCH_SLOPE = 80
LOST_SEARCH_CONF = 55
BASE_SPEED = 7

CMD_PERIOD_MS = 50
KEY_DEBOUNCE_MS = 250

NEAR_ROIS = [
    (12, 176, 296, 24, 0.35),
    (12, 206, 296, 24, 0.45),
]
MID_ROIS = [
    (12, 116, 296, 24, 0.35),
    (12, 146, 296, 24, 0.45),
]
FAR_ROIS = [
    (12, 56, 296, 24, 0.30),
    (12, 86, 296, 24, 0.40),
]
VERY_FAR_ROIS = [
    (12, 12, 296, 20, 0.30),
    (12, 34, 296, 18, 0.40),
]
LAYERS = [NEAR_ROIS, MID_ROIS, FAR_ROIS, VERY_FAR_ROIS]
LAYER_TARGET_WEIGHTS = [0.34, 0.30, 0.24, 0.12]

PARAM_FILE = "/sd/road_sensor_simple_v1.txt"
PARAMS = {
    "LOCAL_L_OFFSET": LOCAL_L_OFFSET,
    "ROAD_A_MIN": ROAD_A_MIN,
    "MIN_ROAD_W": MIN_ROAD_W,
    "MAX_ROAD_GAP": MAX_ROAD_GAP,
    "ERROR_GAIN": ERROR_GAIN,
    "SLOPE_GAIN": SLOPE_GAIN,
    "FAR_SLOPE_GAIN": FAR_SLOPE_GAIN,
    "BASE_SPEED": BASE_SPEED,
}

WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
YELLOW = (255, 220, 0)
BLUE = (0, 100, 255)


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def load_params(params):
    try:
        with open(PARAM_FILE, "r") as f:
            for line in f:
                if "=" not in line:
                    continue
                key, value = line.strip().split("=", 1)
                if key in params:
                    params[key] = float(value)
    except Exception:
        pass


def save_params(params):
    text = ""
    for key in sorted(params.keys()):
        text += "%s=%s\n" % (key, params[key])
    try:
        with open(PARAM_FILE, "w") as f:
            f.write(text)
        return True
    except Exception:
        return False


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

    def send_vision(self, error, slope, confidence, speed):
        error = int(clamp(error, -100, 100))
        slope = int(clamp(slope, -100, 100))
        confidence = int(clamp(confidence, 0, 100))
        speed = int(clamp(speed, 0, 30))
        err_u = error & 0xFF
        slope_u = slope & 0xFF
        speed_u = speed & 0xFF
        chk = (err_u + slope_u + confidence + speed_u) & 0xFF
        frame = [HEAD, err_u, slope_u, confidence, speed_u, chk, TAIL]
        try:
            if self.mode == "ybserial":
                self.serial.send_bytearray(frame)
            elif self.mode == "uart2":
                self.serial.write(bytes(frame))
        except Exception:
            pass


def init_camera():
    lcd.init()
    if ts:
        try:
            ts.init()
        except Exception:
            pass
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.skip_frames(time=800)


def local_road_threshold(img, roi, params):
    roi_rect = roi[:4]
    try:
        stats = img.get_statistics(roi=roi_rect)
        l_max = int(stats.l_max() - params["LOCAL_L_OFFSET"] - 10)
    except Exception:
        l_max = 55
    l_max = int(clamp(l_max, LOCAL_L_MIN, LOCAL_L_MAX))
    a_min = int(clamp(params["ROAD_A_MIN"], -35, 20))
    threshold = [(0, l_max, a_min, BLACK_AB_LIMIT,
                  -BLACK_AB_LIMIT, BLACK_AB_LIMIT)]
    return threshold, l_max


def green_edge_bias(img):
    left_roi = (0, 40, 58, 170)
    right_roi = (262, 40, 58, 170)
    green_threshold = [(20, 95, -128, -10, -20, 95)]
    try:
        left_blobs = img.find_blobs(green_threshold,
                                    roi=left_roi,
                                    pixels_threshold=GREEN_PIXELS_THRESHOLD,
                                    area_threshold=GREEN_PIXELS_THRESHOLD,
                                    merge=True,
                                    margin=6)
        right_blobs = img.find_blobs(green_threshold,
                                     roi=right_roi,
                                     pixels_threshold=GREEN_PIXELS_THRESHOLD,
                                     area_threshold=GREEN_PIXELS_THRESHOLD,
                                     merge=True,
                                     margin=6)
    except Exception:
        return 0, [], []

    left_pixels = 0
    right_pixels = 0
    for blob in left_blobs:
        left_pixels += blob.pixels()
    for blob in right_blobs:
        right_pixels += blob.pixels()

    if left_pixels > right_pixels + GREEN_PIXELS_THRESHOLD:
        return GREEN_EDGE_SLOPE, left_blobs, right_blobs
    if right_pixels > left_pixels + GREEN_PIXELS_THRESHOLD:
        return -GREEN_EDGE_SLOPE, left_blobs, right_blobs
    return 0, left_blobs, right_blobs


def road_span_in_roi(img, roi, params, last_x):
    x, y, w, h = roi[:4]
    thresholds, l_max = local_road_threshold(img, roi, params)
    blobs = img.find_blobs(thresholds,
                           roi=(x, y, w, h),
                           pixels_threshold=MIN_PIXELS,
                           area_threshold=MIN_AREA,
                           merge=True,
                           margin=8)
    valid = []
    for blob in blobs:
        if blob.area() < MIN_AREA or blob.area() > MAX_AREA:
            continue
        if blob.pixels() < MIN_PIXELS:
            continue
        if blob.density() < MIN_DENSITY:
            continue
        if blob.w() < params["MIN_ROAD_W"]:
            continue
        valid.append(blob)

    if not valid:
        return None, None, [], l_max

    valid.sort(key=lambda b: b.x())
    groups = []
    for blob in valid:
        left = blob.x()
        right = blob.x() + blob.w()
        if not groups:
            groups.append([left, right, blob.pixels(), [blob]])
            continue
        group = groups[-1]
        if left - group[1] <= params["MAX_ROAD_GAP"]:
            if right > group[1]:
                group[1] = right
            group[2] += blob.pixels()
            group[3].append(blob)
        else:
            groups.append([left, right, blob.pixels(), [blob]])

    best = None
    best_score = -100000
    for group in groups:
        left, right, pixels, members = group
        width = right - left
        if width < params["MIN_ROAD_W"]:
            continue
        center = (left + right) // 2
        if last_x is not None and abs(center - last_x) > PATH_MAX_JUMP:
            continue
        score = pixels + width * 5
        if last_x is not None:
            score -= abs(center - last_x) * 14
        else:
            if abs(center - IMG_CENTER_X) > SEED_CENTER_WINDOW:
                continue
            score -= abs(center - IMG_CENTER_X) * 8
        if score > best_score:
            best_score = score
            best = group

    if best is None:
        return None, None, valid, l_max

    left, right, pixels, members = best
    center = int((left + right) // 2)
    span = (int(left), y, int(right - left), h)
    return center, span, members, l_max


def detect_layer(img, rois, params, last_x):
    sum_x = 0
    sum_w = 0
    spans = []
    blobs = []
    lmax_values = []

    for roi in rois:
        weight = roi[4]
        cx, span, members, l_max = road_span_in_roi(img, roi, params,
                                                    last_x)
        spans.append(span)
        lmax_values.append(l_max)
        for blob in members:
            blobs.append(blob)
        if cx is not None:
            sum_x += cx * weight
            sum_w += weight

    if sum_w <= 0:
        return None, spans, blobs, lmax_values
    return int(sum_x / sum_w), spans, blobs, lmax_values


def detect_path(img, params, last_path_x):
    centers = []
    spans = []
    blobs = []
    green_blobs = []
    lmax_values = []
    expected_x = last_path_x
    green_bias, left_green, right_green = green_edge_bias(img)
    for blob in left_green:
        green_blobs.append(blob)
    for blob in right_green:
        green_blobs.append(blob)

    for layer in LAYERS:
        cx, layer_spans, layer_blobs, layer_lmax = detect_layer(
            img, layer, params, expected_x)
        centers.append(cx)
        for span in layer_spans:
            spans.append(span)
        for l_max in layer_lmax:
            lmax_values.append(l_max)
        for blob in layer_blobs:
            blobs.append(blob)
        if cx is not None:
            expected_x = cx

    near = centers[0]
    mid = centers[1]
    far = centers[2]
    very_far = centers[3]
    seen = 0
    for cx in centers:
        if cx is not None:
            seen += 1

    if seen == 0:
        return {
            "seen": False,
            "centers": centers,
            "spans": spans,
            "blobs": blobs,
            "target_x": None,
            "error": 0,
            "slope": 0,
            "confidence": 0,
            "lmax": lmax_values,
            "green_blobs": green_blobs,
            "green_bias": green_bias,
        }

    anchor = None
    filtered_centers = []
    for cx in centers:
        if cx is None:
            filtered_centers.append(None)
            continue
        if anchor is not None and abs(cx - anchor) > PATH_MAX_JUMP:
            filtered_centers.append(None)
            continue
        filtered_centers.append(cx)
        anchor = cx
    centers = filtered_centers

    sum_x = 0
    sum_w = 0
    for cx, weight in zip(centers, LAYER_TARGET_WEIGHTS):
        if cx is None:
            continue
        sum_x += cx * weight
        sum_w += weight

    if sum_w <= 0:
        return {
            "seen": False,
            "centers": centers,
            "spans": spans,
            "blobs": blobs,
            "target_x": None,
            "error": 0,
            "slope": 0,
            "confidence": 0,
            "lmax": lmax_values,
            "green_blobs": green_blobs,
            "green_bias": green_bias,
        }

    target_x = int(sum_x / sum_w)
    local_base = centers[0] if centers[0] is not None else centers[1]
    if local_base is None:
        local_base = target_x

    mid_hint = 0
    if centers[0] is not None and centers[1] is not None:
        mid_hint = centers[1] - centers[0]

    far_hint = 0
    look = centers[2] if centers[2] is not None else centers[3]
    if look is not None:
        far_hint = look - local_base

    slope_px = mid_hint * params["SLOPE_GAIN"] + \
        far_hint * params["FAR_SLOPE_GAIN"]

    for cx in (centers[2], centers[3]):
        if cx is None:
            continue
        if cx < EDGE_GUARD_MARGIN:
            slope_px -= EDGE_GUARD_SLOPE
        elif cx > IMG_W - EDGE_GUARD_MARGIN:
            slope_px += EDGE_GUARD_SLOPE

    slope_px += green_bias

    confidence = seen * 20
    if centers[0] is not None:
        confidence += 15
    if centers[1] is not None:
        confidence += 12
    confidence = int(clamp(confidence, 0, 100))

    error_px = target_x - IMG_CENTER_X
    if abs(error_px) <= CENTER_DEAD_ZONE:
        error_px = 0
    if abs(slope_px) <= SLOPE_DEAD_ZONE:
        slope_px = 0

    error_i8 = int(error_px * 100 / IMG_CENTER_X * params["ERROR_GAIN"])
    slope_i8 = int(slope_px * 100 / IMG_CENTER_X)

    return {
        "seen": True,
        "centers": centers,
        "spans": spans,
        "blobs": blobs,
        "target_x": target_x,
        "error": int(clamp(error_i8, -100, 100)),
        "slope": int(clamp(slope_i8, -100, 100)),
        "confidence": confidence,
        "lmax": lmax_values,
        "green_blobs": green_blobs,
        "green_bias": green_bias,
    }


def make_lost_search(last_valid, params, now):
    if last_valid is None:
        return None
    if time.ticks_diff(now, last_valid["time"]) > LOST_SEARCH_MS:
        return None
    turn_dir = last_valid["dir"]
    if turn_dir == 0:
        return None
    return {
        "seen": True,
        "lost_search": True,
        "centers": [None, None, None, None],
        "spans": [None, None, None, None, None, None, None, None],
        "blobs": [],
        "target_x": IMG_CENTER_X + LOST_SEARCH_ERROR * turn_dir,
        "error": LOST_SEARCH_ERROR * turn_dir,
        "slope": LOST_SEARCH_SLOPE * turn_dir,
        "confidence": LOST_SEARCH_CONF,
    }


def touch_params(params, save_msg):
    if not ts:
        return save_msg
    try:
        status, x, y = ts.read()
    except Exception:
        return save_msg
    if status != ts.STATUS_PRESS:
        return save_msg
    if 0 <= y < 34:
        if x < 100:
            params["LOCAL_L_OFFSET"] = clamp(params["LOCAL_L_OFFSET"] - 1,
                                             5, 35)
            return "LO-"
        if x > 220:
            params["LOCAL_L_OFFSET"] = clamp(params["LOCAL_L_OFFSET"] + 1,
                                             5, 35)
            return "LO+"
    if 40 <= y < 74:
        if x < 100:
            params["ROAD_A_MIN"] = clamp(params["ROAD_A_MIN"] - 1, -35, 20)
            return "GA-"
        if x > 220:
            params["ROAD_A_MIN"] = clamp(params["ROAD_A_MIN"] + 1, -35, 20)
            return "GA+"
    if 80 <= y < 114:
        if x < 100:
            params["MAX_ROAD_GAP"] = clamp(params["MAX_ROAD_GAP"] - 2, 20, 110)
            return "GP-"
        if x > 220:
            params["MAX_ROAD_GAP"] = clamp(params["MAX_ROAD_GAP"] + 2, 20, 110)
            return "GP+"
    if 120 <= y < 154:
        if x < 100:
            params["ERROR_GAIN"] = clamp(params["ERROR_GAIN"] - 0.05, 0.3, 1.8)
            return "EG-"
        if x > 220:
            params["ERROR_GAIN"] = clamp(params["ERROR_GAIN"] + 0.05, 0.3, 1.8)
            return "EG+"
    if 160 <= y < 194:
        if x < 100:
            params["SLOPE_GAIN"] = clamp(params["SLOPE_GAIN"] - 0.05, 0.2, 2.0)
            return "SG-"
        if x > 220:
            params["SLOPE_GAIN"] = clamp(params["SLOPE_GAIN"] + 0.05, 0.2, 2.0)
            return "SG+"
    if 206 <= y < 240:
        if x < 100:
            params["BASE_SPEED"] = clamp(params["BASE_SPEED"] - 1, 0, 30)
            return "SP-"
        if x > 220:
            params["BASE_SPEED"] = clamp(params["BASE_SPEED"] + 1, 0, 30)
            return "SP+"
        if 100 <= x <= 220:
            return "SAVED" if save_params(params) else "FAIL"
    return save_msg


def draw_debug(img, path, running, fps, params, save_msg, link_mode):
    colors = [GREEN, BLUE, WHITE, RED]
    ys = [206, 146, 86, 32]
    for layer, color in zip(LAYERS, colors):
        for roi in layer:
            img.draw_rectangle(roi[:4], color=color, thickness=1)
    for blob in path["blobs"]:
        img.draw_rectangle(blob.rect(), color=YELLOW, thickness=1)
    for blob in path.get("green_blobs", []):
        img.draw_rectangle(blob.rect(), color=GREEN, thickness=2)
    for span in path["spans"]:
        if span is not None:
            img.draw_rectangle(span, color=YELLOW, thickness=2)
    for cx, y, color in zip(path["centers"], ys, colors):
        if cx is not None:
            img.draw_cross(int(cx), y, color=color, thickness=2)

    img.draw_line(IMG_CENTER_X, 0, IMG_CENTER_X, IMG_H - 1,
                  color=WHITE, thickness=1)
    if path.get("target_x") is not None:
        img.draw_line(int(path["target_x"]), 0, int(path["target_x"]),
                      IMG_H - 1, color=RED, thickness=2)

    title = "RUN" if running else "STOP"
    if path.get("lost_search"):
        title = "SEARCH"
    lmax = path.get("lmax", [0, 0])
    l0 = lmax[0] if len(lmax) > 0 else 0
    l1 = lmax[1] if len(lmax) > 1 else 0
    img.draw_string(0, 0, "%s fps:%2.1f e:%d s:%d c:%d g:%d" %
                    (title, fps, path["error"], path["slope"],
                     path["confidence"], path.get("green_bias", 0)),
                    color=GREEN, scale=1)
    img.draw_string(0, 12, "uart:%s sp:%d L:%d/%d" %
                    (link_mode, int(params["BASE_SPEED"]), l0, l1),
                    color=WHITE, scale=1)

    if not running:
        rows = [
            ("LO", "LOCAL_L_OFFSET", 18, "%d"),
            ("GA", "ROAD_A_MIN", 54, "%d"),
            ("GP", "MAX_ROAD_GAP", 90, "%d"),
            ("EG", "ERROR_GAIN", 126, "%1.2f"),
            ("SG", "SLOPE_GAIN", 162, "%1.2f"),
            ("SP", "BASE_SPEED", 206, "%d"),
        ]
        for name, key, y, fmt in rows:
            img.draw_rectangle((0, y, 100, 22), color=BLUE, thickness=1)
            img.draw_string(10, y + 7, name + "-", color=WHITE, scale=1)
            value = params[key]
            if fmt == "%d":
                value = int(value)
            img.draw_string(116, y + 7, ("%s:" + fmt) % (name, value),
                            color=WHITE, scale=1)
            img.draw_rectangle((220, y, 100, 22), color=BLUE, thickness=1)
            img.draw_string(250, y + 7, name + "+", color=WHITE, scale=1)
        img.draw_rectangle((110, 206, 100, 34), color=GREEN, thickness=1)
        img.draw_string(132, 218, "SAVE", color=WHITE, scale=1)
        if save_msg:
            img.draw_string(110, 218, save_msg, color=GREEN, scale=1)


def main():
    init_camera()
    key = KeyButton()
    link = Stm32Link()
    clock = time.clock()
    params = PARAMS.copy()
    load_params(params)

    running = False
    last_send = time.ticks_ms()
    display_count = 0
    save_msg = ""
    save_msg_until = 0
    last_path_x = None
    last_valid_path = None
    last_turn_dir = 0

    while True:
        try:
            clock.tick()
            img = sensor.snapshot()
            now = time.ticks_ms()

            if key.pressed_event():
                running = not running
                if not running:
                    link.send_vision(0, 0, 0, 0)

            if not running:
                msg = touch_params(params, save_msg)
                if msg != save_msg:
                    save_msg = msg
                    save_msg_until = now + 900
            if save_msg and time.ticks_diff(now, save_msg_until) > 0:
                save_msg = ""

            path = detect_path(img, params, last_path_x)
            if path["seen"] and path["target_x"] is not None:
                last_path_x = path["target_x"]
                turn_dir = 0
                if path["slope"] > 10:
                    turn_dir = 1
                elif path["slope"] < -10:
                    turn_dir = -1
                elif path["error"] > 16:
                    turn_dir = 1
                elif path["error"] < -16:
                    turn_dir = -1
                if turn_dir != 0:
                    last_turn_dir = turn_dir
                else:
                    turn_dir = last_turn_dir
                last_valid_path = {
                    "time": now,
                    "dir": turn_dir,
                }
            elif running:
                search = make_lost_search(last_valid_path, params, now)
                if search is not None:
                    path = search

            if running and time.ticks_diff(now, last_send) >= CMD_PERIOD_MS:
                if path["target_x"] is not None:
                    link.send_vision(path["error"], path["slope"],
                                     path["confidence"],
                                     params["BASE_SPEED"])
                else:
                    link.send_vision(0, 0, 0, 0)
                last_send = now

            show_display = DEBUG_MODE and (DISPLAY_WHILE_RUNNING or
                                           (not running))
            display_count += 1
            if not show_display:
                display_count = 0
            if show_display and display_count >= DISPLAY_EVERY_N:
                display_count = 0
                draw_debug(img, path, running, clock.fps(), params, save_msg,
                           link.mode)
                lcd.display(img)
        except Exception as err:
            try:
                link.send_vision(0, 0, 0, 0)
                img.draw_string(0, 44, "ERR:%s" % err, color=RED, scale=1)
                lcd.display(img)
            except Exception:
                pass
            running = False


main()
