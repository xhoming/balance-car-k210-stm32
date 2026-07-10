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

THRESHOLD_UPDATE_EVERY_N = 4
CMD_PERIOD_MS = 10
KEY_DEBOUNCE_MS = 250

# 13 strips, 5 px high. Index 0 is nearest/bottom, index 12 is farther/top.
SCAN_ROIS = [
    (12, 218, 296, 5, 8),
    (12, 202, 296, 5, 8),
    (12, 186, 296, 5, 9),
    (12, 170, 296, 5, 10),
    (12, 154, 296, 5, 11),
    (12, 138, 296, 5, 12),
    (12, 122, 296, 5, 12),
    (12, 106, 296, 5, 12),
    (12, 90, 296, 5, 13),
    (12, 74, 296, 5, 13),
    (12, 58, 296, 5, 12),
    (12, 42, 296, 5, 11),
    (12, 26, 296, 5, 10),
]

BLACK_AB_LIMIT = 42
LOCAL_L_MIN = 20
LOCAL_L_MAX = 88
MIN_PIXELS = 45
MIN_AREA = 45
MIN_ROAD_W = 20
MIN_VALID_BANDS = 3
PATH_MAX_JUMP = 220
FIT_NEAR_Y = 194
FIT_FAR_Y = 42
LOOKAHEAD_ORDER = (7, 6, 8, 5, 9, 4, 3, 2, 1, 0)
CONTINUITY_MARGIN_MIN = 32
CONTINUITY_JUMP_MIN = 42
ROBUST_RESIDUAL_STRAIGHT = 34
ROBUST_RESIDUAL_CURVE = 48
SLOPE_BOOST_START_PX = 18
SLOPE_BOOST_NUM = 3
SLOPE_BOOST_DEN = 2
MIN_RUN_SPEED = 10

PARAM_FILE = "/sd/road_sensor_simple.txt"
PARAMS = {
    "LO": 15,      # local L offset. Bigger means stricter/darker road.
    "GA": -6,      # road A min. Bigger filters green harder.
    "GP": 58,      # max gap for merging nearby road pieces.
    "DZ": 8,       # center dead zone in pixels. Inside it, turn is 0.
    "EG": 0.78,    # error gain sent to STM32.
    "SG": 0.90,    # slope gain sent to STM32.
    "SP": 15,      # fixed run speed sent to STM32.
}

WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
YELLOW = (255, 220, 0)
BLUE = (0, 110, 255)


def clamp(v, low, high):
    if v < low:
        return low
    if v > high:
        return high
    return v


def load_params(params):
    try:
        with open(PARAM_FILE, "r") as f:
            for line in f:
                if "=" not in line:
                    continue
                k, v = line.strip().split("=", 1)
                if k in params:
                    params[k] = float(v)
    except Exception:
        pass


def save_params(params):
    try:
        with open(PARAM_FILE, "w") as f:
            for k in sorted(params.keys()):
                f.write("%s=%s\n" % (k, params[k]))
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

    def send(self, error, slope, confidence, speed):
        error = int(clamp(error, -100, 100))
        slope = int(clamp(slope, -100, 100))
        confidence = int(clamp(confidence, 0, 100))
        speed = int(clamp(speed, 0, 30))
        error_u = error & 0xFF
        slope_u = slope & 0xFF
        speed_u = speed & 0xFF
        chk = (error_u + slope_u + confidence + speed_u) & 0xFF
        frame = [HEAD, error_u, slope_u, confidence, speed_u, chk, TAIL]
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


def make_road_threshold(l_max, params):
    l_max = int(clamp(l_max, LOCAL_L_MIN, LOCAL_L_MAX))
    a_min = int(clamp(params["GA"], -35, 20))
    return [(0, l_max, a_min, BLACK_AB_LIMIT, -BLACK_AB_LIMIT, BLACK_AB_LIMIT)]


def stats_lmax(img, roi, params):
    try:
        stats = img.get_statistics(roi=roi)
        l_max = int(stats.l_max() - params["LO"] - 10)
    except Exception:
        l_max = 55
    return int(clamp(l_max, LOCAL_L_MIN, LOCAL_L_MAX))


def build_threshold_cache(img, params):
    low_l = stats_lmax(img, (12, 162, 296, 64), params)
    mid_l = stats_lmax(img, (12, 88, 296, 70), params)
    top_l = stats_lmax(img, (12, 22, 296, 54), params)
    return [
        make_road_threshold(low_l, params),
        make_road_threshold(mid_l, params),
        make_road_threshold(top_l, params),
    ]


def threshold_for_index(cache, index):
    if index <= 3:
        return cache[0]
    if index <= 8:
        return cache[1]
    return cache[2]


def find_road(img, roi, params, expected_x, threshold):
    x, y, w, h, _ = roi
    try:
        blobs = img.find_blobs(threshold,
                               roi=(x, y, w, h),
                               pixels_threshold=MIN_PIXELS,
                               area_threshold=MIN_AREA,
                               merge=True,
                               margin=8)
    except Exception:
        return None, None

    blobs.sort(key=lambda b: b.x())
    groups = []
    max_gap = int(clamp(params["GP"], 20, 120))
    for b in blobs:
        if b.w() < MIN_ROAD_W:
            continue
        left = b.x()
        right = b.x() + b.w()
        pixels = b.pixels()
        if groups and left - groups[-1][1] <= max_gap:
            if right > groups[-1][1]:
                groups[-1][1] = right
            groups[-1][2] += pixels
        else:
            groups.append([left, right, pixels])

    best = None
    best_score = -100000
    for left, right, pixels in groups:
        center = (left + right) // 2
        if abs(center - expected_x) > PATH_MAX_JUMP:
            continue
        width = right - left
        score = pixels + width * 4 - abs(center - expected_x) * 5
        if score > best_score:
            best_score = score
            best = (left, right)

    if best is None:
        return None, None
    left, right = best
    return (left + right) // 2, (int(left), y, int(right - left), h)


def fit_center_line(centers):
    sum_w = 0
    sum_y = 0
    sum_x = 0
    sum_yy = 0
    sum_yx = 0

    for i, cx in enumerate(centers):
        if cx is None:
            continue
        _, y, _, h, weight = SCAN_ROIS[i]
        cy = y + h // 2
        sum_w += weight
        sum_y += cy * weight
        sum_x += cx * weight
        sum_yy += cy * cy * weight
        sum_yx += cy * cx * weight

    if sum_w <= 0:
        return None, None

    denom = sum_w * sum_yy - sum_y * sum_y
    if denom == 0:
        a = 0.0
    else:
        a = (sum_w * sum_yx - sum_y * sum_x) / denom
    b = (sum_x - a * sum_y) / sum_w
    return a, b


def count_valid_centers(centers):
    count = 0
    for cx in centers:
        if cx is not None:
            count += 1
    return count


def roi_center_y(index):
    _, y, _, h, _ = SCAN_ROIS[index]
    return y + h // 2


def choose_track_y(centers):
    for index in LOOKAHEAD_ORDER:
        if index < len(centers) and centers[index] is not None:
            return roi_center_y(index)
    return FIT_NEAR_Y


def span_continuous(span, cx, prev_span, prev_cx):
    if span is None or cx is None:
        return False
    if prev_span is None or prev_cx is None:
        return True

    left = span[0]
    right = span[0] + span[2]
    prev_left = prev_span[0]
    prev_right = prev_span[0] + prev_span[2]
    width = span[2]
    prev_width = prev_span[2]

    margin = max(CONTINUITY_MARGIN_MIN, min(width, prev_width) // 3)
    if right < prev_left - margin:
        return False
    if left > prev_right + margin:
        return False

    jump_limit = max(CONTINUITY_JUMP_MIN,
                     int(max(width, prev_width) * 0.55))
    if abs(cx - prev_cx) > jump_limit:
        return False
    return True


def robust_refit_centers(centers):
    line_a, line_b = fit_center_line(centers)
    if line_a is None:
        return centers, None, None

    near_x = line_a * FIT_NEAR_Y + line_b
    far_x = line_a * FIT_FAR_Y + line_b
    residual_limit = ROBUST_RESIDUAL_STRAIGHT
    if abs(far_x - near_x) > 45:
        residual_limit = ROBUST_RESIDUAL_CURVE

    refined = []
    for i, cx in enumerate(centers):
        if cx is None:
            refined.append(None)
            continue
        _, y, _, h, _ = SCAN_ROIS[i]
        cy = y + h // 2
        pred_x = line_a * cy + line_b
        if abs(cx - pred_x) > residual_limit:
            refined.append(None)
        else:
            refined.append(cx)

    if count_valid_centers(refined) < MIN_VALID_BANDS:
        return centers, line_a, line_b

    line_a2, line_b2 = fit_center_line(refined)
    if line_a2 is None:
        return centers, line_a, line_b
    return refined, line_a2, line_b2


def dynamic_speed(params, error, slope, confidence):
    base = int(clamp(params["SP"], 0, 30))
    if base <= 0 or confidence < 15:
        return 0

    turn = max(abs(error), abs(slope))
    speed = base - int(turn * 0.16)

    if turn >= 70:
        speed = min(speed, 8)
    elif turn >= 45:
        speed = min(speed, 12)
    elif turn >= 25:
        speed = min(speed, 16)

    if confidence < 35:
        speed = min(speed, 8)
    elif confidence < 55:
        speed = min(speed, 12)

    return int(clamp(speed, MIN_RUN_SPEED, base))


def detect_path(img, params, last_x, threshold_cache):
    expected_x = last_x if last_x is not None else IMG_CENTER_X
    centers = []
    spans = []
    prev_span = None
    prev_cx = None

    for i, roi in enumerate(SCAN_ROIS):
        cx, span = find_road(img, roi, params, expected_x,
                             threshold_for_index(threshold_cache, i))
        if cx is not None:
            if span_continuous(span, cx, prev_span, prev_cx):
                expected_x = cx
                prev_cx = cx
                prev_span = span
            else:
                cx = None
        centers.append(cx)
        spans.append(span)

    valid = count_valid_centers(centers)
    if valid < MIN_VALID_BANDS:
        return {
            "seen": False, "centers": centers, "spans": spans,
            "target_x": None,
            "error": 0, "slope": 0, "confidence": 0, "speed": 0,
            "roadw": 0, "line": None,
        }

    centers, line_a, line_b = robust_refit_centers(centers)
    if line_a is None:
        return {
            "seen": False, "centers": centers, "spans": spans,
            "target_x": None,
            "error": 0, "slope": 0, "confidence": 0, "speed": 0,
            "roadw": 0, "line": None,
        }

    valid = 0
    width_sum = 0
    for i, cx in enumerate(centers):
        if cx is None:
            continue
        valid += 1
        if spans[i] is not None:
            width_sum += spans[i][2]

    track_y = choose_track_y(centers)
    track_x = line_a * track_y + line_b
    far_x = line_a * FIT_FAR_Y + line_b
    target_x = int(clamp(track_x, 0, IMG_W - 1))
    line = (int(clamp(far_x, 0, IMG_W - 1)), FIT_FAR_Y,
            int(clamp(track_x, 0, IMG_W - 1)), int(track_y))
    error_px = target_x - IMG_CENTER_X
    if abs(error_px) <= int(clamp(params["DZ"], 0, 30)):
        error_px = 0
    error = int(error_px * 100 / IMG_CENTER_X * params["EG"])
    error = int(clamp(error, -100, 100))

    slope_px = far_x - track_x
    if abs(slope_px) > SLOPE_BOOST_START_PX:
        slope_px = slope_px * SLOPE_BOOST_NUM / SLOPE_BOOST_DEN
    if abs(slope_px) <= int(clamp(params["DZ"], 0, 30)):
        slope_px = 0
    slope = int(slope_px * 100 / IMG_CENTER_X * params["SG"])
    slope = int(clamp(slope, -100, 100))
    confidence = int(clamp(valid * 8, 0, 100))
    speed = dynamic_speed(params, error, slope, confidence)

    return {
        "seen": True, "centers": centers, "spans": spans,
        "target_x": target_x,
        "error": error, "slope": slope, "confidence": confidence,
        "speed": speed,
        "roadw": int(width_sum / valid) if valid else 0,
        "line": line,
    }


def touch_params(params, msg):
    if not ts:
        return msg
    try:
        status, x, y = ts.read()
    except Exception:
        return msg
    if status != ts.STATUS_PRESS:
        return msg

    rows = [
        ("LO", 24, 50, 1, 5, 35),
        ("GA", 52, 78, 1, -35, 20),
        ("GP", 80, 106, 2, 20, 120),
        ("DZ", 108, 134, 1, 0, 30),
        ("EG", 136, 158, 0.05, 0.3, 1.8),
        ("SG", 160, 182, 0.05, 0.0, 2.0),
        ("SP", 184, 206, 1, 0, 30),
    ]
    for key, y0, y1, step, low, high in rows:
        if y0 <= y < y1:
            if x < 96:
                params[key] = clamp(params[key] - step, low, high)
                return key + "-"
            if x > 224:
                params[key] = clamp(params[key] + step, low, high)
                return key + "+"
    if 208 <= y < 235 and 96 <= x <= 224:
        return "SAVED" if save_params(params) else "FAIL"
    return msg


def draw_debug(img, path, running, fps, params, msg):
    if not running:
        for roi in SCAN_ROIS:
            img.draw_rectangle(roi[:4], color=BLUE, thickness=1)
        for span in path["spans"]:
            if span:
                img.draw_rectangle(span, color=YELLOW, thickness=1)

    img.draw_line(IMG_CENTER_X, 0, IMG_CENTER_X, IMG_H - 1,
                  color=WHITE, thickness=1)
    for i, cx in enumerate(path["centers"]):
        if cx is None:
            continue
        _, y, _, h, _ = SCAN_ROIS[i]
        img.draw_cross(int(cx), y + h // 2, color=YELLOW, thickness=1)
    if path["line"] is not None:
        x0, y0, x1, y1 = path["line"]
        img.draw_line(x0, y0, x1, y1, color=RED, thickness=2)
        img.draw_cross(x1, y1, color=RED, thickness=2)

    img.draw_string(0, 0, "%s fps:%2.1f e:%d s:%d c:%d" %
                    ("RUN" if running else "STOP", fps,
                     path["error"], path["slope"], path["confidence"]),
                    color=GREEN, scale=1)
    img.draw_string(0, 12, "sp:%d w:%d" %
                    (path["speed"], path["roadw"]),
                    color=WHITE, scale=1)

    if running:
        return

    rows = [("LO", 24), ("GA", 52), ("GP", 80), ("DZ", 108),
            ("EG", 136), ("SG", 160), ("SP", 184)]
    for key, y in rows:
        img.draw_rectangle((0, y, 90, 24), color=BLUE, thickness=1)
        img.draw_string(10, y + 8, key + "-", color=WHITE, scale=1)
        if key == "EG" or key == "SG":
            text = "%s:%1.2f" % (key, params[key])
        else:
            text = "%s:%d" % (key, int(params[key]))
        img.draw_string(112, y + 8, text, color=WHITE, scale=1)
        img.draw_rectangle((230, y, 90, 24), color=BLUE, thickness=1)
        img.draw_string(256, y + 8, key + "+", color=WHITE, scale=1)
    img.draw_rectangle((98, 208, 124, 26), color=GREEN, thickness=1)
    img.draw_string(138, 216, "SAVE", color=WHITE, scale=1)
    if msg:
        img.draw_string(110, 226, msg, color=GREEN, scale=1)


def main():
    init_camera()
    key = KeyButton()
    link = Stm32Link()
    clock = time.clock()
    params = PARAMS.copy()
    load_params(params)

    running = False
    last_send = time.ticks_ms()
    msg = ""
    msg_until = 0
    last_x = None
    threshold_cache = None
    threshold_count = 0

    while True:
        try:
            clock.tick()
            img = sensor.snapshot()
            now = time.ticks_ms()

            if key.pressed_event():
                running = not running
                if not running:
                    link.send(0, 0, 0, 0)

            if not running:
                new_msg = touch_params(params, msg)
                if new_msg != msg:
                    msg = new_msg
                    msg_until = now + 900
                    threshold_cache = None
            if msg and time.ticks_diff(now, msg_until) > 0:
                msg = ""

            if threshold_cache is None or threshold_count <= 0:
                threshold_cache = build_threshold_cache(img, params)
                threshold_count = THRESHOLD_UPDATE_EVERY_N - 1
            else:
                threshold_count -= 1

            path = detect_path(img, params, last_x, threshold_cache)
            if path["target_x"] is not None:
                last_x = path["target_x"]

            if running and time.ticks_diff(now, last_send) >= CMD_PERIOD_MS:
                if path["seen"]:
                    link.send(path["error"], path["slope"],
                              path["confidence"], path["speed"])
                else:
                    link.send(0, 0, 0, 0)
                last_send = now

            draw_debug(img, path, running, clock.fps(), params, msg)
            lcd.display(img)
        except Exception as err:
            try:
                link.send(0, 0, 0, 0)
                img.draw_string(0, 44, "ERR:%s" % err, color=RED, scale=1)
                lcd.display(img)
            except Exception:
                pass
            running = False


main()
