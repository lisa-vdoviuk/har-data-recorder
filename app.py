#!/usr/bin/env python3
"""
MetaWear Recorder - local web app for recording data from several MbientLab sensors.
Run:  python app.py   ->   http://127.0.0.1:8000
"""
import bisect
import collections
import csv
import io
import json
import re
import shutil
import statistics
import time
import zipfile
from ctypes import byref, cast
from datetime import datetime
from pathlib import Path
from threading import Event, Lock, RLock, Thread

from flask import Flask, abort, jsonify, request, send_file, send_from_directory

from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *  # noqa: F401,F403
from mbientlab.warble import *  # noqa: F401,F403  (BleScanner)

BASE = Path(__file__).resolve().parent
REC_DIR = BASE / "recordings"
REC_DIR.mkdir(exist_ok=True)
CFG_FILE = BASE / "sensors.json"
PORT = 8000
DOWNLOAD_IDLE_TIMEOUT = 45
LOG_READY_TIMEOUT = 25

OPTIONS = {
    "acc_odr": [12.5, 25, 50, 100, 200, 400, 800, 1600],   # Hz
    "acc_range": [2, 4, 8, 16],                            # g
    "gyro_odr": [25, 50, 100, 200, 400, 800, 1600, 3200],  # Hz
    "gyro_range": [125, 250, 500, 1000, 2000],             # deg/s
}

app = Flask(__name__, static_folder=str(BASE / "static"), static_url_path="/static")
lock = RLock()


# --------------------------------------------------------------------------- #
#  Config (sensor labels, movement history)
# --------------------------------------------------------------------------- #
def load_cfg():
    try:
        return json.loads(CFG_FILE.read_text("utf-8"))
    except Exception:
        return {"sensors": {}, "movements": []}


CFG = load_cfg()


def save_cfg():
    with lock:
        CFG["sensors"] = {
            m: {"label": s.label, "name": s.name, "selected": s.selected}
            for m, s in SENSORS.items()
        }
        CFG_FILE.write_text(json.dumps(CFG, ensure_ascii=False, indent=2), "utf-8")


def slug(text, default="x"):
    t = re.sub(r"[^\w\-]+", "_", str(text), flags=re.UNICODE).strip("_")
    return t[:40] or default


def par(fn, items):
    """Run fn for every item in parallel."""
    ts = [Thread(target=fn, args=(i,), daemon=True) for i in items]
    for t in ts:
        t.start()
    for t in ts:
        t.join()


# --------------------------------------------------------------------------- #
#  Sensor
# --------------------------------------------------------------------------- #
class Sensor:
    def __init__(self, mac, name="MetaWear"):
        self.mac = mac
        self.name = name
        self.rssi = None
        self.nearby = False
        self.label = ""
        self.selected = True
        self.state = "disconnected"  # disconnected | connecting | connected | error
        self.error = None
        self.identifying = False
        self.rec_state = "idle"
        self.progress = 0.0
        self.device = None
        self.board = None
        self.loggers = None
        self.logger_ids = {}
        self.battery = None
        self.dl_left, self.dl_total, self.dl_t = 0, 0, 0.0
        self.dl_ok = False
        self.dl_samples = 0
        self.dl_done = None
        self._bat_sub, self._bat_cb = False, None
        self.live, self._live_keep = False, []
        self.acc = []
        self.gyro = []
        self._keep = []
        self._download_keep = []
        self._length_cb = None
        self.op = Lock()

    # ---- connection ---- #
    def connect(self):
        with self.op:
            self.state, self.error = "connecting", None
            try:
                self.device = MetaWear(self.mac)
                self.device.connect()
                self.board = self.device.board
                self.loggers = None
                if self.logger_ids:
                    loggers = {name: (libmetawear.mbl_mw_logger_lookup_id(self.board, logger_id), None)
                               for name, logger_id in self.logger_ids.items()}
                    if all(logger for logger, _ in loggers.values()):
                        self.loggers = loggers
                if self.dl_done is not None and not self.dl_done.is_set():
                    self.dl_done = None
                self._bat_sub = False
                try:
                    self.device.on_disconnect = lambda status: self._lost()
                except Exception:
                    pass
                self.state = "connected"
            except Exception as e:
                self.state, self.error = "error", str(e) or "Could not connect"
                return
        self.read_battery()
        self.identify()

    def _lost(self):
        if self.state == "connected":
            self.state, self.error = "disconnected", "Connection lost"
            self.loggers = None

    def disconnect(self):
        with self.op:
            try:
                if self.device:
                    self.device.disconnect()
            except Exception:
                pass
            self.state, self.error, self.loggers = "disconnected", None, None

    def identify(self):
        """Vibration (if a motor exists) + blue LED blink for ~2.5 s."""
        if self.state != "connected" or self.identifying:
            return
        self.identifying = True
        b = self.board
        try:
            try:
                p = LedPattern()
                libmetawear.mbl_mw_led_load_preset_pattern(byref(p), LedPreset.BLINK)
                p.pulse_duration_ms, p.high_time_ms, p.repeat_count = 400, 200, 6
                libmetawear.mbl_mw_led_write_pattern(b, byref(p), LedColor.BLUE)
                libmetawear.mbl_mw_led_play(b)
            except Exception:
                pass
            for _ in range(3):
                try:
                    libmetawear.mbl_mw_haptic_start_motor(b, 100.0, 500)
                except Exception:
                    break
                time.sleep(0.8)
            time.sleep(0.2)
            try:
                libmetawear.mbl_mw_led_stop_and_clear(b)
            except Exception:
                pass
        finally:
            self.identifying = False

    # ---- loggers ---- #
    def _make_logger(self, signal):
        ev, res = Event(), {}

        def cb(ctx, logger):
            res["l"] = logger
            ev.set()

        fn = FnVoid_VoidP_VoidP(cb)
        self._keep.append(fn)
        libmetawear.mbl_mw_datasignal_log(signal, None, fn)
        if not ev.wait(10):
            raise RuntimeError("Sensor did not respond to logger creation. Reconnect it and retry")
        if not res.get("l"):
            raise RuntimeError("Sensor could not allocate a logger. Its logging slots may be full")
        self._keep.remove(fn)
        return res["l"], fn

    def _ensure_loggers(self):
        if self.loggers and all(self.loggers.get(name) for name in ("acc", "gyro")):
            return
        if has_pending(self.mac):
            raise RuntimeError("Reconnect this sensor to restore the pending recording loggers")
        board = self.board
        # Query the actual board, including triggers left by an earlier app run.
        # This also rebuilds the SDK logger registry from the device.
        ready, result = Event(), {}

        def discovered(ctx, device, signals, count):
            result["ok"] = bool(signals) or count == 0
            result["status"] = count
            ready.set()

        callback = FnVoid_VoidP_VoidP_VoidP_UInt(discovered)
        self._keep.append(callback)
        libmetawear.mbl_mw_metawearboard_create_anonymous_datasignals(board, None, callback)
        if not ready.wait(40):
            raise RuntimeError("Sensor did not respond while reading existing loggers. Reconnect it and retry")
        if not result["ok"]:
            raise RuntimeError(f"Could not read existing sensor loggers (SDK status {result['status']})")
        self._keep.remove(callback)
        signals = {
            "acc": libmetawear.mbl_mw_acc_get_acceleration_data_signal(board),
            "gyro": libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal(board),
        }
        self.loggers = {}
        for logger_id in range(32):
            logger = libmetawear.mbl_mw_logger_lookup_id(board, logger_id)
            if not logger:
                continue
            source = libmetawear.mbl_mw_logger_get_signal(logger)
            for name, signal in signals.items():
                if source == signal:
                    self.loggers.setdefault(name, (logger, None))
        for name, signal in signals.items():
            if not signal:
                raise RuntimeError(f"The sensor does not provide the requested {name} channel")
            if name not in self.loggers:
                # Retain each successful logger even if the next creation fails.
                self.loggers[name] = self._make_logger(signal)
        self.logger_ids = {name: libmetawear.mbl_mw_logger_get_id(logger)
                           for name, (logger, _) in self.loggers.items()}
        self.device.serialize()

    # ---- sensor configuration ---- #
    def _cfg_acc(self, odr, rng):
        b = self.board
        libmetawear.mbl_mw_acc_set_odr(b, float(odr))
        libmetawear.mbl_mw_acc_set_range(b, float(rng))
        libmetawear.mbl_mw_acc_write_acceleration_config(b)

    def _cfg_gyro(self, odr, rng):
        b = self.board
        libmetawear.mbl_mw_gyro_bmi270_set_range(b, getattr(GyroBoschRange, f"_{int(rng)}dps"))
        libmetawear.mbl_mw_gyro_bmi270_set_odr(b, getattr(GyroBoschOdr, f"_{int(odr)}Hz"))
        libmetawear.mbl_mw_gyro_bmi270_write_config(b)

    # ---- battery ---- #
    def read_battery(self):
        """Ask the sensor for its battery state; the answer arrives asynchronously."""
        if self.state != "connected":
            return
        try:
            sig = libmetawear.mbl_mw_settings_get_battery_state_data_signal(self.board)
            if not self._bat_sub:
                def cb(ctx, p):
                    v = parse_value(p)
                    self.battery = {"charge": int(v.charge), "voltage": int(v.voltage)}
                self._bat_cb = FnVoid_VoidP_DataP(cb)
                libmetawear.mbl_mw_datasignal_subscribe(sig, None, self._bat_cb)
                self._bat_sub = True
            libmetawear.mbl_mw_datasignal_read(sig)
        except Exception:
            pass

    # ---- live preview (streaming, nothing is stored) ---- #
    def live_start(self, use_acc, use_gyro, odr, acc_range, gyro_range):
        b = self.board
        self._live_keep = []
        if use_acc:
            self._cfg_acc(odr, acc_range)
            sig = libmetawear.mbl_mw_acc_get_acceleration_data_signal(b)
            cb = FnVoid_VoidP_DataP(lambda ctx, p: live_push("acc", p))
            libmetawear.mbl_mw_datasignal_subscribe(sig, None, cb)
            libmetawear.mbl_mw_acc_enable_acceleration_sampling(b)
            libmetawear.mbl_mw_acc_start(b)
            self._live_keep.append(cb)
        if use_gyro:
            self._cfg_gyro(odr, gyro_range)
            sig = libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal(b)
            cb = FnVoid_VoidP_DataP(lambda ctx, p: live_push("gyro", p))
            libmetawear.mbl_mw_datasignal_subscribe(sig, None, cb)
            libmetawear.mbl_mw_gyro_bmi270_enable_rotation_sampling(b)
            libmetawear.mbl_mw_gyro_bmi270_start(b)
            self._live_keep.append(cb)
        self.live = True

    def live_stop(self):
        b = self.board
        self.live = False
        for fn in (libmetawear.mbl_mw_acc_stop, libmetawear.mbl_mw_gyro_bmi270_stop,
                   libmetawear.mbl_mw_acc_disable_acceleration_sampling,
                   libmetawear.mbl_mw_gyro_bmi270_disable_rotation_sampling):
            try:
                fn(b)
            except Exception:
                pass
        for getter in (libmetawear.mbl_mw_acc_get_acceleration_data_signal,
                       libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal):
            try:
                libmetawear.mbl_mw_datasignal_unsubscribe(getter(b))
            except Exception:
                pass

    # ---- recording ---- #
    def configure(self, s):
        self.rec_state, self.error, self.progress = "configuring", None, 0.0
        try:
            b = self.board
            if s.get("use_acc", True):
                self._cfg_acc(s["acc_odr"], s["acc_range"])
            if s.get("use_gyro", True):
                self._cfg_gyro(s["gyro_odr"], s["gyro_range"])
            self._ensure_loggers()
            # MMS flash garbage collection after a clear can silently prevent logging.
            # A normal readout already consumes the entries. Only clear on explicit discard.
            self.rec_state = "ready"
        except Exception as e:
            self.rec_state, self.error = "error", f"Configuration failed: {e}"

    def start(self, s):
        if self.rec_state != "ready":
            return
        try:
            b = self.board
            libmetawear.mbl_mw_logging_start(b, 0)
            if s.get("use_acc", True):
                libmetawear.mbl_mw_acc_enable_acceleration_sampling(b)
                libmetawear.mbl_mw_acc_start(b)
            if s.get("use_gyro", True):
                libmetawear.mbl_mw_gyro_bmi270_enable_rotation_sampling(b)
                libmetawear.mbl_mw_gyro_bmi270_start(b)
            self.rec_state = "recording"
        except Exception as e:
            self.rec_state, self.error = "error", f"Start failed: {e}"

    def stop(self):
        if self.rec_state != "recording":
            return
        self.rec_state = "stopping"
        b = self.board
        for fn in (
            libmetawear.mbl_mw_acc_stop,
            libmetawear.mbl_mw_gyro_bmi270_stop,
            libmetawear.mbl_mw_acc_disable_acceleration_sampling,
            libmetawear.mbl_mw_gyro_bmi270_disable_rotation_sampling,
            libmetawear.mbl_mw_logging_stop,
        ):
            try:
                fn(b)
            except Exception:
                pass
        try:
            libmetawear.mbl_mw_logging_flush_page(b)  # MetaMotionS
        except Exception:
            pass
        self.rec_state = "stored"

    def _wait_for_log(self):
        """The final NAND page on MetaMotion S takes time to become readable."""
        signal = libmetawear.mbl_mw_logging_get_length_data_signal(self.board)
        if not signal:
            raise RuntimeError("Sensor log length is unavailable")
        received = Event()
        length = None

        def on_length(ctx, data):
            nonlocal length
            length = parse_value(data)
            received.set()

        self._length_cb = FnVoid_VoidP_DataP(on_length)
        libmetawear.mbl_mw_datasignal_subscribe(signal, None, self._length_cb)
        try:
            libmetawear.mbl_mw_logging_flush_page(self.board)
            deadline = time.monotonic() + LOG_READY_TIMEOUT
            previous = None
            while time.monotonic() < deadline:
                if self.state != "connected":
                    raise RuntimeError("Connection lost while checking sensor memory")
                received.clear()
                libmetawear.mbl_mw_datasignal_read(signal)
                if received.wait(2):
                    if length >= 2 and length == previous:
                        return
                    previous = length
                    time.sleep(2)
                else:
                    previous = None
            if length is None:
                raise RuntimeError("Sensor did not respond to the log length request")
            raise RuntimeError("Sensor memory did not report a stable, non-empty log")
        finally:
            libmetawear.mbl_mw_datasignal_unsubscribe(signal)

    def _start_readout(self):
        self.dl_done = done = Event()
        self.dl_left, self.dl_total = 0, 0

        def data_handler(store):
            # An interrupted page may be sent again on reconnect.
            seen = collections.Counter(store)

            def receive(ctx, data):
                if self.dl_done is not done or done.is_set():
                    return
                self.dl_t = time.monotonic()
                value = parse_value(data)
                sample = (data.contents.epoch, value.x, value.y, value.z)
                if seen[sample]:
                    seen[sample] -= 1
                else:
                    store.append(sample)
                    self.dl_samples += 1

            return FnVoid_VoidP_DataP(receive)

        def progress(ctx, left, total):
            if self.dl_done is not done or done.is_set():
                return
            if (left, total) != (self.dl_left, self.dl_total):
                self.dl_t = time.monotonic()
            self.dl_left, self.dl_total = left, total
            self.progress = max(0.0, (total - left) / total) if total else 0.0
            if left == 0:
                done.set()

        cb_a, cb_g = data_handler(self.acc), data_handler(self.gyro)
        prog = FnVoid_VoidP_UInt_UInt(progress)
        handler = LogDownloadHandler(
            context=None,
            received_progress_update=prog,
            received_unknown_entry=cast(None, FnVoid_VoidP_UByte_Long_UByteP_UByte),
            received_unhandled_entry=cast(None, FnVoid_VoidP_DataP),
        )
        libmetawear.mbl_mw_logger_subscribe(self.loggers["acc"][0], None, cb_a)
        libmetawear.mbl_mw_logger_subscribe(self.loggers["gyro"][0], None, cb_g)
        libmetawear.mbl_mw_logging_download(self.board, 100, byref(handler))
        self._download_keep = [cb_a, cb_g, prog, handler]

    def download_all(self):
        """Read one sensor, keeping received samples until its CSVs are saved."""
        self.error = None
        try:
            if self.dl_done is None:
                if self.state != "connected":
                    raise RuntimeError("Connect this sensor and press Download all again")
                if not self.loggers:
                    raise RuntimeError("The recording loggers could not be restored")
                self.rec_state, self.dl_t = "preparing", time.monotonic()
                self._wait_for_log()
                self.rec_state, self.dl_t = "downloading", time.monotonic()
                self._start_readout()
            elif not self.dl_done.is_set():
                raise RuntimeError("Previous readout stalled. Reconnect this sensor and retry")

            while not self.dl_done.wait(0.5):
                if self.state != "connected":
                    raise RuntimeError("Connection lost during transfer. Reconnect this sensor and retry")
                if time.monotonic() - self.dl_t > DOWNLOAD_IDLE_TIMEOUT:
                    raise RuntimeError("No transfer activity for 45 s. Reconnect this sensor and retry")
            if not (self.acc or self.gyro):
                if self.dl_total == 0:
                    self.dl_done = None
                raise RuntimeError("The sensor returned no readable accelerometer or gyroscope samples")
            self.progress, self.dl_ok = 1.0, True
        except Exception as e:
            self.rec_state, self.error = "error", str(e)

    def finish_download(self):
        self.dl_done, self.dl_ok, self.dl_samples = None, False, 0
        self.acc.clear()
        self.gyro.clear()

PENDING = []  # takes recorded but not yet downloaded from the sensors


LIVE = {"mac": None}
LIVE_BUF = collections.deque(maxlen=6000)
LIVE_SEQ = [0]
LIVE_LOCK = Lock()


def live_push(kind, p):
    try:
        d = parse_value(p)
        with LIVE_LOCK:
            LIVE_SEQ[0] += 1
            LIVE_BUF.append((LIVE_SEQ[0], kind, p.contents.epoch, d.x, d.y, d.z))
    except Exception:
        pass


def stop_live():
    with lock:
        mac, LIVE["mac"] = LIVE["mac"], None
    s = SENSORS.get(mac) if mac else None
    if s and s.live:
        try:
            s.live_stop()
        except Exception:
            pass


def has_pending(mac):
    return any(mac in t["sensors"] for t in PENDING)


SENSORS = {}


def get_sensor(mac, name="MetaWear"):
    mac = mac.upper()
    with lock:
        if mac not in SENSORS:
            SENSORS[mac] = Sensor(mac, name)
        return SENSORS[mac]


for _mac, _v in CFG.get("sensors", {}).items():
    s = get_sensor(_mac, _v.get("name", "MetaWear"))
    s.label, s.selected = _v.get("label", ""), _v.get("selected", True)

# --------------------------------------------------------------------------- #
#  Scanning
# --------------------------------------------------------------------------- #
SCAN = {"active": False, "error": None}
MW_SERVICE = "326a9000-85cb-9195-d9dd-464cfbbae75a"


def _scan_bleak(seconds):
    """Scan with bleak (most reliable on Windows). Returns {mac: (name, rssi)}."""
    import asyncio
    from bleak import BleakScanner

    async def run():
        res = await BleakScanner.discover(timeout=seconds, return_adv=True)
        found = {}
        for addr, (dev, adv) in res.items():
            name = dev.name or adv.local_name or ""
            uuids = [u.lower() for u in (adv.service_uuids or [])]
            if MW_SERVICE in uuids or name.lower().startswith(("metawear", "metamotion")):
                found[addr.upper()] = (name or "MetaWear", adv.rssi)
        return found

    return asyncio.run(run())


def _scan_warble(seconds):
    """Fallback: scan with the warble library bundled with the MetaWear SDK."""
    found = {}

    def handler(result):
        try:
            name = result.name or ""
            is_mw = False
            try:
                is_mw = result.has_service_uuid(MetaWear.GATT_SERVICE)
            except Exception:
                pass
            if is_mw or name.lower().startswith(("metawear", "metamotion")):
                found[result.mac.upper()] = (name or "MetaWear", getattr(result, "rssi", None))
        except Exception:
            pass

    BleScanner.set_handler(handler)
    BleScanner.start()
    time.sleep(seconds)
    BleScanner.stop()
    return found


def do_scan(seconds=5):
    SCAN["active"], SCAN["error"] = True, None
    found, errors = None, []
    for fn in (_scan_bleak, _scan_warble):
        try:
            found = fn(seconds)
            break
        except Exception as e:
            errors.append(f"{fn.__name__.replace('_scan_', '')}: {e}")
    if found is None:
        SCAN["error"] = ("Bluetooth scan failed. Make sure Bluetooth is turned on (Windows: Settings > Bluetooth) "
                         "and run 'pip install bleak'. You can still add a sensor by its MAC address. Details: "
                         + " | ".join(errors))
        found = {}
    with lock:
        if errors and not found and SCAN["error"] is None:
            pass
        for s in SENSORS.values():
            s.nearby = False
        for mac, (name, rssi) in found.items():
            s = get_sensor(mac, name)
            s.name, s.rssi, s.nearby = name, rssi, True
        save_cfg()
    SCAN["active"] = False


# --------------------------------------------------------------------------- #
#  Recording session
# --------------------------------------------------------------------------- #
REC_STOP = Event()
REC = {
    "phase": "idle",  # idle|countdown|starting|recording|stopping|downloading|saving
    "movement": "", "session": None, "t0": None, "planned": None,
    "countdown_end": None, "macs": [], "settings": {},
}
NOTICE = {"id": 0, "text": "", "level": "ok"}


def notice(text, level="ok"):
    NOTICE.update(id=NOTICE["id"] + 1, text=text, level=level)


def merge_streams(acc, gyro):
    """Merge accelerometer and gyroscope into one table (as-of join on time)."""
    acc.sort(key=lambda r: r[0])
    gyro.sort(key=lambda r: r[0])
    base_is_acc = len(acc) >= len(gyro)
    base, other = (acc, gyro) if base_is_acc else (gyro, acc)
    oe = [r[0] for r in other]
    tol = 1e9
    if len(oe) > 2:
        tol = statistics.median([b - a for a, b in zip(oe[:1000], oe[1:1001])]) * 1.5 + 1
    rows = []
    for r in base:
        i = bisect.bisect_right(oe, r[0]) - 1
        o = other[i] if i >= 0 and r[0] - oe[i] <= tol else None
        rows.append((r[0], r if base_is_acc else o, o if base_is_acc else r))
    return rows


CORE_HEAD = ["subject_id", "timestamp", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z", "activity"]
META_HEAD = ["time_s", "sensor_position", "sensor_mac", "acc_odr_hz", "acc_range_g",
             "gyro_odr_hz", "gyro_range_dps", "recording_id"]


def write_csv(path, rows, subject, activity, meta=None):
    """timestamp = Unix epoch in ms; acc in g, gyro in deg/s. `meta` adds extra columns after the core ones."""
    t0 = rows[0][0] if rows else 0
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(CORE_HEAD + (META_HEAD if meta else []))
        for epoch, a, g in rows:
            row = [subject, epoch, *(a[1:4] if a else ["", "", ""]), *(g[1:4] if g else ["", "", ""]), activity]
            if meta:
                row += [f"{(epoch - t0) / 1000:.3f}", *meta]
            w.writerow(row)


def folder_name(t):
    return f"{t['id']}_{slug(t['subject'], 'subj')}_{slug(t['movement'], 'act')}"


def split_takes(acc, gyro, takes):
    """Split one long download into per-take parts. Takes are separated by time gaps
    (sensor is not sampling between takes); falls back to host start/stop times."""
    acc.sort(key=lambda r: r[0])
    gyro.sort(key=lambda r: r[0])
    n = len(takes)
    if n <= 1:
        return [(acc, gyro)]
    ts = sorted({r[0] for r in acc} | {r[0] for r in gyro})
    cuts = []
    if len(ts) > 1:
        diffs = [b - a for a, b in zip(ts[:1000], ts[1:1001]) if b > a]
        period = statistics.median(diffs) if diffs else 10
        thr = max(300, 8 * period)
        gaps = [(ts[i + 1] - ts[i], ts[i + 1]) for i in range(len(ts) - 1) if ts[i + 1] - ts[i] > thr]
        if len(gaps) >= n - 1:
            cuts = sorted(g[1] for g in sorted(gaps, reverse=True)[: n - 1])
    if not cuts:
        cuts = [int((takes[i]["t_stop"] + takes[i + 1]["t_start"]) / 2 * 1000) for i in range(n - 1)]
    parts = [([], []) for _ in range(n)]
    for r in acc:
        parts[bisect.bisect_right(cuts, r[0])][0].append(r)
    for r in gyro:
        parts[bisect.bisect_right(cuts, r[0])][1].append(r)
    return parts


def process_pending():
    """Download all pending takes from their sensors and write one CSV per take and sensor."""
    with lock:
        takes = list(PENDING)
    if not takes:
        return
    macs = sorted({m for t in takes for m in t["sensors"]})
    sens = [SENSORS[m] for m in macs]
    with lock:
        for s in sens:
            s.rec_state, s.error, s.progress = "queued", None, 0.0
        REC["macs"] = [s.mac for s in sens]
    # Keep recording parallel, but transfer one log at a time over the Bluetooth adapter.
    for s in sens:
        s.download_all()
        if s.dl_ok:
            s.rec_state = "downloaded"

    with lock:
        REC["phase"] = "saving"
    saved, failed = 0, []
    for s in sens:
        if not s.dl_ok:
            failed.append(f"{s.label or s.mac} ({s.error})")
            continue
        s.rec_state = "saving"
        mine = [t for t in takes if s.mac in t["sensors"]]
        # Ignore old flash data left by a previous app session.
        cutoff = int(mine[0]["t_start"] * 1000) - 500
        acc = [row for row in s.acc if row[0] >= cutoff]
        gyro = [row for row in s.gyro if row[0] >= cutoff]
        try:
            files = [(t, a, g) for t, (a, g) in zip(mine, split_takes(acc, gyro, mine))
                     if not t.get("discard")]
            for t, a, g in files:
                if a or g:
                    save_take(t, s, a, g)
                    saved += 1
            if any(not (a or g) for _, a, g in files):
                raise RuntimeError("No samples found for one or more pending takes")
        except Exception as e:
            s.rec_state, s.error = "error", str(e)
            failed.append(f"{s.label or s.mac} ({s.error})")
            continue
        s.finish_download()
        s.rec_state = "done"
        with lock:
            for t in mine:
                t["sensors"].pop(s.mac, None)

    with lock:
        PENDING[:] = [t for t in PENDING if t["sensors"]]
    if failed:
        notice(f"Saved {saved} CSV. " + " · ".join(failed), "warn")
    else:
        notice(f"Saved {saved} CSV file{'s' if saved != 1 else ''}")


def save_take(t, sensor, acc, gyro):
    settings = t["settings"]
    folder = folder_name(t)
    directory = REC_DIR / folder
    directory.mkdir(parents=True, exist_ok=True)
    label = t["sensors"][sensor.mac]
    fname = (f"{slug(t['subject'], 'subj')}__{slug(t['movement'], 'act')}__"
             f"{slug(label, 'sensor')}__{sensor.mac[-5:].replace(':', '')}.csv")
    columns = None
    if t.get("meta_cols", True):
        columns = [label, sensor.mac,
                   settings["acc_odr"] if settings.get("use_acc", True) else "",
                   settings["acc_range"] if settings.get("use_acc", True) else "",
                   settings["gyro_odr"] if settings.get("use_gyro", True) else "",
                   settings["gyro_range"] if settings.get("use_gyro", True) else "", folder]
    rows = merge_streams(acc, gyro)
    path = directory / fname
    temp = path.with_suffix(".csv.tmp")
    write_csv(temp, rows, t["subject"], t["movement"], columns)
    temp.replace(path)

    mp = directory / "meta.json"
    if mp.exists():
        meta = json.loads(mp.read_text("utf-8"))
    else:
        meta = {"id": folder, "subject_id": t["subject"], "movement": t["movement"], "created": t["created"],
                "planned_s": t["planned"], "host_duration_s": round(t["t_stop"] - t["t_start"], 2),
                "settings": settings, "files": []}
    duration = (rows[-1][0] - rows[0][0]) / 1000
    meta["files"] = [f for f in meta["files"] if f["file"] != fname]
    meta["files"].append({"file": fname, "sensor_label": label, "mac": sensor.mac,
                          "duration_s": round(duration, 2), "samples_acc": len(acc),
                          "samples_gyro": len(gyro), "rows": len(rows)})
    temp = mp.with_suffix(".json.tmp")
    temp.write_text(json.dumps(meta, ensure_ascii=False, indent=2), "utf-8")
    temp.replace(mp)


def run_session(sid, subject, movement, planned, countdown, settings, macs, transfer, meta_cols):
    sensors = [SENSORS[m] for m in macs]
    try:
        if countdown > 0:
            with lock:
                REC.update(phase="countdown", countdown_end=time.time() + countdown)
            if REC_STOP.wait(countdown):
                notice("Recording cancelled", "warn")
                return
        with lock:
            REC["phase"] = "starting"
        par(lambda s: s.configure(settings), sensors)
        if REC_STOP.is_set():
            notice("Recording cancelled", "warn")
            return
        par(lambda s: s.start(settings), sensors)
        active = [s for s in sensors if s.rec_state == "recording"]
        if not active:
            notice("No sensor started recording: " + "; ".join(s.error or "?" for s in sensors), "err")
            return
        with lock:
            REC.update(phase="recording", t0=time.time())
        REC_STOP.wait(planned)  # planned=None -> wait for the Stop button
        t_start, t_stop = REC["t0"], time.time()

        with lock:
            REC["phase"] = "stopping"
        par(lambda s: s.stop(), active)
        stored = [s for s in active if s.rec_state == "stored"]
        take = {
            "id": sid, "subject": subject, "movement": movement, "settings": settings, "planned": planned,
            "meta_cols": meta_cols, "created": datetime.fromtimestamp(t_start).isoformat(timespec="seconds"),
            "t_start": t_start, "t_stop": t_stop, "discard": False,
            "sensors": {s.mac: (s.label or f"sensor_{s.mac[-5:].replace(':', '')}") for s in stored},
        }
        with lock:
            if stored:
                PENDING.append(take)
        failed = [s.label or s.mac for s in sensors if s.rec_state == "error"]
        if transfer == "now":
            with lock:
                REC["phase"] = "downloading"
            process_pending()
        else:
            n = len(PENDING)
            notice(f"Take stored on {len(stored)} sensor{'s' if len(stored) != 1 else ''}. "
                   f"{n} take{'s' if n != 1 else ''} waiting for download"
                   + (f". Failed: {', '.join(failed)}" if failed else ""), "warn" if failed else "ok")
    except Exception as e:
        notice(f"Recording error: {e}", "err")
    finally:
        with lock:
            for s in sensors:
                if s.rec_state in ("configuring", "ready", "stopping"):
                    s.rec_state = "idle"
            REC.update(phase="idle", t0=None, countdown_end=None)


# --------------------------------------------------------------------------- #
#  API
# --------------------------------------------------------------------------- #
def jerr(msg, code=400):
    return jsonify(error=msg), code


def sensor_dict(s):
    return dict(mac=s.mac, name=s.name, rssi=s.rssi, nearby=s.nearby, label=s.label, selected=s.selected,
                state=s.state, error=s.error, identifying=s.identifying, rec_state=s.rec_state,
                progress=round(s.progress, 3), battery=s.battery, live=s.live,
                dl_left=s.dl_left, dl_total=s.dl_total, dl_samples=s.dl_samples,
                dl_idle=int(time.monotonic() - s.dl_t) if s.rec_state in ("preparing", "downloading") else 0)


@app.get("/")
def index():
    for d in (BASE / "static", BASE):
        if (d / "index.html").exists():
            r = send_from_directory(d, "index.html")
            r.headers["Cache-Control"] = "no-store"
            return r
    return "index.html not found: put it next to app.py or in a static folder", 404


@app.get("/favicon.ico")
def favicon():
    return "", 204


@app.get("/api/state")
def api_state():
    with lock:
        now = time.time()
        rec = dict(REC)
        rec["elapsed"] = (now - REC["t0"]) if REC["t0"] else 0
        rec["countdown_left"] = max(0, REC["countdown_end"] - now) if REC["countdown_end"] else 0
        for k in ("t0", "countdown_end"):
            rec.pop(k)
        return jsonify(
            sensors=[sensor_dict(s) for s in sorted(SENSORS.values(), key=lambda x: x.mac)],
            scan=SCAN, recording=rec, notice=NOTICE, options=OPTIONS, movements=CFG.get("movements", []),
            live={"mac": LIVE["mac"]},
            pending=[{"id": t["id"], "subject": t["subject"], "movement": t["movement"], "created": t["created"], "discard": t.get("discard", False),
                      "duration": round(t["t_stop"] - t["t_start"], 1),
                      "sensors": [{"mac": m, "label": l} for m, l in t["sensors"].items()]} for t in PENDING],
        )


@app.post("/api/scan")
def api_scan():
    if SCAN["active"]:
        return jsonify(ok=True)
    if REC["phase"] != "idle":
        return jerr("Scanning is unavailable while recording", 409)
    Thread(target=do_scan, daemon=True).start()
    return jsonify(ok=True)


@app.post("/api/sensors")
def api_add_sensor():
    mac = ((request.get_json(silent=True) or {}).get("mac") or "").strip().upper().replace("-", ":")
    if not re.fullmatch(r"([0-9A-F]{2}:){5}[0-9A-F]{2}", mac):
        return jerr("Invalid MAC address (example: F0:F3:20:73:56:3A)")
    get_sensor(mac)
    save_cfg()
    return jsonify(ok=True)


@app.delete("/api/sensors/<mac>")
def api_forget_sensor(mac):
    s = SENSORS.get(mac.upper())
    if s and has_pending(s.mac):
        return jerr("Download the pending takes from this sensor first", 409)
    if s:
        if s.state == "connected":
            s.disconnect()
        with lock:
            SENSORS.pop(s.mac, None)
        save_cfg()
    return jsonify(ok=True)


@app.post("/api/sensors/<mac>/<action>")
def api_sensor_action(mac, action):
    s = SENSORS.get(mac.upper())
    if not s:
        return jerr("Sensor not found", 404)
    data = request.get_json(silent=True) or {}
    if action == "label":
        s.label = (data.get("label") or "").strip()[:40]
        save_cfg()
        return jsonify(ok=True)
    if action == "select":
        s.selected = bool(data.get("selected"))
        save_cfg()
        return jsonify(ok=True)
    if REC["phase"] != "idle":
        return jerr("This action is unavailable while recording", 409)
    if action == "connect":
        if s.state in ("connecting", "connected"):
            return jsonify(ok=True)
        Thread(target=s.connect, daemon=True).start()
    elif action == "disconnect":
        if has_pending(s.mac) and s.rec_state != "error":
            return jerr("Download the pending takes from this sensor first", 409)
        if LIVE["mac"] == s.mac:
            stop_live()
        Thread(target=s.disconnect, daemon=True).start()
    elif action == "battery":
        Thread(target=s.read_battery, daemon=True).start()
    elif action == "identify":
        Thread(target=s.identify, daemon=True).start()
    else:
        return jerr("Unknown action", 404)
    return jsonify(ok=True)


@app.post("/api/record/start")
def api_record_start():
    d = request.get_json(silent=True) or {}
    movement = (d.get("movement") or "").strip()[:60]
    if not movement:
        return jerr("Enter an activity label")
    try:
        settings = {k: float(d["settings"][k]) for k in OPTIONS}
    except Exception:
        return jerr("Invalid settings")
    for k, v in settings.items():
        if v not in OPTIONS[k]:
            return jerr(f"Invalid value {k}={v}")
    settings = {k: (int(v) if float(v).is_integer() else v) for k, v in settings.items()}
    ch = d.get("channels") or {}
    settings["use_acc"], settings["use_gyro"] = bool(ch.get("acc", True)), bool(ch.get("gyro", True))
    if not (settings["use_acc"] or settings["use_gyro"]):
        return jerr("Select the accelerometer, the gyroscope, or both")
    subject = (d.get("subject") or "").strip()[:40]
    if not subject:
        return jerr("Enter a subject ID")
    meta_cols = bool(d.get("meta_cols", True))
    planned = d.get("duration")
    planned = float(planned) if planned else None
    if planned is not None and not (0.5 <= planned <= 36000):
        return jerr("Duration must be at least 0.5 s")
    countdown = max(0, min(30, int(d.get("countdown") or 0)))
    transfer = d.get("transfer") if d.get("transfer") in ("now", "later") else "later"
    with lock:
        if REC["phase"] != "idle":
            return jerr("A recording is already running", 409)
        macs = [m for m, s in SENSORS.items() if s.selected and s.state == "connected"]
        if not macs:
            return jerr("Connect at least one sensor and tick \"In recording\" on it")
        if any(SENSORS[m].dl_done is not None or SENSORS[m].dl_samples for m in macs):
            return jerr("Finish downloading or discard the pending takes before recording on these sensors", 409)
        sid = datetime.now().strftime("%Y%m%d_%H%M%S")
        REC.update(phase="starting", subject=subject, movement=movement, session=sid, planned=planned, macs=macs,
                   settings=settings, t0=None)
        REC_STOP.clear()
        mv = CFG.setdefault("movements", [])
        if movement in mv:
            mv.remove(movement)
        mv.insert(0, movement)
        del mv[20:]
        save_cfg()
    stop_live()
    Thread(target=run_session, args=(sid, subject, movement, planned, countdown, settings, macs, transfer, meta_cols), daemon=True).start()
    return jsonify(ok=True)


@app.post("/api/record/stop")
def api_record_stop():
    REC_STOP.set()
    return jsonify(ok=True)


@app.post("/api/live/start")
def api_live_start():
    d = request.get_json(silent=True) or {}
    s = SENSORS.get((d.get("mac") or "").upper())
    if not s or s.state != "connected":
        return jerr("Connect the sensor first")
    if REC["phase"] != "idle":
        return jerr("Live preview is unavailable while recording", 409)
    use_acc, use_gyro = bool(d.get("acc", True)), bool(d.get("gyro", True))
    if not (use_acc or use_gyro):
        return jerr("Select the accelerometer, the gyroscope, or both")
    odr = float(d.get("rate") or 50)
    if odr not in (25, 50, 100):
        return jerr("Preview rate must be 25, 50 or 100 Hz")
    acc_range, gyro_range = float(d.get("acc_range") or 8), float(d.get("gyro_range") or 2000)
    if acc_range not in OPTIONS["acc_range"] or gyro_range not in OPTIONS["gyro_range"]:
        return jerr("Invalid range")
    stop_live()
    with LIVE_LOCK:
        LIVE_BUF.clear()
    try:
        s.live_start(use_acc, use_gyro, odr, acc_range, gyro_range)
    except Exception as e:
        s.live_stop()
        return jerr(f"Could not start preview: {e}", 500)
    with lock:
        LIVE["mac"] = s.mac
    return jsonify(ok=True)


@app.post("/api/live/stop")
def api_live_stop():
    stop_live()
    return jsonify(ok=True)


@app.get("/api/live")
def api_live():
    after = request.args.get("after", 0, type=int)
    mac = LIVE["mac"]
    s = SENSORS.get(mac) if mac else None
    with LIVE_LOCK:
        rows = [r for r in LIVE_BUF if r[0] > after]
        seq = LIVE_SEQ[0]
    return jsonify(active=bool(s and s.state == "connected" and s.live), mac=mac, seq=seq, samples=rows)


@app.post("/api/pending/download")
def api_pending_download():
    with lock:
        if REC["phase"] != "idle":
            return jerr("A recording is running", 409)
        if not PENDING:
            return jerr("Nothing to download")
        REC.update(phase="downloading", movement="Pending takes", macs=[], t0=None, planned=None)
    stop_live()

    def job():
        try:
            process_pending()
        except Exception as e:
            notice(f"Download error: {e}", "err")
        finally:
            with lock:
                REC["phase"] = "idle"

    Thread(target=job, daemon=True).start()
    return jsonify(ok=True)


@app.post("/api/pending/discard")
def api_pending_discard():
    tid = (request.get_json(silent=True) or {}).get("id")
    with lock:
        if REC["phase"] != "idle":
            return jerr("A recording is running", 409)
        if tid:
            for t in PENDING:
                if t["id"] == tid:
                    t["discard"] = True  # still counted when splitting the sensor log
            if not all(t.get("discard") for t in PENDING):
                return jsonify(ok=True)
        macs = {m for t in PENDING for m in t["sensors"]}
        for m in macs:
            s = SENSORS[m]
            if s.state == "connected" and s.dl_done is not None and not s.dl_done.is_set():
                return jerr("Disconnect the sensor with the stalled readout before discarding", 409)
        for m in macs:
            s = SENSORS[m]
            if s.state == "connected":
                libmetawear.mbl_mw_logging_clear_entries(s.board)
            s.finish_download()
            s.rec_state, s.error = "idle", None
        PENDING.clear()
    return jsonify(ok=True)


# ---- recordings on disk ---- #
def list_sessions():
    out = []
    for d in sorted(REC_DIR.iterdir(), reverse=True):
        m = d / "meta.json"
        if d.is_dir() and m.exists():
            try:
                meta = json.loads(m.read_text("utf-8"))
                for f in meta["files"]:
                    p = d / f["file"]
                    f["size"] = p.stat().st_size if p.exists() else 0
                out.append(meta)
            except Exception:
                continue
    return out


def safe_path(sid, fname=None):
    p = (REC_DIR / sid / fname) if fname else (REC_DIR / sid)
    p = p.resolve()
    if REC_DIR.resolve() not in p.parents:
        abort(404)
    return p


@app.get("/api/recordings")
def api_recordings():
    return jsonify(list_sessions())


@app.get("/api/recordings/<sid>/<fname>")
def api_download(sid, fname):
    p = safe_path(sid, fname)
    if not p.exists():
        abort(404)
    return send_file(p, as_attachment=True, download_name=fname, mimetype="text/csv")


@app.delete("/api/recordings/<sid>/<fname>")
def api_delete_file(sid, fname):
    p = safe_path(sid, fname)
    mp = safe_path(sid) / "meta.json"
    if p.exists():
        p.unlink()
    if mp.exists():
        meta = json.loads(mp.read_text("utf-8"))
        meta["files"] = [f for f in meta["files"] if f["file"] != fname]
        if meta["files"]:
            mp.write_text(json.dumps(meta, ensure_ascii=False, indent=2), "utf-8")
        else:
            shutil.rmtree(safe_path(sid), ignore_errors=True)
    return jsonify(ok=True)


@app.delete("/api/recordings/<sid>")
def api_delete_session(sid):
    shutil.rmtree(safe_path(sid), ignore_errors=True)
    return jsonify(ok=True)


def zip_response(sessions, name):
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        for meta in sessions:
            for f in meta["files"]:
                p = REC_DIR / meta["id"] / f["file"]
                if p.exists():
                    z.write(p, f"{meta['id']}/{f['file']}")
    buf.seek(0)
    return send_file(buf, as_attachment=True, download_name=name, mimetype="application/zip")


@app.get("/api/zip")
def api_zip_all():
    return zip_response(list_sessions(), f"recordings_{datetime.now():%Y%m%d_%H%M%S}.zip")


@app.get("/api/zip/<sid>")
def api_zip_session(sid):
    safe_path(sid)
    return zip_response([m for m in list_sessions() if m["id"] == sid], f"{sid}.zip")


def battery_loop():
    while True:
        time.sleep(90)
        if REC["phase"] != "idle" or LIVE["mac"]:
            continue
        for s in list(SENSORS.values()):
            if s.state == "connected":
                s.read_battery()


if __name__ == "__main__":
    Thread(target=battery_loop, daemon=True).start()
    print(f"\n  MetaWear Recorder: http://127.0.0.1:{PORT}\n")
    app.run(host="127.0.0.1", port=PORT, threaded=True, debug=False)
