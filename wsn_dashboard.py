import spidev
import Jetson.GPIO as GPIO
import time
import threading
import sqlite3
import argparse
import os
import sys
import json
import csv
from datetime import datetime, timezone
from flask import Flask, jsonify, request, Response

RESET_PIN = 11
BUSY_PIN = 13
DIO1_PIN = 15
RXEN_PIN = 18
TXEN_PIN = 22

# 868.000000 MHz register value, assumes 32MHz crystal, must match the ESP32 side
FREQ_REG = [0x36, 0x40, 0x00, 0x00]

# private network sync word, must match on both ends or packets are silently dropped
SYNC_WORD = 0x1424

IRQ_TX_DONE = 0x0001
IRQ_RX_DONE = 0x0002
IRQ_TIMEOUT = 0x0200

# how long each rx window stays open before it is restarted, node's actual
# wake time will drift so this just needs to comfortably cover that drift
RX_WINDOW_MS = 5 * 60 * 1000

# how long to wait for an incoming node packet before re-arming rx
DB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "wsn_data.db")
DEFAULT_SLEEP_S = 10
HOST = "0.0.0.0"
PORT = 5000

# charging sign convention, the battery monitor is wired in series with the
# battery itself so its current already is net battery current
# flip BATTERY_CURRENT_SIGN to -1 if your shunt is wired so charging reads negative
BATTERY_CURRENT_SIGN = 1

# small deadband in mA so sensor noise near zero does not flicker the flag
CHARGING_THRESHOLD_MA = 5.0

spi = None

FIELD_TYPES = {
    "co2": int, "t": float, "h": float,
    "clear": int, "nir": int,
    "sm": float, "st": float, "ec": int, "ph": float,
    "n": int, "p": int, "k": int,
    "bv": float, "bi": float, "bp": float,
    "g": int, "gsat": int, "si": int, "aw": int,
}


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_reading = None
        self.last_seen = None
        self.rssi = None
        self.snr = None
        self.packet_count = 0

    def snapshot(self):
        with self.lock:
            return {
                "last_reading": self.last_reading,
                "last_seen": self.last_seen,
                "rssi": self.rssi,
                "snr": self.snr,
                "packet_count": self.packet_count,
            }

    def record_packet(self, reading, rssi, snr):
        with self.lock:
            self.last_reading = reading
            self.last_seen = datetime.now(timezone.utc).isoformat()
            self.rssi = rssi
            self.snr = snr
            self.packet_count += 1


state = SharedState()


def gpio_setup():
    global spi
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(RESET_PIN, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(BUSY_PIN, GPIO.IN)
    GPIO.setup(DIO1_PIN, GPIO.IN)
    GPIO.setup(RXEN_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(TXEN_PIN, GPIO.OUT, initial=GPIO.LOW)

    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.max_speed_hz = 2000000
    spi.mode = 0


def wait_busy(timeout=2.0):
    start = time.time()
    while GPIO.input(BUSY_PIN) == GPIO.HIGH:
        if time.time() - start > timeout:
            return False
        time.sleep(0.001)
    return True


def reset_chip():
    GPIO.output(RESET_PIN, GPIO.LOW)
    time.sleep(0.001)
    GPIO.output(RESET_PIN, GPIO.HIGH)
    time.sleep(0.005)
    wait_busy()


def write_command(opcode, params=None):
    if params is None:
        params = []
    spi.xfer2([opcode] + list(params))
    wait_busy()


def read_command(opcode, num_bytes):
    # first returned byte after the opcode is always the chip status, callers discard it
    response = spi.xfer2([opcode] + [0x00] * num_bytes)
    wait_busy()
    return response[1:]


def write_register(addr, value):
    write_command(0x0D, [(addr >> 8) & 0xFF, addr & 0xFF, value])


def write_buffer(data):
    write_command(0x0E, [0x00] + list(data))


def read_buffer(length):
    response = spi.xfer2([0x1E, 0x00] + [0x00] * (1 + length))
    wait_busy()
    return bytes(response[3:])


def get_status():
    return read_command(0xC0, 1)[0]


def get_irq_status():
    data = read_command(0x12, 3)
    return (data[1] << 8) | data[2]


def get_rx_buffer_status():
    data = read_command(0x13, 3)
    return data[1], data[2]


def get_packet_status():
    data = read_command(0x14, 4)
    rssi = -data[1] / 2
    snr_raw = data[2]
    snr = snr_raw / 4 if snr_raw < 128 else (snr_raw - 256) / 4
    return rssi, snr


def set_standby_rc():
    write_command(0x80, [0x00])


def set_regulator_mode():
    # 0x01 selects DC-DC + LDO, needed for the PA to draw enough current during TX
    write_command(0x96, [0x01])


def clear_device_errors():
    write_command(0x07, [0x00, 0x00])


def set_dio3_as_tcxo_ctrl(voltage_code=0x01, delay_steps=320):
    write_command(0x97, [voltage_code,
                          (delay_steps >> 16) & 0xFF,
                          (delay_steps >> 8) & 0xFF,
                          delay_steps & 0xFF])


def set_packet_type_lora():
    write_command(0x8A, [0x01])


def set_rf_frequency():
    write_command(0x86, FREQ_REG)


def set_pa_config():
    write_command(0x95, [0x02, 0x02, 0x00, 0x01])


def set_tx_params(power_dbm=14, ramp=0x04):
    write_command(0x8E, [power_dbm & 0xFF, ramp])


def set_buffer_base_address():
    write_command(0x8F, [0x00, 0x00])


def set_modulation_params(sf=7, bw=0x04, cr=0x01, ldro=0x00):
    write_command(0x8B, [sf, bw, cr, ldro])


def set_packet_params(payload_len, preamble=8, header_type=0x00, crc_on=0x01, invert_iq=0x00):
    write_command(0x8C, [(preamble >> 8) & 0xFF, preamble & 0xFF, header_type, payload_len, crc_on, invert_iq])


def set_dio_irq_params(irq_mask, dio1_mask):
    write_command(0x08, [(irq_mask >> 8) & 0xFF, irq_mask & 0xFF,
                          (dio1_mask >> 8) & 0xFF, dio1_mask & 0xFF,
                          0x00, 0x00, 0x00, 0x00])


def clear_irq_status():
    write_command(0x02, [0xFF, 0xFF])


def set_tx(timeout_ms=2000):
    steps = int(timeout_ms / 0.015625)
    write_command(0x83, [(steps >> 16) & 0xFF, (steps >> 8) & 0xFF, steps & 0xFF])


def set_rx(timeout_ms):
    steps = int(timeout_ms / 0.015625)
    write_command(0x82, [(steps >> 16) & 0xFF, (steps >> 8) & 0xFF, steps & 0xFF])


def set_sleep(warm_start=True):
    GPIO.output(RXEN_PIN, GPIO.LOW)
    GPIO.output(TXEN_PIN, GPIO.LOW)
    config = 0x04 if warm_start else 0x00
    spi.xfer2([0x84, config])


def calibrate_all():
    # recalibrate all blocks against the tcxo clock after the reference switch
    write_command(0x89, [0x7F])
    wait_busy()
    time.sleep(0.005)


def get_device_errors():
    data = read_command(0x17, 3)
    return (data[1] << 8) | data[2]


def print_device_errors(tag=""):
    mask = get_device_errors()
    names = [(0x0004, "PLL_CALIB"), (0x0010, "IMG_CALIB"),
             (0x0020, "XOSC_START"), (0x0040, "PLL_LOCK"), (0x0100, "PA_RAMP")]
    if mask == 0:
        print(f"device errors{tag}: none (0x0000)")
    else:
        flags = ", ".join(n for b, n in names if mask & b)
        print(f"device errors{tag}: 0x{mask:04X} [{flags}]")


def init_radio():
    reset_chip()
    set_standby_rc()
    set_dio3_as_tcxo_ctrl()
    calibrate_all()
    clear_device_errors()
    set_regulator_mode()
    set_packet_type_lora()
    set_rf_frequency()
    set_pa_config()
    set_tx_params()
    set_buffer_base_address()
    set_modulation_params()
    write_register(0x0740, (SYNC_WORD >> 8) & 0xFF)
    write_register(0x0741, SYNC_WORD & 0xFF)
    clear_irq_status()
    print("radio initialised: 868.000 MHz, SF7, BW125, CR4/5")
    print_device_errors(" post-init")


def send(message):
    data = message.encode()
    set_packet_params(len(data))
    write_buffer(data)
    set_dio_irq_params(IRQ_TX_DONE | IRQ_TIMEOUT, IRQ_TX_DONE | IRQ_TIMEOUT)
    clear_irq_status()
    GPIO.output(TXEN_PIN, GPIO.HIGH)
    GPIO.output(RXEN_PIN, GPIO.LOW)
    set_tx(timeout_ms=2000)

    start = time.time()
    while time.time() - start < 3:
        if GPIO.input(DIO1_PIN) == GPIO.HIGH:
            break
        time.sleep(0.01)

    irq = get_irq_status()
    clear_irq_status()
    GPIO.output(TXEN_PIN, GPIO.LOW)
    return bool(irq & IRQ_TX_DONE)


def listen_once(timeout_ms):
    # arms one rx window and blocks until a packet arrives or it times out
    # returns (payload_str, rssi, snr) or (None, None, None) on timeout
    set_packet_params(255)
    set_dio_irq_params(IRQ_RX_DONE | IRQ_TIMEOUT, IRQ_RX_DONE | IRQ_TIMEOUT)
    clear_irq_status()
    GPIO.output(RXEN_PIN, GPIO.HIGH)
    GPIO.output(TXEN_PIN, GPIO.LOW)
    set_rx(timeout_ms=timeout_ms)

    start = time.time()
    while time.time() - start < (timeout_ms / 1000) + 1:
        if GPIO.input(DIO1_PIN) == GPIO.HIGH:
            break
        time.sleep(0.01)

    irq = get_irq_status()
    clear_irq_status()
    GPIO.output(RXEN_PIN, GPIO.LOW)

    if irq & IRQ_RX_DONE:
        length, _ = get_rx_buffer_status()
        payload = read_buffer(length)
        rssi, snr = get_packet_status()
        # floor rssi with a 0xff buffer is a false completion, not a packet
        if rssi <= -120:
            return None, None, None
        return payload.decode(errors="replace"), rssi, snr
    return None, None, None


def console_listen(timeout_s=10):
    payload, rssi, snr = listen_once(timeout_s * 1000)
    if payload is not None:
        print(f"received: \"{payload}\"  RSSI: {rssi} dBm  SNR: {snr} dB")
    else:
        print("no packet received before timeout")


def sleep_test(duration_s):
    print(f"entering sleep for {duration_s}s")
    set_sleep(warm_start=True)
    time.sleep(duration_s)
    status = get_status()
    print(f"woke up, status: 0x{status:02X}")
    set_standby_rc()


def parse_payload(text):
    # payload looks like "co2:412,t:23.4,h:55.2,clear:1234,nir:567,sm:34.5,...,sv:4.1,si:250.5,sp:1030,lv:3.9,li:85,lp:335"
    # any field the node omitted (sensor read failed) is simply absent here
    reading = {}
    for part in text.split(","):
        if ":" not in part:
            continue
        key, _, value = part.partition(":")
        caster = FIELD_TYPES.get(key)
        if caster is None:
            continue
        try:
            reading[key] = caster(value)
        except ValueError:
            reading[key] = None
    return reading


def compute_charging(battery_i):
    # single source of truth for charging state, used at insert time and stored
    # per row so realtime and historical views never diverge
    if battery_i is None:
        return None
    return (battery_i * BATTERY_CURRENT_SIGN) > CHARGING_THRESHOLD_MA


def db_connect():
    return sqlite3.connect(DB_PATH, timeout=5)


def ensure_columns(conn):
    # adds the power monitor columns to a pre-existing readings table so an
    # older wsn_data.db keeps working without a manual migration
    existing = {row[1] for row in conn.execute("PRAGMA table_info(readings)")}
    additions = [
        ("battery_v", "REAL"), ("battery_i", "REAL"), ("battery_p", "REAL"),
        ("charging", "INTEGER"),
        # duty-cycle and light-gain context, needed to compare readings taken
        # under different sleep intervals and different illumination
        ("light_gain", "INTEGER"), ("light_saturated", "INTEGER"),
        ("sleep_interval_s", "INTEGER"), ("awake_ms", "INTEGER"),
    ]
    for col, decl in additions:
        if col not in existing:
            conn.execute(f"ALTER TABLE readings ADD COLUMN {col} {decl}")


def init_db():
    conn = db_connect()
    conn.execute("PRAGMA journal_mode=WAL")
    conn.execute("""
        CREATE TABLE IF NOT EXISTS readings (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            ts TEXT NOT NULL,
            co2 INTEGER,
            temp REAL,
            hum REAL,
            clear INTEGER,
            nir INTEGER,
            soil_moisture REAL,
            soil_temp REAL,
            soil_ec INTEGER,
            soil_ph REAL,
            soil_n INTEGER,
            soil_p INTEGER,
            soil_k INTEGER,
            rssi REAL,
            snr REAL,
            battery_v REAL,
            battery_i REAL,
            battery_p REAL,
            charging INTEGER,
            light_gain INTEGER,
            light_saturated INTEGER,
            sleep_interval_s INTEGER,
            awake_ms INTEGER
        )
    """)
    conn.execute("""
        CREATE TABLE IF NOT EXISTS config (
            key TEXT PRIMARY KEY,
            value TEXT
        )
    """)
    ensure_columns(conn)
    conn.commit()
    conn.close()
    # seed the persisted interval only if it has never been set
    if get_config("sleep_interval_s") is None:
        set_config("sleep_interval_s", DEFAULT_SLEEP_S)


def get_config(key, default=None):
    conn = db_connect()
    row = conn.execute("SELECT value FROM config WHERE key=?", (key,)).fetchone()
    conn.close()
    return row[0] if row else default


def set_config(key, value):
    conn = db_connect()
    conn.execute(
        "INSERT INTO config(key, value) VALUES(?, ?) "
        "ON CONFLICT(key) DO UPDATE SET value=excluded.value",
        (key, str(value)),
    )
    conn.commit()
    conn.close()


def get_sleep_interval():
    return int(get_config("sleep_interval_s", DEFAULT_SLEEP_S))


def set_sleep_interval(seconds):
    set_config("sleep_interval_s", int(seconds))


# Counterbalanced interval rotation.
# Running one interval per day confounds the duty cycle with the weather that
# day, so instead the trial is split into fixed blocks and the interval is
# rotated between them, with the rotation offset advancing each day. Over a
# few days every interval accumulates a comparable share of morning, midday,
# afternoon and night conditions.

def get_rotation():
    plan_raw = get_config("rotation_plan")
    if not plan_raw:
        return None
    try:
        plan = [int(v) for v in plan_raw.split(",") if v.strip()]
    except ValueError:
        return None
    if not plan:
        return None
    return {
        "plan": plan,
        "block_h": float(get_config("rotation_block_h", 3.0)),
        "start": get_config("rotation_start"),
    }


def set_rotation(plan, block_h):
    set_config("rotation_plan", ",".join(str(int(v)) for v in plan))
    set_config("rotation_block_h", float(block_h))
    set_config("rotation_start", datetime.now(timezone.utc).isoformat())


def clear_rotation():
    set_config("rotation_plan", "")


# One-shot characterisation sweep.
# Runs a fixed sequence of intervals for fixed durations and then stops, which
# is what the dark-run baseline needs: each interval held long enough to
# accumulate a stable number of cycles, with no manual intervention overnight.
# Stored as "interval:hours,interval:hours,..." in the config table.

def get_sweep():
    raw = get_config("sweep_plan")
    if not raw:
        return None
    stages = []
    for part in raw.split(","):
        if ":" not in part:
            continue
        iv, _, hrs = part.partition(":")
        try:
            stages.append((int(iv), float(hrs)))
        except ValueError:
            return None
    if not stages:
        return None
    return {"stages": stages, "start": get_config("sweep_start")}


def set_sweep(stages):
    set_config("sweep_plan", ",".join(f"{int(iv)}:{float(h)}" for iv, h in stages))
    set_config("sweep_start", datetime.now(timezone.utc).isoformat())


def clear_sweep():
    set_config("sweep_plan", "")


def sweep_state(now=None):
    # returns (interval_s, stage_index, elapsed_h, stage_remaining_h) while the
    # sweep is running, or None once it has finished or was never started
    sw = get_sweep()
    if sw is None or not sw["start"]:
        return None
    now = now or datetime.now(timezone.utc)
    elapsed = (now - datetime.fromisoformat(sw["start"])).total_seconds() / 3600
    if elapsed < 0:
        elapsed = 0
    cursor = 0.0
    for idx, (iv, hours) in enumerate(sw["stages"]):
        if elapsed < cursor + hours:
            return iv, idx, elapsed, (cursor + hours) - elapsed
        cursor += hours
    # past the end, the sweep is over and control returns to whatever the
    # manual interval was, the plan is left in place so --sweep-status can
    # still report that it completed
    return None


def sweep_finished():
    sw = get_sweep()
    return sw is not None and sw["start"] and sweep_state() is None


def rotation_state(now=None):
    # returns (interval_s, block_index, plan_position) or None when rotation
    # is off or has not been started
    rot = get_rotation()
    if rot is None or not rot["start"]:
        return None
    now = now or datetime.now(timezone.utc)
    elapsed_h = (now - datetime.fromisoformat(rot["start"])).total_seconds() / 3600
    if elapsed_h < 0:
        elapsed_h = 0
    block = int(elapsed_h // rot["block_h"])
    n = len(rot["plan"])
    # within a day the plan cycles in order, and the starting offset advances
    # by one block each day, so over n days every interval visits every slot
    # in the day rather than settling into the same two or three
    blocks_per_day = max(1, int(round(24 / rot["block_h"])))
    day, block_in_day = divmod(block, blocks_per_day)
    pos = (block_in_day + day) % n
    return rot["plan"][pos], block, pos


def current_interval():
    # the interval actually sent in the next ack. a running sweep wins over a
    # rotation, which in turn wins over the manually set value, so a sweep left
    # running cannot be silently overridden partway through
    sweep_now = sweep_state()
    if sweep_now is not None:
        interval = sweep_now[0]
        if interval != get_sleep_interval():
            set_sleep_interval(interval)
        return interval

    state_now = rotation_state()
    if state_now is None:
        return get_sleep_interval()
    interval = state_now[0]
    # mirror it into the config table so the dashboard and --get-latest show
    # what the node is really being told, not a stale manual setting
    if interval != get_sleep_interval():
        set_sleep_interval(interval)
    return interval


def insert_reading(reading, rssi, snr):
    charging = compute_charging(reading.get("bi"))
    conn = db_connect()
    conn.execute(
        """INSERT INTO readings
           (ts, co2, temp, hum, clear, nir, soil_moisture, soil_temp, soil_ec, soil_ph,
            soil_n, soil_p, soil_k, rssi, snr,
            battery_v, battery_i, battery_p, charging,
            light_gain, light_saturated, sleep_interval_s, awake_ms)
           VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
        (
            datetime.now(timezone.utc).isoformat(),
            reading.get("co2"), reading.get("t"), reading.get("h"),
            reading.get("clear"), reading.get("nir"),
            reading.get("sm"), reading.get("st"), reading.get("ec"), reading.get("ph"),
            reading.get("n"), reading.get("p"), reading.get("k"),
            rssi, snr,
            reading.get("bv"), reading.get("bi"), reading.get("bp"),
            None if charging is None else int(charging),
            reading.get("g"), reading.get("gsat"),
            # the interval the node reports is the one it actually ran, which
            # is what the analysis needs to group by
            reading.get("si"), reading.get("aw"),
        ),
    )
    conn.commit()
    conn.close()


def fetch_history(limit=100):
    conn = db_connect()
    conn.row_factory = sqlite3.Row
    rows = conn.execute(
        "SELECT * FROM readings ORDER BY id DESC LIMIT ?", (limit,)
    ).fetchall()
    conn.close()
    return [dict(r) for r in reversed(rows)]


def fetch_for_export(limit=None):
    # chronological order for csv, optional last-N slice
    conn = db_connect()
    conn.row_factory = sqlite3.Row
    if limit:
        rows = conn.execute(
            "SELECT * FROM readings ORDER BY id DESC LIMIT ?", (limit,)
        ).fetchall()
        rows = list(reversed(rows))
    else:
        rows = conn.execute("SELECT * FROM readings ORDER BY id ASC").fetchall()
    conn.close()
    return [dict(r) for r in rows]


def count_readings():
    conn = db_connect()
    count = conn.execute("SELECT COUNT(*) FROM readings").fetchone()[0]
    conn.close()
    return count


def wipe_readings():
    # deletes all logged readings and resets the autoincrement counter, the
    # config table (sleep interval) is left untouched
    conn = db_connect()
    deleted = conn.execute("SELECT COUNT(*) FROM readings").fetchone()[0]
    conn.execute("DELETE FROM readings")
    conn.execute("DELETE FROM sqlite_sequence WHERE name='readings'")
    conn.commit()
    conn.close()
    return deleted


def radio_loop():
    gpio_setup()
    init_radio()
    print(f"listening for node packets, sync word 0x{SYNC_WORD:04X}")
    while True:
        payload, rssi, snr = listen_once(RX_WINDOW_MS)
        if payload is None:
            continue
        reading = parse_payload(payload)

        # ack before the db write so the node's short rx window is hit with the
        # least possible turnaround, keeping its radio-on time (energy) minimal
        interval = current_interval()
        ok = send(f"sleep:{interval}")
        print(f"ack sent, next interval {interval}s" if ok else "ack send failed")

        insert_reading(reading, rssi, snr)
        state.record_packet(reading, rssi, snr)
        print(f"logged: {reading}  RSSI:{rssi}  SNR:{snr}")


app = Flask(__name__)


@app.route("/api/latest")
def api_latest():
    snap = state.snapshot()
    if snap["last_seen"] is not None:
        age_s = (datetime.now(timezone.utc) - datetime.fromisoformat(snap["last_seen"])).total_seconds()
    else:
        age_s = None
    snap["age_s"] = age_s
    snap["sleep_interval_s"] = get_sleep_interval()
    reading = snap["last_reading"]
    snap["charging"] = compute_charging(reading.get("bi")) if reading else None
    return jsonify(snap)


@app.route("/api/history")
def api_history():
    limit = request.args.get("limit", default=200, type=int)
    return jsonify(fetch_history(limit))


@app.route("/api/config", methods=["GET", "POST"])
def api_config():
    if request.method == "POST":
        body = request.get_json(force=True, silent=True) or {}
        seconds = body.get("sleep_interval_s")
        if not isinstance(seconds, int) or seconds < 5:
            return jsonify({"error": "sleep_interval_s must be an integer >= 5"}), 400
        set_sleep_interval(seconds)
    return jsonify({"sleep_interval_s": get_sleep_interval()})


HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>WSN node telemetry</title>
<style>
  :root {
    --bg: #0b0f0d;
    --panel-bg: #121815;
    --line: #223028;
    --fg: #d8ece2;
    --dim: #6f8a7d;
    --live: #4fe38c;
    --warn: #e3b74f;
    --dead: #e35f4f;
    --mono: ui-monospace, "JetBrains Mono", "Cascadia Code", "SF Mono", Menlo, Consolas, monospace;
    --sans: -apple-system, BlinkMacSystemFont, "Segoe UI", system-ui, sans-serif;
  }
  * { box-sizing: border-box; }
  body {
    background: var(--bg);
    color: var(--fg);
    font-family: var(--sans);
    margin: 0;
    padding: 32px;
  }
  header {
    display: flex;
    align-items: baseline;
    justify-content: space-between;
    border-bottom: 1px solid var(--line);
    padding-bottom: 16px;
    margin-bottom: 24px;
    flex-wrap: wrap;
    gap: 12px;
  }
  h1 {
    font-size: 15px;
    letter-spacing: 0.12em;
    text-transform: uppercase;
    font-weight: 600;
    color: var(--dim);
    margin: 0;
  }
  .link-status {
    display: flex;
    align-items: center;
    gap: 8px;
    font-family: var(--mono);
    font-size: 13px;
  }
  .dot {
    width: 9px;
    height: 9px;
    border-radius: 50%;
    background: var(--dead);
    box-shadow: 0 0 8px var(--dead);
  }
  .dot.live { background: var(--live); box-shadow: 0 0 8px var(--live); }
  .dot.warn { background: var(--warn); box-shadow: 0 0 8px var(--warn); }
  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fill, minmax(150px, 1fr));
    gap: 1px;
    background: var(--line);
    border: 1px solid var(--line);
    margin-bottom: 28px;
  }
  .card {
    background: var(--panel-bg);
    padding: 16px;
  }
  .card .label {
    font-size: 10px;
    text-transform: uppercase;
    letter-spacing: 0.1em;
    color: var(--dim);
    margin-bottom: 6px;
  }
  .card .value {
    font-family: var(--mono);
    font-size: 26px;
    line-height: 1;
  }
  .card .unit {
    font-family: var(--mono);
    font-size: 12px;
    color: var(--dim);
    margin-left: 4px;
  }
  svg.spark {
    width: 100%;
    height: 34px;
    margin-top: 10px;
    display: block;
  }
  svg.spark path { fill: none; stroke: var(--live); stroke-width: 1.5; }
  .panel {
    border: 1px solid var(--line);
    background: var(--panel-bg);
    padding: 20px;
    margin-bottom: 20px;
  }
  .panel h2 {
    font-size: 11px;
    text-transform: uppercase;
    letter-spacing: 0.1em;
    color: var(--dim);
    margin: 0 0 14px 0;
  }
  .config-row {
    display: flex;
    align-items: center;
    gap: 10px;
    font-family: var(--mono);
    font-size: 13px;
  }
  input[type=number] {
    background: var(--bg);
    border: 1px solid var(--line);
    color: var(--fg);
    font-family: var(--mono);
    padding: 8px 10px;
    width: 100px;
    font-size: 13px;
  }
  button {
    background: var(--live);
    color: #06110b;
    border: none;
    font-family: var(--sans);
    font-weight: 600;
    font-size: 13px;
    padding: 9px 16px;
    cursor: pointer;
  }
  button:hover { opacity: 0.85; }
  .note { color: var(--dim); font-size: 12px; margin-top: 10px; line-height: 1.5; }
  .meta { display: flex; gap: 24px; flex-wrap: wrap; font-family: var(--mono); font-size: 13px; color: var(--dim); }
  .meta b { color: var(--fg); font-weight: 600; }
  .charge-tag { font-family: var(--mono); font-size: 13px; padding: 2px 8px; border: 1px solid var(--line); }
  .charge-tag.charging { color: var(--live); border-color: var(--live); }
  .charge-tag.discharging { color: var(--warn); border-color: var(--warn); }
</style>
</head>
<body>

<header>
  <h1>WSN node telemetry</h1>
  <div class="link-status">
    <div class="dot" id="linkDot"></div>
    <span id="linkText">no contact yet</span>
  </div>
</header>

<div class="panel">
  <h2>Link</h2>
  <div class="meta">
    <div>last packet <b id="lastSeen">-</b></div>
    <div>rssi <b id="rssi">-</b> dBm</div>
    <div>snr <b id="snr">-</b> dB</div>
    <div>packets received <b id="packetCount">0</b></div>
    <div>battery <span class="charge-tag" id="chargeTag">-</span></div>
  </div>
</div>

<div class="grid" id="cardGrid"></div>

<div class="panel">
  <h2>Cycle control</h2>
  <div class="config-row">
    <span>sleep interval</span>
    <input type="number" id="intervalInput" min="5" step="1">
    <span>seconds</span>
    <button onclick="applyInterval()">Apply</button>
    <span id="configStatus"></span>
  </div>
  <div class="note">
    The node only picks up a new interval when it wakes, transmits, and briefly
    listens for an acknowledgement right after. A change here takes effect
    starting from the node's next wake cycle, not immediately.
  </div>
</div>

<script>
const METRICS = [
  { key: "co2", label: "CO2", unit: "ppm" },
  { key: "temp", label: "Temp", unit: "C" },
  { key: "hum", label: "Humidity", unit: "%" },
  { key: "clear", label: "Light clear", unit: "" },
  { key: "nir", label: "Light NIR", unit: "" },
  { key: "soil_moisture", label: "Soil moisture", unit: "%" },
  { key: "soil_temp", label: "Soil temp", unit: "C" },
  { key: "soil_ec", label: "Soil EC", unit: "" },
  { key: "soil_ph", label: "Soil pH", unit: "" },
  { key: "soil_n", label: "Soil N", unit: "" },
  { key: "soil_p", label: "Soil P", unit: "" },
  { key: "soil_k", label: "Soil K", unit: "" },
  { key: "battery_v", label: "Battery V", unit: "V" },
  { key: "battery_i", label: "Battery I", unit: "mA" },
  { key: "battery_p", label: "Battery P", unit: "mW" },
];

function buildGrid() {
  const grid = document.getElementById("cardGrid");
  grid.innerHTML = METRICS.map(m => `
    <div class="card" data-key="${m.key}">
      <div class="label">${m.label}</div>
      <div><span class="value" id="val_${m.key}">-</span><span class="unit">${m.unit}</span></div>
      <svg class="spark" id="spark_${m.key}" viewBox="0 0 100 34" preserveAspectRatio="none"></svg>
    </div>
  `).join("");
}

function sparkPath(values) {
  const clean = values.filter(v => v !== null && v !== undefined && !isNaN(v));
  if (clean.length < 2) return "";
  const min = Math.min(...clean);
  const max = Math.max(...clean);
  const range = (max - min) || 1;
  const step = 100 / (values.length - 1);
  return values.map((v, i) => {
    const x = i * step;
    const y = v === null || v === undefined ? 17 : 32 - ((v - min) / range) * 30;
    return `${i === 0 ? "M" : "L"}${x.toFixed(2)},${y.toFixed(2)}`;
  }).join(" ");
}

async function refreshLatest() {
  const res = await fetch("/api/latest");
  const data = await res.json();
  const dot = document.getElementById("linkDot");
  const text = document.getElementById("linkText");

  if (data.age_s === null) {
    dot.className = "dot";
    text.textContent = "no contact yet";
  } else if (data.age_s < data.sleep_interval_s * 2 + 30) {
    dot.className = "dot live";
    text.textContent = "live";
  } else {
    dot.className = "dot warn";
    text.textContent = "stale";
  }

  document.getElementById("lastSeen").textContent = data.last_seen ? new Date(data.last_seen).toLocaleTimeString() : "-";
  document.getElementById("rssi").textContent = data.rssi ?? "-";
  document.getElementById("snr").textContent = data.snr ?? "-";
  document.getElementById("packetCount").textContent = data.packet_count;

  const chargeTag = document.getElementById("chargeTag");
  if (data.charging === null || data.charging === undefined) {
    chargeTag.textContent = "-";
    chargeTag.className = "charge-tag";
  } else if (data.charging) {
    chargeTag.textContent = "charging";
    chargeTag.className = "charge-tag charging";
  } else {
    chargeTag.textContent = "discharging";
    chargeTag.className = "charge-tag discharging";
  }

  const intervalInput = document.getElementById("intervalInput");
  if (document.activeElement !== intervalInput) {
    intervalInput.value = data.sleep_interval_s;
  }

  if (data.last_reading) {
    const map = { temp: "t", hum: "h", soil_moisture: "sm", soil_temp: "st", soil_ec: "ec", soil_ph: "ph", soil_n: "n", soil_p: "p", soil_k: "k", battery_v: "bv", battery_i: "bi", battery_p: "bp" };
    METRICS.forEach(m => {
      const srcKey = map[m.key] || m.key;
      const v = data.last_reading[srcKey];
      const el = document.getElementById(`val_${m.key}`);
      if (el) el.textContent = (v === undefined || v === null) ? "-" : v;
    });
  }
}

async function refreshHistory() {
  const res = await fetch("/api/history?limit=60");
  const rows = await res.json();
  const map = { temp: "temp", hum: "hum", soil_moisture: "soil_moisture", soil_temp: "soil_temp", soil_ec: "soil_ec", soil_ph: "soil_ph", soil_n: "soil_n", soil_p: "soil_p", soil_k: "soil_k", co2: "co2", clear: "clear", nir: "nir", battery_v: "battery_v", battery_i: "battery_i", battery_p: "battery_p" };
  METRICS.forEach(m => {
    const col = map[m.key];
    const values = rows.map(r => r[col]);
    const path = document.getElementById(`spark_${m.key}`);
    if (path) path.innerHTML = `<path d="${sparkPath(values)}" />`;
  });
}

async function applyInterval() {
  const val = parseInt(document.getElementById("intervalInput").value, 10);
  const status = document.getElementById("configStatus");
  const res = await fetch("/api/config", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ sleep_interval_s: val }),
  });
  status.textContent = res.ok ? "saved" : "error";
  setTimeout(() => { status.textContent = ""; }, 2000);
}

buildGrid();
refreshLatest();
refreshHistory();
setInterval(refreshLatest, 3000);
setInterval(refreshHistory, 8000);
</script>
</body>
</html>
"""


@app.route("/")
def dashboard():
    return Response(HTML_PAGE, mimetype="text/html")


def cli_get_latest():
    rows = fetch_history(1)
    latest = rows[0] if rows else None
    out = {"sleep_interval_s": current_interval()}
    sweep_now = sweep_state()
    if sweep_now is not None:
        sw = get_sweep()
        out["sweep"] = {
            "stage": sweep_now[1] + 1,
            "stages": len(sw["stages"]),
            "elapsed_h": round(sweep_now[2], 2),
            "stage_remaining_h": round(sweep_now[3], 2),
        }
    elif sweep_finished():
        out["sweep"] = {"finished": True}
    state_now = rotation_state()
    if state_now is not None:
        rot = get_rotation()
        out["rotation"] = {
            "plan": rot["plan"],
            "block_h": rot["block_h"],
            "block": state_now[1],
            "position": state_now[2],
        }
    if latest is None:
        out["link"] = {"last_seen": None, "age_s": None, "rssi": None, "snr": None}
        out["charging"] = None
        out["reading"] = None
        out["battery"] = None
        print(json.dumps(out))
        return

    age_s = (datetime.now(timezone.utc) - datetime.fromisoformat(latest["ts"])).total_seconds()
    out["link"] = {
        "last_seen": latest["ts"],
        "age_s": age_s,
        "rssi": latest["rssi"],
        "snr": latest["snr"],
    }
    out["charging"] = None if latest["charging"] is None else bool(latest["charging"])
    out["reading"] = {
        "co2": latest["co2"], "temp": latest["temp"], "hum": latest["hum"],
        "clear": latest["clear"], "nir": latest["nir"],
        "soil_moisture": latest["soil_moisture"], "soil_temp": latest["soil_temp"],
        "soil_ec": latest["soil_ec"], "soil_ph": latest["soil_ph"],
        "soil_n": latest["soil_n"], "soil_p": latest["soil_p"], "soil_k": latest["soil_k"],
        "light_gain": latest["light_gain"],
    }
    out["battery"] = {"v": latest["battery_v"], "i": latest["battery_i"], "p": latest["battery_p"]}
    print(json.dumps(out))


def cli_export_csv(limit=None):
    rows = fetch_for_export(limit)
    columns = [
        "id", "ts", "co2", "temp", "hum", "clear", "nir",
        "soil_moisture", "soil_temp", "soil_ec", "soil_ph", "soil_n", "soil_p", "soil_k",
        "rssi", "snr",
        "battery_v", "battery_i", "battery_p", "charging",
        "light_gain", "light_saturated", "sleep_interval_s", "awake_ms",
    ]
    writer = csv.writer(sys.stdout)
    writer.writerow(columns)
    for r in rows:
        writer.writerow([r.get(c) for c in columns])


def cli_sweep(plan_raw):
    # plan looks like "900:10,300:6,60:3,30:3", interval seconds to hours
    stages = []
    for part in plan_raw.split(","):
        if ":" not in part:
            print(json.dumps({"status": "error", "error": f"bad stage '{part}', expected interval:hours"}))
            return
        iv, _, hrs = part.partition(":")
        try:
            iv, hrs = int(iv), float(hrs)
        except ValueError:
            print(json.dumps({"status": "error", "error": f"bad stage '{part}'"}))
            return
        if iv < 5 or hrs <= 0:
            print(json.dumps({"status": "error", "error": "interval must be >= 5 and hours > 0"}))
            return
        stages.append((iv, hrs))
    if not stages:
        print(json.dumps({"status": "error", "error": "empty plan"}))
        return

    # a sweep overrides any rotation, running both at once would interleave
    # two schedules and make the logged intervals meaningless
    clear_rotation()
    set_sweep(stages)
    state_now = sweep_state()
    total_h = sum(h for _, h in stages)
    print(json.dumps({
        "status": "ok",
        "stages": [{"interval_s": iv, "hours": h} for iv, h in stages],
        "total_hours": total_h,
        "sleep_interval_s": state_now[0] if state_now else None,
    }))


def cli_sweep_stop():
    clear_sweep()
    print(json.dumps({"status": "ok", "sleep_interval_s": get_sleep_interval()}))


def cli_sweep_status():
    sw = get_sweep()
    if sw is None:
        print(json.dumps({"status": "ok", "sweep": None}))
        return
    state_now = sweep_state()
    stages = [{"interval_s": iv, "hours": h} for iv, h in sw["stages"]]
    if state_now is None:
        print(json.dumps({"status": "ok", "sweep": {"finished": True, "stages": stages,
                                                    "started": sw["start"]}}))
        return
    iv, idx, elapsed, remaining = state_now
    total_h = sum(h for _, h in sw["stages"])
    print(json.dumps({
        "status": "ok",
        "sweep": {
            "finished": False,
            "started": sw["start"],
            "stages": stages,
            "stage": idx + 1,
            "interval_s": iv,
            "elapsed_h": round(elapsed, 2),
            "stage_remaining_h": round(remaining, 2),
            "total_remaining_h": round(total_h - elapsed, 2),
        },
    }))


def cli_rotate(plan_raw, block_h):
    # starting a rotation overrides manual interval control until it is stopped
    try:
        plan = [int(v) for v in plan_raw.split(",") if v.strip()]
    except ValueError:
        print(json.dumps({"status": "error", "error": "plan must be comma-separated integers"}))
        return
    if len(plan) < 2:
        print(json.dumps({"status": "error", "error": "plan needs at least two intervals"}))
        return
    if any(v < 5 for v in plan):
        print(json.dumps({"status": "error", "error": "every interval must be >= 5"}))
        return
    set_rotation(plan, block_h)
    interval = current_interval()
    print(json.dumps({
        "status": "ok", "plan": plan, "block_h": block_h,
        "sleep_interval_s": interval,
    }))


def cli_rotate_stop():
    clear_rotation()
    print(json.dumps({"status": "ok", "sleep_interval_s": get_sleep_interval()}))


def cli_rotate_status():
    state_now = rotation_state()
    if state_now is None:
        print(json.dumps({"status": "ok", "rotation": None, "sleep_interval_s": get_sleep_interval()}))
        return
    rot = get_rotation()
    print(json.dumps({
        "status": "ok",
        "rotation": {
            "plan": rot["plan"], "block_h": rot["block_h"],
            "started": rot["start"], "block": state_now[1], "position": state_now[2],
        },
        "sleep_interval_s": state_now[0],
    }))


def cli_set_interval(seconds):
    if seconds < 5:
        print(json.dumps({"status": "error", "error": "interval must be >= 5"}))
        return
    set_sleep_interval(seconds)
    print(json.dumps({"status": "ok", "sleep_interval_s": get_sleep_interval()}))


def cli_wipe(confirm):
    # requires an explicit --confirm alongside --wipe so a bare typo cannot
    # nuke the dataset, the config table (sleep interval) is left untouched
    pending = count_readings()
    if not confirm:
        print(json.dumps({
            "status": "needs_confirm",
            "row_count": pending,
            "message": "pass --wipe --confirm to actually delete these rows",
        }))
        return
    deleted = wipe_readings()
    print(json.dumps({"status": "ok", "deleted": deleted}))


def run_console():
    gpio_setup()
    init_radio()
    print("commands: send <message> | listen [seconds] | sleep <seconds> | status | quit")
    while True:
        try:
            line = input("> ").strip()
        except EOFError:
            break
        if not line:
            continue
        parts = line.split(maxsplit=1)
        action = parts[0].lower()
        if action == "quit":
            break
        elif action == "send" and len(parts) > 1:
            send(parts[1])
        elif action == "listen":
            console_listen(int(parts[1]) if len(parts) > 1 else 10)
        elif action == "sleep" and len(parts) > 1:
            sleep_test(int(parts[1]))
        elif action == "status":
            print(f"status: 0x{get_status():02X}")
        else:
            print("unknown command")
    spi.close()
    GPIO.cleanup()


def run_dashboard():
    init_db()
    thread = threading.Thread(target=radio_loop, daemon=True)
    thread.start()
    print(f"dashboard at http://<jetson-ip>:{PORT}")
    app.run(host=HOST, port=PORT, threaded=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--console", action="store_true", help="run the original interactive console instead of the dashboard")
    parser.add_argument("--get-latest", action="store_true", help="print latest reading, power, link, and interval as JSON, then exit")
    parser.add_argument("--export-csv", action="store_true", help="print the readings table as CSV to stdout, then exit")
    parser.add_argument("--limit", type=int, default=None, help="limit --export-csv to the last N rows")
    parser.add_argument("--set-interval", type=int, default=None, metavar="SECONDS", help="persist a new sleep interval, then exit")
    parser.add_argument("--sweep", metavar="PLAN", default=None, help="run a one-shot interval sweep, e.g. 900:10,300:6,60:3,30:3 as interval_seconds:hours")
    parser.add_argument("--sweep-stop", action="store_true", help="abandon a running sweep")
    parser.add_argument("--sweep-status", action="store_true", help="print sweep progress as JSON")
    parser.add_argument("--rotate", metavar="PLAN", default=None, help="start counterbalanced interval rotation, e.g. 30,60,300,900")
    parser.add_argument("--rotate-block-hours", type=float, default=3.0, help="hours per rotation block")
    parser.add_argument("--rotate-stop", action="store_true", help="stop rotation and return to the manual interval")
    parser.add_argument("--rotate-status", action="store_true", help="print the current rotation state as JSON")
    parser.add_argument("--wipe", action="store_true", help="delete all logged readings, then exit")
    parser.add_argument("--confirm", action="store_true", help="required alongside --wipe to actually delete")
    args = parser.parse_args()

    # db-only invocations, safe to run over ssh alongside the long-lived radio
    # process because they never touch spi or gpio
    db_only = (args.get_latest or args.export_csv or args.set_interval is not None
               or args.wipe or args.rotate or args.rotate_stop or args.rotate_status
               or args.sweep or args.sweep_stop or args.sweep_status)
    if db_only:
        init_db()
        if args.wipe:
            cli_wipe(args.confirm)
        elif args.sweep:
            cli_sweep(args.sweep)
        elif args.sweep_stop:
            cli_sweep_stop()
        elif args.sweep_status:
            cli_sweep_status()
        elif args.rotate:
            cli_rotate(args.rotate, args.rotate_block_hours)
        elif args.rotate_stop:
            cli_rotate_stop()
        elif args.rotate_status:
            cli_rotate_status()
        elif args.set_interval is not None:
            cli_set_interval(args.set_interval)
        elif args.get_latest:
            cli_get_latest()
        elif args.export_csv:
            cli_export_csv(args.limit)
        sys.exit(0)

    try:
        if args.console:
            run_console()
        else:
            run_dashboard()
    except KeyboardInterrupt:
        pass
