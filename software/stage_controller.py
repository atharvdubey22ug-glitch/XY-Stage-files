"""
=============================================================
  FLEXURE STAGE WEB CONTROLLER  —  v2 (metrology build)
  Flask + Serial -> ESP32 + TMC2208 + 28BYJ-48
=============================================================
  Run:  python stage_controller.py
  Open: http://<your-ip>:5002  (any device on same WiFi)

  New in this build
  -----------------
  * Repeatability test engine (unidirectional, bidirectional,
    staircase, XY square closure, stationary drift)
  * Timestamped event log + CSV export for Tracker alignment
  * Optical frame marker burned into the video stream so you
    can find measurement frames in the recording
  * Manual override: any jog/arrow key instantly halts the
    running test and hands control back
=============================================================
"""

import serial
import cv2
import csv
import io
import json
import math
import os
import threading
import time
from collections import deque
from datetime import datetime
from flask import Flask, Response, request, jsonify

app = Flask(__name__)

# ─────────────────────────────────────────────
#  CONFIGURATION
# ─────────────────────────────────────────────
SERIAL_PORT = 'COM4'       # Windows: 'COM4' | Linux/RPi: '/dev/ttyACM0'
BAUD_RATE   = 115200
CAM_INDEX   = 1            # 0 = built-in, 1 = USB microscope camera

# ─────────────────────────────────────────────
#  MIRROR OF FIRMWARE VARIABLES (server-side)
# ─────────────────────────────────────────────
stage_params = {
    "motor_speed":           50.0,    # steps/sec  → send S:<val>
    "timeout_ms":            350,     # ms         → send M:<val>  (backup safety only now)
    "jog_steps":             25,      # steps/tap  → send J:<val>  (main step size control)
    "startup_accel":         200.0,   # steps/s²   → send C:<val>  (smooths startup jerk)
    "microsteps":            16,      # → send MS:<val>  (calc only — MS1+MS2 both high on the shield)
    "screw_pitch_mm":        0.5,     # M3 = 0.5 mm/rev → send P:<val>
    "gear_ratio":            64.0,    # 28BYJ-48 bipolar → send G:<val>
    "motor_steps_per_rev":   32.0,    # bipolar 28BYJ-48 → send N:<val>
    "flexure_attenuation":   0.82,    # measured: 400nm/488nm → send A:<val>
    "hold_when_idle":        True,    # → send H:1 or H:0
    "x_speed":               50.0,    # → send XS:<val>
    "y_speed":               50.0,    # → send YS:<val>
    "x_accel":               200.0,   # → send XA:<val>
    "y_accel":               200.0,   # → send YA:<val>
    "x_steps":               25,      # → send XJ:<val>
    "y_steps":               25,      # → send YJ:<val>
    "x_comp_ratio":          0.0,     # X ratio fired with every Y tap → send XC:<val>
    "y_comp_ratio":          0.0,     # Y ratio fired with every X tap → send YC:<val>
    "led_brightness":        200,     # 0-255 → send B:<val>
}

def nm_per_microstep(p):
    microsteps_per_rev = p["motor_steps_per_rev"] * p["gear_ratio"] * p["microsteps"]
    return (p["screw_pitch_mm"] * 1_000_000.0) / microsteps_per_rev * p["flexure_attenuation"]

def nm_per_tap(p):
    if p["jog_steps"] > 0:
        return nm_per_microstep(p) * p["jog_steps"]
    return nm_per_microstep(p) * (p["motor_speed"] * p["timeout_ms"] / 1000.0)

# ─────────────────────────────────────────────
#  SERIAL
# ─────────────────────────────────────────────
try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"✓ Serial connected: {SERIAL_PORT}")
except Exception as e:
    print(f"✗ Serial failed: {e}")
    arduino = None

serial_lock = threading.Lock()
serial_rx = deque(maxlen=250)          # recent firmware replies, shown in the console

# v7 firmware prints DONE:<axis> the instant an axis finishes its queue.
# Waiting on that is exact; the trapezoidal estimate is only a fallback for
# older firmware. Preflight refuses to start a run unless v7 acks are present.
# Every host-commanded move carries a token. MOVE:<axis>:<id> means the
# firmware parsed and accepted the command; DONE:<axis>:<id> means that
# specific move finished. Matching on the token is what stops a stale ack
# from an earlier move satisfying the wrong wait and firing a measurement
# mark while the stage is still travelling.
ack_lock  = threading.Lock()
move_ids  = {'X': None, 'Y': None, 'Z': None}   # token of the most recent accepted move
done_ids  = {'X': 0, 'Y': 0, 'Z': 0}            # token of the most recent completed move
fw_version = None                               # int, from VERSION:<n>
version_event = threading.Event()

# Absolute firmware step counters, mirrored from POS: replies.
fw_pos    = {'X': None, 'Y': None, 'Z': None}
pos_event = threading.Event()

def send_serial(cmd: str):
    """Thread-safe write. Auto-tap, the test engine and the UI all share this."""
    if arduino:
        with serial_lock:
            arduino.write((cmd + '\n').encode())
        print(f"→ {cmd}")

def global_version(line):
    global fw_version
    try:
        fw_version = int(line.split(':')[1])
        version_event.set()
    except (ValueError, IndexError):
        pass

def check_firmware_version(timeout=2.0):
    """Ask the board what it is. Returns the version int, or None."""
    global fw_version
    if not arduino:
        return None
    version_event.clear()
    send_serial("V")
    version_event.wait(timeout)
    return fw_version

def serial_reader():
    """One reader thread for the whole process. Writes stay under serial_lock;
    pyserial tolerates a single reader alongside a single writer."""
    while True:
        if not arduino:
            time.sleep(0.5)
            continue
        try:
            raw = arduino.readline()
        except Exception:
            time.sleep(0.2)
            continue
        if not raw:
            continue
        line = raw.decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        serial_rx.append(line)
        if line.startswith('MOVE:') or line.startswith('DONE:'):
            try:
                kind, ax, tok = line.split(':')
                ax = ax.upper()
                with ack_lock:
                    if kind == 'MOVE':
                        move_ids[ax] = int(tok)
                    else:
                        done_ids[ax] = int(tok)
            except (ValueError, KeyError):
                pass
        elif line.startswith('VERSION:'):
            global_version(line)
        elif line.startswith('POS:'):
            try:
                for part in line[4:].split(','):
                    fw_pos[part[0].upper()] = int(part[1:])
                pos_event.set()
            except (ValueError, IndexError):
                pass

threading.Thread(target=serial_reader, daemon=True).start()

# ─────────────────────────────────────────────
#  AUTO-TAP STATE (unchanged behaviour)
# ─────────────────────────────────────────────
auto_tap = {
    "running":    False,
    "axis":       "X",
    "direction":  "R",
    "delay_ms":   500,
    "tap_count":  0,
    "total_taps": 0,
}
auto_tap_lock = threading.Lock()
auto_tap_thread = None

def auto_tap_worker():
    while True:
        with auto_tap_lock:
            if not auto_tap["running"]:
                break
            cmd     = auto_tap["direction"]
            delay_s = auto_tap["delay_ms"] / 1000.0

        send_serial(cmd)
        _track_manual_position(cmd)     # keep the commanded-position tracker honest

        with auto_tap_lock:
            auto_tap["tap_count"]  += 1
            auto_tap["total_taps"] += 1

        time.sleep(delay_s)

def start_auto_tap():
    global auto_tap_thread
    with auto_tap_lock:
        if auto_tap["running"]:
            return
        auto_tap["running"] = True
    auto_tap_thread = threading.Thread(target=auto_tap_worker, daemon=True)
    auto_tap_thread.start()

def stop_auto_tap():
    with auto_tap_lock:
        auto_tap["running"] = False

# ═════════════════════════════════════════════
#  REPEATABILITY TEST ENGINE
# ═════════════════════════════════════════════
#  Modes
#  -----
#  unidirectional : every target is approached from the same side (back-off
#                   loop past the origin). Isolates pure repeatability with
#                   backlash/hysteresis removed. ISO 230-2 style.
#  bidirectional  : target approached forward, origin approached in reverse.
#                   The forward/reverse mean difference IS your hysteresis.
#  staircase      : ladder up then back down in equal increments. Gives
#                   linearity + a full hysteresis loop in one run.
#  square         : +X, +Y, -X, -Y back to start. Residual offset at closure
#                   is your cross-axis coupling error (parasitic diagonal).
#  drift          : no motion at all, just timed marks. Thermal drift and
#                   vibration noise floor.
# ─────────────────────────────────────────────

seq = {
    "running":        False,
    "paused":         False,
    "mode":           "bidirectional",
    "axis":           "X",
    "travel_steps":   200,     # steps per leg
    "cycles":         20,
    "backoff_steps":  400,     # unidirectional approach run-up
    "ladder_points":  5,       # staircase: rungs each way
    "ladder_steps":   50,      # staircase: steps per rung
    "settle_ms":      400,     # quiet time after a move, before the mark
    "dwell_ms":       1500,    # mark duration (this is your measurement window)
    "marker_enabled": True,    # burn a white square into the video during dwell
    "on_manual":      "pause", # "pause" or "abort"
    "use_comp":       False,   # fire cross-axis compensation during test moves
    "verify":         True,    # re-read the firmware counters at every mark
    "max_excursion":  4000,    # steps from origin before the run aborts itself

    # live state
    "cycle":          0,
    "point":          0,
    "phase":          "idle",
    "marker":         False,
    "pos":            {"X": 0, "Y": 0, "Z": 0},
    "started_at":     None,
    "fw_origin":      None,    # firmware counters at run start, for resync
    "elapsed":        0.0,
    "marks":          0,
    "last_note":      "",
    "integrity":      "ok",    # ok | FAILED — surfaced in the UI and the CSV
    "csv_path":       "",
}
seq_lock = threading.Lock()
seq_thread = None
seq_log = []          # list of dicts, mirrored to disk as it is produced

RUN_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'runs')
CSV_COLS = ["wall_clock", "t_monotonic", "elapsed_s", "mode", "cycle", "point",
            "event", "axis", "x_steps", "y_steps", "z_steps", "x_nm", "y_nm",
            "nm_per_step", "note"]
_csv_file = None
_csv_writer = None

def _open_run_file():
    """Open a fresh CSV and write a metadata preamble. Every row is flushed as
    it is written, so an unattended overnight run survives a browser crash, a
    Flask restart, or the machine being closed — the file on disk is always
    complete up to the last event."""
    global _csv_file, _csv_writer
    _close_run_file()
    os.makedirs(RUN_DIR, exist_ok=True)
    name = f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{seq['mode']}_{seq['axis']}.csv"
    path = os.path.join(RUN_DIR, name)
    _csv_file = open(path, 'w', newline='', encoding='utf-8')
    meta = {
        "generated": datetime.now().isoformat(timespec='seconds'),
        "firmware_version": fw_version,
        "firmware_acks": fw_version is not None and fw_version >= 7,
        **{k: seq[k] for k in ("mode", "axis", "travel_steps", "cycles", "backoff_steps",
                               "ladder_points", "ladder_steps", "settle_ms", "dwell_ms",
                               "use_comp", "verify", "max_excursion")},
        **stage_params,
        "nm_per_microstep": round(nm_per_microstep(stage_params), 4),
    }
    # '#' preamble so the run is self-describing without a separate file.
    # Read it back with pandas.read_csv(path, comment='#').
    for k, v in meta.items():
        _csv_file.write(f"# {k}={v}\n")
    _csv_writer = csv.DictWriter(_csv_file, fieldnames=CSV_COLS, extrasaction='ignore')
    _csv_writer.writeheader()
    _csv_file.flush()
    seq["csv_path"] = path
    return path

def _close_run_file():
    global _csv_file, _csv_writer
    if _csv_file:
        try:
            _csv_file.flush()
            _csv_file.close()
        except Exception:
            pass
    _csv_file = None
    _csv_writer = None

def _lround(x):
    """C's lround() rounds halves away from zero; Python's round() rounds them
    to even. The firmware computes compensation steps with lround, so on an
    exact .5 (ratio 0.125 x 100 steps, say) the two disagree by one step — the
    stage would move a step the host never recorded. Match the firmware."""
    return int(math.copysign(math.floor(abs(x) + 0.5), x))

def _axis_motion(axis):
    if axis == 'X':
        return stage_params['x_speed'], stage_params['x_accel']
    if axis == 'Y':
        return stage_params['y_speed'], stage_params['y_accel']
    return stage_params['motor_speed'], stage_params['startup_accel']

def move_time_s(steps, axis):
    """Trapezoidal-profile duration estimate. There is no encoder feedback,
    so the engine waits this long plus settle_ms before marking."""
    n = abs(float(steps))
    if n == 0:
        return 0.0
    v, a = _axis_motion(axis)
    v = max(float(v), 1.0)
    a = max(float(a), 1.0)
    n_ramp = v * v / a               # steps consumed by both ramps
    if n >= n_ramp:
        return 2.0 * (v / a) + (n - n_ramp) / v
    return 2.0 * math.sqrt(n / a)

def _log_event(kind, note=""):
    p = stage_params
    nm = nm_per_microstep(p)
    rec = {
        "wall_clock":  datetime.now().strftime("%H:%M:%S.%f")[:-3],
        "t_monotonic": round(time.monotonic(), 4),
        "elapsed_s":   round(time.time() - (seq["started_at"] or time.time()), 3),
        "mode":        seq["mode"],
        "cycle":       seq["cycle"],
        "point":       seq["point"],
        "event":       kind,
        "axis":        seq["axis"],
        "x_steps":     seq["pos"]["X"],
        "y_steps":     seq["pos"]["Y"],
        "z_steps":     seq["pos"]["Z"],
        "x_nm":        round(seq["pos"]["X"] * nm, 1),
        "y_nm":        round(seq["pos"]["Y"] * nm, 1),
        "nm_per_step": round(nm, 3),
        "note":        note,
    }
    seq_log.append(rec)
    seq["last_note"] = f"{kind} {note}".strip()
    if _csv_writer:
        try:
            _csv_writer.writerow(rec)
            _csv_file.flush()          # flush per row: never lose an unattended run
        except Exception as e:
            print(f"CSV write failed: {e}")
    if len(seq_log) > 20000:
        del seq_log[:5000]             # in-memory tail only; disk keeps everything

def _seq_sleep(sec):
    """Interruptible, pause-aware sleep. False means the run was stopped."""
    deadline = time.time() + sec
    while True:
        if not seq["running"]:
            return False
        if seq["paused"]:
            # The worker may have set a phase label just before the pause landed;
            # reassert it here so the UI never reads "settling" while stopped.
            if not seq["phase"].startswith("PAUSED"):
                seq["phase"] = "PAUSED — you have control"
            time.sleep(0.05)
            deadline += 0.05          # freeze the clock while paused
            continue
        now = time.time()
        seq["elapsed"] = now - (seq["started_at"] or now)
        if now >= deadline:
            return True
        time.sleep(min(0.02, deadline - now))

def _wait_unpaused():
    while seq["running"] and seq["paused"]:
        if not seq["phase"].startswith("PAUSED"):
            seq["phase"] = "PAUSED — you have control"
        time.sleep(0.05)
    return seq["running"]

def _query_position(timeout=1.0):
    """Ask the firmware for its absolute counters. None if it doesn't answer."""
    if not arduino:
        return None
    pos_event.clear()
    send_serial("@")
    if not pos_event.wait(timeout):
        return None
    return {a: fw_pos[a] for a in ('X', 'Y', 'Z') if fw_pos[a] is not None}

def _capture_origin():
    seq["fw_origin"] = _query_position()

def resync_position(note="resync"):
    """Correct the run's tracker against the firmware's real counters. A halt
    mid-move, or an operator jog during a pause, leaves the commanded position
    ahead of or behind the truth; this is what makes the CSV honest again."""
    if seq["fw_origin"] is None:
        return
    now = _query_position()
    if not now:
        return
    delta = {}
    for a in ('X', 'Y', 'Z'):
        if a in now and a in seq["fw_origin"]:
            real = now[a] - seq["fw_origin"][a]
            if real != seq["pos"][a]:
                delta[a] = real - seq["pos"][a]
                seq["pos"][a] = real
    if delta:
        _log_event("RESYNC", f"{note} corrected {delta}")

def _wait_accepted(axes, timeout=2.0):
    """Wait for MOVE:<axis>:<id> — proof the firmware parsed the command.
    A command mangled in transit produces no MOVE line, so a dropped or
    corrupted move is caught here instead of being silently recorded as
    having happened. Returns the tokens, or None."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        with ack_lock:
            got = {a: move_ids[a] for a in axes}
        if all(v is not None for v in got.values()):
            return got
        if not seq["running"]:
            return None
        time.sleep(0.01)
    return None

def _wait_done(tokens, timeout):
    """Block until each axis reports DONE with the matching token. Abort-aware
    but not pause-aware: the stage is physically moving, so a pause takes
    effect at the next _wait_unpaused rather than mid-travel."""
    deadline = time.time() + timeout
    while True:
        if not seq["running"]:
            return False
        with ack_lock:
            if all(done_ids[a] >= tok for a, tok in tokens.items()):
                return True
        if time.time() > deadline:
            return False
        time.sleep(0.01)
        seq["elapsed"] = time.time() - (seq["started_at"] or time.time())

def _fail(reason):
    """Stop the run and mark the data as untrustworthy. Nothing downstream of
    this point should end up in a report without being looked at."""
    seq["integrity"] = "FAILED"
    _log_event("INTEGRITY FAIL", reason)
    stop_sequence(reason)
    return False

def _move(steps, axis, note=""):
    """Issue one deterministic microstep move and wait for the stage to land.
    Every step of this is confirmed by the firmware; nothing is assumed."""
    steps = int(round(steps))
    if steps == 0:
        return True
    if not _wait_unpaused():
        return False

    # Cross-axis compensation is off for test moves by default: a repeatability
    # run should see what the mechanism does unaided. Turn it on to score
    # whether a comp ratio actually cancels the parasitic diagonal.
    comp = seq["use_comp"] and axis in ('X', 'Y')
    other = 'Y' if axis == 'X' else 'X'
    comp_steps = 0
    if comp:
        ratio = stage_params['y_comp_ratio'] if axis == 'X' else stage_params['x_comp_ratio']
        comp_steps = _lround(ratio * steps)

    # Soft travel limit. A flexure has finite range; running an unattended
    # 20-cycle test into the end of it produces garbage data at best.
    lim = seq["max_excursion"]
    if abs(seq["pos"][axis] + steps) > lim:
        return _fail(f"{axis} would reach {seq['pos'][axis] + steps} steps, past the "
                     f"{lim}-step travel limit")
    if comp_steps and abs(seq["pos"][other] + comp_steps) > lim:
        return _fail(f"{other} would reach {seq['pos'][other] + comp_steps} steps, past the "
                     f"{lim}-step travel limit")

    axes = [axis] + ([other] if comp_steps else [])
    with ack_lock:
        for a in axes:
            move_ids[a] = None

    seq["phase"] = f"move {axis}{steps:+d}"
    send_serial(f"T{'C' if comp else ''}{axis}{steps}")

    tokens = _wait_accepted(axes)
    if tokens is None:
        if not seq["running"]:
            return False
        return _fail(f"firmware never acknowledged T{axis}{steps} — command dropped or garbled")

    seq["pos"][axis] += steps
    if comp_steps:
        seq["pos"][other] += comp_steps
    _log_event("move", note or f"{axis}{steps:+d}" + (f" (+{other}{comp_steps:+d})" if comp_steps else ""))

    est = max(move_time_s(steps, axis),
              move_time_s(comp_steps, other) if comp_steps else 0.0)
    if _wait_done(tokens, timeout=est * 3.0 + 5.0):
        return seq["running"]
    if not seq["running"]:
        return False
    return _fail(f"move {axis}{steps:+d} never completed within {est * 3.0 + 5.0:.1f} s")

def _verify_position(label):
    """Re-read the firmware's own counters and confirm they match what this run
    thinks it commanded. This is the check that makes the data defensible: it
    catches a dropped command, a mis-parsed move, a halt nobody logged. It
    cannot catch a physically lost step — the system is open-loop, and that
    limitation belongs in the report, not in the code."""
    if not seq["verify"] or seq["fw_origin"] is None:
        return True
    now = _query_position()
    if not now:
        return _fail(f"position query returned nothing before mark '{label}'")
    for a in ('X', 'Y', 'Z'):
        if a not in now or a not in seq["fw_origin"]:
            continue
        real = now[a] - seq["fw_origin"][a]
        if real != seq["pos"][a]:
            return _fail(f"{a} counter disagrees before mark '{label}': "
                         f"firmware {real}, expected {seq['pos'][a]}")
    return True

def _mark(label):
    """Settle, verify, then hold a measurement window with the marker lit."""
    seq["phase"] = "settling"
    if not _seq_sleep(seq["settle_ms"] / 1000.0):
        return False
    if not _verify_position(label):
        return False
    seq["marker"] = True
    seq["phase"] = f"MARK · {label}"
    seq["marks"] += 1
    _log_event("MARK", label)
    ok = _seq_sleep(seq["dwell_ms"] / 1000.0)
    seq["marker"] = False
    return ok

def sequence_worker():
    mode    = seq["mode"]
    axis    = seq["axis"]
    other   = 'Y' if axis == 'X' else 'X'
    travel  = int(seq["travel_steps"])
    backoff = int(seq["backoff_steps"])
    cycles  = int(seq["cycles"])
    rungs   = int(seq["ladder_points"])
    rung    = int(seq["ladder_steps"])

    _capture_origin()
    _log_event("RUN START", f"{mode} on {axis}")

    try:
        # Establish the reference before any motion.
        if not _mark("origin reference"):
            return

        for c in range(1, cycles + 1):
            seq["cycle"] = c
            seq["point"] = 0
            if not seq["running"]:
                return

            if mode == "unidirectional":
                # target: approached in the + direction
                if not _move(+travel, axis, "to target"): return
                if not _mark(f"target (fwd approach) c{c}"): return
                # run past the origin, then come back up onto it — same approach side
                if not _move(-(travel + backoff), axis, "back off past origin"): return
                if not _seq_sleep(seq["settle_ms"] / 1000.0): return
                if not _move(+backoff, axis, "re-approach origin"): return
                if not _mark(f"origin (fwd approach) c{c}"): return

            elif mode == "bidirectional":
                if not _move(+travel, axis, "to target"): return
                if not _mark(f"target (fwd approach) c{c}"): return
                if not _move(-travel, axis, "to origin"): return
                if not _mark(f"origin (rev approach) c{c}"): return

            elif mode == "staircase":
                for i in range(1, rungs + 1):
                    seq["point"] = i
                    if not _move(+rung, axis, f"up rung {i}"): return
                    if not _mark(f"up {i}/{rungs} c{c}"): return
                for i in range(rungs, 0, -1):
                    seq["point"] = i
                    if not _move(-rung, axis, f"down rung {i}"): return
                    if not _mark(f"down {i}/{rungs} c{c}"): return

            elif mode == "square":
                legs = [(+travel, axis, "leg 1"), (+travel, other, "leg 2"),
                        (-travel, axis, "leg 3"), (-travel, other, "leg 4")]
                for i, (s, ax, name) in enumerate(legs, 1):
                    seq["point"] = i
                    if not _move(s, ax, name): return
                    if not _mark(f"corner {i} c{c}"): return
                if not _mark(f"closure c{c}"): return

            elif mode == "drift":
                if not _mark(f"drift sample {c}"): return

        seq["phase"] = "complete"
        _log_event("RUN COMPLETE", f"{seq['marks']} marks, integrity {seq['integrity']}")

    finally:
        seq["marker"] = False
        seq["running"] = False
        seq["paused"] = False
        if seq["phase"] != "complete":
            seq["phase"] = "stopped"
            _log_event("RUN STOPPED", f"integrity {seq['integrity']}")
        _close_run_file()

def preflight():
    """Everything that must be true before a run is allowed to produce data.
    Refusing to start is always cheaper than discovering afterwards that a
    night of measurements can't be defended."""
    problems = []
    if arduino is None:
        problems.append("No serial connection to the ESP32.")
    else:
        v = check_firmware_version()
        if v is None:
            problems.append("Firmware did not answer the version query. "
                            "Pre-v7 firmware cannot confirm moves, so marks "
                            "could land mid-travel — flash v7 before recording.")
        elif v < 7:
            problems.append(f"Firmware v{v} has no move acknowledgements. Flash v7.")
        if _query_position() is None:
            problems.append("Firmware did not report its step counters, so "
                            "position cannot be verified.")
    if seq["marker_enabled"] and not camera.isOpened():
        problems.append(f"Camera {CAM_INDEX} is not open, so no marker frames "
                        f"will be recorded. Turn the marker off or fix the camera.")
    if seq["travel_steps"] > seq["max_excursion"]:
        problems.append(f"Travel per leg ({seq['travel_steps']}) exceeds the "
                        f"{seq['max_excursion']}-step limit.")
    return problems

def start_sequence():
    global seq_thread
    with seq_lock:
        if seq["running"]:
            return False
        stop_auto_tap()                 # the two engines must not share the axis
        seq_log.clear()
        seq["running"]    = True
        seq["paused"]     = False
        seq["cycle"]      = 0
        seq["point"]      = 0
        seq["marks"]      = 0
        seq["elapsed"]    = 0.0
        seq["phase"]      = "starting"
        seq["pos"]        = {"X": 0, "Y": 0, "Z": 0}
        seq["started_at"] = time.time()
        seq["fw_origin"]  = None
        seq["integrity"]  = "ok"
        with ack_lock:
            for a in ('X', 'Y', 'Z'):
                move_ids[a] = None
                done_ids[a] = 0
        _open_run_file()
    seq_thread = threading.Thread(target=sequence_worker, daemon=True)
    seq_thread.start()
    return True

def stop_sequence(reason="user stop"):
    if seq["running"]:
        _log_event("ABORT", reason)
    seq["running"] = False
    seq["paused"]  = False
    seq["marker"]  = False
    send_serial("!")          # dump whatever is still queued in the firmware

# ─────────────────────────────────────────────
#  MANUAL OVERRIDE
# ─────────────────────────────────────────────
JOG_CMDS = {'U', 'D', 'L', 'R', '+', '-'}

def _track_manual_position(cmd):
    """Keep the commanded-position tracker in step with jogs and T-moves,
    so a paused run can be resumed without losing the coordinate frame."""
    try:
        if cmd == 'R':   seq["pos"]["X"] += int(stage_params['x_steps'])
        elif cmd == 'L': seq["pos"]["X"] -= int(stage_params['x_steps'])
        elif cmd == 'U': seq["pos"]["Y"] += int(stage_params['y_steps'])
        elif cmd == 'D': seq["pos"]["Y"] -= int(stage_params['y_steps'])
        elif cmd == '+': seq["pos"]["Z"] += int(stage_params['jog_steps'])
        elif cmd == '-': seq["pos"]["Z"] -= int(stage_params['jog_steps'])
        elif len(cmd) > 2 and cmd[0] == 'T' and cmd[1] in 'XYZ':
            seq["pos"][cmd[1]] += int(cmd[2:])
    except Exception:
        pass

def _is_motion_cmd(cmd):
    return cmd in JOG_CMDS or (len(cmd) > 2 and cmd[0] == 'T' and cmd[1] in 'XYZ')

def manual_override(cmd):
    """Called whenever a motion command arrives from the browser. Anything
    the operator does takes precedence over whatever is running."""
    took_over = False
    if auto_tap["running"]:
        stop_auto_tap()
        took_over = True
    if seq["running"]:
        if seq["on_manual"] == "abort":
            _log_event("MANUAL OVERRIDE", f"aborted by {cmd}")
            stop_sequence(f"manual {cmd}")
        else:
            seq["paused"] = True
            seq["marker"] = False
            seq["phase"]  = "PAUSED — manual control"
            _log_event("MANUAL OVERRIDE", f"paused by {cmd}")
            # Halt whatever is mid-flight. Without this the operator's jog
            # would queue up behind the remaining steps of the current move.
            send_serial("!")
            # Release any wait on the halted move. It will never report DONE,
            # and a deliberate pause must not be recorded as an integrity
            # failure. The position it did not finish travelling is recovered
            # by resync_position() on resume, before the next mark is verified.
            with ack_lock:
                for a in done_ids:
                    done_ids[a] = max(done_ids[a], move_ids[a] or 0)
        took_over = True
    elif took_over:
        send_serial("!")
    return took_over

# ─────────────────────────────────────────────
#  CAMERA
# ─────────────────────────────────────────────
camera = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
camera.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

def generate_frames():
    while True:
        ok, frame = camera.read()
        if not ok:
            break
        if seq["marker_enabled"] and seq["running"]:
            # Solid white square during the measurement window, dark otherwise.
            # Tracker can threshold this patch to find exactly which frames count.
            colour = (255, 255, 255) if seq["marker"] else (35, 35, 35)
            cv2.rectangle(frame, (8, 8), (52, 52), colour, -1)
            cv2.rectangle(frame, (8, 8), (52, 52), (120, 120, 120), 1)
            label = f"{seq['mode'][:5]} c{seq['cycle']} {'MARK' if seq['marker'] else ''}"
            cv2.putText(frame, label, (60, 38),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        _, buf = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buf.tobytes() + b'\r\n')

# ─────────────────────────────────────────────
#  HTML + JS (single-file, no CDN needed)
# ─────────────────────────────────────────────
HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>Stage Controller</title>
<style>
  *, *::before, *::after { box-sizing: border-box; }

  body {
    margin: 0; padding: 12px 12px 84px;
    background: #1a1a2e;
    color: #e0e0e0;
    font-family: 'Segoe UI', system-ui, sans-serif;
    text-align: center;
  }

  h1 { font-size: 1.2rem; color: #7ec8e3; margin: 0 0 12px; letter-spacing: 2px; text-transform: uppercase; }

  /* ── VIDEO ── */
  #feed {
    width: 100%; max-width: 540px;
    border: 2px solid #2e4a6a;
    border-radius: 10px;
    display: block; margin: 0 auto 16px;
  }

  /* ── MOTION GRID ── */
  .dpad {
    display: grid;
    grid-template-columns: repeat(3, 62px);
    grid-template-rows: repeat(3, 62px);
    gap: 6px;
    justify-content: center;
    margin: 0 auto 12px;
  }

  .btn {
    background: #2a2a4a;
    border: 1px solid #3a3a6a;
    border-radius: 10px;
    color: #e0e0e0;
    font-size: 22px;
    cursor: pointer;
    display: flex; align-items: center; justify-content: center;
    touch-action: manipulation;
    user-select: none;
    transition: background 0.08s, transform 0.05s;
  }
  .btn:active, .btn.held { background: #1a6eb5; transform: scale(0.95); }
  .btn.empty { background: transparent; border-color: transparent; pointer-events: none; }
  .btn:focus-visible, .go-btn:focus-visible, .dir-btn:focus-visible { outline: 2px solid #7ec8e3; outline-offset: 2px; }

  .flip-btn {
    background: #2a2a4a;
    border: 1px solid #3a3a6a;
    border-radius: 6px;
    color: #e0e0e0;
    font-size: 0.72rem;
    font-weight: 600;
    padding: 4px 10px;
    cursor: pointer;
    white-space: nowrap;
    transition: background 0.08s, transform 0.05s;
  }
  .flip-btn:active { background: #1a6eb5; transform: scale(0.95); }

  /* ── Z ROW ── */
  .z-row { display: flex; justify-content: center; gap: 10px; margin-bottom: 20px; }
  .z-btn { padding: 10px 22px; font-size: 15px; font-weight: 600; }

  /* ── PANELS ── */
  .panel {
    background: #22223a;
    border: 1px solid #2e4a6a;
    border-radius: 12px;
    padding: 16px;
    max-width: 540px;
    margin: 0 auto 16px;
    text-align: left;
  }
  .panel h2 {
    margin: 0 0 14px;
    font-size: 0.85rem;
    letter-spacing: 1.5px;
    text-transform: uppercase;
    color: #7ec8e3;
    border-bottom: 1px solid #2e4a6a;
    padding-bottom: 8px;
  }
  .panel.metro { border-color: #6a4a2e; }
  .panel.metro h2 { color: #e3b87e; border-bottom-color: #6a4a2e; }

  /* ── SLIDERS ── */
  .param-row {
    display: flex; align-items: center;
    gap: 8px; margin-bottom: 12px;
  }
  .param-row label {
    width: 130px; font-size: 0.8rem; color: #aaa; flex-shrink: 0;
  }
  .param-row input[type=range] {
    flex: 1; accent-color: #1a6eb5;
  }
  .param-val {
    width: 54px; text-align: right;
    font-size: 0.8rem; font-weight: 600;
    color: #7ec8e3; font-variant-numeric: tabular-nums;
  }

  /* ── ESTIMATE BOX ── */
  .estimate-box {
    background: #12122a;
    border: 1px solid #1a6eb5;
    border-radius: 8px;
    padding: 10px 14px;
    margin-bottom: 14px;
    display: flex; flex-wrap: wrap; gap: 8px 24px;
  }
  .est-item { font-size: 0.78rem; color: #aaa; }
  .est-item span { color: #7ec8e3; font-weight: 700; font-size: 0.88rem; }
  .estimate-box.amber { border-color: #b5821a; }
  .estimate-box.amber .est-item span { color: #e3b87e; }

  /* ── TOGGLE ── */
  .toggle-row {
    display: flex; align-items: center; gap: 10px;
    margin-bottom: 12px;
  }
  .toggle-label { font-size: 0.8rem; color: #aaa; flex: 1; }
  .toggle {
    position: relative; display: inline-block; width: 44px; height: 24px; flex-shrink: 0;
  }
  .toggle input { opacity: 0; width: 0; height: 0; }
  .slider-tog {
    position: absolute; cursor: pointer; inset: 0;
    background: #444; border-radius: 24px; transition: 0.2s;
  }
  .slider-tog:before {
    content: ''; position: absolute;
    width: 18px; height: 18px; left: 3px; bottom: 3px;
    background: white; border-radius: 50%; transition: 0.2s;
  }
  input:checked + .slider-tog { background: #1a6eb5; }
  input:checked + .slider-tog:before { transform: translateX(20px); }

  /* ── PRECISION ── */
  .precision-row {
    display: flex; gap: 6px; align-items: center; margin-bottom: 8px; flex-wrap: wrap;
  }
  .precision-row select, .precision-row input[type=number] {
    background: #12122a; border: 1px solid #3a3a6a;
    color: #e0e0e0; border-radius: 6px;
    padding: 6px 8px; font-size: 0.85rem;
  }
  .precision-row select { width: 70px; }
  .precision-row input[type=number] { width: 70px; }
  .go-btn {
    flex: 1; padding: 8px 14px;
    background: #1a6eb5; border: none; border-radius: 6px;
    color: white; font-weight: 700; font-size: 0.85rem; cursor: pointer;
  }
  .go-btn:hover { background: #1a80d0; }
  .go-btn:disabled { background: #333; color: #666; cursor: default; }

  /* ── DIRECTION / MODE BUTTONS ── */
  .dir-btn {
    padding: 7px 13px; font-size: 0.82rem; font-weight: 600;
    background: #2a2a4a; border: 1px solid #3a3a6a;
    border-radius: 7px; color: #aaa; cursor: pointer;
    transition: background 0.1s, color 0.1s, border-color 0.1s;
  }
  .dir-btn:hover    { background: #333360; color: #ddd; }
  .dir-btn.active-dir {
    background: #1a6eb5; border-color: #2a8adf; color: white;
  }
  .mode-btn.active-dir { background: #8a5a1a; border-color: #e3b87e; color: white; }

  /* ── STATUS PILL STATES ── */
  .pill-running { background: #1a4a1a !important; color: #28a745 !important; }
  .pill-idle    { background: #333    !important; color: #888    !important; }
  .pill-paused  { background: #4a3a1a !important; color: #e3b87e !important; }
  #status {
    font-size: 0.7rem; color: #555;
    margin-top: 8px; min-height: 16px;
  }
  #status.ok  { color: #28a745; }
  #status.err { color: #dc3545; }

  /* ── MICROSTEP INFO ── */
  .info-note {
    font-size: 0.72rem; color: #666;
    margin-top: 4px; line-height: 1.4;
  }

  /* ── TEST LOG TABLE ── */
  #seq-log {
    background: #12122a; border-radius: 6px; padding: 8px;
    font-size: 0.68rem; font-family: ui-monospace, monospace;
    max-height: 150px; overflow-y: auto; text-align: left; color: #8fa;
  }
  #seq-log .mark { color: #e3b87e; font-weight: 700; }
  #seq-log .man  { color: #ff8080; font-weight: 700; }

  /* ── OVERRIDE BAR ── */
  #override-bar {
    position: fixed; left: 0; right: 0; bottom: 0;
    background: #4a0f1a; border-top: 2px solid #dc3545;
    padding: 10px 12px; z-index: 999;
    display: none; gap: 10px; align-items: center; justify-content: center;
    flex-wrap: wrap;
  }
  #override-bar.show { display: flex; }
  #override-bar .ov-msg {
    font-size: 0.75rem; color: #ffb3bd; flex: 1 1 100%; text-align: center;
  }
  .ov-btn {
    padding: 11px 18px; border-radius: 8px; border: none;
    font-size: 0.9rem; font-weight: 800; cursor: pointer; color: white;
  }
  .ov-take   { background: #b5821a; }
  .ov-abort  { background: #dc3545; }
  .ov-resume { background: #28a745; }

  @media (prefers-reduced-motion: reduce) {
    * { transition: none !important; }
  }
</style>
</head>
<body>

<h1>⚙ Flexure Stage</h1>
<img id="feed" src="/video_feed" alt="Camera Feed">

<!-- ── D-PAD ── -->
<div class="dpad">
  <div class="btn empty"></div>
  <button class="btn" id="btn-U" onpointerdown="hold('U')" onpointerup="release()" onpointerleave="release()">▲</button>
  <div class="btn empty"></div>
  <button class="btn" id="btn-L" onpointerdown="hold('L')" onpointerup="release()" onpointerleave="release()">◀</button>
  <div class="btn empty"></div>
  <button class="btn" id="btn-R" onpointerdown="hold('R')" onpointerup="release()" onpointerleave="release()">▶</button>
  <div class="btn empty"></div>
  <button class="btn" id="btn-D" onpointerdown="hold('D')" onpointerup="release()" onpointerleave="release()">▼</button>
  <div class="btn empty"></div>
</div>

<!-- ── Z ── -->
<div class="z-row">
  <button class="btn z-btn" id="btn-plus"  onpointerdown="hold('+')" onpointerup="release()" onpointerleave="release()">Focus ▲</button>
  <button class="btn z-btn" id="btn-minus" onpointerdown="hold('-')" onpointerup="release()" onpointerleave="release()">Focus ▼</button>
</div>

<!-- ── LIVE ESTIMATES ── -->
<div class="panel">
  <h2>📐 Live Estimates</h2>
  <div class="estimate-box">
    <div class="est-item">nm / microstep: <span id="est-nm-us">—</span></div>
    <div class="est-item">nm / tap: <span id="est-nm-tap">—</span></div>
    <div class="est-item">steps / tap: <span id="est-steps-tap">—</span></div>
  </div>
  <p class="info-note">
    Estimates update instantly as you move sliders. Microstep count is hardware-set via MS1/MS2 pins —
    change the display below to match your wiring and estimates will recalculate.
  </p>
</div>

<!-- ══════════════════════════════════════════ -->
<!-- ── REPEATABILITY TEST SUITE ── -->
<!-- ══════════════════════════════════════════ -->
<div class="panel metro">
  <h2>📊 Repeatability Test Suite</h2>

  <div style="display:flex; gap:6px; flex-wrap:wrap; margin-bottom:10px;">
    <button class="dir-btn mode-btn active-dir" data-mode="bidirectional"  onclick="setMode(this)">Bidirectional</button>
    <button class="dir-btn mode-btn"            data-mode="unidirectional" onclick="setMode(this)">Unidirectional</button>
    <button class="dir-btn mode-btn"            data-mode="staircase"      onclick="setMode(this)">Staircase</button>
    <button class="dir-btn mode-btn"            data-mode="square"         onclick="setMode(this)">XY square</button>
    <button class="dir-btn mode-btn"            data-mode="drift"          onclick="setMode(this)">Drift</button>
  </div>
  <p class="info-note" id="mode-desc" style="margin:0 0 12px; color:#e3b87e;"></p>

  <div class="param-row">
    <label>Axis</label>
    <select id="seq-axis" onchange="pushSeqCfg(); refreshSeqEstimates();"
            style="flex:1; background:#12122a; border:1px solid #3a3a6a; color:#e0e0e0; border-radius:6px; padding:6px 8px;">
      <option value="X" selected>X</option>
      <option value="Y">Y</option>
      <option value="Z">Z</option>
    </select>
    <div class="param-val" style="width:auto; font-size:0.7rem; color:#666;" id="axis-note">primary</div>
  </div>

  <div class="param-row" id="row-travel">
    <label>Travel / leg</label>
    <input type="range" id="sl-travel" min="5" max="2000" value="200" step="5"
           oninput="seqParam('travel_steps', this.value, 'lbl-travel', ' steps')"
           onchange="pushSeqCfg()">
    <div class="param-val" id="lbl-travel">200 steps</div>
  </div>
  <p class="info-note" id="travel-nm" style="margin:-8px 0 10px;">≈ — nm per leg</p>

  <div class="param-row" id="row-backoff">
    <label>Back-off run-up</label>
    <input type="range" id="sl-backoff" min="10" max="2000" value="400" step="10"
           oninput="seqParam('backoff_steps', this.value, 'lbl-backoff', ' steps')"
           onchange="pushSeqCfg()">
    <div class="param-val" id="lbl-backoff">400 steps</div>
  </div>
  <p class="info-note" id="note-backoff" style="margin:-8px 0 10px;">
    ↑ Unidirectional only. The stage overshoots the origin by this much then comes back up onto it,
    so every mark is reached from the same side. Make it comfortably larger than your expected
    hysteresis (~100 nm ≈ a few steps at your current resolution) or backlash will leak into the result.
  </p>

  <div class="param-row" id="row-rungsize">
    <label>Rung size</label>
    <input type="range" id="sl-rungsize" min="1" max="500" value="50" step="1"
           oninput="seqParam('ladder_steps', this.value, 'lbl-rungsize', ' steps')"
           onchange="pushSeqCfg()">
    <div class="param-val" id="lbl-rungsize">50 steps</div>
  </div>
  <div class="param-row" id="row-rungs">
    <label>Rungs each way</label>
    <input type="range" id="sl-rungs" min="2" max="30" value="5" step="1"
           oninput="seqParam('ladder_points', this.value, 'lbl-rungs', '')"
           onchange="pushSeqCfg()">
    <div class="param-val" id="lbl-rungs">5</div>
  </div>

  <div class="param-row">
    <label id="lbl-cycles-name">Cycles</label>
    <input type="range" id="sl-cycles" min="1" max="200" value="20" step="1"
           oninput="seqParam('cycles', this.value, 'lbl-cycles', '')"
           onchange="pushSeqCfg()">
    <div class="param-val" id="lbl-cycles">20</div>
  </div>
  <p class="info-note" style="margin:-8px 0 10px;">
    ISO 230-2 uses 5 approaches per target; 20+ gives you a standard deviation you can actually trust.
  </p>

  <div class="param-row">
    <label>Settle before mark</label>
    <input type="range" id="sl-settle" min="0" max="5000" value="400" step="50"
           oninput="seqParam('settle_ms', this.value, 'lbl-settle', ' ms')"
           onchange="pushSeqCfg()">
    <div class="param-val" id="lbl-settle">400 ms</div>
  </div>
  <div class="param-row">
    <label>Mark window</label>
    <input type="range" id="sl-dwell" min="200" max="10000" value="1500" step="100"
           oninput="seqParam('dwell_ms', this.value, 'lbl-dwell', ' ms')"
           onchange="pushSeqCfg()">
    <div class="param-val" id="lbl-dwell">1500 ms</div>
  </div>
  <p class="info-note" style="margin:-8px 0 12px;">
    ↑ The mark window is your measurement interval — long enough for several clean video frames
    with the stage fully at rest. Motion timing is open-loop (there is no encoder), so the engine
    waits out a trapezoidal estimate from the current speed/accel plus the settle time. If a mark
    ever lands mid-move, raise the settle.
  </p>

  <div class="toggle-row">
    <span class="toggle-label">Burn a marker patch into the video during each mark — Tracker can threshold it to find measurement frames</span>
    <label class="toggle">
      <input type="checkbox" id="chk-marker" checked onchange="pushSeqCfg()">
      <span class="slider-tog"></span>
    </label>
  </div>

  <div class="param-row">
    <label>Travel limit</label>
    <input type="range" id="sl-limit" min="100" max="20000" value="4000" step="100"
           oninput="seqParam('max_excursion', this.value, 'lbl-limit', ' steps')"
           onchange="pushSeqCfg()">
    <div class="param-val" id="lbl-limit">4000 steps</div>
  </div>
  <p class="info-note" style="margin:-8px 0 10px;">
    ↑ Hard stop on how far from the origin the run may travel. Set it just inside
    your flexure's usable range — an unattended run that drives into the end of
    the travel produces data that looks plausible and is worthless.
  </p>

  <div class="toggle-row">
    <span class="toggle-label">Verify the firmware's step counters before every mark, and abort on disagreement</span>
    <label class="toggle">
      <input type="checkbox" id="chk-verify" checked onchange="pushSeqCfg()">
      <span class="slider-tog"></span>
    </label>
  </div>

  <div class="toggle-row">
    <span class="toggle-label">Apply cross-axis compensation during test moves — off measures the bare mechanism, on scores whether your comp ratios actually cancel the diagonal</span>
    <label class="toggle">
      <input type="checkbox" id="chk-use-comp" onchange="pushSeqCfg()">
      <span class="slider-tog"></span>
    </label>
  </div>

  <div class="toggle-row">
    <span class="toggle-label">Manual input aborts the run outright (off = pause, so you can resume where you left off)</span>
    <label class="toggle">
      <input type="checkbox" id="chk-abort-on-manual" onchange="pushSeqCfg()">
      <span class="slider-tog"></span>
    </label>
  </div>
  <p class="info-note" style="margin:-4px 0 12px;">
    Run the XY square both ways to get a number for your comp tuning: closure error uncompensated
    minus closure error compensated is exactly how much the correction bought you.
  </p>

  <div class="estimate-box amber">
    <div class="est-item">Marks: <span id="est-marks">—</span></div>
    <div class="est-item">Run time: <span id="est-runtime">—</span></div>
    <div class="est-item">Span: <span id="est-span">—</span></div>
  </div>

  <div style="display:flex; gap:10px; margin-bottom:14px;">
    <button id="btn-seq-start" class="go-btn" style="flex:1; padding:13px; font-size:1rem; background:#28a745;"
            onclick="seqStart()">▶ Start run</button>
    <button id="btn-seq-pause" class="go-btn" style="flex:0 0 auto; padding:13px 16px; font-size:1rem; background:#b5821a; display:none;"
            onclick="seqPause()">⏸</button>
    <button id="btn-seq-stop"  class="go-btn" style="flex:1; padding:13px; font-size:1rem; background:#dc3545; display:none;"
            onclick="seqStop()">⏹ Stop run</button>
  </div>

  <!-- Live progress -->
  <div style="background:#12122a; border-radius:10px; padding:14px; margin-bottom:12px;">
    <div style="display:flex; justify-content:space-between; align-items:flex-start; flex-wrap:wrap; gap:10px;">
      <div>
        <div style="font-size:0.72rem; color:#666; text-transform:uppercase; letter-spacing:1px;">Cycle</div>
        <div id="seq-cycle" style="font-size:2.4rem; font-weight:800; color:#e3b87e;
             font-variant-numeric:tabular-nums; line-height:1.1;">0 / 0</div>
        <div id="seq-phase" style="font-size:0.8rem; color:#aaa; margin-top:2px;">idle</div>
      </div>
      <div style="text-align:right;">
        <div style="font-size:0.72rem; color:#666; text-transform:uppercase; letter-spacing:1px;">Marks / elapsed</div>
        <div id="seq-marks" style="font-size:1.4rem; font-weight:600; color:#7ec8e3;
             font-variant-numeric:tabular-nums;">0</div>
        <div id="seq-elapsed" style="font-size:0.8rem; color:#666;">0:00</div>
      </div>
    </div>
    <div style="margin-top:10px; font-size:0.75rem; color:#888; font-variant-numeric:tabular-nums;">
      Commanded position — <span id="seq-pos">X 0 · Y 0 · Z 0</span>
    </div>
    <div style="margin-top:8px; text-align:center;">
      <span id="seq-pill" style="display:inline-block; padding:4px 14px; border-radius:20px;
            font-size:0.75rem; font-weight:600; background:#333; color:#888;">IDLE</span>
    </div>
    <div id="integrity-banner" style="display:none; margin-top:10px; padding:10px;
         border-radius:8px; background:#4a0f1a; border:1px solid #dc3545;
         font-size:0.78rem; color:#ffb3bd; text-align:center;"></div>
    <div id="csv-path" style="margin-top:8px; font-size:0.68rem; color:#555;
         font-family:monospace; word-break:break-all;"></div>
  </div>

  <!-- Log -->
  <div id="seq-log">No run yet.</div>
  <div style="display:flex; gap:8px; margin-top:8px;">
    <a class="go-btn" href="/seq/log.csv" download
       style="flex:1; text-decoration:none; text-align:center; line-height:1.6; background:#2a5a8a;">⬇ Download event CSV</a>
  </div>
  <p class="info-note">
    The CSV carries wall-clock and elapsed time for every move and mark, plus commanded position in
    steps and nm. Line it up against your Tracker positions to get per-approach deviation, then
    take the standard deviation across cycles for repeatability and the forward/reverse mean
    difference for hysteresis. Starting a new run clears this log — download it first.
  </p>
</div>

<!-- ── MOTION TUNING ── -->
<div class="panel">
  <h2>🎛 Motion Tuning</h2>

  <div class="param-row">
    <label>Jog Steps / Tap</label>
    <input type="range" id="sl-jogsteps" min="1" max="200" value="25" step="1"
           oninput="updateParam('jog_steps', this.value, 'lbl-jogsteps', ' steps')"
           onchange="sendParam('J:' + this.value)">
    <div class="param-val" id="lbl-jogsteps">25 steps</div>
  </div>
  <p class="info-note" style="margin:-8px 0 10px;">
    ↑ Primary step-size control. Exact microsteps per tap — deterministic regardless of USB latency.
  </p>

  <div class="param-row">
    <label>Motor Speed</label>
    <input type="range" id="sl-speed" min="5" max="500" value="50" step="5"
           oninput="updateParam('motor_speed', this.value, 'lbl-speed', ' stp/s')"
           onchange="sendParam('S:' + this.value)">
    <div class="param-val" id="lbl-speed">50 stp/s</div>
  </div>

  <div class="param-row">
    <label>Startup Accel</label>
    <input type="range" id="sl-accel" min="50" max="2000" value="200" step="25"
           oninput="updateParam('startup_accel', this.value, 'lbl-accel', ' stp/s²')"
           onchange="sendParam('C:' + this.value)">
    <div class="param-val" id="lbl-accel">200 stp/s²</div>
  </div>
  <p class="info-note" style="margin:-8px 0 10px;">
    ↑ Smooths startup jerk. Lower = gentler ramp, less vibration. Higher = snappier but more jolt.
    50–300 recommended for precision work.
  </p>

  <div class="param-row">
    <label>Safety Timeout</label>
    <input type="range" id="sl-timeout" min="50" max="2000" value="350" step="10"
           oninput="updateParam('timeout_ms', this.value, 'lbl-timeout', ' ms')"
           onchange="sendParam('M:' + this.value)">
    <div class="param-val" id="lbl-timeout">350 ms</div>
  </div>
  <p class="info-note" style="margin:-8px 0 10px;">
    ↑ Backup safety cutoff only (motor stops if no command arrives). Step size now controlled by Jog Steps above.
  </p>

  <div class="param-row">
    <label>Flexure Atten.</label>
    <input type="range" id="sl-atten" min="0.3" max="1.0" value="0.82" step="0.01"
           oninput="updateParam('flexure_attenuation', this.value, 'lbl-atten', '')"
           onchange="sendParam('A:' + this.value)">
    <div class="param-val" id="lbl-atten">0.82</div>
  </div>
  <p class="info-note" style="margin:-8px 0 10px;">
    ↑ Update this whenever you move to a different point on the flexure's range — attenuation
    varies with deflection (measured ~0.82 near neutral, ~0.43 at higher deflection).
  </p>

  <div class="param-row">
    <label>LED Brightness</label>
    <input type="range" id="sl-led" min="0" max="255" value="200" step="5"
           oninput="updateParam('led_brightness', this.value, 'lbl-led', '')"
           onchange="sendParam('B:' + this.value)">
    <div class="param-val" id="lbl-led">200</div>
  </div>

  <div class="toggle-row">
    <span class="toggle-label">Hold coils when idle — eliminates startup jerk entirely (motor stays warm)</span>
    <label class="toggle">
      <input type="checkbox" id="chk-hold" checked onchange="sendParam('H:' + (this.checked ? '1' : '0'))">
      <span class="slider-tog"></span>
    </label>
  </div>
</div>

<!-- ── DIAGONAL MOTION TUNING ── -->
<div class="panel">
  <h2>⤢ Diagonal Motion Tuning</h2>
  <p class="info-note" style="margin:0 0 10px;">
    Your X and Y share the same V-springs, so tapping one naturally nudges the other. Tap R/L a few
    times while watching the platform, then adjust "Y comp" below until the drift disappears.
    Do the same for U/D using "X comp". Speed/accel let you shape each motor's ramp so both finish
    together (a mismatched finish looks like a snap-then-hold instead of a clean diagonal cancel).
    Use the XY square run above to score how well a comp setting actually closes.
  </p>

  <div class="param-row">
    <label>X Speed</label>
    <input type="range" id="sl-xspeed" min="5" max="500" value="50" step="5"
           oninput="updateParam('x_speed', this.value, 'lbl-xspeed', ' stp/s')"
           onchange="sendParam('XS:' + this.value)">
    <div class="param-val" id="lbl-xspeed">50 stp/s</div>
  </div>
  <div class="param-row">
    <label>Y Speed</label>
    <input type="range" id="sl-yspeed" min="5" max="500" value="50" step="5"
           oninput="updateParam('y_speed', this.value, 'lbl-yspeed', ' stp/s')"
           onchange="sendParam('YS:' + this.value)">
    <div class="param-val" id="lbl-yspeed">50 stp/s</div>
  </div>

  <div class="param-row">
    <label>X Accel</label>
    <input type="range" id="sl-xaccel" min="50" max="2000" value="200" step="25"
           oninput="updateParam('x_accel', this.value, 'lbl-xaccel', ' stp/s²')"
           onchange="sendParam('XA:' + this.value)">
    <div class="param-val" id="lbl-xaccel">200 stp/s²</div>
  </div>
  <div class="param-row">
    <label>Y Accel</label>
    <input type="range" id="sl-yaccel" min="50" max="2000" value="200" step="25"
           oninput="updateParam('y_accel', this.value, 'lbl-yaccel', ' stp/s²')"
           onchange="sendParam('YA:' + this.value)">
    <div class="param-val" id="lbl-yaccel">200 stp/s²</div>
  </div>

  <div class="param-row">
    <label>X Steps/Tap</label>
    <input type="range" id="sl-xsteps" min="1" max="200" value="25" step="1"
           oninput="updateParam('x_steps', this.value, 'lbl-xsteps', ' steps')"
           onchange="sendParam('XJ:' + this.value)">
    <div class="param-val" id="lbl-xsteps">25 steps</div>
  </div>
  <div class="param-row">
    <label>Y Steps/Tap</label>
    <input type="range" id="sl-ysteps" min="1" max="200" value="25" step="1"
           oninput="updateParam('y_steps', this.value, 'lbl-ysteps', ' steps')"
           onchange="sendParam('YJ:' + this.value)">
    <div class="param-val" id="lbl-ysteps">25 steps</div>
  </div>

  <div class="param-row">
    <label>Y Comp (on X tap)</label>
    <input type="range" id="sl-ycomp" min="-150" max="150" value="0" step="0.5"
           oninput="updateComp('y_comp_ratio', this.value, 'lbl-ycomp')"
           onchange="sendParam('YC:' + (this.value/100))">
    <div class="param-val" id="lbl-ycomp">0%</div>
    <button class="flip-btn" onclick="flipComp('sl-ycomp', 'y_comp_ratio', 'lbl-ycomp', 'YC:')">⇄ Flip</button>
  </div>
  <div class="param-row">
    <label>X Comp (on Y tap)</label>
    <input type="range" id="sl-xcomp" min="-150" max="150" value="0" step="0.5"
           oninput="updateComp('x_comp_ratio', this.value, 'lbl-xcomp')"
           onchange="sendParam('XC:' + (this.value/100))">
    <div class="param-val" id="lbl-xcomp">0%</div>
    <button class="flip-btn" onclick="flipComp('sl-xcomp', 'x_comp_ratio', 'lbl-xcomp', 'XC:')">⇄ Flip</button>
  </div>
  <p class="info-note" style="margin:-8px 0 0;">
    ↑ This is a RATIO (% of the primary move), not a fixed step count — so it scales correctly
    automatically no matter what X/Y Steps-per-Tap you're using. "Flip" instantly reverses which
    way the compensating motor turns (same magnitude, opposite sign) — use it if a correction is
    making the drift worse instead of better. The OTHER direction (L instead of R, D instead of U)
    already mirrors automatically — you never need to re-tune per direction, only per axis.
  </p>
</div>

<!-- ── HARDWARE CONFIG ── -->
<div class="panel">
  <h2>🔧 Hardware Config</h2>

  <div class="param-row">
    <label>Microsteps</label>
    <select onchange="updateHardware('microsteps', this.value); sendParam('MS:' + this.value)">
      <option value="8">1/8 &mdash; MS1=0, MS2=0</option>
      <option value="2">1/2 &mdash; MS1=1, MS2=0</option>
      <option value="4">1/4 &mdash; MS1=0, MS2=1</option>
      <option value="16" selected>1/16 &mdash; MS1=1, MS2=1</option>
    </select>
    <div class="param-val" style="width:auto; font-size:0.7rem; color:#666;">rewire pins</div>
  </div>

  <div class="param-row">
    <label>Screw Pitch</label>
    <select onchange="updateHardware('screw_pitch_mm', this.value); sendParam('P:' + this.value)">
      <option value="0.5" selected>M3 — 0.5 mm/rev</option>
      <option value="0.4">M2.5 — 0.4 mm/rev</option>
      <option value="0.35">M2 — 0.35 mm/rev</option>
      <option value="0.25">M2 fine — 0.25 mm/rev</option>
    </select>
    <div class="param-val" style="width:auto;"></div>
  </div>

  <div class="param-row">
    <label>Gear Ratio</label>
    <input type="number" id="in-gear" value="64" min="1" max="500" step="1"
           style="flex:1; background:#12122a; border:1px solid #3a3a6a; color:#e0e0e0; border-radius:6px; padding:6px 8px;"
           onchange="updateHardware('gear_ratio', this.value); sendParam('G:' + this.value)">
    <div class="param-val" style="width:auto;">:1</div>
  </div>

  <div class="param-row">
    <label>Motor Steps/Rev</label>
    <input type="number" id="in-mspr" value="32" min="1" max="400" step="1"
           style="flex:1; background:#12122a; border:1px solid #3a3a6a; color:#e0e0e0; border-radius:6px; padding:6px 8px;"
           onchange="updateHardware('motor_steps_per_rev', this.value); sendParam('N:' + this.value)">
    <div class="param-val" style="width:auto;"></div>
  </div>

  <p class="info-note">
    Microsteps and gear/motor constants are physically set by your wiring — these controls update
    the calculation so nm figures stay accurate, they do not change the hardware.
  </p>
  <p class="info-note" style="color:#e3b87e;">
    The TMC220x standalone table is not the A4988 ordering — 1/8 is both pins low, and
    <strong>1/16 is the ceiling on pins alone</strong>. microPlyer's 256-step interpolation smooths
    the current waveform between commanded steps; it is not addressable resolution. True 1/256
    needs the UART interface. Whatever is selected here scales every nm number the system reports,
    so check it against the actual pins before a recording session.
  </p>
</div>

<!-- ── PRECISION MOVE ── -->
<div class="panel">
  <h2>🎯 Precision Microstep Move</h2>
  <div class="precision-row">
    <select id="prec-axis">
      <option>X</option>
      <option>Y</option>
      <option>Z</option>
    </select>
    <input type="number" id="prec-steps" value="1" min="-500" max="500">
    <button class="go-btn" onclick="precisionMove()">Move exactly N steps →</button>
  </div>
  <div id="prec-estimate" class="info-note">≈ — nm per step</div>
</div>

<!-- ── AUTO-TAP ── -->
<div class="panel" id="auto-panel">
  <h2>🤖 Auto-Tap Sequence</h2>

  <div style="display:flex; gap:8px; flex-wrap:wrap; margin-bottom:12px; align-items:center;">
    <span style="font-size:0.8rem; color:#aaa; width:80px;">Direction</span>
    <div style="display:flex; gap:6px; flex-wrap:wrap;">
      <button class="dir-btn active-dir" data-cmd="R"  onclick="setDir(this)">X  ▶</button>
      <button class="dir-btn"            data-cmd="L"  onclick="setDir(this)">X  ◀</button>
      <button class="dir-btn"            data-cmd="U"  onclick="setDir(this)">Y  ▲</button>
      <button class="dir-btn"            data-cmd="D"  onclick="setDir(this)">Y  ▼</button>
      <button class="dir-btn"            data-cmd="+"  onclick="setDir(this)">Z  ▲</button>
      <button class="dir-btn"            data-cmd="-"  onclick="setDir(this)">Z  ▼</button>
    </div>
  </div>

  <div class="param-row">
    <label>Tap Delay</label>
    <input type="range" id="sl-delay" min="50" max="5000" value="500" step="50"
           oninput="autoDelayInput(this.value)"
           onchange="autoDelayCommit(this.value)">
    <div class="param-val" id="lbl-delay">500 ms</div>
  </div>
  <p class="info-note" style="margin:-8px 0 12px;">
    Minimum 50 ms. For data collection at ~20 nm/step allow ≥ 1000 ms so the stage fully settles.
  </p>

  <div style="display:flex; gap:10px; margin-bottom:14px;">
    <button id="btn-auto-start" class="go-btn"
            style="flex:1; padding:13px; font-size:1rem; background:#28a745;"
            onclick="autoStart()">▶ Start</button>
    <button id="btn-auto-stop"  class="go-btn"
            style="flex:1; padding:13px; font-size:1rem; background:#dc3545; display:none;"
            onclick="autoStop()">⏹ Stop</button>
  </div>

  <div style="background:#12122a; border-radius:10px; padding:14px; display:flex;
              align-items:center; justify-content:space-between; flex-wrap:wrap; gap:10px;">
    <div>
      <div style="font-size:0.72rem; color:#666; text-transform:uppercase; letter-spacing:1px;">Taps (resettable)</div>
      <div id="tap-count"
           style="font-size:2.8rem; font-weight:800; color:#7ec8e3;
                  font-variant-numeric: tabular-nums; line-height:1.1;">0</div>
      <div id="tap-dist"
           style="font-size:0.78rem; color:#aaa; margin-top:2px;">≈ 0 nm total</div>
    </div>
    <div style="text-align:right;">
      <div style="font-size:0.72rem; color:#666; text-transform:uppercase; letter-spacing:1px;">Session total</div>
      <div id="tap-total"
           style="font-size:1.4rem; font-weight:600; color:#555;
                  font-variant-numeric: tabular-nums;">0</div>
      <button onclick="resetCounter()"
              style="margin-top:8px; padding:7px 14px; background:#444; border:1px solid #666;
                     border-radius:6px; color:#ccc; font-size:0.78rem; cursor:pointer;">
        ↺ Reset counter
      </button>
    </div>
  </div>

  <div style="margin-top:10px; text-align:center;">
    <span id="auto-status-pill"
          style="display:inline-block; padding:4px 14px; border-radius:20px;
                 font-size:0.75rem; font-weight:600; background:#333; color:#888;">
      IDLE
    </span>
  </div>
</div>

<!-- ── SERIAL CONSOLE ── -->
<div class="panel">
  <h2>🖥 Serial Console</h2>
  <div class="precision-row">
    <input type="text" id="raw-cmd" placeholder='e.g.  ?  or  S:80  or  TX1'
           style="flex:1; background:#12122a; border:1px solid #3a3a6a; color:#e0e0e0;
                  border-radius:6px; padding:8px; font-size:0.85rem;"
           onkeydown="if(event.key==='Enter') sendRaw()">
    <button class="go-btn" style="flex:0 0 auto; width:80px;" onclick="sendRaw()">Send</button>
  </div>
  <div id="console-log"
       style="background:#12122a; border-radius:6px; padding:8px; margin-top:8px;
              font-size:0.72rem; font-family:monospace; color:#7ec8e3;
              max-height:100px; overflow-y:auto; text-align:left;">
    Ready.
  </div>
  <p class="info-note" style="margin:8px 0 4px;">Firmware replies <span id="ack-state"></span></p>
  <div id="rx-log"
       style="background:#12122a; border-radius:6px; padding:8px;
              font-size:0.72rem; font-family:monospace; color:#9c9;
              max-height:120px; overflow-y:auto; text-align:left;">
    —
  </div>
</div>

<div id="status"></div>

<!-- ── MANUAL OVERRIDE BAR ── -->
<div id="override-bar">
  <div class="ov-msg" id="ov-msg">Something is running. Any arrow key or jog button takes control immediately.</div>
  <button class="ov-btn ov-take"   onclick="takeControl()">✋ Take control · Esc</button>
  <button class="ov-btn ov-resume" id="ov-resume" style="display:none;" onclick="seqResume()">▶ Resume run</button>
  <button class="ov-btn ov-abort"  onclick="stopEverything()">⏹ Stop everything</button>
</div>

<script>
// ── State mirror (matches server-side stage_params) ──
const p = {
  motor_speed:          50,
  timeout_ms:           350,
  jog_steps:            25,
  startup_accel:        200,
  microsteps:           16,
  screw_pitch_mm:       0.5,
  gear_ratio:           64,
  motor_steps_per_rev:  32,
  flexure_attenuation:  0.82,
  x_speed:              50,
  y_speed:              50,
  x_accel:              200,
  y_accel:              200,
  x_steps:              25,
  y_steps:              25,
  x_comp_ratio:         0,
  y_comp_ratio:         0,
  led_brightness:       200,
};

// ── Test config mirror ──
const sq = {
  mode: 'bidirectional', axis: 'X',
  travel_steps: 200, backoff_steps: 400,
  ladder_steps: 50, ladder_points: 5,
  cycles: 20, settle_ms: 400, dwell_ms: 1500,
  max_excursion: 4000,
};

function nmPerMicrostep() {
  const usteps_per_rev = p.motor_steps_per_rev * p.gear_ratio * p.microsteps;
  return (p.screw_pitch_mm * 1e6) / usteps_per_rev * p.flexure_attenuation;
}
function nmPerTap() {
  if (p.jog_steps > 0) return nmPerMicrostep() * p.jog_steps;
  return nmPerMicrostep() * (p.motor_speed * p.timeout_ms / 1000);
}
function stepsPerTap() {
  if (p.jog_steps > 0) return p.jog_steps;
  return (p.motor_speed * p.timeout_ms / 1000).toFixed(1);
}

function refreshEstimates() {
  document.getElementById('est-nm-us').textContent   = nmPerMicrostep().toFixed(1) + ' nm';
  document.getElementById('est-nm-tap').textContent  = nmPerTap().toFixed(0) + ' nm';
  document.getElementById('est-steps-tap').textContent = stepsPerTap();
  document.getElementById('prec-estimate').textContent =
    '≈ ' + (parseFloat(document.getElementById('prec-steps').value || 1) * nmPerMicrostep()).toFixed(1) + ' nm';
  refreshSeqEstimates();
}

// ── Param update (slider moves) ──
function updateParam(key, val, lblId, unit) {
  p[key] = parseFloat(val);
  const disp = key === 'flexure_attenuation' ? parseFloat(val).toFixed(2) : val;
  document.getElementById(lblId).textContent = disp + unit;
  refreshEstimates();
}

function updateComp(key, pctVal, lblId) {
  p[key] = parseFloat(pctVal) / 100;
  document.getElementById(lblId).textContent = pctVal + '%';
  refreshEstimates();
}

function flipComp(sliderId, key, lblId, cmdPrefix) {
  const slider = document.getElementById(sliderId);
  const newPct = -parseFloat(slider.value);
  slider.value = newPct;
  updateComp(key, newPct, lblId);
  sendParam(cmdPrefix + (newPct / 100));
}

function updateHardware(key, val) {
  p[key] = parseFloat(val);
  refreshEstimates();
}

// ── Send serial command to ESP32 ──
function send(cmd) {
  fetch('/send/' + encodeURIComponent(cmd))
    .then(r => r.text())
    .then(t => setStatus(t, 'ok'))
    .catch(() => setStatus('Serial error', 'err'));
}
function sendParam(cmd) { send(cmd); }

// ── Motion hold (D-pad). Any of these overrides a running job. ──
let moveTimer = null;

function hold(cmd) {
  if (moveTimer) return;
  send(cmd);
  document.querySelectorAll('.btn').forEach(b => b.classList.remove('held'));
  const id = {'U':'btn-U','D':'btn-D','L':'btn-L','R':'btn-R','+':'btn-plus','-':'btn-minus'}[cmd];
  if (id) document.getElementById(id).classList.add('held');
  moveTimer = setInterval(() => send(cmd), 150);
  pollSeq();   // reflect the override in the UI right away
}

function release() {
  if (moveTimer) { clearInterval(moveTimer); moveTimer = null; }
  document.querySelectorAll('.btn').forEach(b => b.classList.remove('held'));
}

// ── Precision move ──
function precisionMove() {
  const axis  = document.getElementById('prec-axis').value;
  const steps = parseInt(document.getElementById('prec-steps').value);
  if (isNaN(steps) || steps === 0) return;
  const cmd = 'T' + axis + steps;
  send(cmd);
  logConsole('→ ' + cmd + '  (' + (steps * nmPerMicrostep()).toFixed(1) + ' nm)');
  pollSeq();
}

// ── Raw serial ──
function sendRaw() {
  const cmd = document.getElementById('raw-cmd').value.trim();
  if (!cmd) return;
  send(cmd);
  logConsole('→ ' + cmd);
  document.getElementById('raw-cmd').value = '';
}

function logConsole(msg) {
  const el = document.getElementById('console-log');
  el.innerHTML += '<br>' + msg;
  el.scrollTop = el.scrollHeight;
}

function setStatus(msg, cls) {
  const el = document.getElementById('status');
  el.textContent = msg;
  el.className = cls || '';
}

// ── Keyboard ──
const KEY_MAP = {
  ArrowUp: 'U', ArrowDown: 'D', ArrowLeft: 'L', ArrowRight: 'R',
  '=': '+', '+': '+', '-': '-', '_': '-'
};
let heldKey = null;
document.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT' && e.target.type === 'text') return;
  if (e.key === 'Escape') { takeControl(); return; }
  if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.key)) e.preventDefault();
  const cmd = KEY_MAP[e.key];
  if (cmd && heldKey !== e.key) { heldKey = e.key; hold(cmd); }
});
document.addEventListener('keyup', e => {
  if (heldKey === e.key) { heldKey = null; release(); }
});

document.getElementById('prec-steps').addEventListener('input', refreshEstimates);

// ═══════════════════════════════════════════
//  MANUAL OVERRIDE
// ═══════════════════════════════════════════
function takeControl() {
  release();
  fetch('/override', { method: 'POST' })
    .then(r => r.json())
    .then(() => { pollSeq(); pollStatus(); setStatus('Manual control', 'ok'); });
}

function stopEverything() {
  release();
  fetch('/stop_all', { method: 'POST' })
    .then(r => r.json())
    .then(() => { autoRunning = false; pollSeq(); pollStatus(); setStatus('All stopped', 'ok'); });
}

function updateOverrideBar(seqState, autoState) {
  const bar = document.getElementById('override-bar');
  const busy = seqState.running || autoState;
  bar.classList.toggle('show', busy);
  document.getElementById('ov-resume').style.display = seqState.paused ? 'inline-block' : 'none';
  document.getElementById('ov-msg').textContent = seqState.paused
    ? 'Run paused at cycle ' + seqState.cycle + '. Jog freely — position is still being tracked — then resume.'
    : 'Running. Any arrow key or jog button takes control immediately.';
}

// ═══════════════════════════════════════════
//  REPEATABILITY TEST SUITE
// ═══════════════════════════════════════════
const MODE_DESC = {
  bidirectional:  'Target approached forwards, origin approached in reverse, every cycle. Spread within each direction is repeatability; the gap between the two means is your hysteresis.',
  unidirectional: 'Every mark is reached from the same side after a back-off run-up, so backlash is excluded. This is your true positioning repeatability.',
  staircase:      'Equal rungs up then back down. Gives linearity across the travel and a full hysteresis loop in one run — useful for mapping how attenuation changes with deflection.',
  square:         'Four legs around a square back to the start. Whatever offset is left at closure is cross-axis coupling — the parasitic diagonal you are compensating for.',
  drift:          'No motion at all. Timed marks only, so you can measure thermal drift and the vibration noise floor of the whole setup.'
};

function setMode(btn) {
  document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active-dir'));
  btn.classList.add('active-dir');
  sq.mode = btn.dataset.mode;
  document.getElementById('mode-desc').textContent = MODE_DESC[sq.mode];
  applyModeVisibility();
  pushSeqCfg();
  refreshSeqEstimates();
}

function applyModeVisibility() {
  const show = (id, on) => document.getElementById(id).style.display = on ? '' : 'none';
  show('row-travel',   sq.mode !== 'staircase' && sq.mode !== 'drift');
  show('travel-nm',    sq.mode !== 'staircase' && sq.mode !== 'drift');
  show('row-backoff',  sq.mode === 'unidirectional');
  show('note-backoff', sq.mode === 'unidirectional');
  show('row-rungsize', sq.mode === 'staircase');
  show('row-rungs',    sq.mode === 'staircase');
  document.getElementById('lbl-cycles-name').textContent =
    sq.mode === 'drift' ? 'Samples' : 'Cycles';
  document.getElementById('axis-note').textContent =
    sq.mode === 'square' ? 'starts here' : (sq.mode === 'drift' ? 'logged only' : 'primary');
}

function seqParam(key, val, lblId, unit) {
  sq[key] = parseFloat(val);
  document.getElementById(lblId).textContent = val + unit;
  refreshSeqEstimates();
}

function pushSeqCfg() {
  sq.axis = document.getElementById('seq-axis').value;
  fetch('/seq/config', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({
      ...sq,
      marker_enabled: document.getElementById('chk-marker').checked,
      use_comp: document.getElementById('chk-use-comp').checked,
      verify: document.getElementById('chk-verify').checked,
      on_manual: document.getElementById('chk-abort-on-manual').checked ? 'abort' : 'pause'
    })
  }).catch(() => {});
}

// Mirror of the server's trapezoidal move-time estimate
function moveTimeS(steps, axis) {
  const n = Math.abs(steps);
  if (n === 0) return 0;
  let v, a;
  if (axis === 'X')      { v = p.x_speed; a = p.x_accel; }
  else if (axis === 'Y') { v = p.y_speed; a = p.y_accel; }
  else                   { v = p.motor_speed; a = p.startup_accel; }
  v = Math.max(v, 1); a = Math.max(a, 1);
  const nRamp = v * v / a;
  return n >= nRamp ? 2 * (v / a) + (n - nRamp) / v : 2 * Math.sqrt(n / a);
}

function fmtTime(s) {
  s = Math.round(s);
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), ss = s % 60;
  return h ? h + 'h ' + m + 'm' : (m ? m + 'm ' + ss + 's' : ss + 's');
}

function refreshSeqEstimates() {
  const ax    = document.getElementById('seq-axis').value;
  const nm    = nmPerMicrostep();
  const dwell = (sq.settle_ms + sq.dwell_ms) / 1000;
  let marks = 1, secs = dwell, spanSteps = 0;   // origin reference mark

  if (sq.mode === 'bidirectional') {
    marks += 2 * sq.cycles;
    secs  += sq.cycles * (2 * moveTimeS(sq.travel_steps, ax) + 2 * dwell);
    spanSteps = sq.travel_steps;
  } else if (sq.mode === 'unidirectional') {
    marks += 2 * sq.cycles;
    secs  += sq.cycles * (moveTimeS(sq.travel_steps, ax)
                        + moveTimeS(sq.travel_steps + sq.backoff_steps, ax)
                        + moveTimeS(sq.backoff_steps, ax)
                        + 2 * dwell + sq.settle_ms / 1000);
    spanSteps = sq.travel_steps + sq.backoff_steps;
  } else if (sq.mode === 'staircase') {
    const n = 2 * sq.ladder_points;
    marks += n * sq.cycles;
    secs  += sq.cycles * n * (moveTimeS(sq.ladder_steps, ax) + dwell);
    spanSteps = sq.ladder_points * sq.ladder_steps;
  } else if (sq.mode === 'square') {
    marks += 5 * sq.cycles;
    secs  += sq.cycles * (4 * moveTimeS(sq.travel_steps, ax) + 5 * dwell);
    spanSteps = sq.travel_steps;
  } else if (sq.mode === 'drift') {
    marks += sq.cycles;
    secs  += sq.cycles * dwell;
    spanSteps = 0;
  }

  document.getElementById('est-marks').textContent   = marks;
  document.getElementById('est-runtime').textContent = fmtTime(secs);
  document.getElementById('est-span').textContent    =
    spanSteps ? spanSteps + ' steps · ' + (spanSteps * nm / 1000).toFixed(2) + ' µm' : 'stationary';
  const tn = document.getElementById('travel-nm');
  if (tn) tn.textContent = '≈ ' + (sq.travel_steps * nm).toFixed(0) + ' nm ('
                         + (sq.travel_steps * nm / 1000).toFixed(3) + ' µm) per leg';
}

function seqStart(force) {
  pushSeqCfg();
  fetch('/seq/start', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({ force: !!force })
  })
    .then(r => r.json())
    .then(res => {
      if (res.preflight) {
        const msg = 'This run would not produce defensible data:\n\n  • '
                  + res.preflight.join('\n  • ')
                  + '\n\nStart anyway? The CSV will record that checks were bypassed.';
        if (confirm(msg)) seqStart(true);
        else setStatus('Run blocked by preflight', 'err');
        return;
      }
      if (!res.ok) { setStatus(res.msg || 'Could not start', 'err'); return; }
      if (res.forced) setStatus('Started with preflight bypassed', 'err');
      pollSeq();
      startSeqPolling();
    });
}
function seqStop()   { fetch('/seq/stop',   { method: 'POST' }).then(pollSeq); }
function seqPause()  { fetch('/seq/pause',  { method: 'POST' }).then(pollSeq); }
function seqResume() { fetch('/seq/resume', { method: 'POST' }).then(pollSeq); }

let seqPollTimer = null;
function startSeqPolling() {
  if (seqPollTimer) return;
  seqPollTimer = setInterval(pollSeq, 400);
}

function pollSeq() {
  return fetch('/seq/status')
    .then(r => r.json())
    .then(s => {
      const running = s.running, paused = s.paused;
      document.getElementById('btn-seq-start').style.display = running ? 'none' : 'block';
      document.getElementById('btn-seq-pause').style.display = running ? 'block' : 'none';
      document.getElementById('btn-seq-stop').style.display  = running ? 'block' : 'none';
      document.getElementById('btn-seq-pause').textContent   = paused ? '▶' : '⏸';
      document.getElementById('btn-seq-pause').onclick        = paused ? seqResume : seqPause;

      document.getElementById('seq-cycle').textContent   = s.cycle + ' / ' + s.cycles;
      document.getElementById('seq-phase').textContent   = s.phase;
      document.getElementById('seq-marks').textContent   = s.marks;
      document.getElementById('seq-elapsed').textContent = fmtTime(s.elapsed);
      const nm = nmPerMicrostep();
      document.getElementById('seq-pos').textContent =
        'X ' + s.pos.X + ' (' + (s.pos.X * nm / 1000).toFixed(3) + ' µm) · ' +
        'Y ' + s.pos.Y + ' (' + (s.pos.Y * nm / 1000).toFixed(3) + ' µm) · Z ' + s.pos.Z;

      const pill = document.getElementById('seq-pill');
      if (paused)       { pill.textContent = '⏸ PAUSED — you have control'; pill.className = 'pill-paused'; }
      else if (running) { pill.textContent = '● RUNNING — ' + s.mode + ' ' + s.axis; pill.className = 'pill-running'; }
      else              { pill.textContent = s.phase === 'complete' ? '✓ COMPLETE' : 'IDLE'; pill.className = 'pill-idle'; }

      // log tail
      if (s.log && s.log.length) {
        document.getElementById('seq-log').innerHTML = s.log.map(r => {
          const cls = r.event === 'MARK' ? 'mark' : (r.event === 'MANUAL OVERRIDE' ? 'man' : '');
          return '<div class="' + cls + '">' + r.elapsed_s.toFixed(2).padStart(8) + 's  c' + r.cycle +
                 '  ' + r.event + '  ' + r.note + '  [X' + r.x_steps + ' Y' + r.y_steps + ']</div>';
        }).join('');
      }

      const ib = document.getElementById('integrity-banner');
      if (s.integrity === 'FAILED') {
        ib.style.display = 'block';
        ib.textContent = 'Run aborted on an integrity check — ' + s.last_note
                       + '. Do not use this data. The CSV records what happened.';
      } else { ib.style.display = 'none'; }
      document.getElementById('csv-path').textContent =
        s.csv_path ? 'Recording to ' + s.csv_path : '';

      updateOverrideBar(s, autoRunning);
      if (!running && seqPollTimer && !paused) { clearInterval(seqPollTimer); seqPollTimer = null; }
      return s;
    })
    .catch(() => {});
}

// ═══════════════════════════════════════════
//  AUTO-TAP
// ═══════════════════════════════════════════
let autoDir      = 'R';
let autoRunning  = false;
let pollTimer    = null;

function setDir(btn) {
  document.querySelectorAll('.dir-btn:not(.mode-btn)').forEach(b => b.classList.remove('active-dir'));
  btn.classList.add('active-dir');
  autoDir = btn.dataset.cmd;
  if (autoRunning) {
    fetch('/auto/set', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({ direction: autoDir })
    });
  }
}

function autoDelayInput(val) {
  document.getElementById('lbl-delay').textContent = val + ' ms';
}

function autoDelayCommit(val) {
  document.getElementById('lbl-delay').textContent = val + ' ms';
  if (autoRunning) {
    fetch('/auto/set', {
      method: 'POST',
      headers: {'Content-Type':'application/json'},
      body: JSON.stringify({ delay_ms: parseInt(val) })
    });
  }
}

function autoStart() {
  const delay = parseInt(document.getElementById('sl-delay').value);
  fetch('/auto/start', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({ direction: autoDir, delay_ms: delay })
  })
  .then(r => r.json())
  .then(() => {
    autoRunning = true;
    document.getElementById('btn-auto-start').style.display = 'none';
    document.getElementById('btn-auto-stop').style.display  = 'block';
    setPill(true);
    startPolling();
    pollSeq();
  });
}

function autoStop() {
  fetch('/auto/stop', { method: 'POST' })
  .then(() => {
    autoRunning = false;
    document.getElementById('btn-auto-start').style.display = 'block';
    document.getElementById('btn-auto-stop').style.display  = 'none';
    setPill(false);
    stopPolling();
    pollSeq();
  });
}

function resetCounter() {
  fetch('/auto/reset_counter', { method: 'POST' })
  .then(() => {
    document.getElementById('tap-count').textContent = '0';
    document.getElementById('tap-dist').textContent  = '≈ 0 nm total';
  });
}

function setPill(running) {
  const pill = document.getElementById('auto-status-pill');
  if (running) {
    pill.textContent = '● RUNNING — ' + autoDir;
    pill.className = 'pill-running';
  } else {
    pill.textContent = 'IDLE';
    pill.className = 'pill-idle';
  }
}

function startPolling() {
  if (pollTimer) return;
  pollTimer = setInterval(pollStatus, 300);
}
function stopPolling() {
  if (pollTimer) { clearInterval(pollTimer); pollTimer = null; }
  pollStatus();
}

function pollStatus() {
  fetch('/auto/status')
  .then(r => r.json())
  .then(s => {
    const count = s.tap_count;
    const total = s.total_taps;
    const dist  = (count * nmPerTap()).toFixed(0);
    document.getElementById('tap-count').textContent = count;
    document.getElementById('tap-total').textContent = total;
    document.getElementById('tap-dist').textContent  = '≈ ' + dist + ' nm total  (' + (dist/1000).toFixed(3) + ' µm)';
    if (!s.running && autoRunning) { autoStop(); }
  })
  .catch(() => {});
}

// ── Firmware reply stream ──
function pollRx() {
  fetch('/serial/rx')
    .then(r => r.json())
    .then(d => {
      const el = document.getElementById('rx-log');
      if (d.lines && d.lines.length) {
        el.textContent = d.lines.join('\n');
        el.scrollTop = el.scrollHeight;
      }
      const a = document.getElementById('ack-state');
      a.textContent = d.acks === true  ? '— firmware v' + d.version + ', move acks active'
                    : d.acks === false ? '— firmware v' + d.version + ' is too old, flash v7'
                    : '— version unknown';
      a.style.color = d.acks === false ? '#e3b87e' : '#666';
    })
    .catch(() => {});
}
setInterval(pollRx, 1000);
pollRx();

// ── Init ──
document.getElementById('mode-desc').textContent = MODE_DESC[sq.mode];
applyModeVisibility();
refreshEstimates();
pollStatus();
pollSeq();
setInterval(() => { if (!seqPollTimer) pollSeq(); }, 1500);   // catch state changes from other tabs
</script>
</body>
</html>
"""

# ─────────────────────────────────────────────
#  ROUTES
# ─────────────────────────────────────────────

@app.route('/')
def index():
    return HTML

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/send/<path:cmd>')
def send_command(cmd):
    """Forward any command string directly to the ESP32 over serial."""
    cmd = cmd.strip()
    if not cmd:
        return "Empty command", 400

    # Operator input beats any running job.
    overridden = False
    if _is_motion_cmd(cmd):
        overridden = manual_override(cmd)
        _track_manual_position(cmd)

    # Update server-side mirror so estimates stay in sync
    if cmd.startswith('S:'):
        try: stage_params['motor_speed']   = float(cmd[2:])
        except: pass
    elif cmd.startswith('M:'):
        try: stage_params['timeout_ms']    = int(cmd[2:])
        except: pass
    elif cmd.startswith('J:'):
        try: stage_params['jog_steps']     = int(cmd[2:])
        except: pass
    elif cmd.startswith('C:'):
        try: stage_params['startup_accel'] = float(cmd[2:])
        except: pass
    elif cmd.startswith('H:'):
        stage_params['hold_when_idle'] = cmd[2:] == '1'
    elif cmd.startswith('XS:'):
        try: stage_params['x_speed'] = float(cmd[3:])
        except: pass
    elif cmd.startswith('YS:'):
        try: stage_params['y_speed'] = float(cmd[3:])
        except: pass
    elif cmd.startswith('XA:'):
        try: stage_params['x_accel'] = float(cmd[3:])
        except: pass
    elif cmd.startswith('YA:'):
        try: stage_params['y_accel'] = float(cmd[3:])
        except: pass
    elif cmd.startswith('XJ:'):
        try: stage_params['x_steps'] = int(cmd[3:])
        except: pass
    elif cmd.startswith('YJ:'):
        try: stage_params['y_steps'] = int(cmd[3:])
        except: pass
    elif cmd.startswith('XC:'):
        try: stage_params['x_comp_ratio'] = float(cmd[3:])
        except: pass
    elif cmd.startswith('YC:'):
        try: stage_params['y_comp_ratio'] = float(cmd[3:])
        except: pass
    elif cmd.startswith('A:'):
        try: stage_params['flexure_attenuation'] = float(cmd[2:])
        except: pass
    elif cmd.startswith('MS:'):
        try: stage_params['microsteps'] = int(cmd[3:])
        except: pass
    elif cmd.startswith('P:'):
        try: stage_params['screw_pitch_mm'] = float(cmd[2:])
        except: pass
    elif cmd.startswith('G:'):
        try: stage_params['gear_ratio'] = float(cmd[2:])
        except: pass
    elif cmd.startswith('N:'):
        try: stage_params['motor_steps_per_rev'] = float(cmd[2:])
        except: pass
    elif cmd.startswith('B:'):
        try: stage_params['led_brightness'] = int(cmd[2:])
        except: pass

    send_serial(cmd)
    return (f"Sent: {cmd} — manual override" if overridden else f"Sent: {cmd}"), 200

@app.route('/serial/rx')
def serial_rx_route():
    return jsonify({"lines": list(serial_rx)[-40:],
                    "acks": (fw_version >= 7) if fw_version is not None else None,
                    "version": fw_version})

@app.route('/params')
def get_params():
    return jsonify({
        **stage_params,
        "nm_per_microstep": round(nm_per_microstep(stage_params), 2),
        "nm_per_tap":        round(nm_per_tap(stage_params), 1),
    })

# ── Manual override routes ───────────────────

@app.route('/override', methods=['POST'])
def route_override():
    """Take control: stop auto-tap, pause (or abort) the test run."""
    manual_override('operator')
    return jsonify({"ok": True, "seq_paused": seq["paused"], "seq_running": seq["running"]})

@app.route('/stop_all', methods=['POST'])
def route_stop_all():
    stop_auto_tap()
    stop_sequence("stop everything")
    return jsonify({"ok": True})

# ── Auto-tap routes ──────────────────────────

@app.route('/auto/status')
def auto_status():
    with auto_tap_lock:
        return jsonify(dict(auto_tap))

@app.route('/auto/start', methods=['POST'])
def auto_start():
    if seq["running"]:
        return jsonify({"ok": False, "msg": "A test run is active — stop it first"})
    data = request.get_json(silent=True) or {}
    with auto_tap_lock:
        if 'direction' in data:
            auto_tap['direction'] = data['direction']
        if 'delay_ms'  in data:
            auto_tap['delay_ms']  = max(50, int(data['delay_ms']))
    start_auto_tap()
    return jsonify({"ok": True, "direction": auto_tap["direction"], "delay_ms": auto_tap["delay_ms"]})

@app.route('/auto/stop', methods=['POST'])
def auto_stop():
    stop_auto_tap()
    return jsonify({"ok": True})

@app.route('/auto/reset_counter', methods=['POST'])
def auto_reset_counter():
    with auto_tap_lock:
        auto_tap['tap_count'] = 0
    return jsonify({"ok": True})

@app.route('/auto/set', methods=['POST'])
def auto_set():
    data = request.get_json(silent=True) or {}
    with auto_tap_lock:
        if 'direction' in data:
            auto_tap['direction'] = data['direction']
        if 'delay_ms'  in data:
            auto_tap['delay_ms']  = max(50, int(data['delay_ms']))
    return jsonify(dict(auto_tap))

# ── Repeatability test routes ────────────────

SEQ_INT_KEYS   = ("travel_steps", "cycles", "backoff_steps", "ladder_points",
                  "ladder_steps", "settle_ms", "dwell_ms", "max_excursion")
SEQ_OTHER_KEYS = ("mode", "axis", "marker_enabled", "on_manual", "use_comp", "verify")

@app.route('/seq/config', methods=['POST'])
def seq_config():
    data = request.get_json(silent=True) or {}
    for k in SEQ_INT_KEYS:
        if k in data:
            try: seq[k] = int(float(data[k]))
            except: pass
    for k in SEQ_OTHER_KEYS:
        if k in data:
            seq[k] = data[k]
    seq["settle_ms"] = max(0, seq["settle_ms"])
    seq["dwell_ms"]  = max(100, seq["dwell_ms"])
    return jsonify({"ok": True})

@app.route('/seq/start', methods=['POST'])
def seq_start():
    if auto_tap["running"]:
        return jsonify({"ok": False, "msg": "Auto-tap is running — stop it first"})
    data = request.get_json(silent=True) or {}
    problems = preflight()
    if problems and not data.get('force'):
        return jsonify({"ok": False, "preflight": problems})
    ok = start_sequence()
    return jsonify({"ok": ok, "msg": "" if ok else "Already running",
                    "forced": bool(problems)})

@app.route('/seq/preflight')
def seq_preflight():
    return jsonify({"problems": preflight(), "firmware_version": fw_version})

@app.route('/seq/stop', methods=['POST'])
def seq_stop():
    stop_sequence("stop button")
    return jsonify({"ok": True})

@app.route('/seq/pause', methods=['POST'])
def seq_pause():
    if seq["running"]:
        seq["paused"] = True
        seq["marker"] = False
        seq["phase"]  = "PAUSED"
        _log_event("PAUSE")
    return jsonify({"ok": True})

@app.route('/seq/resume', methods=['POST'])
def seq_resume():
    if seq["running"]:
        resync_position("after manual control")
        seq["paused"] = False
        _log_event("RESUME")
    return jsonify({"ok": True})

@app.route('/seq/status')
def seq_status():
    s = {k: seq[k] for k in seq if k != "pos"}
    s["pos"] = dict(seq["pos"])
    s["log"] = seq_log[-14:]
    return jsonify(s)

@app.route('/seq/log.csv')
def seq_log_csv():
    """Serve the file already on disk, so what you download is byte-for-byte what
    was recorded — metadata preamble included. Falls back to the in-memory tail
    only if no run file exists yet."""
    path = seq.get("csv_path")
    if path and os.path.exists(path):
        if _csv_file:
            try: _csv_file.flush()
            except Exception: pass
        with open(path, 'r', encoding='utf-8') as f:
            body = f.read()
        return Response(body, mimetype='text/csv',
                        headers={"Content-Disposition":
                                 f"attachment; filename={os.path.basename(path)}"})
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=CSV_COLS, extrasaction='ignore')
    w.writeheader()
    for r in list(seq_log):
        w.writerow(r)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Response(buf.getvalue(), mimetype='text/csv',
                    headers={"Content-Disposition": f"attachment; filename=stage_{stamp}.csv"})

# ─────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────
if __name__ == '__main__':
    print("\n" + "═"*50)
    print("  FLEXURE STAGE CONTROLLER — metrology build")
    print("═"*50)
    print(f"  Serial port : {SERIAL_PORT}")
    print(f"  Camera index: {CAM_INDEX}")
    print(f"  nm/microstep: {nm_per_microstep(stage_params):.2f} nm")
    print(f"  nm/tap (est): {nm_per_tap(stage_params):.1f} nm")
    print("═"*50)
    print("  Open in browser:  http://<your-ip>:5002")
    print("  Debug params:     http://<your-ip>:5002/params")
    print("  Run event CSV:    http://<your-ip>:5002/seq/log.csv")
    print("═"*50 + "\n")
    app.run(host='0.0.0.0', port=5002, threaded=True)
