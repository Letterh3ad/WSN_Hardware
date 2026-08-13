#!/usr/bin/env python3
# Laptop-side control for the WSN base station.
# Talks to the Jetson over SSH (key-based auth assumed) and drives the CLI flags
# on wsn_dashboard.py. Uses the system ssh binary via subprocess so it needs no
# third-party packages and nothing extra installed on the Jetson.

import argparse
import json
import subprocess
import sys
import time

# defaults, override per-invocation with --host/--user/--remote-script
JETSON_HOST = "192.168.55.1"
JETSON_USER = "letterhead"
REMOTE_PYTHON = "python3"
REMOTE_SCRIPT = "~/wsn/wsn_dashboard.py"

SENSOR_ORDER = [
    ("co2", "CO2", "ppm"),
    ("temp", "Temp", "C"),
    ("hum", "Humidity", "%"),
    ("clear", "Light clear", ""),
    ("nir", "Light NIR", ""),
    ("light_gain", "Light gain", "x"),
    ("soil_moisture", "Soil moisture", "%"),
    ("soil_temp", "Soil temp", "C"),
    ("soil_ec", "Soil EC", ""),
    ("soil_ph", "Soil pH", ""),
    ("soil_n", "Soil N", ""),
    ("soil_p", "Soil P", ""),
    ("soil_k", "Soil K", ""),
]

CLEAR_SCREEN = "\033[2J\033[H"


def ssh_target(args):
    return f"{args.user}@{args.host}"


def build_remote_cmd(args, flags):
    inner = f"{args.remote_python} {args.remote_script} " + " ".join(flags)
    return ["ssh", ssh_target(args), inner]


def run_remote(args, flags, timeout=20):
    # returns stdout on success, raises RuntimeError with stderr on failure
    cmd = build_remote_cmd(args, flags)
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        raise RuntimeError("ssh timed out")
    if proc.returncode != 0:
        raise RuntimeError(proc.stderr.strip() or f"ssh exited {proc.returncode}")
    return proc.stdout


def fmt_value(v):
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:g}"
    return str(v)


def charging_label(charging):
    # charging is read straight from the payload, never recomputed here, so the
    # live view and the exported csv always agree on this field
    if charging is None:
        return "unknown"
    return "charging" if charging else "discharging"


def render_latest(data):
    lines = []
    interval = data.get("sleep_interval_s")
    link = data.get("link") or {}
    age = link.get("age_s")
    last_seen = link.get("last_seen")

    if age is None:
        link_state = "no contact yet"
    elif interval and age < interval * 2 + 30:
        link_state = "live"
    else:
        link_state = "stale"

    lines.append("WSN NODE  " + link_state.upper())
    lines.append("")
    lines.append(f"  last packet   {last_seen or '-'}")
    lines.append(f"  age           {fmt_value(age)} s" if age is not None else "  age           -")
    lines.append(f"  rssi          {fmt_value(link.get('rssi'))} dBm")
    lines.append(f"  snr           {fmt_value(link.get('snr'))} dB")
    lines.append(f"  sleep interval {fmt_value(interval)} s")
    sw = data.get("sweep")
    if sw and not sw.get("finished"):
        lines.append(f"  sweep         stage {sw['stage']}/{sw['stages']}, {sw['stage_remaining_h']:g}h left")
    elif sw:
        lines.append("  sweep         finished")
    rot = data.get("rotation")
    if rot:
        lines.append(f"  rotation      {rot['plan']} @ {rot['block_h']}h, block {rot['block']}")
    lines.append(f"  battery       {charging_label(data.get('charging'))}")
    lines.append("")

    reading = data.get("reading") or {}
    lines.append("  SENSORS")
    for key, label, unit in SENSOR_ORDER:
        v = reading.get(key)
        unit_str = f" {unit}" if unit else ""
        lines.append(f"    {label:<16}{fmt_value(v)}{unit_str}")
    lines.append("")

    battery = data.get("battery") or {}
    lines.append("  POWER")
    lines.append(f"    Battery        {fmt_value(battery.get('v'))} V   {fmt_value(battery.get('i'))} mA   {fmt_value(battery.get('p'))} mW")
    return "\n".join(lines)


def cmd_once(args):
    out = run_remote(args, ["--get-latest"])
    data = json.loads(out)
    print(render_latest(data))


def cmd_live(args):
    while True:
        try:
            out = run_remote(args, ["--get-latest"])
            data = json.loads(out)
            sys.stdout.write(CLEAR_SCREEN)
            print(render_latest(data))
            print(f"\n  refreshing every {args.interval}s, ctrl-c to stop")
        except RuntimeError as e:
            sys.stdout.write(CLEAR_SCREEN)
            print(f"WSN NODE  SSH ERROR\n\n  {e}\n\n  retrying in {args.interval}s")
        except json.JSONDecodeError:
            sys.stdout.write(CLEAR_SCREEN)
            print("WSN NODE  BAD RESPONSE\n\n  could not parse --get-latest output")
        try:
            time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\nstopped")
            return


def cmd_export(args):
    flags = ["--export-csv"]
    if args.limit is not None:
        flags += ["--limit", str(args.limit)]
    out = run_remote(args, flags, timeout=120)
    with open(args.out, "w", newline="") as f:
        f.write(out)
    row_count = max(out.count("\n") - 1, 0)
    print(f"wrote {row_count} rows to {args.out}")


def cmd_set_interval(args):
    out = run_remote(args, ["--set-interval", str(args.seconds)])
    try:
        data = json.loads(out)
    except json.JSONDecodeError:
        print(out.strip())
        return
    if data.get("status") == "ok":
        print(f"interval set to {data['sleep_interval_s']}s, node applies it on its next wake")
    else:
        print(f"error: {data.get('error', 'unknown')}")


def cmd_sweep(args):
    if args.stop:
        flags = ["--sweep-stop"]
    elif args.status or not args.plan:
        flags = ["--sweep-status"]
    else:
        flags = ["--sweep", args.plan]

    out = run_remote(args, flags)
    try:
        data = json.loads(out)
    except json.JSONDecodeError:
        print(out.strip())
        return

    if data.get("status") != "ok":
        print(f"error: {data.get('error', 'unknown')}")
        return

    if args.stop:
        print(f"sweep abandoned, interval back to {data.get('sleep_interval_s')}s")
        return

    if args.plan and not args.status:
        print(f"sweep started, {data['total_hours']:g}h total")
        for s in data["stages"]:
            print(f"  {s['interval_s']:>5}s for {s['hours']:g}h")
        print(f"currently running {data['sleep_interval_s']}s")
        return

    sw = data.get("sweep")
    if sw is None:
        print("no sweep configured")
        return
    if sw.get("finished"):
        print(f"sweep finished, started {sw['started']}")
        return
    print(f"sweep stage {sw['stage']} of {len(sw['stages'])}, running {sw['interval_s']}s")
    print(f"  elapsed          {sw['elapsed_h']:g}h")
    print(f"  stage remaining  {sw['stage_remaining_h']:g}h")
    print(f"  sweep remaining  {sw['total_remaining_h']:g}h")


def cmd_lightrun(args):
    # a light-run is a sweep with the panel connected and each interval held
    # long enough to span at least one full day-night cycle, so every interval
    # sees comparable insolation. mechanically it is the same sweep scheduler,
    # the difference is the stage length and that the panel stays connected
    intervals = [s.strip() for s in args.intervals.split(",") if s.strip()]
    try:
        ivs = [int(s) for s in intervals]
    except ValueError:
        print("intervals must be comma-separated integers, e.g. 30,60,300,900")
        return
    if len(ivs) < 2:
        print("give at least two intervals")
        return

    plan = ",".join(f"{iv}:{args.hours_each:g}" for iv in ivs)
    total = len(ivs) * args.hours_each
    days = total / 24
    print(f"light-run: {len(ivs)} intervals x {args.hours_each:g}h = {total:g}h ({days:.1f} days)")
    print("panel MUST stay connected for this run, it measures energy balance under real light")
    if args.hours_each < 24:
        print(f"warning: {args.hours_each:g}h per interval is less than a full day, each interval will")
        print("see a different slice of the daily light cycle. 24h or more is recommended, or use")
        print("'rotate' instead, which interleaves intervals so they share conditions in less total time")

    out = run_remote(args, ["--sweep", plan])
    try:
        data = json.loads(out)
    except json.JSONDecodeError:
        print(out.strip())
        return
    if data.get("status") != "ok":
        print(f"error: {data.get('error', 'unknown')}")
        return
    print(f"started, currently running {data['sleep_interval_s']}s")


def cmd_rotate(args):
    if args.stop:
        flags = ["--rotate-stop"]
    elif args.status:
        flags = ["--rotate-status"]
    elif args.plan:
        flags = ["--rotate", args.plan, "--rotate-block-hours", str(args.block_hours)]
    else:
        print("give a plan, or use --status or --stop")
        return

    out = run_remote(args, flags)
    try:
        data = json.loads(out)
    except json.JSONDecodeError:
        print(out.strip())
        return

    if data.get("status") != "ok":
        print(f"error: {data.get('error', 'unknown')}")
        return

    rot = data.get("rotation")
    if rot is None and not args.plan:
        print(f"rotation off, fixed interval {data.get('sleep_interval_s')}s")
        return
    if args.plan:
        print(f"rotating {data['plan']} in {data['block_h']}h blocks, currently {data['sleep_interval_s']}s")
        return
    print(f"rotating {rot['plan']} in {rot['block_h']}h blocks")
    print(f"  started      {rot['started']}")
    print(f"  block        {rot['block']} (plan position {rot['position']})")
    print(f"  current      {data['sleep_interval_s']}s")


def cmd_wipe(args):
    # first call without --confirm just to report how many rows are at stake
    out = run_remote(args, ["--wipe"])
    data = json.loads(out)
    row_count = data.get("row_count", 0)

    if row_count == 0:
        print("nothing to wipe, the readings table is already empty")
        return

    if not args.yes:
        reply = input(f"this will permanently delete {row_count} logged readings on the jetson. type 'yes' to continue: ")
        if reply.strip().lower() != "yes":
            print("cancelled, nothing deleted")
            return

    out = run_remote(args, ["--wipe", "--confirm"])
    data = json.loads(out)
    if data.get("status") == "ok":
        print(f"deleted {data['deleted']} rows")
    else:
        print(f"error: {data.get('error', 'unknown')}")


def add_common(sub):
    sub.add_argument("--host", default=JETSON_HOST)
    sub.add_argument("--user", default=JETSON_USER)
    sub.add_argument("--remote-python", default=REMOTE_PYTHON)
    sub.add_argument("--remote-script", default=REMOTE_SCRIPT)


def main():
    parser = argparse.ArgumentParser(description="WSN laptop control over SSH")
    subs = parser.add_subparsers(dest="command", required=True)

    p_live = subs.add_parser("live", help="live-updating terminal view")
    add_common(p_live)
    p_live.add_argument("--interval", type=float, default=5.0, help="poll interval in seconds")
    p_live.set_defaults(func=cmd_live)

    p_once = subs.add_parser("once", help="print latest reading once and exit")
    add_common(p_once)
    p_once.set_defaults(func=cmd_once)

    p_export = subs.add_parser("export", help="download the readings table to a local csv")
    add_common(p_export)
    p_export.add_argument("--out", default="wsn_export.csv", help="local output path")
    p_export.add_argument("--limit", type=int, default=None, help="only the last N rows")
    p_export.set_defaults(func=cmd_export)

    p_set = subs.add_parser("set-interval", help="change the node sleep interval")
    add_common(p_set)
    p_set.add_argument("seconds", type=int, help="new sleep interval in seconds")
    p_set.set_defaults(func=cmd_set_interval)

    p_sweep = subs.add_parser("sweep", help="one-shot interval sweep for the dark-run baseline")
    add_common(p_sweep)
    p_sweep.add_argument("plan", nargs="?", default=None, help="interval:hours pairs, e.g. 900:10,300:6,60:3,30:3")
    p_sweep.add_argument("--stop", action="store_true", help="abandon a running sweep")
    p_sweep.add_argument("--status", action="store_true", help="show sweep progress")
    p_sweep.set_defaults(func=cmd_sweep)

    p_light = subs.add_parser("lightrun", help="interval sweep with the panel connected, for energy-balance data under real light")
    add_common(p_light)
    p_light.add_argument("intervals", help="comma-separated intervals, e.g. 30,60,300,900")
    p_light.add_argument("--hours-each", type=float, default=24.0, help="hours to hold each interval, 24 or more spans a full day")
    p_light.set_defaults(func=cmd_lightrun)

    p_rot = subs.add_parser("rotate", help="counterbalanced interval rotation for solar-varying trials")
    add_common(p_rot)
    p_rot.add_argument("plan", nargs="?", default=None, help="comma-separated intervals, e.g. 30,60,300,900")
    p_rot.add_argument("--block-hours", type=float, default=3.0, help="hours per rotation block")
    p_rot.add_argument("--stop", action="store_true", help="stop rotation, return to fixed interval")
    p_rot.add_argument("--status", action="store_true", help="show current rotation state")
    p_rot.set_defaults(func=cmd_rotate)

    p_wipe = subs.add_parser("wipe", help="delete all logged readings on the jetson")
    add_common(p_wipe)
    p_wipe.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    p_wipe.set_defaults(func=cmd_wipe)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
