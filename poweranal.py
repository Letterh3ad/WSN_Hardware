#!/usr/bin/env python3
# Analyses one or more WSN CSV exports, each assumed to come from a different
# deep sleep interval configuration (e.g. wsn_export-120.csv for a 120s sleep).
# For each export it estimates battery state of charge, splits power draw and
# solar gain into day/night and charge/discharge buckets, and (when more than
# one file is given) compares the configs against each other so you can see
# which sleep interval is actually the most power efficient.
# Usage: python3 poweranal.py export1.csv [export2.csv ...] [--capacity-mah 2000] [--hours 6] [--save-dir out/]
# Requires: pip install pandas matplotlib numpy

import argparse
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# rough single-cell li-ion open-circuit-voltage to state-of-charge curve, used
# only to anchor the very first reading, everything after that is coulomb
# counted from measured current, not re-read from voltage
OCV_TABLE_V = [3.0, 3.3, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0, 4.1, 4.2]
OCV_TABLE_SOC = [0, 5, 15, 25, 40, 55, 70, 82, 92, 100]

# if the gap between two consecutive readings is longer than this, skip
# integrating across it rather than assuming the last known current held
# constant for the whole gap, since that would badly distort the estimate
MAX_GAP_MINUTES = 15

# recent-window length used to compute the trend for the forward projection
TREND_WINDOW_MINUTES = 30

# voltage readings below this are treated as boot transients, not real
# battery voltage, and excluded before anchoring the SOC estimate
MIN_PLAUSIBLE_V = 2.0

# gain-normalised clear counts at or below this are treated as dark, i.e. no
# meaningful harvest. this is what makes a night block usable as an in-situ
# dark run, where net battery current is the node's load current and nothing
# else, and so the only condition under which load can be measured directly
# from a single battery-side monitor
DARK_INDEX_THRESHOLD = 2.0

# a dark block shorter than this is not long enough to average out the
# per-cycle overheads, so it is excluded from the load estimate
MIN_DARK_BLOCK_MINUTES = 20

# 'clear' light channel counts at or above this are treated as daylight,
# below it as night. the sensor saturates at 1000 in direct sun and sits
# near 0 at night, so this threshold sits comfortably in the gap between them
DAY_LIGHT_THRESHOLD = 50


def voltage_to_soc(v):
    return float(np.interp(v, OCV_TABLE_V, OCV_TABLE_SOC))


def infer_label(path):
    # pull a config number (the sleep interval in seconds) out of the
    # filename if there is one, otherwise just fall back to the filename
    m = re.search(r"(\d+)", Path(path).stem)
    return f"{m.group(1)}s sleep" if m else Path(path).stem


def load_data(path):
    try:
        df = pd.read_csv(path)
    except FileNotFoundError:
        print(f"file not found: {path}")
        sys.exit(1)
    if df.empty:
        print(f"{path}: csv has no rows")
        sys.exit(0)
    df["ts"] = pd.to_datetime(df["ts"])
    df = df.sort_values("ts").reset_index(drop=True)
    return df


def detect_schema(df, path):
    # different firmware/hardware revisions logged different columns, so
    # each export needs to be handled a little differently
    if "battery_v" in df.columns and "battery_i" in df.columns:
        return "battery"
    if "solar_i" in df.columns:
        if "load_i" in df.columns and df["load_i"].notna().any():
            return "solar_load"
        return "solar_only"
    print(f"{path}: no recognised power columns, skipping")
    return None


def clean_for_schema(df, schema):
    if schema == "battery":
        df = df.dropna(subset=["battery_v", "battery_i"]).reset_index(drop=True)
        if df.empty:
            return df
        # drop any leading rows below a plausible li-ion voltage, these are
        # typically boot transients or a disconnected sensor settling, not
        # the real battery voltage, and would badly skew the SOC anchor
        plausible = df[df["battery_v"] >= MIN_PLAUSIBLE_V]
        if plausible.empty:
            print("no readings above the plausible battery voltage floor, check the sensor wiring")
            return plausible
        first_plausible_idx = plausible.index[0]
        if first_plausible_idx > 0:
            print(f"dropped {first_plausible_idx} leading reading(s) below {MIN_PLAUSIBLE_V}V as boot transients")
        df = df.loc[first_plausible_idx:].reset_index(drop=True)
    elif schema in ("solar_load", "solar_only"):
        df = df.dropna(subset=["solar_v", "solar_i"]).reset_index(drop=True)
    return df


def net_current_ma(df, schema):
    # positive = net current into the battery (charging), negative = net
    # current out of the battery (discharging), matching the battery_i
    # convention already used in the battery-schema exports
    if schema == "battery":
        return df["battery_i"]
    if schema == "solar_load":
        return df["solar_i"] - df["load_i"]
    return None


def net_power_mw(df, schema, net_ma):
    if schema == "battery":
        if "battery_p" in df.columns:
            return df["battery_p"]
        return net_ma * df["battery_v"]
    if schema == "solar_load":
        if "solar_p" in df.columns and "load_p" in df.columns:
            return df["solar_p"] - df["load_p"]
        return net_ma * df["solar_v"]
    if schema == "solar_only":
        if "solar_p" in df.columns:
            return df["solar_p"]
        return df["solar_i"] * df["solar_v"]
    return None


def day_night_labels(df):
    if "clear" in df.columns and df["clear"].notna().any():
        return np.where(df["clear"].fillna(0) >= DAY_LIGHT_THRESHOLD, "day", "night")
    # fallback if there's no light sensor reading: treat 06:00-19:00 UTC as day
    hour = df["ts"].dt.hour
    return np.where((hour >= 6) & (hour < 19), "day", "night")


def charge_discharge_labels(df, net_ma):
    if "charging" in df.columns and df["charging"].notna().any():
        return np.where(df["charging"].fillna(0) > 0, "charge", "discharge")
    if net_ma is not None:
        return np.where(net_ma >= 0, "charge", "discharge")
    return np.array(["n/a"] * len(df))


def coulomb_count(df, net_ma):
    # cumulative mAh added/removed since the first reading, trapezoidal
    # integration over actual elapsed seconds, skipping oversized gaps
    cumulative = [0.0]
    for i in range(1, len(df)):
        dt_s = (df["ts"].iloc[i] - df["ts"].iloc[i - 1]).total_seconds()
        if dt_s > MAX_GAP_MINUTES * 60:
            cumulative.append(cumulative[-1])
            continue
        avg_ma = (net_ma.iloc[i] + net_ma.iloc[i - 1]) / 2
        delta_mah = avg_ma * (dt_s / 3600)
        cumulative.append(cumulative[-1] + delta_mah)
    return pd.Series(cumulative, index=df.index)


def integrate_by_category(ts, values, categories):
    # splits a value series (mA or mW) into totals per category by
    # trapezoidal integration over elapsed hours, tagging each interval with
    # the category of the interval's starting sample. skips oversized gaps
    # the same way coulomb_count does, so the two stay consistent
    totals = {}
    for i in range(1, len(ts)):
        dt_s = (ts.iloc[i] - ts.iloc[i - 1]).total_seconds()
        if dt_s <= 0 or dt_s > MAX_GAP_MINUTES * 60:
            continue
        hours = dt_s / 3600
        avg_val = (values.iloc[i] + values.iloc[i - 1]) / 2
        if pd.isna(avg_val):
            # a single missing reading shouldn't poison the whole running
            # total for its category, so just skip this interval
            continue
        cat = categories[i - 1]
        bucket = totals.setdefault(cat, {"total": 0.0, "hours": 0.0})
        bucket["total"] += avg_val * hours
        bucket["hours"] += hours
    return totals


def average_by_category(totals):
    # converts accumulated totals into a time-weighted average value per
    # category, which is what actually makes different sleep intervals with
    # different sample counts and total durations comparable
    return {cat: (b["total"] / b["hours"] if b["hours"] > 0 else float("nan")) for cat, b in totals.items()}


def build_soc_series(df, net_ma, capacity_mah):
    delta_mah = coulomb_count(df, net_ma)
    soc0 = voltage_to_soc(df["battery_v"].iloc[0])
    soc = soc0 + (delta_mah / capacity_mah) * 100
    return soc.clip(lower=0, upper=100)


def project_forward(df, net_ma, soc, capacity_mah, hours_ahead):
    window_start = df["ts"].iloc[-1] - pd.Timedelta(minutes=TREND_WINDOW_MINUTES)
    recent = net_ma[df["ts"] >= window_start]
    trend_ma = recent.mean() if len(recent) else net_ma.iloc[-1]

    last_ts = df["ts"].iloc[-1]
    last_soc = soc.iloc[-1]
    future_ts = [last_ts]
    future_soc = [last_soc]

    step_minutes = 5
    steps = int(hours_ahead * 60 / step_minutes)
    for _ in range(steps):
        delta_mah = trend_ma * (step_minutes * 60 / 3600)
        next_soc = future_soc[-1] + (delta_mah / capacity_mah) * 100
        next_soc = max(0, min(100, next_soc))
        future_ts.append(future_ts[-1] + pd.Timedelta(minutes=step_minutes))
        future_soc.append(next_soc)
        if next_soc in (0, 100):
            break

    return future_ts, future_soc, trend_ma


class Dataset:
    # bundles everything computed for a single export so the per-file
    # plotting and cross-file comparison can both draw on it
    def __init__(self, path, capacity_mah):
        self.path = path
        self.label = infer_label(path)
        df = load_data(path)
        self.schema = detect_schema(df, path)
        if self.schema is None:
            self.df = df.iloc[0:0]
            return
        df = clean_for_schema(df, self.schema)
        self.df = df
        if df.empty:
            return

        self.net_ma = net_current_ma(df, self.schema)
        self.net_mw = net_power_mw(df, self.schema, self.net_ma)
        self.day_labels = day_night_labels(df)

        dt = df["ts"].diff().dt.total_seconds().dropna()
        plausible_dt = dt[dt <= MAX_GAP_MINUTES * 60]
        self.median_interval_s = plausible_dt.median() if len(plausible_dt) else float("nan")

        if self.schema == "solar_only":
            # no net battery current here, only solar generation, so the
            # only meaningful split is day vs night, not charge/discharge
            self.charge_labels = None
            self.mw_by_combo = average_by_category(integrate_by_category(df["ts"], self.net_mw, self.day_labels))
        elif self.net_mw is not None:
            self.charge_labels = charge_discharge_labels(df, self.net_ma)
            combined = np.array([f"{d}_{c}" for d, c in zip(self.day_labels, self.charge_labels)])
            self.mw_by_combo = average_by_category(integrate_by_category(df["ts"], self.net_mw, combined))
        else:
            self.charge_labels = None
            self.mw_by_combo = {}

        if self.schema == "battery":
            self.soc = build_soc_series(df, self.net_ma, capacity_mah)
        else:
            self.soc = None

    def has_data(self):
        return not self.df.empty and self.schema is not None

    def night_draw_mw(self):
        # positive magnitude of average power pulled from the battery at
        # night, or None if this export can't tell us that
        if self.schema == "solar_only":
            return None
        val = self.mw_by_combo.get("night_discharge")
        return abs(val) if val is not None else None

    def day_gain_mw(self):
        # positive magnitude of average power going in during the day,
        # either into the battery or straight off the solar panel
        if self.schema == "solar_only":
            return self.mw_by_combo.get("day")
        return self.mw_by_combo.get("day_charge")


def summarize(ds):
    df = ds.df
    print(f"\n=== {ds.label} ({ds.path}) ===")
    print(f"schema: {ds.schema}, {len(df)} usable readings, measured wake interval ~{ds.median_interval_s:.0f}s")

    if ds.schema == "solar_only":
        print("no load/battery current logged in this export, only solar generation stats are available")
        for key, desc in (("day", "day solar generation"), ("night", "night solar generation")):
            if key in ds.mw_by_combo:
                print(f"  {desc}: {ds.mw_by_combo[key]:.1f} mW average")
    else:
        labels = {
            "night_discharge": "night draw (battery out)",
            "day_discharge": "day draw (battery out)",
            "night_charge": "night gain (battery in)",
            "day_charge": "day gain (battery in)",
        }
        for key, desc in labels.items():
            if key in ds.mw_by_combo:
                magnitude = abs(ds.mw_by_combo[key])
                print(f"  {desc}: {magnitude:.1f} mW average")

    if ds.soc is not None and len(ds.soc):
        print(f"  SOC range over export: {ds.soc.min():.0f}% - {ds.soc.max():.0f}%, ends at {ds.soc.iloc[-1]:.0f}%")


def plot_dataset(ds, save_dir, hours_ahead, capacity_mah):
    df = ds.df
    if ds.schema == "battery":
        fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=False)

        axes[0].plot(df["ts"], df["battery_v"], color="tab:blue", linewidth=1)
        axes[0].set_ylabel("Battery V")
        axes[0].set_title(f"{ds.label}: measured battery voltage")

        axes[1].plot(df["ts"], ds.net_ma, color="tab:orange", linewidth=1)
        axes[1].axhline(0, color="gray", linewidth=0.8)
        axes[1].set_ylabel("mA")
        axes[1].set_title("Battery current (measured directly)")

        future_ts, future_soc, trend_ma = project_forward(df, ds.net_ma, ds.soc, capacity_mah, hours_ahead)
        axes[2].plot(df["ts"], ds.soc, color="tab:green", linewidth=1.5, label="estimated SOC")
        axes[2].plot(future_ts, future_soc, color="tab:green", linewidth=1.5, linestyle="--", label="projected")
        axes[2].axvline(df["ts"].iloc[-1], color="gray", linewidth=0.8, linestyle=":")
        axes[2].set_ylabel("SOC %")
        axes[2].set_ylim(-5, 105)
        axes[2].set_title("Estimated state of charge and forward projection")
        axes[2].legend(fontsize=8)
    else:
        fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=False)

        axes[0].plot(df["ts"], df["solar_v"], color="tab:blue", linewidth=1)
        axes[0].set_ylabel("Solar V")
        axes[0].set_title(f"{ds.label}: measured solar panel voltage")

        power = ds.net_mw if ds.net_mw is not None else df["solar_i"] * df["solar_v"]
        axes[1].plot(df["ts"], power, color="tab:orange", linewidth=1)
        axes[1].axhline(0, color="gray", linewidth=0.8)
        axes[1].set_ylabel("mW")
        title = "Net solar - load power" if ds.schema == "solar_load" else "Solar panel power (no load telemetry)"
        axes[1].set_title(title)

    for ax in axes:
        ax.tick_params(axis="x", rotation=30, labelsize=8)
    fig.tight_layout()

    if save_dir:
        out_path = Path(save_dir) / f"{Path(ds.path).stem}.png"
        fig.savefig(out_path, dpi=150)
        print(f"saved {out_path}")
        plt.close(fig)
    else:
        plt.show()


def plot_comparison(datasets, save_dir):
    usable = [ds for ds in datasets if ds.has_data() and ds.mw_by_combo]
    if len(usable) < 2:
        return

    labels = [ds.label for ds in usable]
    night_draw = [ds.night_draw_mw() if ds.night_draw_mw() is not None else np.nan for ds in usable]
    day_gain = [ds.day_gain_mw() if ds.day_gain_mw() is not None else np.nan for ds in usable]

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))

    axes[0].bar(labels, night_draw, color="tab:red")
    axes[0].set_ylabel("mW")
    axes[0].set_title("Average night-time draw (higher = worse)")
    axes[0].tick_params(axis="x", rotation=20, labelsize=8)

    axes[1].bar(labels, day_gain, color="tab:green")
    axes[1].set_ylabel("mW")
    axes[1].set_title("Average day-time gain (higher = better)")
    axes[1].tick_params(axis="x", rotation=20, labelsize=8)

    fig.suptitle("Deep sleep config comparison")
    fig.tight_layout()

    if save_dir:
        out_path = Path(save_dir) / "comparison.png"
        fig.savefig(out_path, dpi=150)
        print(f"saved {out_path}")
        plt.close(fig)
    else:
        plt.show()

    # a second figure overlaying SOC trends for the configs that have real
    # battery telemetry, normalised to hours elapsed so runs starting on
    # different days can still be compared directly
    battery_sets = [ds for ds in usable if ds.soc is not None]
    if len(battery_sets) >= 2:
        fig2, ax = plt.subplots(figsize=(9, 4.5))
        for ds in battery_sets:
            elapsed_h = (ds.df["ts"] - ds.df["ts"].iloc[0]).dt.total_seconds() / 3600
            ax.plot(elapsed_h, ds.soc, label=ds.label, linewidth=1.5)
        ax.set_xlabel("Hours since export start")
        ax.set_ylabel("Estimated SOC %")
        ax.set_title("SOC drain comparison, aligned by elapsed time")
        ax.legend(fontsize=8)
        fig2.tight_layout()
        if save_dir:
            out_path = Path(save_dir) / "soc_comparison.png"
            fig2.savefig(out_path, dpi=150)
            print(f"saved {out_path}")
            plt.close(fig2)
        else:
            plt.show()


def print_comparison_table(datasets):
    usable = [ds for ds in datasets if ds.has_data()]
    if len(usable) < 2:
        return
    print("\n=== Config comparison ===")
    header = f"{'config':<14}{'wake interval':<16}{'night draw':<14}{'day gain':<14}{'data quality'}"
    print(header)
    for ds in usable:
        interval = f"~{ds.median_interval_s:.0f}s"
        night = ds.night_draw_mw()
        day = ds.day_gain_mw()
        night_s = f"{night:.0f} mW" if night is not None else "n/a"
        day_s = f"{day:.0f} mW" if day is not None else "n/a"
        quality = {
            "battery": "full battery telemetry",
            "solar_load": "derived net current",
            "solar_only": "solar generation only",
        }[ds.schema]
        print(f"{ds.label:<14}{interval:<16}{night_s:<14}{day_s:<14}{quality}")




# Solar-normalised comparison.
#
# The single INA219 sits on the battery, so its reading is net current: under
# illumination the load is hidden beneath the charge current and cannot be
# recovered from that row alone. These functions work around that by splitting
# the record into dark periods, where net current is load current exactly, and
# lit periods, where the harvest is inferred by difference against the load
# figure measured in the dark.


def insolation_index(df):
    # gain-normalised clear-channel counts. uncalibrated, so the units are
    # arbitrary, which is all a covariate needs to be. dividing by gain is
    # what makes a reading taken at 512x comparable with one taken at 1x
    if "clear" not in df.columns:
        return None
    clear = pd.to_numeric(df["clear"], errors="coerce")
    if "light_gain" in df.columns:
        gain = pd.to_numeric(df["light_gain"], errors="coerce")
        gain = gain.where(gain > 0)
        index = clear / gain
        # rows logged before the gain was recorded fall back to raw counts,
        # which is wrong but visibly wrong rather than silently wrong
        index = index.fillna(clear)
    else:
        index = clear
    if "light_saturated" in df.columns:
        sat = pd.to_numeric(df["light_saturated"], errors="coerce").fillna(0)
        index = index.mask(sat > 0)
    return index


def interval_labels(df):
    # prefers the interval the node reported over anything inferred from
    # timestamps, since a missed packet stretches the apparent gap
    if "sleep_interval_s" in df.columns and df["sleep_interval_s"].notna().any():
        return pd.to_numeric(df["sleep_interval_s"], errors="coerce")
    dt = df["ts"].diff().dt.total_seconds()
    return dt.round(-1)


def dark_blocks(df, index, min_minutes=MIN_DARK_BLOCK_MINUTES):
    # contiguous runs of dark rows, returned as (start_idx, end_idx) pairs
    if index is None:
        return []
    dark = (index.fillna(0) <= DARK_INDEX_THRESHOLD).to_numpy()
    blocks = []
    start = None
    for i, is_dark in enumerate(dark):
        if is_dark and start is None:
            start = i
        elif not is_dark and start is not None:
            blocks.append((start, i - 1))
            start = None
    if start is not None:
        blocks.append((start, len(dark) - 1))

    keep = []
    for a, b in blocks:
        if b <= a:
            continue
        span_min = (df["ts"].iloc[b] - df["ts"].iloc[a]).total_seconds() / 60
        if span_min >= min_minutes:
            keep.append((a, b))
    return keep


def dark_run_energy(df, net_ma, index):
    # per-interval load energy measured over dark blocks only
    # returns {interval_s: {"mwh_per_cycle", "mean_mw", "cycles", "hours"}}
    intervals = interval_labels(df)
    voltage = pd.to_numeric(df["battery_v"], errors="coerce") if "battery_v" in df.columns else None
    if voltage is None:
        return {}

    acc = {}
    for a, b in dark_blocks(df, index):
        for i in range(a + 1, b + 1):
            dt_s = (df["ts"].iloc[i] - df["ts"].iloc[i - 1]).total_seconds()
            if dt_s <= 0 or dt_s > MAX_GAP_MINUTES * 60:
                continue
            iv = intervals.iloc[i]
            if pd.isna(iv):
                continue
            mw_a = net_ma.iloc[i - 1] * voltage.iloc[i - 1]
            mw_b = net_ma.iloc[i] * voltage.iloc[i]
            if pd.isna(mw_a) or pd.isna(mw_b):
                continue
            # net is negative while discharging, the load is its magnitude
            avg_mw = -((mw_a + mw_b) / 2)
            if avg_mw <= 0:
                # positive net current in the dark means the panel is not
                # actually dark, or the sign convention is inverted, either
                # way this sample cannot be read as load
                continue
            bucket = acc.setdefault(float(iv), {"mwh": 0.0, "hours": 0.0})
            hours = dt_s / 3600
            bucket["mwh"] += avg_mw * hours
            bucket["hours"] += hours

    out = {}
    for iv, b in acc.items():
        if b["hours"] <= 0:
            continue
        cycles = b["hours"] * 3600 / iv
        out[iv] = {
            "mwh_per_cycle": b["mwh"] / cycles if cycles > 0 else float("nan"),
            "mean_mw": b["mwh"] / b["hours"],
            "cycles": cycles,
            "hours": b["hours"],
        }
    return out


def daily_balance(df, net_ma, index):
    # one row per (date, interval): energy into or out of the battery over
    # that period, alongside the integrated insolation index for it
    intervals = interval_labels(df)
    voltage = pd.to_numeric(df["battery_v"], errors="coerce") if "battery_v" in df.columns else None
    if voltage is None or index is None:
        return pd.DataFrame()

    rows = {}
    for i in range(1, len(df)):
        dt_s = (df["ts"].iloc[i] - df["ts"].iloc[i - 1]).total_seconds()
        if dt_s <= 0 or dt_s > MAX_GAP_MINUTES * 60:
            continue
        iv = intervals.iloc[i]
        if pd.isna(iv):
            continue
        hours = dt_s / 3600
        key = (df["ts"].iloc[i].date(), float(iv))
        bucket = rows.setdefault(key, {"delta_mwh": 0.0, "insol": 0.0, "hours": 0.0})

        mw_a = net_ma.iloc[i - 1] * voltage.iloc[i - 1]
        mw_b = net_ma.iloc[i] * voltage.iloc[i]
        if not (pd.isna(mw_a) or pd.isna(mw_b)):
            bucket["delta_mwh"] += ((mw_a + mw_b) / 2) * hours
        ia, ib = index.iloc[i - 1], index.iloc[i]
        if not (pd.isna(ia) or pd.isna(ib)):
            bucket["insol"] += ((ia + ib) / 2) * hours
        bucket["hours"] += hours

    if not rows:
        return pd.DataFrame()
    recs = [{"date": d, "interval_s": iv, **v} for (d, iv), v in rows.items()]
    out = pd.DataFrame(recs).sort_values(["date", "interval_s"]).reset_index(drop=True)
    # normalise to a full day so partial blocks are comparable
    out["delta_mwh_per_h"] = out["delta_mwh"] / out["hours"]
    out["insol_per_h"] = out["insol"] / out["hours"]
    return out


def fit_normalised(balance):
    # regresses hourly energy balance on hourly insolation with a separate
    # offset per interval and one shared slope. the slope absorbs the weather,
    # the offsets are the duty-cycle effect, which is the whole point of
    # doing it this way rather than comparing raw daily totals
    usable = balance.dropna(subset=["delta_mwh_per_h", "insol_per_h"])
    intervals = sorted(usable["interval_s"].unique())
    if len(usable) < len(intervals) + 2:
        return None

    # design matrix: one indicator column per interval plus the shared slope
    cols = [(usable["interval_s"] == iv).astype(float).to_numpy() for iv in intervals]
    cols.append(usable["insol_per_h"].to_numpy())
    A = np.column_stack(cols)
    y = usable["delta_mwh_per_h"].to_numpy()
    coeffs, residuals, rank, _ = np.linalg.lstsq(A, y, rcond=None)
    if rank < A.shape[1]:
        return None

    fitted = A @ coeffs
    ss_res = float(((y - fitted) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    return {
        "intervals": intervals,
        "offsets": {iv: float(c) for iv, c in zip(intervals, coeffs[:-1])},
        "slope": float(coeffs[-1]),
        "r2": 1 - ss_res / ss_tot if ss_tot > 0 else float("nan"),
        "n": len(usable),
        "data": usable,
    }


def wake_phase_power(df, net_ma):
    # mean load power over wake phases only, per interval, taken from the
    # logged samples directly. every logged row is a wake sample, so this is
    # exactly what the ina219 can measure. returns {interval_s: mean_wake_mw}
    intervals = interval_labels(df)
    voltage = pd.to_numeric(df["battery_v"], errors="coerce") if "battery_v" in df.columns else None
    if voltage is None:
        return {}, {}
    load_mw = -(net_ma * voltage)  # discharge is negative, load is its magnitude

    acc = {}
    awake = {}
    aw_col = pd.to_numeric(df["awake_ms"], errors="coerce") if "awake_ms" in df.columns else None
    for i in range(len(df)):
        iv = intervals.iloc[i]
        p_mw = load_mw.iloc[i]
        if pd.isna(iv) or pd.isna(p_mw) or p_mw <= 0:
            continue
        b = acc.setdefault(float(iv), [])
        b.append(p_mw)
        if aw_col is not None and not pd.isna(aw_col.iloc[i]):
            awake.setdefault(float(iv), []).append(aw_col.iloc[i])

    wake_mw = {iv: float(np.median(v)) for iv, v in acc.items() if v}
    wake_ms = {iv: float(np.median(v)) for iv, v in awake.items() if v}
    return wake_mw, wake_ms


def composite_cycle_energy(df, net_ma, sleep_mw, default_awake_ms=None):
    # E_cycle = P_wake * t_awake + P_sleep * (interval - t_awake)
    # this is the primary per-cycle figure. it does not integrate across the
    # sleep gap, which the ina219 never samples, but instead uses the measured
    # sleep floor for that portion of the cycle
    wake_mw, wake_ms = wake_phase_power(df, net_ma)
    if not wake_mw:
        return {}

    out = {}
    for iv in sorted(wake_mw):
        t_awake_ms = wake_ms.get(iv, default_awake_ms)
        if t_awake_ms is None:
            # no awake_ms logged and none supplied, cannot split the cycle
            continue
        t_awake_s = t_awake_ms / 1000
        if t_awake_s >= iv:
            # awake longer than the interval, the node never actually slept at
            # this setting, so the whole cycle is wake power
            t_awake_s = iv
        t_sleep_s = max(iv - t_awake_s, 0.0)
        e_wake = wake_mw[iv] * t_awake_s
        e_sleep = sleep_mw * t_sleep_s
        e_cycle_mwh = (e_wake + e_sleep) / 3600
        mean_mw = (e_wake + e_sleep) / iv
        out[iv] = {
            "mwh_per_cycle": e_cycle_mwh,
            "mean_mw": mean_mw,
            "wake_mw": wake_mw[iv],
            "wake_s": t_awake_s,
            "sleep_mw": sleep_mw,
            "duty_pct": 100 * t_awake_s / iv,
            "wake_share_pct": 100 * e_wake / (e_wake + e_sleep) if (e_wake + e_sleep) > 0 else float("nan"),
        }
    return out


def report_composite(ds, sleep_mw):
    print(f"\n=== composite cycle energy ({ds.path}) ===")
    print(f"measured sleep floor: {sleep_mw:.2f} mW")
    comp = composite_cycle_energy(ds.df, ds.net_ma, sleep_mw)
    if not comp:
        print("no usable wake samples with an interval label and awake_ms")
        return None

    print(f"\n{'interval':<11}{'mWh/cycle':<13}{'mean mW':<11}{'wake mW':<11}"
          f"{'wake s':<9}{'duty %':<9}{'wake %E'}")
    for iv in sorted(comp):
        c = comp[iv]
        print(f"{iv:<11.0f}{c['mwh_per_cycle']:<13.4f}{c['mean_mw']:<11.2f}"
              f"{c['wake_mw']:<11.1f}{c['wake_s']:<9.1f}{c['duty_pct']:<9.1f}{c['wake_share_pct']:.0f}")

    print("\nprojected daily consumption at each interval")
    print(f"{'interval':<11}{'cycles/day':<13}{'mWh/day':<11}{'mAh/day (nominal 3.8V)'}")
    for iv in sorted(comp):
        c = comp[iv]
        cyc = 86400 / iv
        mwh_day = c["mwh_per_cycle"] * cyc
        print(f"{iv:<11.0f}{cyc:<13.0f}{mwh_day:<11.1f}{mwh_day / 3.8:.1f}")
    return comp


def report_normalised(ds):
    index = insolation_index(ds.df)
    if index is None or ds.schema != "battery":
        print("\nsolar-normalised analysis needs battery telemetry and a clear-channel reading")
        return None

    print(f"\n=== solar-normalised comparison ({ds.path}) ===")

    dark = dark_run_energy(ds.df, ds.net_ma, index)
    if dark:
        print("\ndark-period load, measured where harvest is zero")
        print(f"{'interval':<12}{'mWh/cycle':<14}{'mean mW':<12}{'cycles':<10}{'dark hours'}")
        for iv in sorted(dark):
            d = dark[iv]
            print(f"{iv:<12.0f}{d['mwh_per_cycle']:<14.4f}{d['mean_mw']:<12.2f}{d['cycles']:<10.0f}{d['hours']:.1f}")
    else:
        print("\nno dark periods long enough to measure load directly")

    balance = daily_balance(ds.df, ds.net_ma, index)
    if balance.empty:
        print("not enough data for a daily energy balance")
        return None

    print("\ndaily energy balance against insolation")
    print(f"{'date':<12}{'interval':<11}{'mWh/h':<12}{'insol/h':<12}{'hours'}")
    for _, r in balance.iterrows():
        print(f"{str(r['date']):<12}{r['interval_s']:<11.0f}{r['delta_mwh_per_h']:<12.2f}"
              f"{r['insol_per_h']:<12.1f}{r['hours']:.1f}")

    fit = fit_normalised(balance)
    if fit is None:
        print("\nnot enough spread across intervals and conditions to fit the model yet,")
        print("keep rotating intervals across days with differing cloud cover")
        return balance

    print(f"\nfitted model: balance = offset(interval) + {fit['slope']:.3f} x insolation")
    print(f"R2 = {fit['r2']:.3f} over {fit['n']} interval-days")
    print("\ninterval offsets, higher is better, this is the duty-cycle effect")
    print("with the weather held constant")
    for iv in fit["intervals"]:
        print(f"  {iv:>7.0f}s   {fit['offsets'][iv]:+.2f} mWh/h")

    if dark:
        common = sorted(set(dark) & set(fit["intervals"]))
        if common:
            print("\ncross-check, dark-run load against fitted offset")
            print(f"{'interval':<12}{'dark load mW':<16}{'fitted offset'}")
            for iv in common:
                print(f"{iv:<12.0f}{dark[iv]['mean_mw']:<16.2f}{fit['offsets'][iv]:+.2f}")

    return balance, fit


def plot_normalised(result, save_dir, label):
    if not isinstance(result, tuple):
        return
    balance, fit = result
    fig, ax = plt.subplots(figsize=(8, 5))
    xs = np.linspace(balance["insol_per_h"].min(), balance["insol_per_h"].max(), 50)
    for iv in fit["intervals"]:
        sub = fit["data"][fit["data"]["interval_s"] == iv]
        pts = ax.scatter(sub["insol_per_h"], sub["delta_mwh_per_h"], s=40, label=f"{iv:.0f}s")
        ax.plot(xs, fit["offsets"][iv] + fit["slope"] * xs, linewidth=1.2,
                color=pts.get_facecolor()[0])
    ax.axhline(0, color="gray", linewidth=0.8, linestyle=":")
    ax.set_xlabel("insolation index per hour (arbitrary units)")
    ax.set_ylabel("battery energy balance (mWh/h)")
    ax.set_title("Energy balance vs insolation, one line per sleep interval")
    ax.legend(fontsize=8, title="interval")
    fig.tight_layout()

    if save_dir:
        out_path = Path(save_dir) / f"{label}_normalised.png"
        fig.savefig(out_path, dpi=150)
        print(f"saved {out_path}")
        plt.close(fig)
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_paths", nargs="+", help="one or more WSN export CSVs to analyse and compare")
    parser.add_argument("--capacity-mah", type=float, default=2000, help="battery capacity in mAh")
    parser.add_argument("--hours", type=float, default=6, help="how far ahead to project SOC, per file")
    parser.add_argument("--save-dir", metavar="DIR", help="save figures here instead of showing them interactively")
    parser.add_argument("--normalise", action="store_true", help="compare sleep intervals with solar input held constant, needs sleep_interval_s and clear columns")
    parser.add_argument("--no-plots", action="store_true", help="print the tables without drawing any figures")
    parser.add_argument("--sleep-mw", type=float, default=None, help="externally measured deep-sleep power in mW, enables the composite per-cycle energy calculation")
    parser.add_argument("--sleep-ma", type=float, default=None, help="alternatively give the measured sleep current in mA, converted using each file's mean battery voltage")
    args = parser.parse_args()

    if args.save_dir:
        Path(args.save_dir).mkdir(parents=True, exist_ok=True)

    datasets = [Dataset(p, args.capacity_mah) for p in args.csv_paths]

    for ds in datasets:
        if not ds.has_data():
            continue
        summarize(ds)
        if args.sleep_mw is not None or args.sleep_ma is not None:
            sleep_mw = args.sleep_mw
            if sleep_mw is None:
                v = pd.to_numeric(ds.df["battery_v"], errors="coerce").mean()
                sleep_mw = args.sleep_ma * v if pd.notna(v) else None
            if sleep_mw is not None:
                report_composite(ds, sleep_mw)
        if not args.no_plots:
            plot_dataset(ds, args.save_dir, args.hours, args.capacity_mah)
        if args.normalise:
            result = report_normalised(ds)
            if not args.no_plots:
                plot_normalised(result, args.save_dir, Path(ds.path).stem)

    print_comparison_table(datasets)
    if not args.no_plots:
        plot_comparison(datasets, args.save_dir)


if __name__ == "__main__":
    main()
