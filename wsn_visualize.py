#!/usr/bin/env python3
# Quick full-history viewer for a wsn_laptop.py CSV export.
# Usage: python3 wsn_visualize.py [path/to/export.csv] [--save out.png]
# Requires: pip install pandas matplotlib

import argparse
import sys

import pandas as pd
import matplotlib.pyplot as plt

METRICS = [
    ("co2", "CO2 (ppm)"),
    ("temp", "Temp (C)"),
    ("hum", "Humidity (%)"),
    ("clear", "Light clear"),
    ("nir", "Light NIR"),
    ("soil_moisture", "Soil moisture (%)"),
    ("soil_temp", "Soil temp (C)"),
    ("soil_ec", "Soil EC"),
    ("soil_ph", "Soil pH"),
    ("soil_n", "Soil N"),
    ("soil_p", "Soil P"),
    ("soil_k", "Soil K"),
    ("battery_v", "Battery V"),
    ("battery_i", "Battery I (mA)"),
    ("battery_p", "Battery P (mW)"),
    ("rssi", "RSSI (dBm)"),
    ("snr", "SNR (dB)"),
]


def shade_charging(ax, df):
    # draws a light green span behind any stretch where charging == 1, so
    # every metric subplot shows charging state without a dedicated panel
    if "charging" not in df.columns:
        return
    charging = df["charging"].fillna(0).astype(int)
    in_span = False
    start = None
    for ts, val in zip(df["ts"], charging):
        if val == 1 and not in_span:
            in_span = True
            start = ts
        elif val == 0 and in_span:
            in_span = False
            ax.axvspan(start, ts, color="tab:green", alpha=0.08, zorder=0)
    if in_span:
        ax.axvspan(start, df["ts"].iloc[-1], color="tab:green", alpha=0.08, zorder=0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", nargs="?", default="wsn_export.csv")
    parser.add_argument("--save", metavar="PATH", help="save the figure instead of showing it interactively")
    args = parser.parse_args()

    try:
        df = pd.read_csv(args.csv_path)
    except FileNotFoundError:
        print(f"file not found: {args.csv_path}")
        sys.exit(1)

    if df.empty:
        print("csv has no rows, nothing to plot")
        sys.exit(0)

    df["ts"] = pd.to_datetime(df["ts"])
    present = [(key, label) for key, label in METRICS if key in df.columns and df[key].notna().any()]

    if not present:
        print("none of the expected columns have any data")
        sys.exit(0)

    cols = 4
    rows = (len(present) + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 2.6 * rows), sharex=True)
    axes = axes.flatten()

    for ax, (key, label) in zip(axes, present):
        ax.plot(df["ts"], df[key], linewidth=1.2, color="tab:blue")
        shade_charging(ax, df)
        ax.set_title(label, fontsize=10)
        ax.tick_params(axis="x", rotation=45, labelsize=7)
        ax.tick_params(axis="y", labelsize=7)

    for ax in axes[len(present):]:
        ax.axis("off")

    fig.suptitle(f"WSN node history — {len(df)} readings, green = charging", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    if args.save:
        fig.savefig(args.save, dpi=150)
        print(f"saved to {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
