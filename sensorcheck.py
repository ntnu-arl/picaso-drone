#!/usr/bin/env python3
"""
sensorcheck.py — Sichtprüfung: läuft der Sensor noch?

Vier Standardbilder, wie in der Sonartechnik üblich — keine Interpretation,
nur die Rohlage:

  1 Spektrogramm  Kommt der Sendesweep (20–80 kHz) an, kommen Echos zurück?
  2 A-Scan        Echostärke über Entfernung. Eine Linie, ein Blick.
  3 B-Scan        Derselbe A-Scan für jeden Schuss, gestapelt: wiederholbar?
  4 Kanalcheck    Pegel aller 16 Mikrofone. Ist eins taub?

Dazu ein Klartext-Urteil auf der Konsole.

Aufruf
------
    python sensorcheck.py 2026_10_02_114119_test_campaign
    python sensorcheck.py <ordner> --rmax 4 --limit 20
"""
import argparse
from pathlib import Path

import numpy as np
from scipy.signal import butter, lfilter, correlate, hilbert, spectrogram
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from beamform_frtis import (N_MICS, C, FS_RAW, FS_DEC, DECIM, MF_BAND, BLANK,
                            load_reference, capture_list)

DB_FLOOR = -60.0


def decode_raw(path):
    """Wie beamform_frtis.decode, aber OHNE Blanking — der Sendepuls soll sichtbar
    bleiben, sonst kann man nicht prüfen, ob überhaupt gesendet wird."""
    raw = np.fromfile(path, dtype=np.uint16)
    bits = ((raw[:, None] >> np.arange(N_MICS)) & 1).astype(float)
    b, a = butter(6, 100e3 / (FS_RAW / 2))
    filt = lfilter(b, a, bits - bits.mean(axis=0, keepdims=True), axis=0)
    return filt[::DECIM, :]


def envelope_ascan(dec, ref_bp):
    """Matched Filter je Mikro -> Hüllkurve -> inkohärent über die Mikros gemittelt.
    Inkohärent, nicht gebeamformt: zeigt Echostärke über Entfernung ohne Richtung."""
    acc = np.zeros(dec.shape[0])
    for m in range(N_MICS):
        mf = correlate(dec[:, m], ref_bp, mode="same")
        acc += np.abs(hilbert(mf))
    return acc / N_MICS


def to_db(x, ref=None):
    ref = np.max(x) if ref is None else ref
    return 20 * np.log10(np.maximum(x, 1e-20) / (ref + 1e-20))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder")
    ap.add_argument("--sweep", default="sweeps/sweep_512_taper30.csv")
    ap.add_argument("--rmax", type=float, default=3.0, help="Anzeige bis RMAX Meter")
    ap.add_argument("--limit", type=int, default=None, help="nur erste N Schüsse")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    folder = Path(args.folder)
    files = capture_list(folder)
    if not files:
        raise SystemExit(f"Keine capture_*.bin in {folder}")
    if args.limit:
        files = files[:args.limit]

    ref = load_reference(args.sweep)
    b_bp, a_bp = butter(4, list(MF_BAND), btype="band", fs=FS_DEC)
    ref_bp = lfilter(b_bp, a_bp, ref)
    ref_bp /= np.linalg.norm(ref_bp) + 1e-9

    blank_r = BLANK / FS_DEC * C / 2

    # ── alle Schüsse einlesen ────────────────────────────────────────────────
    ascans, rms = [], []
    dec0 = None
    for f in files:
        dec = decode_raw(f)
        if dec0 is None:
            dec0 = dec
        n = dec.shape[0]
        rng = np.arange(n) / FS_DEC * C / 2
        keep = rng <= args.rmax
        ascans.append(envelope_ascan(dec, ref_bp)[keep])
        rms.append(dec[BLANK:, :].std(axis=0))          # Pegel je Mikro, ohne Sendepuls
    rng = rng[keep]
    A = np.vstack(ascans)                                # (n_shots, n_range)
    rms = np.vstack(rms).mean(axis=0)                    # (16,)

    a_mean = A.mean(axis=0)
    far = rng > blank_r
    a_db = to_db(a_mean, ref=a_mean[far].max())

    # ── Bild ────────────────────────────────────────────────────────────────
    fig, axs = plt.subplots(2, 2, figsize=(13.5, 9))
    fig.suptitle(f"Sensorcheck — {folder.name}   ({len(files)} Schüsse)",
                 fontsize=13, fontweight="bold")

    # 1 Spektrogramm (lautester Kanal, erster Schuss)
    ax = axs[0, 0]
    ch = int(np.argmax(rms))
    f_sp, t_sp, S = spectrogram(dec0[:, ch], fs=FS_DEC, nperseg=128, noverlap=112,
                                window="hann", scaling="spectrum")
    r_sp = t_sp * C / 2
    S_db = to_db(np.sqrt(S))
    sel = (f_sp <= 120e3)
    m = ax.pcolormesh(r_sp, f_sp[sel] / 1e3, S_db[sel], cmap="viridis",
                      vmin=DB_FLOOR, vmax=0, shading="auto")
    ax.set_xlim(0, args.rmax)
    ax.axhline(20, color="w", lw=.8, ls="--", alpha=.6)
    ax.axhline(80, color="w", lw=.8, ls="--", alpha=.6)
    ax.axvline(blank_r, color="w", lw=1, alpha=.7)
    ax.text(blank_r, 115, " Ende Sendepuls", color="w", fontsize=8, va="top")
    ax.set_title(f"1 · Spektrogramm, Mikro {ch+1}, Schuss 1", fontsize=10, loc="left")
    ax.set_xlabel("Entfernung [m]"); ax.set_ylabel("Frequenz [kHz]")
    fig.colorbar(m, ax=ax, label="dB", pad=.02)

    # 2 A-Scan
    ax = axs[0, 1]
    for row in A:
        ax.plot(rng, to_db(row, ref=a_mean[far].max()), color="0.75", lw=.6)
    ax.plot(rng, a_db, color="#1F7A8C", lw=1.8, label="Mittel aller Schüsse")
    ax.axvspan(0, blank_r, color="0.9", zorder=0)
    ax.text(blank_r / 2, DB_FLOOR + 3, "Sendepuls\n(blind)", ha="center",
            fontsize=8, color="0.35")
    pk = np.argmax(a_mean * far)
    ax.plot(rng[pk], a_db[pk], "o", ms=7, mfc="none", mec="#C2452D", mew=2)
    ax.annotate(f"{rng[pk]:.2f} m", (rng[pk], a_db[pk]), textcoords="offset points",
                xytext=(8, 6), fontsize=9, color="#C2452D")
    ax.set_ylim(DB_FLOOR, 5); ax.set_xlim(0, args.rmax)
    ax.grid(alpha=.25, lw=.6)
    ax.set_title("2 · A-Scan — Echostärke über Entfernung", fontsize=10, loc="left")
    ax.set_xlabel("Entfernung [m]"); ax.set_ylabel("dB (0 = stärkstes Echo)")
    ax.legend(fontsize=8, frameon=False, loc="upper right")

    # 3 B-Scan
    ax = axs[1, 0]
    B = to_db(A, ref=a_mean[far].max())
    m = ax.pcolormesh(rng, np.arange(1, len(files) + 1), B, cmap="magma",
                      vmin=DB_FLOOR, vmax=0, shading="auto")
    ax.axvline(blank_r, color="w", lw=1, alpha=.7)
    ax.set_title("3 · B-Scan — jeder Schuss eine Zeile", fontsize=10, loc="left")
    ax.set_xlabel("Entfernung [m]"); ax.set_ylabel("Schuss Nr.")
    ax.set_xlim(0, args.rmax)
    if len(files) <= 20:
        ax.set_yticks(np.arange(1, len(files) + 1))
    fig.colorbar(m, ax=ax, label="dB", pad=.02)

    # 4 Kanalcheck
    ax = axs[1, 1]
    rel = 20 * np.log10(rms / np.median(rms))
    bad = np.abs(rel) > 6
    ax.bar(np.arange(1, N_MICS + 1), rel,
           color=np.where(bad, "#C2452D", "#1F7A8C"), width=.7)
    ax.axhspan(-6, 6, color="0.9", zorder=0)
    ax.axhline(0, color="0.4", lw=.8)
    ax.set_ylim(min(-10, rel.min() - 2), max(10, rel.max() + 2))
    ax.set_xticks(np.arange(1, N_MICS + 1))
    ax.set_title("4 · Kanalcheck — Pegel je Mikrofon", fontsize=10, loc="left")
    ax.set_xlabel("Mikrofon"); ax.set_ylabel("dB zum Median")
    ax.text(.5, -8.5, "grauer Streifen = normal (±6 dB)", fontsize=8, color="0.35")
    ax.grid(axis="y", alpha=.25, lw=.6)

    fig.tight_layout(rect=[0, 0, 1, .96])
    out = Path(args.out) if args.out else Path(f"{folder.name}_sensorcheck.png")
    fig.savefig(out, dpi=130)
    print(f"Bild: {out}")

    # ── Klartext-Urteil ─────────────────────────────────────────────────────
    band = (f_sp >= 20e3) & (f_sp <= 80e3)
    tx = S_db[:, r_sp < blank_r]
    in_band = tx[band].max() - tx[~band & (f_sp < 120e3)].max()
    snr = a_db[pk] - np.median(a_db[far])
    spread = A.max(axis=1)
    print("\n── Urteil ───────────────────────────────────────────")
    print(f"  Sendepuls      {'OK' if in_band > 6 else 'VERDÄCHTIG'} — "
          f"Energie im Band 20–80 kHz liegt {in_band:.0f} dB über dem Rest")
    print(f"  Mikrofone      {'alle 16 OK' if not bad.any() else 'AUFFÄLLIG: ' + ', '.join(str(i+1) for i in np.where(bad)[0])}"
          f" — Streuung {rel.min():+.1f} … {rel.max():+.1f} dB zum Median")
    print(f"  Stärkstes Echo {rng[pk]:.2f} m, {snr:.0f} dB über dem Rauschen")
    print(f"  Wiederholbar   Schuss-zu-Schuss {20*np.log10(spread.max()/spread.min()):.1f} dB Spanne"
          f" ({'stabil' if 20*np.log10(spread.max()/spread.min()) < 3 else 'unruhig'})")
    print("─────────────────────────────────────────────────────")


if __name__ == "__main__":
    main()
