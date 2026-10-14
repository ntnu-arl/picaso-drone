#!/usr/bin/env python3
"""
miccheck.py — Sind alle 16 Mikrofone richtig angeschlossen?

Ein reiner Pegeltest reicht nicht: eine abgerissene PDM-Datenleitung rauscht
oft munter weiter und sieht im Pegel gesund aus. Deshalb fuenf unabhaengige
Pruefungen je Kanal, davon drei, die ein totes Kabel nicht vortaeuschen kann:

  1 Bitdichte     Mittelwert des 1-Bit-Stroms. Gesund ~0.5; 0 oder 1 = Leitung
                  haengt fest (nicht angeschlossen oder Mikro ohne Takt).
  2 Flankenrate   Wie oft kippt das Bit? Eine tote Leitung kippt nie.
  3 Pulsantwort   Jedes Mikro MUSS den eigenen Sendepuls hoeren — er ist das
                  lauteste Signal ueberhaupt. Fehlt er, ist das Mikro akustisch
                  oder elektrisch ab.
  4 Gleichlauf    Korrelation des Sendepulses mit dem Mittel der anderen Mikros.
                  Ein Kanal, der nur rauscht, korreliert nicht. Das ist der
                  schaerfste Test.
  5 Bandlage      Liegt die Energie des Pulses im Sendeband 20-80 kHz?

Die Zaehlung folgt der Platine (Mikro 1..16), nicht dem Bit im Datenwort —
das "swap"-Mapping aus beamformFRTIS.m ist beruecksichtigt.

Aufruf
------
    python miccheck.py 2026_10_02_125623_recheck
"""
import argparse
from pathlib import Path

import numpy as np
from scipy.signal import butter, lfilter, welch
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from beamform_frtis import N_MICS, FS_RAW, FS_DEC, DECIM, MF_BAND, capture_list

PULSE_LO, PULSE_HI = 60, 900     # dezimierte Samples: der laute Teil des Sendepulses
OK, WARN, DEAD = "OK", "?", "TOT"


def bit_to_mic(b):
    """Bit b im Datenwort -> Mikrofonnummer auf der Platine (pos_swap)."""
    return 2 * b + 2 if b < 8 else 2 * (b - 8) + 1


def load(path):
    """Rohbits (ungefiltert) und gefiltert+dezimiert zurueckgeben."""
    raw = np.fromfile(path, dtype=np.uint16)
    bits = ((raw[:, None] >> np.arange(N_MICS)) & 1).astype(np.uint8)
    b, a = butter(6, 100e3 / (FS_RAW / 2))
    f = lfilter(b, a, bits.astype(float) - bits.mean(axis=0, keepdims=True), axis=0)
    return bits, f[::DECIM, :]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder")
    ap.add_argument("--limit", type=int, default=5, help="so viele Schuesse mitteln")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    folder = Path(args.folder)
    files = capture_list(folder)[:args.limit]
    if not files:
        raise SystemExit(f"Keine capture_*.bin in {folder}")

    dens, togg, pulse_rms, inband, dec0 = [], [], [], [], None
    for f in files:
        bits, dec = load(f)
        if dec0 is None:
            dec0 = dec
        dens.append(bits.mean(axis=0))
        togg.append((np.diff(bits.astype(np.int8), axis=0) != 0).mean(axis=0))
        seg = dec[PULSE_LO:PULSE_HI, :]
        pulse_rms.append(seg.std(axis=0))
        fq, P = welch(seg, fs=FS_DEC, nperseg=256, axis=0)
        band = (fq >= MF_BAND[0]) & (fq <= MF_BAND[1])
        inband.append(P[band].sum(axis=0) / (P.sum(axis=0) + 1e-20))
    dens = np.mean(dens, 0); togg = np.mean(togg, 0)
    pulse_rms = np.mean(pulse_rms, 0); inband = np.mean(inband, 0)

    # Gleichlauf: jeder Kanal gegen das Mittel der uebrigen 15
    seg = dec0[PULSE_LO:PULSE_HI, :]
    segn = seg / (seg.std(axis=0, keepdims=True) + 1e-20)
    corr = np.empty(N_MICS)
    for m in range(N_MICS):
        others = np.delete(segn, m, axis=1).mean(axis=1)
        others /= others.std() + 1e-20
        corr[m] = float(np.dot(segn[:, m], others) / len(others))

    pulse_db = 20 * np.log10(pulse_rms / np.median(pulse_rms))
    mics = np.array([bit_to_mic(b) for b in range(N_MICS)])

    # ── Urteil je Kanal ─────────────────────────────────────────────────────
    verdict, reason = [], []
    for m in range(N_MICS):
        why = []
        if dens[m] < 0.02 or dens[m] > 0.98: why.append("Bit haengt fest")
        if togg[m] < 0.01:                   why.append("keine Flanken")
        if corr[m] < 0.3:                    why.append("kein Gleichlauf")
        if abs(pulse_db[m]) > 6:             why.append(f"Pegel {pulse_db[m]:+.0f} dB")
        if inband[m] < 0.25:                 why.append("Energie ausserhalb 20-80 kHz")
        if any(w in why for w in ("Bit haengt fest", "keine Flanken")) or corr[m] < 0.15:
            verdict.append(DEAD)
        elif why:
            verdict.append(WARN)
        else:
            verdict.append(OK)
        reason.append(", ".join(why))

    order = np.argsort(mics)      # nach Platinennummer sortiert ausgeben

    print(f"\n  {len(files)} Schuesse aus {folder.name}\n")
    print("  Mikro  Bit   Bitdichte  Flanken  Puls[dB]  Gleichlauf  Band   Urteil")
    print("  " + "─" * 70)
    for m in order:
        print(f"  {mics[m]:>4}  {m:>4}   {dens[m]:8.3f}  {togg[m]:7.3f}  "
              f"{pulse_db[m]:+8.1f}  {corr[m]:10.2f}  {inband[m]*100:4.0f}%   "
              f"{verdict[m]:>4}  {reason[m]}")
    print("  " + "─" * 70)
    n_ok = verdict.count(OK)
    print(f"\n  {n_ok}/16 einwandfrei"
          + (f", {verdict.count(WARN)} auffaellig" if WARN in verdict else "")
          + (f", {verdict.count(DEAD)} TOT" if DEAD in verdict else ""))
    print("  → " + ("alle Mikrofone sind angeschlossen und arbeiten."
                    if n_ok == N_MICS else
                    "Kanaele pruefen: " + ", ".join(
                        f"Mikro {mics[m]}" for m in order if verdict[m] != OK)))

    # ── Bild ────────────────────────────────────────────────────────────────
    col = {OK: "#1F7A8C", WARN: "#D98324", DEAD: "#C2452D"}
    cols = [col[v] for v in np.array(verdict)[order]]
    lbl = mics[order]
    x = np.arange(N_MICS)

    fig, axs = plt.subplots(2, 2, figsize=(13.5, 8.6))
    fig.suptitle(f"Mikrofon-Anschlusstest — {folder.name}   ({len(files)} Schüsse)",
                 fontsize=13, fontweight="bold")

    ax = axs[0, 0]
    ax.plot(np.arange(PULSE_LO, PULSE_HI) / FS_DEC * 1e3, seg[:, order], lw=.7, alpha=.75)
    ax.set_title("1 · Sendepuls, alle 16 Kanäle übereinander", fontsize=10, loc="left")
    ax.set_xlabel("Zeit seit Auslösung [ms]"); ax.set_ylabel("Amplitude")
    ax.grid(alpha=.25, lw=.6)
    ax.text(.98, .04, "ein abweichender Kanal fällt hier sofort auf",
            transform=ax.transAxes, ha="right", fontsize=8, color="0.4")

    ax = axs[0, 1]
    ax.bar(x, corr[order], color=cols, width=.7)
    ax.axhspan(0.3, 1.05, color="0.92", zorder=0)
    ax.axhline(0.3, color="0.45", lw=.8, ls="--")
    ax.set_ylim(min(-.1, corr.min() - .1), 1.05)
    ax.set_xticks(x); ax.set_xticklabels(lbl, fontsize=8)
    ax.set_title("2 · Gleichlauf mit den übrigen Mikrofonen", fontsize=10, loc="left")
    ax.set_xlabel("Mikrofon"); ax.set_ylabel("Korrelation")
    ax.text(.3, .02, "grau = angeschlossen (> 0.3)", transform=ax.transAxes,
            fontsize=8, color="0.4")
    ax.grid(axis="y", alpha=.25, lw=.6)

    ax = axs[1, 0]
    ax.bar(x, pulse_db[order], color=cols, width=.7)
    ax.axhspan(-6, 6, color="0.92", zorder=0)
    ax.axhline(0, color="0.45", lw=.8)
    ax.set_ylim(min(-12, pulse_db.min() - 2), max(12, pulse_db.max() + 2))
    ax.set_xticks(x); ax.set_xticklabels(lbl, fontsize=8)
    ax.set_title("3 · Lautstärke des Sendepulses je Mikrofon", fontsize=10, loc="left")
    ax.set_xlabel("Mikrofon"); ax.set_ylabel("dB zum Median")
    ax.grid(axis="y", alpha=.25, lw=.6)

    ax = axs[1, 1]
    ax.bar(x - .19, dens[order], width=.36, color=cols, label="Bitdichte (Soll ~0.5)")
    ax.bar(x + .19, togg[order], width=.36, color=cols, alpha=.45,
           label="Flankenrate (Soll > 0)")
    ax.axhline(.5, color="0.45", lw=.8, ls="--")
    ax.set_ylim(0, 1)
    ax.set_xticks(x); ax.set_xticklabels(lbl, fontsize=8)
    ax.set_title("4 · Lebenszeichen des 1-Bit-Datenstroms", fontsize=10, loc="left")
    ax.set_xlabel("Mikrofon"); ax.set_ylabel("Anteil")
    ax.legend(fontsize=8, frameon=False, loc="upper right")
    ax.grid(axis="y", alpha=.25, lw=.6)

    fig.tight_layout(rect=[0, 0, 1, .95])
    out = Path(args.out) if args.out else Path(f"{folder.name}_miccheck.png")
    fig.savefig(out, dpi=130)
    print(f"\n  Bild: {out}")


if __name__ == "__main__":
    main()
