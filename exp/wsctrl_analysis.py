import csv
import math
import re
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# Configuration
# ============================================================

FILE_PATTERN = "left_kp_*.txt"

CONTROL_PERIOD_S = 0.010  # nominal 100 Hz

# First test segment begins from rest.
FIRST_SEGMENT_INITIAL_CPS = 0.0

# Analyse the last 50% of each target segment for steady state.
STEADY_STATE_FRACTION = 0.50

# Settling band:
# ± max(5% of commanded step, 150 CPS)
#
# The minimum matters because the encoder-derived speed is
# quantised by roughly 100 CPS at a 10 ms measurement interval.
SETTLING_FRACTION = 0.05
SETTLING_MIN_CPS = 150.0

PWM_SATURATION = 99.5

OUTPUT_DIR = Path("kp_analysis_left")


# ============================================================
# GDB dump parser
# ============================================================

STRUCT_RE = re.compile(
    r"\{targetCps = ([^,]+), "
    r"measuredCps = ([^,]+), "
    r"pwm = ([^,]+), "
    r"error = ([^,]+), "
    r"pidOut = ([^}]+)\}"
)

REPEAT_RE = re.compile(r"<repeats\s+(\d+)\s+times>")


def load_gdb_dump(filename):
    """
    Parse WheelDebugSample structs from a GDB text dump.

    Also understands GDB output such as:

        {...} <repeats 142 times>

    where the displayed struct occurs 142 times total.
    """

    text = Path(filename).read_text(errors="replace")

    events = []

    for m in STRUCT_RE.finditer(text):
        values = tuple(float(v.strip()) for v in m.groups())
        events.append((m.start(), "sample", values))

    for m in REPEAT_RE.finditer(text):
        count = int(m.group(1))
        events.append((m.start(), "repeat", count))

    events.sort(key=lambda x: x[0])

    rows = []
    last_row = None

    for _, kind, value in events:

        if kind == "sample":
            last_row = value
            rows.append(last_row)

        elif kind == "repeat":
            if last_row is None:
                raise RuntimeError(
                    f"Repeat marker found before any sample in {filename}"
                )

            # GDB has already printed one copy of the repeated value.
            rows.extend([last_row] * (value - 1))

    if not rows:
        raise RuntimeError(f"No WheelDebugSample entries found in {filename}")

    data = np.asarray(rows, dtype=float)

    return {
        "target": data[:, 0],
        "measured": data[:, 1],
        "pwm": data[:, 2],
        "error_logged": data[:, 3],
        "pid_out": data[:, 4],
    }


# ============================================================
# Filename handling
# ============================================================

KP_RE = re.compile(r"left_kp_" r"([0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)" r"\.txt$")


def extract_kp(filename):
    m = KP_RE.search(Path(filename).name)

    if not m:
        raise ValueError(f"Could not extract Kp from filename: {filename}")

    return float(m.group(1))


# ============================================================
# Segment detection
# ============================================================


def find_segments(target):
    """
    Returns:
        [(start, end, target_cps), ...]

    end is exclusive.
    """

    transitions = np.where(np.diff(target) != 0)[0] + 1

    starts = np.concatenate(([0], transitions))
    ends = np.concatenate((transitions, [len(target)]))

    return [
        (int(start), int(end), float(target[start])) for start, end in zip(starts, ends)
    ]


# ============================================================
# Transient metrics
# ============================================================


def first_crossing_time(progress, threshold, dt):
    """
    Find the first threshold crossing and linearly interpolate
    between samples.
    """

    indices = np.where(progress >= threshold)[0]

    if len(indices) == 0:
        return math.nan

    i = int(indices[0])

    if i == 0:
        return 0.0

    y0 = progress[i - 1]
    y1 = progress[i]

    if y1 == y0:
        fraction = 0.0
    else:
        fraction = (threshold - y0) / (y1 - y0)
        fraction = float(np.clip(fraction, 0.0, 1.0))

    return ((i - 1) + fraction) * dt


def calculate_settling_time(measured, target, step_magnitude, dt):
    """
    Standard settling-time definition:

    Earliest point after which the response remains inside
    the settling band for the rest of the target segment.
    """

    tolerance = max(SETTLING_FRACTION * step_magnitude, SETTLING_MIN_CPS)

    within_band = np.abs(measured - target) <= tolerance

    # stays_inside[i] is True only if every sample from i onward
    # remains within the band.
    stays_inside = np.logical_and.accumulate(within_band[::-1])[::-1]

    indices = np.where(stays_inside)[0]

    if len(indices) == 0:
        return math.nan, tolerance

    return float(indices[0] * dt), tolerance


def analyse_transition(measured, pwm, pid_out, old_target, new_target, dt):
    """
    Analyse one non-zero target segment.
    """

    step = new_target - old_target

    if step == 0:
        raise ValueError("Transition has zero step size.")

    step_magnitude = abs(step)

    # Normalised response:
    #
    #   0 -> previous target
    #   1 -> new target
    #
    # This works for both positive and negative-going steps.
    progress = (measured - old_target) / step

    t10 = first_crossing_time(progress, 0.10, dt)
    t90 = first_crossing_time(progress, 0.90, dt)

    if math.isnan(t10) or math.isnan(t90):
        rise_time = math.nan
    else:
        rise_time = max(0.0, t90 - t10)

    # --------------------------------------------------------
    # Overshoot
    # --------------------------------------------------------

    max_progress_index = int(np.argmax(progress))
    max_progress = float(progress[max_progress_index])

    overshoot_fraction = max(0.0, max_progress - 1.0)

    overshoot_percent = overshoot_fraction * 100.0
    overshoot_cps = overshoot_fraction * step_magnitude

    peak_time = max_progress_index * dt

    # --------------------------------------------------------
    # Settling
    # --------------------------------------------------------

    settling_time, settling_band = calculate_settling_time(
        measured, new_target, step_magnitude, dt
    )

    # --------------------------------------------------------
    # Steady-state metrics
    # --------------------------------------------------------

    n = len(measured)

    steady_start = int(n * (1.0 - STEADY_STATE_FRACTION))

    ss_measured = measured[steady_start:]
    ss_pwm = pwm[steady_start:]
    ss_pid = pid_out[steady_start:]

    ss_error = new_target - ss_measured

    mean_error = float(np.mean(ss_error))
    mae = float(np.mean(np.abs(ss_error)))
    rmse = float(np.sqrt(np.mean(ss_error**2)))

    error_std = float(np.std(ss_error))
    speed_std = float(np.std(ss_measured))

    mean_pwm = float(np.mean(ss_pwm))
    pwm_std = float(np.std(ss_pwm))

    mean_abs_pid = float(np.mean(np.abs(ss_pid)))
    peak_abs_pid = float(np.max(np.abs(pid_out)))

    saturation_fraction = float(np.mean(np.abs(pwm) >= PWM_SATURATION))

    return {
        "old_target": old_target,
        "target": new_target,
        "step_cps": step,
        "rise_time_s": rise_time,
        "peak_time_s": peak_time,
        "overshoot_cps": overshoot_cps,
        "overshoot_percent": overshoot_percent,
        "settling_time_s": settling_time,
        "settling_band_cps": settling_band,
        "steady_mean_cps": float(np.mean(ss_measured)),
        "steady_mean_error_cps": mean_error,
        "steady_mae_cps": mae,
        "steady_rmse_cps": rmse,
        "steady_error_std_cps": error_std,
        "steady_speed_std_cps": speed_std,
        "steady_mean_pwm": mean_pwm,
        "steady_pwm_std": pwm_std,
        "steady_mean_abs_pid": mean_abs_pid,
        "peak_abs_pid": peak_abs_pid,
        "saturation_fraction": saturation_fraction,
    }


# ============================================================
# Analyse a complete Kp test
# ============================================================


def analyse_run(filename):
    kp = extract_kp(filename)
    data = load_gdb_dump(filename)

    segments = find_segments(data["target"])

    metrics = []

    previous_target = FIRST_SEGMENT_INITIAL_CPS

    for segment_number, (start, end, target) in enumerate(segments):

        measured = data["measured"][start:end]
        pwm = data["pwm"][start:end]
        pid_out = data["pid_out"][start:end]

        # target = 0 is currently open-loop neutral/coast-down,
        # not closed-loop zero-speed regulation.
        #
        # Don't include it in P-controller tuning metrics.
        if target != 0.0:

            result = analyse_transition(
                measured=measured,
                pwm=pwm,
                pid_out=pid_out,
                old_target=previous_target,
                new_target=target,
                dt=CONTROL_PERIOD_S,
            )

            result["kp"] = kp
            result["filename"] = Path(filename).name
            result["segment"] = segment_number
            result["samples"] = end - start

            metrics.append(result)

        previous_target = target

    return {
        "kp": kp,
        "filename": Path(filename).name,
        "data": data,
        "segments": segments,
        "metrics": metrics,
    }


# ============================================================
# Aggregate one Kp run
# ============================================================


def nanmean(values):
    values = np.asarray(values, dtype=float)

    if np.all(np.isnan(values)):
        return math.nan

    return float(np.nanmean(values))


def aggregate_run(run):
    rows = run["metrics"]

    return {
        "kp": run["kp"],
        "filename": run["filename"],
        "samples": len(run["data"]["target"]),
        "transitions": len(rows),
        "mean_rise_time_s": nanmean([r["rise_time_s"] for r in rows]),
        "mean_settling_time_s": nanmean([r["settling_time_s"] for r in rows]),
        # Worst overshoot is often more useful than average
        # when choosing a controller gain.
        "max_overshoot_percent": float(max(r["overshoot_percent"] for r in rows)),
        "mean_overshoot_percent": float(
            np.mean([r["overshoot_percent"] for r in rows])
        ),
        "mean_steady_abs_bias_cps": float(
            np.mean([abs(r["steady_mean_error_cps"]) for r in rows])
        ),
        "mean_steady_mae_cps": float(np.mean([r["steady_mae_cps"] for r in rows])),
        "mean_steady_rmse_cps": float(np.mean([r["steady_rmse_cps"] for r in rows])),
        "mean_steady_noise_cps": float(
            np.mean([r["steady_speed_std_cps"] for r in rows])
        ),
        "mean_pwm_std": float(np.mean([r["steady_pwm_std"] for r in rows])),
        "mean_abs_pid": float(np.mean([r["steady_mean_abs_pid"] for r in rows])),
        "max_saturation_fraction": float(max(r["saturation_fraction"] for r in rows)),
    }


# ============================================================
# CSV output
# ============================================================


def write_csv(filename, rows):
    if not rows:
        return

    keys = list(rows[0].keys())

    with open(filename, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


# ============================================================
# Console summary
# ============================================================


def print_run_summary(summary):
    print(
        f"Kp={summary['kp']:<8g}  "
        f"samples={summary['samples']:<5d}  "
        f"rise={summary['mean_rise_time_s'] * 1000:6.1f} ms  "
        f"settle={summary['mean_settling_time_s'] * 1000:6.1f} ms  "
        f"max OS={summary['max_overshoot_percent']:5.1f}%  "
        f"RMSE={summary['mean_steady_rmse_cps']:6.1f} CPS"
    )


# ============================================================
# Final comparison graph
# ============================================================


def plot_comparison(runs, summaries):
    """
    One figure with four views:

      1. Full speed response overlay
      2. Rise/settling time vs Kp
      3. Overshoot vs Kp
      4. Steady-state error/noise vs Kp
    """

    summaries = sorted(summaries, key=lambda x: x["kp"])

    kps = np.asarray([s["kp"] for s in summaries], dtype=float)

    rise_ms = np.asarray([s["mean_rise_time_s"] * 1000 for s in summaries])

    settling_ms = np.asarray([s["mean_settling_time_s"] * 1000 for s in summaries])

    overshoot = np.asarray([s["max_overshoot_percent"] for s in summaries])

    rmse = np.asarray([s["mean_steady_rmse_cps"] for s in summaries])

    noise = np.asarray([s["mean_steady_noise_cps"] for s in summaries])

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))

    # --------------------------------------------------------
    # Full response overlay
    # --------------------------------------------------------

    ax = axes[0, 0]

    target_plotted = False

    for run in sorted(runs, key=lambda x: x["kp"]):

        data = run["data"]

        t = np.arange(len(data["target"])) * CONTROL_PERIOD_S

        if not target_plotted:
            ax.plot(t, data["target"], linestyle="--", linewidth=1.5, label="Target")
            target_plotted = True

        ax.plot(t, data["measured"], linewidth=1.0, label=f"Kp={run['kp']:g}")

    ax.set_title("Full speed response")
    ax.set_xlabel("Nominal time (s)")
    ax.set_ylabel("Speed (CPS)")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8)

    # --------------------------------------------------------
    # Rise and settling time
    # --------------------------------------------------------

    ax = axes[0, 1]

    ax.plot(kps, rise_ms, marker="o", label="Mean 10–90% rise time")

    ax.plot(kps, settling_ms, marker="o", label="Mean settling time")

    ax.set_title("Transient speed")
    ax.set_xlabel("Kp")
    ax.set_ylabel("Time (ms)")
    ax.grid(True, alpha=0.25)
    ax.legend()

    # --------------------------------------------------------
    # Overshoot
    # --------------------------------------------------------

    ax = axes[1, 0]

    ax.plot(kps, overshoot, marker="o")

    ax.set_title("Worst transition overshoot")
    ax.set_xlabel("Kp")
    ax.set_ylabel("Overshoot (%)")
    ax.grid(True, alpha=0.25)

    # --------------------------------------------------------
    # Steady-state tracking
    # --------------------------------------------------------

    ax = axes[1, 1]

    ax.plot(kps, rmse, marker="o", label="Mean steady-state RMSE")

    ax.plot(kps, noise, marker="o", label="Mean speed standard deviation")

    ax.set_title("Steady-state tracking")
    ax.set_xlabel("Kp")
    ax.set_ylabel("CPS")
    ax.grid(True, alpha=0.25)
    ax.legend()

    fig.suptitle("Left Wheel P-Controller Gain Comparison")

    fig.tight_layout()

    output = OUTPUT_DIR / "left_kp_comparison.png"

    fig.savefig(output, dpi=200, bbox_inches="tight")

    plt.show()

    return output


# ============================================================
# Main
# ============================================================


def main():
    files = list(Path(".").glob(FILE_PATTERN))

    if not files:
        raise RuntimeError(f"No files matching {FILE_PATTERN!r} found.")

    # Sort numerically by Kp, not alphabetically.
    files = sorted(files, key=extract_kp)

    OUTPUT_DIR.mkdir(exist_ok=True)

    print(f"Found {len(files)} Kp test files:\n")

    for filename in files:
        print(f"  {filename.name:<28} " f"Kp={extract_kp(filename):g}")

    print()

    runs = []
    detailed_rows = []
    summaries = []

    for filename in files:

        run = analyse_run(filename)
        runs.append(run)

        detailed_rows.extend(run["metrics"])

        summary = aggregate_run(run)
        summaries.append(summary)

        print_run_summary(summary)

    # --------------------------------------------------------
    # CSV output
    # --------------------------------------------------------

    detailed_csv = OUTPUT_DIR / "left_kp_transition_metrics.csv"

    summary_csv = OUTPUT_DIR / "left_kp_summary.csv"

    write_csv(detailed_csv, detailed_rows)

    write_csv(summary_csv, summaries)

    # --------------------------------------------------------
    # Comparison figure
    # --------------------------------------------------------

    comparison_png = plot_comparison(runs, summaries)

    print()
    print("Outputs:")
    print(f"  {detailed_csv}")
    print(f"  {summary_csv}")
    print(f"  {comparison_png}")


if __name__ == "__main__":
    main()
