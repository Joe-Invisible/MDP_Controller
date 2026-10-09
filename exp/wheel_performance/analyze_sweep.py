#!/usr/bin/env python3
"""Read one whole-sweep GDB export; emit trace CSV and per-trial measurements.

Only Python's standard library is required. Acceleration results describe
measured wheel-speed response, not an automatically certified operating limit.
"""
import argparse
import csv
import math
from pathlib import Path
import re
import statistics


def section(text, name):
    match = re.search(rf"{name}_BEGIN\s*(.*?)\s*{name}_END", text, re.S)
    if not match:
        raise ValueError(f"Missing {name} section; export may be incomplete")
    return match.group(1)


def scalar(value):
    value = value.strip()
    if value in ("true", "false"):
        return value == "true"
    match = re.match(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", value)
    if not match:
        raise ValueError(f"Expected numeric field, got {value!r}")
    number = float(match.group())
    if not math.isfinite(number):
        raise ValueError("Nonfinite export value")
    return number


def records(text, name):
    blocks = re.findall(r"\{([^{}]*)\}", section(text, name), re.S)
    return [dict((key, scalar(value)) for key, value in
                 re.findall(r"(\w+)\s*=\s*([^,}]+)", block)) for block in blocks]


def slope(samples, wheel):
    if len(samples) < 3:
        return None
    x = [s["timeMs"] / 1000 for s in samples]
    y = [s[wheel] for s in samples]
    xm, ym = statistics.mean(x), statistics.mean(y)
    denominator = sum((t - xm) ** 2 for t in x)
    return (sum((t - xm) * (v - ym) for t, v in zip(x, y)) / denominator
            if denominator else None)


def load_export(path):
    text = Path(path).read_text(encoding="utf-8")
    if "WHEEL_PERFORMANCE_V1" not in text:
        raise ValueError("Not a wheel-performance export")
    configs = records(text, "CONFIG")
    if len(configs) != 1:
        raise ValueError("Expected one frozen configuration")
    metadata = re.findall(r"\$\d+\s*=\s*([^\n]+)", section(text, "METADATA"))
    if len(metadata) != 4:
        raise ValueError("Expected status, conversion, trial count and sample count")
    status, mm_per_count, trial_count, sample_count = map(scalar, metadata)
    trials = records(text, "TRIALS")
    trace = records(text, "TRACE")
    if len(trials) != trial_count or len(trace) != sample_count:
        raise ValueError("Record counts disagree; export is incomplete")
    for index, trial in enumerate(trials):
        start, count = int(trial["firstSample"]), int(trial["sampleCount"])
        samples = trace[start:start + count]
        if len(samples) != count or any(s["trial"] != index for s in samples):
            raise ValueError(f"Trial {index} has an invalid trace range")
        if any(b["timeMs"] <= a["timeMs"] for a, b in zip(samples, samples[1:])):
            raise ValueError(f"Trial {index} timestamps are not increasing")
    return configs[0], status, mm_per_count, trials, trace


def analyze(config, mm_per_count, trials, trace):
    summaries = []
    for index, trial in enumerate(trials):
        start, count = int(trial["firstSample"]), int(trial["sampleCount"])
        samples = trace[start:start + count]
        summary = {"trial": index, **trial}
        sign = 1 if trial["direction"] == 1 else -1
        for wheel, pwm in (("leftCps", "leftPwmX100"), ("rightCps", "rightPwmX100")):
            prefix = "left" if wheel == "leftCps" else "right"
            if config["experiment"] == 1:
                window = config["steadyWindowMs"]
                tail = [s for s in samples if s["phase"] == 1 and
                        s["timeMs"] >= trial["driveMs"] - window]
                mean = statistics.mean(sign * s[wheel] for s in tail) if tail else None
                rate = slope(tail, wheel)
                summary[prefix + "SteadyMeanCps"] = mean
                summary[prefix + "SteadySlopeCps2"] = sign * rate if rate is not None else None
                # A plateau must cover the requested window and change <=5%.
                covered = bool(tail) and (tail[-1]["timeMs"] - tail[0]["timeMs"]
                                         >= window - 2 * 20)
                summary[prefix + "Plateau"] = bool(trial["status"] == 2 and covered and
                    mean and mean > 0 and rate is not None and
                    abs(rate) * window / 1000 <= 0.05 * mean)
            else:
                ramp = [s for s in samples if s["phase"] == 2 and s["timeMs"] > 0]
                rate = slope(ramp, wheel)
                summary[prefix + "RampAccelMmps2"] = (
                    sign * rate * mm_per_count if rate is not None else None)
                summary[prefix + "RampMeanAbsErrorCps"] = (
                    statistics.mean(abs(s["targetCps"] - s[wheel]) for s in ramp)
                    if ramp else None)
                summary[prefix + "RampMaxLagCps"] = (
                    max(sign * (s["targetCps"] - s[wheel]) for s in ramp) if ramp else None)
                summary[prefix + "RampSaturationFraction"] = (
                    statistics.mean(abs(s[pwm]) >= 9990 and s[prefix + "Mode"] == 1
                                    for s in ramp) if ramp else None)
        summaries.append(summary)
    return summaries


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("export", type=Path)
    args = parser.parse_args()
    config, status, mm_per_count, trials, trace = load_export(args.export)
    summaries = analyze(config, mm_per_count, trials, trace)
    write_csv(args.export.with_suffix(".trace.csv"), trace)
    write_csv(args.export.with_suffix(".trials.csv"), summaries)
    print(f"Sweep status={int(status)}, trials={len(trials)}, samples={len(trace)}, "
          f"battery={config['batteryVoltage']:g} V, mm/count={mm_per_count:.8f}")
    if not config["batteryVoltage"]:
        print("Battery voltage was not entered.")
    if status != 2:
        print("Sweep did not complete; inspect the status and retained partial trials.")
    if config["experiment"] == 1:
        for direction, label in ((1, "forward"), (2, "reverse")):
            full = [s for s in summaries if s["direction"] == direction and
                    abs(s["pwmPercent"]) == 100]
            if not full or not all(s["leftPlateau"] and s["rightPlateau"] for s in full):
                print(f"{label}: 100% PWM ceiling not established; missing/unstable trials.")
                continue
            left = min(s["leftSteadyMeanCps"] for s in full)
            right = min(s["rightSteadyMeanCps"] for s in full)
            print(f"{label}: minimum repeated 100% means: left={left:.0f}, "
                  f"right={right:.0f}, shared={min(left, right):.0f} CPS")
    else:
        print("Acceleration CSV includes ramp fits, tracking error and saturation.")
        print("A passing highest point only establishes a tested value, not the upper limit.")
    print(f"CSV files: {args.export.with_suffix('.trace.csv')}, "
          f"{args.export.with_suffix('.trials.csv')}")


if __name__ == "__main__":
    main()
