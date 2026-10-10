"""Checks whole-sweep parser, plateau rejection and acceleration measurements."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import sys

sys.dont_write_bytecode = True

root = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    "analyze_sweep", root / "exp/wheel_performance/analyze_sweep.py")
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


def record(values):
    # Include GDB's uint8 character annotation and multiline formatting.
    return "{" + ",\n".join(
        f"{key} = {value}" + (" '\\001'" if key in ("trial", "phase") else "")
        for key, value in values.items()) + "}"


def export(kind=1, status=2):
    config = dict(experiment=kind, steadyWindowMs=500, batteryVoltage=11.3)
    trace = []
    trials = []
    for index, direction in enumerate((1, 2)):
        sign = 1 if direction == 1 else -1
        start = len(trace)
        for time in range(0, 1501, 20):
            trace.append(dict(timeMs=time, dtMs=10, trial=index,
                phase=1 if kind == 1 else 2,
                targetCps=0 if kind == 1 else sign * time * 3,
                leftCps=sign * (9000 if kind == 1 else time * 3),
                rightCps=sign * (8800 if kind == 1 else time * 2.9),
                leftPwmX100=sign * 10000, rightPwmX100=sign * 9900,
                leftMode=1, rightMode=1))
        trials.append(dict(direction=direction, repetition=1, pwmPercent=sign*100,
            accelerationMmps2=500, targetCps=sign*5000, firstSample=start,
            sampleCount=len(trace)-start, driveMs=1500, status=status))
    sections = {
        "CONFIG": "$1 = " + record(config),
        "METADATA": f"$2 = {status}\n$3 = 0.132\n$4 = 2\n$5 = {len(trace)}",
        "TRIALS": "$6 = {" + ",\n".join(map(record, trials)) + "}",
        "TRACE": "$7 = {" + ",\n".join(map(record, trace)) + "}"}
    return "WHEEL_PERFORMANCE_V1\n" + "\n".join(
        f"{name}_BEGIN\n{data}\n{name}_END" for name, data in sections.items())


class ExportChecks(unittest.TestCase):
    def load(self, text):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sweep.txt"
            path.write_text(text)
            return analysis.load_export(path)

    def test_complete_both_directions(self):
        config, status, mm, trials, trace = self.load(export())
        summaries = analysis.analyze(config, mm, trials, trace)
        self.assertEqual(status, 2)
        self.assertEqual(len(trace), 152)
        for summary in summaries:
            self.assertTrue(summary["leftPlateau"])
            self.assertTrue(summary["rightPlateau"])
            self.assertEqual(summary["leftSteadyMeanCps"], 9000)
            self.assertEqual(summary["rightSteadyMeanCps"], 8800)

    def test_aborted_trial_not_a_ceiling(self):
        config, _, mm, trials, trace = self.load(export(status=4))
        self.assertFalse(analysis.analyze(config, mm, trials, trace)[0]["leftPlateau"])

    def test_unsettled_response_not_a_ceiling(self):
        config, _, mm, trials, trace = self.load(export())
        for sample in trace:
            sample["leftCps"] = sample["timeMs"] * 5
        self.assertFalse(analysis.analyze(config, mm, trials, trace)[0]["leftPlateau"])

    def test_acceleration_fit_and_reverse_sign(self):
        config, _, mm, trials, trace = self.load(export(kind=2))
        summaries = analysis.analyze(config, mm, trials, trace)
        for summary in summaries:
            self.assertAlmostEqual(summary["leftRampAccelMmps2"], 396)
            self.assertAlmostEqual(summary["rightRampAccelMmps2"], 382.8)
            self.assertEqual(summary["leftRampSaturationFraction"], 1)
            self.assertEqual(summary["rightRampSaturationFraction"], 0)

    def test_incomplete_export_rejected(self):
        with self.assertRaises(ValueError):
            self.load(export().replace("TRACE_END", ""))
        with self.assertRaises(ValueError):
            self.load(export().replace("$5 = 152", "$5 = 153"))

    def test_cubeide_mi_console_export(self):
        plain = export()
        wrapped = '=cmd-param-changed,param="logging enabled",value="on"\n' + "\n".join(
            "~" + json.dumps(line + "\n") for line in plain.splitlines())
        self.assertEqual(self.load(wrapped), self.load(plain))

    def test_invalid_mi_console_record_rejected(self):
        with self.assertRaisesRegex(ValueError, "Malformed GDB/MI"):
            self.load('~"WHEEL_PERFORMANCE_V1\\q"')


if __name__ == "__main__":
    unittest.main()
