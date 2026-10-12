#!/usr/bin/env python3
"""Check local FF limits and in-motion cruise changes against production C."""
from pathlib import Path
import test_fused_motion as baseline

if __name__ == "__main__":
    baseline.HARNESS = baseline.HARNESS.replace("int main(void)", "int historicalMain(void)")
    baseline.HARNESS += Path(__file__).with_name("local_feedforward_cruise_harness.inc").read_text()
    baseline.main()
