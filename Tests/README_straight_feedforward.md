# Straight feedforward calibration

`MotionControllerStraightFeedforwardTest.c` is a separate adaptation of
`MotionControllerArcTest_original.c`. The original file and the production
feedforward tables retain the pending radius −275 mm / raw +95 calibration.

## Select and configure the test

In `Tests/Src/TestMain.c`, replace the selected test call with:

```c
MotionControllerStraightFeedforwardTestRun();
```

Its header is already included. The new source is included by the Debug build's
existing `Tests` source entry. Only one test should be called per firmware run.
The sequence test remains selected in the committed checkout.

Edit these settings at the top of `Tests/Src/MotionControllerStraightFeedforwardTest.c`:

```c
#define STRAIGHT_FF_TEST_DISTANCE_MM  (1000.0f)
#define STRAIGHT_FF_TEST_RAW_COMMAND  (-9.52502251f)
#define STRAIGHT_FF_TEST_SPEED_CPS    (2000.0f)
```

Use negative distance for reverse, keeping speed positive. The default raw
command reproduces the centre captured in the 1 October straight acquisition;
it is a starting baseline, not a newly validated feedforward point.

The harness retains deterministic initial centering. After `MoveStraight()`
succeeds, it overrides the captured feedforward and applies the candidate raw
command before the first controller update. The 500 ms preparation period
holds that candidate with the rear wheels stationary. Raw values outside the
servo envelope, or nonfinite values, are rejected rather than silently clamped.

Both shared steering feedback stages have zero gain. Desired path and rear-wheel
reference curvature stay zero, so geometric left/right base targets are equal.
Wheel-speed and synchronization feedback remain enabled; synchronization may
still adjust the individual targets to enforce equal cumulative travel.

Keep the same initial centering and candidate approach procedure for all runs.
Compare yaw at `STRAIGHT -> BRAKING`, because braking recentres steering. Logs
also retain final yaw and distance, applied raw command, captured feedforward,
yaw-rate diagnostics, wheel relationship, brake diagnostics and model validity.
The legacy effective-angle estimates are stale during raw control.

## Automatic CubeIDE export

Duplicate your CubeIDE debug launch for this test. In its post-load Run Commands,
replace the sequence export source line with:

```gdb
cd ${workspace_loc:/MDP_Controller}
source exp/gdb_scripts/auto_export_straight_feedforward.gdb
```

Build the selected test, start debugging, and press SW1 as prompted. The script
stops at `MotionControllerStraightFeedforwardTest_ShowFinal`, after the result
globals are finalized, and exports to a unique timestamped file under
`exp/logs/straight_feedforward/`. The target stays suspended for review.
Resume to show the final OLED screen. No filename edit is needed between runs.

Optional console commands, with the target suspended for manual export:

```gdb
mctrl-straight-ff-battery 11.95
mctrl-straight-ff-export
```

Battery defaults to unknown. Export timestamps describe host export time, not
motion start. Each file records the actual test configuration and requested raw
command, loaded ELF information, checkout metadata, completion/timeout/control
status, exit/final results, and the sample array. A complete file ends with
`=== EXPORT COMPLETE ===`; the console then reports `EXPORTED: <path>`.

The existing sequence runner and shared sequence launch continue to serve
`MotionControllerSequenceTestRun()`. This harness uses its dedicated export
script. Initial fixture failure occurs before motion and does not reach the
completion export hook; the OLED reports initialization failure.
