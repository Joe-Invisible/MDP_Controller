# Sensor integration and calibration TODO

Requested order: front ultrasonic integration and calibration, then left/right
IR integration and calibration.

## Current status — 1 October 2026, afternoon continuation

This section and the final protocol checklist supersede historical entries below.

- U/Q P are implemented and flashed. Last installed image is the corrected
  profile integration plus G reset at the explicit 300 mm/s test cap; normal
  source defaults to 100 mm/s. See mdp_checklist/docs/verification/reset_20261001.json
  for current image hashes. The earlier cancellation used the same U policy.
- Two consecutive 30-cycle F100/B100 tests with sensor polling passed; the
  1 mm straight tolerance remains an acceptance workaround, not a proven fix
  for the earlier intermittent low-speed stall. Friend's controller/driver
  tuning is unchanged (47 protected files hash-checked again this afternoon).
- B100;F100;B100 partial cancellation passed: one command complete, second
  interrupted at 41.95 mm final encoder travel, third cancelled, stable IDLE.
- U200;B20 cancellation passed at 12:45: S requested at 55.66 mm encoder
  travel; STOPPED 0 and Q P at 75.98 mm, zero commands complete, step 1 U200,
  IDLE and unchanged one second later. B20 did not execute. Starting sensor
  gap 509.3 mm, final sensor 428.9 mm; these are not ruler measurements.
- Reusable button-gated test: checks/ultrasonic_stop.py --report NEW_FILE.json.
  Tests use the existing sender and one serial owner; no new production queries,
  controller tasks or buffers. Six new regression tests pass on Mac and Pi;
  full Mac/Pi verification for that phase passed 190 tests, zero skips,
  and 15 entry-point checks.
- A second corrected U approach passed: sensor 569.5 to 204.1 mm; user confirmed
  approximately 200 mm. Twenty stationary readings were 203.9..204.2 mm; saved
  DONE had zero outputs and maximum control interval 13 ms. U400 at sensor
  206.6 mm correctly faulted US_TOO_CLOSE before either U400 or queued B20
  started; S cleared the runtime latch.
- Remaining protocol hardware work: more surface/repeatability measurements
  and missing echo/loss during motion. The current room still returns a valid
  wall/furniture echo (433.5 mm), so missing-echo hardware test is not complete. Remaining
  sensing work: actual-obstacle IR/ultrasonic validation and side-edge behavior.

## Explicit reset addition — 1 October, flashed and stationary verified

- [x] Inspect Team 37's G -> NVIC_SystemReset path and current controller/session.
- [x] Implement standalone idle-only G and INIT fault-loop recovery in our
  application layer. Preserve controller/driver tuning and reject G in batches.
- [x] Pi helper stops first when needed, saves available progress before G,
  requires RESETTING then READY, and starts the new sequence at zero.
  Latest batch/reply/progress use bounded in-memory records; no normal-driving
  disk writes or extra queries. New processes cannot recover full old batches.
- [x] Test stop/save/reset ordering, UART silence, missing/stale READY, rejected
  reset, INIT failure, disk failure, no replay, and explicit new sequence zero.
- [x] Full Mac/Pi verification: 205 tests, no skips, 16 entry-point checks.
- [x] Flash/verify and stationary hardware reboot: G -> RESETTING -> READY in
  about 2 s, next batch ID 0, fresh button counter 0, valid U and both IR channels.
  No button press or driving command required. The saved progress was empty
  because flashing had already reset the session; no interrupted-move hardware
  persistence claim. Existing fault/stop/save paths are covered by software tests.
- [x] Repair deployment after a Pi reboot left two new files and the first remote
  backup empty. Atomic replacements, disk flushes and 27 file hashes verified;
  complete Pi suite then passed. Reboot cause remains unknown. Mac pre-reset
  source/ELF backup remains available; do not use the zero-byte Pi backup.
- [ ] True CPU-lockup recovery needs separately designed watchdog or NRST wiring;
  not supplied by a UART command. No automatic route resume after a reset.

## Delivery priorities and scope

- User schedule: Task 1 next Friday; Task 2 the following week. Prioritize
  stable Task 1 communication/sensing and controlled regression checks.
- Preserve friend's motor, steering, arc, kinematics, calibration and existing
  sensor-driver code. Cleanup is limited to our application integration and
  communication additions. No broad formatting changes.
- Task 2 speed increases, continuous transitions and an asynchronous UART TX
  redesign are separate measured changes, not prerequisites for Task 1 cleanup.

## Source cleanup — 1 October 2026 (flashed; stationary checks passed)

- [x] Centralize acquisition/display periods and separate IR/ultrasonic stale
  thresholds in Apps/Inc/SensorConfig.h. Acquisition periods remain 50/70 ms;
  freshness remains 250 ms. No wire-format or motion-command changes.
- [x] Split SensorTask into initialization, acquisition, snapshot publication
  and display helpers. Check each IR initialization result independently;
  failed configuration publishes ERROR without reading an uninitialized driver.
- [x] Gate OLED work with SENSOR_OLED_DIAGNOSTICS (default 1) and update only
  every 200 ms. Failed display registration does not disable sensor acquisition.
  Existing OLED manager/driver are unchanged; diagnostics still use its mutex.
- [x] Add COMMANDLINK_DIAGNOSTICS (default 0) for GDB-only send/failure counts
  and last/max TX duration. No extra UART messages. Blocking send and 20 ms HAL
  timeout remain unchanged; this is instrumentation, not an async TX fix.
- [x] Test actual SensorTask helpers with fake peripherals: independent failed
  initialization, acquisition timestamps, cadence, tick wrap, invalid/timeout
  replacement, ultrasonic completion/range/start failures and display gating.
- [x] Test CommandLink with diagnostics on/off: unchanged replies, bounded HAL
  timeout argument, failure return and tick-wrap duration accounting.
- [x] Seven host test configurations pass with -Wall -Wextra -Werror: task
  OLED on/off, CommandLink diagnostics on/off, snapshot, ultrasonic driver and
  command session. ARM firmware builds with zero warnings/errors; alternative
  no-OLED and TX-diagnostics variants also cross-compile.
- [x] Restore formatting-only main.c, startup assembly and debug launch file
  to their indexed versions. Restore original wrapper authorship comments.
  All 22 controller files match pre-cleanup hashes. Staged user files untouched.
- [x] Tell the user, flash tested production ELF, verify download and reset (1 October).
- [x] Stationary STATUS/Q returned READY; ten Q U/Q I pairs were fresh and valid.
  Median query latency: ultrasonic 7.86 ms, IR 10.94 ms. Next batch ID stayed 0
  after reset; no movement batches sent. Not a fresh distance calibration.
- [x] Sample query traffic/control timing during supervised movements.
  F100/B100 repeats ran with U/IR polling; partial-batch cancellation with
  U/IR/Q P polling recorded a 14 ms maximum command update interval.
  This is observed timing for those tests, not a worst-case scheduling bound.

Debug build flags (do not put diagnostic strings on the command UART):
`-DSENSOR_OLED_DIAGNOSTICS=0` removes sensor display work;
`-DCOMMANDLINK_DIAGNOSTICS=1` exposes `CommandLink_Diagnostics` to GDB.
Each new host test includes its standalone compile command at the file start.

## Endpoint-stall investigation — 1 October 2026

- [x] Capture and reproduce intermittent near-target stalls with Pi sensor
  polling enabled. Polling-disabled pair passed, but polling-enabled pairs
  also pass; UART query traffic alone is not established as the root cause.
- [x] Add and flash diagnostic-only D W snapshot before fault braking:
  profile/wheel targets, drive/brake PWM, actuator/driver mode and dt/maxdt.
  Gated by MOTION_DIAGNOSTICS; extra query issued only after stopping.
  Host tests and ARM builds (diagnostics on/off) passed. All 47 controller/
  peripheral-driver files unchanged during this diagnostic addition.
- [x] On cycle 17 after 16 completed F100/B100 pairs: F100 stopped at
  99.3257 mm, both wheel speeds 0, DRIVE at 56.2449/57.1318% PWM,
  zero brake PWM, nonzero speed targets 129.088/149.319 CPS.
  Last/max per-command update interval 12/12 ms. STOPPED 16 confirmed.
- [x] D H register snapshot added/tested/flashed (diagnostics only). No-query
  F1 stalled at 0.329767 mm with 10/10 ms last/max dt. PWM timer compares
  matched requested ~56.5/56.8% DRIVE; counters/channel/main output enabled.
  This rules out polling being necessary, not all timing/electrical effects.
- [x] Separate no-query F5/B5 comparison completed three pairs (six DONEs),
  encoder magnitudes 4.55–4.81 mm. F2/B2 were not run after initial F1 fault.
- [ ] Diagnose measured low-speed breakaway under load with controller owner.
  Existing KI=0 means effort does not rise while stalled at fixed error;
  captured commands match current feedforward/P calculation exactly.
  This supports insufficient starting torque but does not measure actual
  voltage/current/PWM or prove encoder integrity. No calibration changed.
- [x] Investigated 1 mm tolerance for UART straight commands as an application
  acceptance workaround. Failed on cycle 3 B100 at -98.4683 mm; no fix proven.
  User-requested retry confirmed seven pairs before Pi/Tailscale connection
  loss during pair eight; final result unknown. STM reset via ST-LINK.
  Candidate removed from source; pre-candidate ELF reproduced byte-for-byte.
  Initial restore attempt could not reach the target; after user reconnection
  the restore retry flashed and download-verified the original 0.5 mm build.
  Stationary READY / next ID 0 and all three sensor queries passed. No gain/
  PWM calibration changes. Pi uptime confirms a reboot; cause unknown.
  Recovered report retained 5 pairs despite 7 confirmed live; no final result.
  No driving test was started after rollback. Power/load check remains open.
- [x] User-requested 1 mm straight-only candidate reinstalled on 1 October:
  exact earlier ELF reproduced, host regression passed, flash verified.
  Fresh-button 30 F100/B100 pairs completed with ultrasonic/IR polling
  (DONE 0..29; 60 movements), final IDLE with zero motor outputs. Candidate
  remains installed and in source; arcs retain shared 0.5 mm tolerance.
  Earlier identical-candidate failure is not erased by this successful run.
  No controller/driver calibration changes or ruler-measured pose validation.
- [ ] Choose/validate any endpoint tolerance or low-speed behavior change
  explicitly; do not silently relax the watchdog or change friend's tuning.

## Historical installed state — 30 September 2026

- Provisional Pi IR preview now deployed: checks/ir_sensors.py --calibrated.
  robot/core/ir_distance.py applies separate reciprocal lookup tables from
  calibration/sensors/ir_provisional.json. Left uses repeat-400 anchor, with
  disagreements retained; raw wire replies/default CSV remain unchanged.
  No extrapolation; stale/failed estimates omitted; retained estimates expire.
  Profile is for stationary preview only, not motion validated. No STM flash.
  All 172 Mac tests pass; Pi 170 pass plus 2 C-bridge skips. Ten live queries
  verified preview: right median 434.6 mm, range 410.2–440.5 mm (last reported
  position 450 mm, not freshly remeasured). Seven deployed hashes match.
  Next: actual obstacle-surface checks at known distances. U200 remains unimplemented.
  Earlier installed-state notes below are historical measurement records.

- Right 450 mm fresh validation: median 783.5 ADC, range 750–883,
  50 distinct valid samples. Unchanged predictions: linear 442.8 mm,
  reciprocal lookup 437.4 mm, global inverse 442.2 mm. Keep held out.
  Both sensors have 450 mm checks; no runtime conversion installed. Latest
  comparison report: ir_comparison_20260930_both450.json. Actual obstacle
  surface checks, fitted model confirmation and motor-noise checks remain.

- Left 450 mm repeat after alignment/reference instructions: median 775.5 ADC,
  range 766–850, 50 distinct valid samples. Repeat-400 candidate estimates:
  linear 448.7 mm, reciprocal lookup 443.2 mm, global inverse 447.5 mm.
  Both 450 mm captures remain held out. New Downloads copies of IMG_0761–0763
  are byte-identical to prior photos; no revised alignment visually verified.
  Shift cause remains unproven. Latest report: ir_comparison_20260930_with450repeat.json.

- Fresh left tissue-box 450 mm validation: 50 distinct valid samples, median
  725 ADC, range 715–810. Nearly identical to the earlier 500 mm median 726.
  Unchanged candidate conversions predict 487–503 mm (reciprocal lookup 500.6).
  This fails as a precise 450 mm range estimate. Keep held out and preserve
  anchors; inspect physical setup/reference/repeatability before installation.
  Latest offline report: ir_comparison_20260930_with450.json. No runtime fit installed.

- Offline IR candidate comparison is now available on Mac/Pi via
  `checks/ir_calibration.py`; explicit anchor/validation manifest and JSON report
  live under mdp_checklist/calibration/sensors. Six analysis tests pass on both.
  Compares linear ADC, reciprocal lookup and global inverse fits; retains both
  left 400 mm alternatives. No candidate selected or runtime conversion installed.
  Reciprocal lookup using repeat-left-400 gives held-out median errors -3.9,
  +5.4 and +6.0 mm; right errors +3.6 and +13.9 mm. These comparisons inform
  method choice, so fresh validation is still needed. No motion/firmware changes.

- Runtime firmware built with zero warnings/errors, flashed and byte-verified.
  `BRANCH_TO_TESTS=0`; standalone ultrasonic OLED test remains selectable in
  TestMain when test mode is needed. Runtime boot centres steering as usual.
- SensorTask samples ultrasonic and raw left/right IR independently of Pi queries.
  MotionTask remains the UART owner. No driving commands were sent during checks.
- `Q` aliases `STATUS`; `Q U` returns corrected ultrasonic distance, raw echo,
  status and age; `Q I` returns raw IR ADC/millivolts/status/age per side.
- Pi monitors installed: `checks/ultrasonic.py` and `checks/ir_sensors.py`.
  Reusable parsing is in `robot/core/sensors.py`, serial ownership in stmlink.
- User requested a provisional −5 mm ultrasonic correction and confirmed it
  looked good after flashing. Earlier approximate raw checks were 93→100,
  220→230, 290→300, and approximately 500→503 mm; the final pair was interpreted
  from an ambiguous message, not an exact recorded calibration measurement.
- First live runtime check: 20 ultrasonic samples, all valid, 112.6–117.1 mm
  with unknown target distance. Subsequent reads exercised both OK and TIMEOUT.
  Both IR ADC channels return data; physical mapping/model checks continue below.
- Left target at user-confirmed 200 mm: 50 valid ADC samples, median 1690.5,
  min 1668, max 1758. CSV preserved on Pi and Mac under mdp_checklist/calibration/sensors/.
  Left target at 300 mm: 50 valid samples, median 1216, min 1204, max 1298.
  The right channel stayed around 535–538 ADC, supporting the left-channel mapping.
  Right target at 100 mm: 50 valid samples, median 3224.5, min 3204, max 3301.
  Right target at 150 mm: 50 valid samples, median 2058.5, min 2027, max 2116.
  Right target at 200 mm: 50 valid samples, median 1618, min 1591, max 1911.
  Right target at 300 mm: 50 valid samples, median 1120.5, min 1101, max 1166.
  Right target at 400 mm: 50 valid samples, median 946.5, min 918, max 1006.
  Right target at 500 mm: 50 valid samples, median 877, min 845, max 943.
  Right target at 600 mm, first capture: median 986.5, min 947, max 1043
  (50 distinct valid ADC samples). Non-monotonic versus 500 mm: questionable,
  exclude from fitting until target alignment/nearby objects are checked and repeated.
  Right 600 mm repeat: 50 valid samples, median 994, min 948, max 1145.
  Anomaly persisted after setup-check request; both 600 mm captures remain
  excluded pending model/mounting/target inspection. Cause not established.
  Subsequent tissue-box capture: right median 587.5, min 575, max 643
  (50 valid readings). User confirmed 600 mm afterward and clarified the
  target is a tissue box. CSV metadata updated; raw samples unchanged.
  Same tissue-box face at 500 mm: median 686.5, min 674, max 744,
  50 distinct valid readings. Ordering is consistent with the tissue-box 600 mm
  capture, but differs from the original target's 500 mm median (877).
  Keep target/setup series separate; validate actual obstacle surfaces before motion.
  Same tissue-box face at 400 mm: median 856, min 845, max 909,
  50 distinct valid samples; tissue-box 400/500/600 mm medians remain ordered.
  Same tissue-box face at 300 mm: median 1098, min 1077, max 1156,
  50 distinct valid samples. Tissue-box 300–600 mm series remains ordered.
  Same tissue-box face at 200 mm: median 1549, min 1532, max 1614,
  50 distinct valid samples. Tissue-box 200–600 mm series remains ordered.
  Same tissue-box face at 150 mm: median 2041, min 2025, max 2099,
  50 distinct valid samples. Tissue-box 150–600 mm series remains ordered.
  Same tissue-box face at 100 mm: median 2848, min 2829, max 2898,
  50 distinct valid samples. Tissue-box 100–600 mm series remains ordered;
  independent intermediate-distance validation is still needed.
  Tissue-box 250 mm held-out check: median 1263, min 1251, max 1329,
  50 distinct valid right samples. User subsequently confirmed 250 mm after
  a ruler-measurement request; only right actual_mm metadata was updated.
  Offline linear ADC interpolation from 200/300 mm anchors predicts 263.4 mm
  at the median (+13.4 mm error). No conversion installed; keep out of fitting.
  Tissue-box 350 mm held-out check: median 928, min 913, max 979,
  50 distinct valid right samples. Offline linear ADC interpolation from
  300/400 mm anchors predicts 370.2 mm (+20.2 mm error). Keep out of fitting;
  no conversion installed. Left same-target calibration is being collected.
  Left sensor, same tissue-box face at confirmed 200 mm: median 1576,
  min 1565, max 1653, 50 distinct valid samples. Right channel unlabelled
  (median 506). This begins the separate left same-target calibration series.
  Left sensor, same tissue-box face at confirmed 300 mm: median 1097.5,
  min 1083, max 1205, 50 distinct valid samples. Right remains unlabelled
  (median 489.5). Left 200/300 mm medians are ordered; more points needed.
  Left sensor, same tissue-box face at confirmed 400 mm: median 898,
  min 859, max 993, 50 distinct valid samples. Right remains unlabelled
  (median 490). Left 200/300/400 mm medians remain ordered; no conversion applied.
  Left sensor, same tissue-box face at confirmed 500 mm: median 726,
  min 705, max 835, 50 distinct valid samples. Right remains unlabelled
  (median 489). Left 200–500 mm medians remain ordered; no conversion applied.
  Left sensor, same tissue-box face at confirmed 600 mm: median 576,
  min 544, max 659, 50 distinct valid samples. Right remains unlabelled
  (median 491, range 464–1640; excursion cause unknown, not a calibration point).
  Left 200–600 mm medians remain ordered; nearer points/validation still needed.
  Left sensor, same tissue-box face at confirmed 150 mm: median 2042,
  min 2027, max 2116, 50 distinct valid samples. Right remains unlabelled
  (median 487.5). Left 150–600 mm medians remain ordered; 100 mm and
  held-out validation remain outstanding. No conversion applied.
  Left sensor, same tissue-box face at confirmed 100 mm: median 2886,
  min 2868, max 2958, 50 distinct valid samples. Right remains unlabelled
  (median 489). Both sensors now have separate same-target anchor captures
  at 100, 150, 200, 300, 400, 500 and 600 mm. Left held-out validation
  remains outstanding; no conversion applied.
  Left tissue-box 250 mm held-out check: median 1307, min 1293, max 1405,
  50 distinct valid samples. Right unlabelled (median 490). Offline linear ADC
  interpolation from left 200/300 mm anchors predicts 256.2 mm (+6.2 mm error).
  Keep out of fitting; left 350 mm validation remains. No conversion installed.
  Left tissue-box 350 mm held-out check: median 926, min 913, max 1000,
  50 distinct valid samples. Right unlabelled (median 489). Offline linear ADC
  interpolation from left 300/400 mm anchors predicts 386.0 mm (+36.0 mm error).
  Left 350/400 mm raw ranges overlap; medians differ by only 28 ADC. Recheck
  400 mm setup/readings before selecting a conversion; cause not established.
  Keep 350 mm held out of fitting. No correction or conversion installed.
  Left tissue-box 400 mm repeat: median 822.5, min 813, max 890,
  50 distinct valid samples. Right unlabelled (median 488). Median differs
  from the original 400 mm capture by -75.5 ADC; preserve both. Diagnostic
  substitution of this anchor predicts 362.4 mm at the held-out 350 mm median
  (+12.4 mm). Cause of shift and representative anchor remain unestablished;
  check repeatability before choosing anchors. No conversion installed.
  Left tissue-box repeat at user-measured 351–352 mm: actual_mm=351.5
  records the interval midpoint, not exact measurement precision. Median
  920.5, min 909, max 1012, 50 distinct valid samples. Right unlabelled
  (median 491.5). Median is 5.5 ADC below prior 350 mm reading; supports
  local repeatability, not a cause for the 400 mm shift. Keep held out.
  Diagnostic interpolation with repeat 400 mm anchor predicts 364.4 mm
  (+12.9 mm against midpoint); original anchor gives 388.7 mm. No conversion installed.
  The 400/500 mm raw ranges overlap; usable distance precision needs validation.
  The left median stayed near 562–564 ADC during right-target captures,
  supporting right-channel mapping. All thirty-three captures contain 50 distinct
  valid sequences for the measured side. More distances/model confirmation
  are needed; no IR distance conversion has been fitted.
- C driver tests cover capture wrap, tick wrap, no echo, capture/timeout race,
  start failure and offset clamping. Snapshot tests cover staleness and independent
  IR status. Pi tests cover malformed replies and sensor/DONE/STOPPED interleaving.
  All 159 local Python tests passed including actual C session bridge; Pi passed
  157 with 2 C-bridge-only skips. Hardware motion
  concurrency and sensor-driven braking remain untested.

The rest of this list separates delivered code from remaining physical calibration.

## Existing implementation

- [x] Inspect IMG_0754.JPG, IMG_0755.JPG and IMG_0756.JPG: detached ultrasonic
  board is marked HC-SR04+; photographs do not establish complete pin routing.
- [x] Locate HC-SR04 timer input-capture driver and standalone OLED test.
  Existing documented mapping: TRIG PB14, ECHO PB15 / TIM12_CH2.
- [x] Locate GP2Y0A21YK ADC driver and SideIRSensorConfig: left PC1 / ADC1_IN11,
  right PC2 / ADC1_IN12. Confirm actual fitted IR model before calibration.
- [x] Identify and close the original runtime gap. The ultrasonic instance now
  belongs to PeripheralDrivers/UltrasonicSensorConfig; standalone tests retain
  a compatibility include. IR still intentionally reports raw ADC/voltage.

## Architecture decision

Use one STM FreeRTOS SensorTask with periodic scheduling for acquisition.
It yields/sleeps between work, runs below motion-control priority and owns
sensor state. Ultrasonic timing stays in TIM12 input capture; the task starts
pings and handles completion/timeouts. Only the short trigger pulse uses the
existing microsecond delay. Do not wait for echo in MotionTask or a UART handler.

Start ultrasonic triggering at 70 ms intervals (generic HC-SR04 guidance is
more than 60 ms), then characterise the actual HC-SR04+ module. Start both IR
reads at 50 ms intervals if fitted models are GP2Y0A21YK. Read their shared ADC
serially in the same owner task. These are initial schedules, not guaranteed
fresh-reading rates. Check scheduler load and actual acquisition timing.

Publish a consistent latest-value snapshot through a short protected copy:
raw value, calibrated distance when available, acquisition timestamp, sample
sequence and explicit status (not-ready, valid, timeout/error, stale/out-of-range).
Retain acquisition time, not query time. Do not report an old successful sample
as current after a failed acquisition. No echo does not mean infinite clearance.

The UART command owner replies with the snapshot without waiting for a new
measurement. Replies must be serialised with DONE/FAULT/STOPPED. On the Pi,
retain one serial reader and route sensor replies separately from batch replies;
a second reader could steal DONE. Querying faster does not refresh the sensor.

Implemented: Q U for ultrasonic and Q I for the pair of IR sensors, with
newline framing and 250 ms stale detection. Full wire fields and Pi usage are
documented in mdp_checklist/docs/SENSORS.md. STATUS compatibility is preserved.
S stays available during motion.
U200 is now a separate movement command using the same snapshot locally;
a query must never move motors.

Calibration is a Pi CLI utility using the same query API as Task 1/2. It records
raw samples and known distances; it does not create a second sensor driver.
The local STM ultrasonic approach consumes that snapshot directly, avoiding
a Pi round trip for stopping decisions.

## Phase 1: front ultrasonic

- [x] Verify board connector routing and PB15 digital input compatibility against
  the schematic/datasheet; user checked wiring and live readings worked.
- [ ] Confirm final rigid ultrasonic mounting and unobstructed forward beam.
- [x] Move sensor instance/callback ownership out of test-only code; preserve
  standalone tests without concurrent use of the same timer/handle.
- [x] Check TIM12 frequency, wrap handling and capture/timeout races.
- [x] Implement SensorTask, snapshot validity/age, query/reply and Pi support.
- [x] Add stationary live-read/CSV calibration CLI with usage at file start.
- [x] Test missing echo, stale reading, malformed reply, repeated queries and
  simulated query interleaving with movement completion/stop. No automatic motion
  in stationary utilities. HAL capture-start failure covered in host driver test.
- [ ] Inject production initialization failure and verify query behavior on hardware;
  test query interleaving during a supervised real movement.
- [ ] Record about 50 readings per ruler-measured distance, initially
  100, 150, 200, 250, 300, 500, 800 and 1000 mm where space permits.
  Measure from a consistent transducer-front reference plane, not the camera.
  Use a large flat perpendicular target first. Record temperature if available.
- [ ] Compare median error, spread and invalid-reading fraction. Fit a scale/
  offset only if data supports it; validate at separate intermediate distances.
  Preserve raw echo times and distinguish sensor correction from mounting offset.
- [ ] Repeat with actual 100 mm obstacle faces, angled targets, no target and
  motors operating. Establish usable range and filtering latency experimentally.
- [ ] Validate any sensor-driven approach at low speed, including loss of echo
  and braking distance, before replacing camera distance control.

## Phase 2: left/right IR

- [ ] Confirm both sensors' exact model, pin routing and installed direction.
- [x] Add serial ADC acquisition to SensorTask; publish raw ADC/voltage first.
- [x] Capture separate left/right tissue-box anchors at 100, 150, 200, 300,
  400, 500 and 600 mm, plus held-out checks. Retain disagreements/repeats.
  These are provisional surface-specific data, not validated driving accuracy.
- [x] Deploy provisional separate monotonic reciprocal lookup tables on Pi;
  preserve raw samples, intermediate checks and explicit out-of-range results.
  No extrapolation, stale values omitted. Runtime preview only; actual obstacle
  surfaces and motor-noise checks remain below.
- [ ] Test actual obstacle surfaces, lighting, edge detection, vibration and
  motors-on noise. Choose small filtering windows from measured response delay.
- [ ] Validate Task 2 side-edge thresholds/hysteresis and turning behaviour.

## Subsequent protocol/motion work

- [x] Add one-letter Q status alias without breaking the existing Pi client.
- [x] Implement Q P current/latest batch progress: completed count, attempted
  command, relative travel/yaw, and NOT_STARTED/ACTIVE/BRAKING/IDLE phase.
  Query only; no added normal-driving UART traffic or command history.
  Final stopped snapshot includes braking; unavailable odometry is explicit.
- [x] Implement U200 as forward ultrasonic-gap approach on STM, with fresh
  readings, low speed caps, bounded travel/time, braking and two post-stop
  confirmations. No motor/steering calibration changes; no blind retry.
- [x] Initial U/Q P firmware flashed; read-only checks, F100/B100 and one
  normal-speed U200 passed. The first 300 mm/s experiment stopped early with
  US_RANGE; that result does not validate fast approach accuracy.
- [x] Replace early predicted-distance braking with live ultrasonic endpoints
  for the friend's unchanged MotionProfile; normal and fast software tests.
- [x] Flash/verify revised profile integration and perform first floor test.
  Corrected 300 mm/s run returned DONE, immediate sensor 203.9 mm, user ruler
  confirmation 200 mm. Later readings 217.6..221.9 mm had unknown intervening
  setup changes; repeatability remains open.
- [x] Cancel U while approaching, with a following command in the same batch.
  U200;B20 returned STOPPED 0 at step 1, 0/2 complete, 75.98 mm final encoder
  travel, stable IDLE; following B20 cancelled. Evidence in afternoon summary.
- [ ] Repeat U200 at multiple starting gaps, recording physical stopped-gap
  measurements; exercise missing-echo and precondition failure handling.
  One successful approach and cancellation do not prove repeatability.
- [x] Supervised partial-batch stop passed: B100;F100;B100, S during command 2.
  STOPPED 1; Q P reported 1/3 completed, step 2 F100 at +41.95 mm encoder
  travel, IDLE. Unchanged one second later; third command cancelled.
  U-specific cancellation also passed as recorded above; no resume/replay is implied.
- [ ] Measure control timing on hardware. Q P is on demand; explicit queries
  still cost bounded blocking UART time. Never infer a global pose or replay
  a partial movement without replanning.
- [ ] Use ultrasonic for front range, side IR for obstacle sides/edges and
  camera for symbol/arrow recognition; these are complementary measurements.

## References

- Existing hardware notes: DEVELOPER_NOTES.md, HC-SR04 section.
- Generic HC-SR04 datasheet (actual + variant still needs characterisation):
  https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
- Sharp GP2Y0A21YK0F datasheet (confirm fitted model):
  https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a21yk_e.pdf
