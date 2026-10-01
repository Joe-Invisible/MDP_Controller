#ifndef ULTRASONIC_APPROACH_CONFIG_H
#define ULTRASONIC_APPROACH_CONFIG_H

/* Ultrasonic endpoint policy: uses the existing straight motion profile.
 * Distances reference the ultrasonic transducer faces, including its existing
 * sensor correction. These settings apply only to U commands.
 */
#define ULTRASONIC_APPROACH_MIN_TARGET_MM       100.0f
#define ULTRASONIC_APPROACH_MAX_TARGET_MM      1000.0f
#define ULTRASONIC_APPROACH_TOLERANCE_MM          20.0f
#define ULTRASONIC_APPROACH_MAX_TRAVEL_MM       1000.0f
#define ULTRASONIC_APPROACH_TRAVEL_MARGIN_MM      50.0f
/* A smaller control endpoint band than the final stationary acceptance band.
 * This avoids commanding unreliable near-zero wheel speeds for sensor noise.
 */
#define ULTRASONIC_APPROACH_STOP_MARGIN_MM       10.0f
/* Compile-time override for an explicitly identified speed-test build only. */
#ifndef ULTRASONIC_APPROACH_CRUISE_MMPS
#define ULTRASONIC_APPROACH_CRUISE_MMPS          100.0f
#endif
#define ULTRASONIC_APPROACH_MAX_AGE_MS           150U
#define ULTRASONIC_APPROACH_TIMEOUT_MS         15000U
#define ULTRASONIC_APPROACH_VERIFY_MS           1000U
#define ULTRASONIC_APPROACH_VERIFY_SAMPLES         2U

#endif
