# Developer Notes

This document records known issues, implementation quirks, observed hardware
behaviour, and design decisions that may be useful during debugging and future
development.

## Wheel Speed Controller

### Transient wheel-speed spikes

**Observed behaviour**
- During some WheelSpeedController tests, a measured wheel-speed sample may briefly spike to an unexpectedly large value.

**Cause**
- In `WheelSpeedController_Update()`, the wheel speed is computed based on the elapsed time `dt` supplied by the caller. Meanwhile, the encoder runs independently in the underlying driver `rwdriver`. This permits cases where `dt` does not necessarily reflect the actual time since last encoder reading. The mismatch in encoder count and time elapsed can generate physically impossible values for the speed.

**Impact**
- A single anomalous measurement can temporarily affect the controller output and can make plots appear much worse than the actual mechanical response.

**Notes / mitigation**
- The spikes are intended, well-defined behaviour when an incorrect time since last control is supplied. It was a consumer-side problem rather than controller flaw. 
- Do not immediately interpret an isolated spike as actual wheel acceleration.
- In the future, perhaps the controller can implement its own clock instead of relying on the consumer for timing. 

---

### Zero-speed behaviour

When `targetCps == 0`, the controller deliberately handles the wheel differently
from normal closed-loop speed control.

**Current intended behaviour**

- actively brake while the wheel is still moving;
- transition to coast once it has become stationary.

**Rationale**

- braking reaches zero speed faster than simply coasting;
- continuously braking a stationary motor is unnecessary.

**Implementation**

- The `WheelSpeedController` will brake actively on zero
target, bypassing the normal control loop.
- The `MotionController` will determine whether the robot is
in a stable stationary state before commanding to coast.
- `WheelSpeedController` really should only care about 
attaining a certain wheel speed and direction but not how
to use the motors to accomplish certain motion.

---