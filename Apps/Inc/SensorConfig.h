#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

/* Application sampling policy. Hardware pin/ADC configuration stays in drivers. */
#define SENSOR_ULTRASONIC_PERIOD_MS       70U
#define SENSOR_IR_PERIOD_MS               50U
#define SENSOR_TASK_POLL_MS                2U
#define SENSOR_ULTRASONIC_STALE_MS        250U
#define SENSOR_IR_STALE_MS                250U

/* Nominal ultrasonic limits, not validated accuracy over the entire range. */
#define SENSOR_ULTRASONIC_MIN_MM          20.0f
#define SENSOR_ULTRASONIC_MAX_MM        4000.0f

/* Build with -DSENSOR_OLED_DIAGNOSTICS=0 to remove sensor display work. */
#ifndef SENSOR_OLED_DIAGNOSTICS
#define SENSOR_OLED_DIAGNOSTICS             1
#endif
#define SENSOR_OLED_PERIOD_MS            200U

#endif /* SENSOR_CONFIG_H */
