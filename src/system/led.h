#ifndef SLIMENRF_SYSTEM_LED
#define SLIMENRF_SYSTEM_LED

/*
LED priorities (0 is highest)
0: boot/power
1: sensor
2: connection (esb)
3: status
4: system (persist)
*/

#define SYS_LED_PRIORITY_HIGHEST 0
#define SYS_LED_PRIORITY_BOOT 0
#define SYS_LED_PRIORITY_SENSOR 1
#define SYS_LED_PRIORITY_CONNECTION 2
#define SYS_LED_PRIORITY_STATUS 3
#define SYS_LED_PRIORITY_SYSTEM 4
#define SYS_LED_PATTERN_DEPTH 5

// RGB
// Red, Green, Blue

// Tri-color
// Red/Amber, Green, YellowGreen/White

// RG
// Red, Green

// Dual color
// Red/Amber, YellowGreen/White

// TODO: these patterns are kinda funky
enum sys_led_pattern {
	SYS_LED_PATTERN_OFF_FORCE, // ignores lower priority patterns

	SYS_LED_PATTERN_OFF, // yield to lower priority patterns
	SYS_LED_PATTERN_ON,																// Default | indicates busy
	SYS_LED_PATTERN_SHORT, // 100ms on 900ms off									// Default | indicates waiting (pairing)
	SYS_LED_PATTERN_LONG, // 500ms on 500ms off										// Default | indicates waiting
	SYS_LED_PATTERN_FLASH, // 200ms on 200ms off									// Default | indicates readiness

	SYS_LED_PATTERN_ONESHOT_POWERON, // 200ms on 200ms off, 3 times					// Default
	SYS_LED_PATTERN_ONESHOT_POWEROFF, // 250ms off, 1000ms fade to off				// Default
	SYS_LED_PATTERN_ONESHOT_PROGRESS, // 200ms on 200ms off, 2 times				// Success
	SYS_LED_PATTERN_ONESHOT_COMPLETE, // 200ms on 200ms off, 4 times				// Success

	SYS_LED_PATTERN_ON_PERSIST, // 20% duty cycle									// Success | indicates charged
	SYS_LED_PATTERN_LONG_PERSIST, // 20% duty cycle, 500ms on 500ms off				// Charging| indicates low battery
	SYS_LED_PATTERN_PULSE_PERSIST, // 5000ms pulsing								// Charging| indicates charging
	SYS_LED_PATTERN_ACTIVE_PERSIST, // 300ms on 9700ms off							// Default | indicates normal operation

	SYS_LED_PATTERN_ERROR_A, // 500ms on 500ms off, 2 times, every 5000ms			// Error
	SYS_LED_PATTERN_ERROR_B, // 500ms on 500ms off, 3 times, every 5000ms			// Error
	SYS_LED_PATTERN_ERROR_C, // 500ms on 500ms off, 4 times, every 5000ms			// Error
	SYS_LED_PATTERN_ERROR_D, // 500ms on 500ms off (same as SYS_LED_PATTERN_LONG)	// Error
};

enum sys_led_color {
	SYS_LED_COLOR_DEFAULT,
	SYS_LED_COLOR_SUCCESS,
	SYS_LED_COLOR_ERROR,
	SYS_LED_COLOR_CHARGING,
};

void set_led(enum sys_led_pattern led_pattern, int priority);

#endif